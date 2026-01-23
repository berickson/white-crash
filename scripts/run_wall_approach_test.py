#!/usr/bin/env python3
"""
Wall Approach Test & Analysis Script
=====================================

Tests the wall approach trajectory controller with model-based braking.
Collects data from robot during approach and plots velocity profile vs target.

Usage:
    # Run single test with default parameters
    python3 run_wall_approach_test.py
    
    # Run with custom parameters
    python3 run_wall_approach_test.py --target 0.2 --speed 1.0 --accel 3.0 --decel 2.0 --safety 0.1
    
    # Run a sweep of parameters
    python3 run_wall_approach_test.py --sweep

Requirements:
    - Robot in hand mode, facing wall ~1m away
    - micro-ROS agent running
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from white_crash_msgs.msg import Update
from rcl_interfaces.msg import Log
import argparse
import time
import pickle
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend - saves to file without display
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from datetime import datetime
import sys
import threading
from pathlib import Path


# Global variable to hold the output directory for the current run
_run_output_dir = None


def get_output_dir(timestamp=None, create=True):
    """Get the output directory for this run, creating if needed."""
    global _run_output_dir
    
    if _run_output_dir is None:
        if timestamp is None:
            timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        
        script_dir = Path(__file__).parent.parent
        _run_output_dir = script_dir / "test_outputs" / f"wall_approach_{timestamp}"
        
        if create:
            _run_output_dir.mkdir(parents=True, exist_ok=True)
            print(f"Output directory: {_run_output_dir}")
    
    return _run_output_dir


def output_path(filename):
    """Get full path for an output file in the current run's directory."""
    return str(get_output_dir() / filename)


class WallApproachTestClient(Node):
    """Client for running wall approach tests"""
    
    def __init__(self):
        super().__init__('wall_approach_test_client')
        
        # QoS profile to match robot
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Publisher for commands
        self.command_pub = self.create_publisher(
            String,
            '/white_crash/command',
            qos_profile
        )
        
        # Subscriber for responses
        self.response_sub = self.create_subscription(
            String,
            '/white_crash/command_response',
            self.response_callback,
            qos_profile
        )
        
        # Subscriber for update messages
        self.update_sub = self.create_subscription(
            Update,
            '/white_crash/update',
            self.update_callback,
            qos_profile
        )
        
        # Subscriber for TOF distance
        self.tof_sub = self.create_subscription(
            Range,
            '/white_crash/center_distance',
            self.tof_callback,
            qos_profile
        )
        
        # Subscriber for rosout (robot log messages)
        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout_best_effort',
            self.rosout_callback,
            qos_profile
        )
        
        # Data for current test
        self.current_test_data = []
        self.collecting = False
        self.test_start_time = None
        self.last_tof_distance = float('nan')
        
        self.last_response = None
        self.response_received = threading.Event()
        
    def rosout_callback(self, msg):
        """Print robot log messages"""
        if 'white_crash' in msg.name or msg.name == '':
            print(f"  [ROBOT] {msg.msg}")
        
    def response_callback(self, msg):
        """Handle command responses from robot"""
        self.last_response = msg.data
        self.response_received.set()
        
    def tof_callback(self, msg):
        """Update TOF distance"""
        self.last_tof_distance = msg.range
        # Debug: uncomment to verify TOF is being received
        # print(f"  [TOF] range={msg.range:.3f}m")
        
    def update_callback(self, msg):
        """Collect data from Update messages"""
        if self.collecting:
            current_time = time.time() - self.test_start_time
            
            # Average wheel velocities for linear velocity
            v_actual = (msg.left_speed + msg.right_speed) / 2.0
            v_target = msg.v_left_target + msg.v_right_target
            if not (np.isnan(msg.v_left_target) or np.isnan(msg.v_right_target)):
                v_target = (msg.v_left_target + msg.v_right_target) / 2.0
            else:
                v_target = float('nan')
            
            self.current_test_data.append({
                'time': current_time,
                'tof_distance': self.last_tof_distance,
                'v_actual': v_actual,
                'v_target': v_target,
                'v_target_linear': msg.twist_target_linear,
                'accel_target': msg.twist_target_accel_linear,
                'left_speed': msg.left_speed,
                'right_speed': msg.right_speed,
                'left_motor_command': msg.left_motor_command,
                'right_motor_command': msg.right_motor_command,
                'battery_voltage': msg.battery_voltage,
            })
            
    def send_command(self, command_text, timeout=3.0, retries=2):
        """Send a command and wait for response, with retries"""
        for attempt in range(retries + 1):
            msg = String()
            msg.data = command_text
            self.command_pub.publish(msg)
            
            self.response_received.clear()
            if self.response_received.wait(timeout=timeout):
                return self.last_response
            elif attempt < retries:
                self.get_logger().warning(f"Command timeout, retry {attempt + 1}/{retries}")
                time.sleep(0.5)
        return None
    
    def wait_for_robot_ready(self, timeout=10.0):
        """Wait until we're receiving updates from robot"""
        self.get_logger().info("Checking robot connectivity...")
        start = time.time()
        initial_count = len(self.current_test_data) if self.collecting else 0
        
        was_collecting = self.collecting
        self.collecting = True
        self.test_start_time = time.time()
        
        while time.time() - start < timeout:
            time.sleep(0.5)
            if len(self.current_test_data) > initial_count + 2:
                self.collecting = was_collecting
                self.current_test_data = []
                self.get_logger().info("Robot is responsive")
                return True
        
        self.collecting = was_collecting
        self.current_test_data = []
        self.get_logger().error("Robot not responding!")
        return False
            
    def run_wall_test(self, target_dist, max_speed, accel, decel, safety_dist, rest_time=2.0):
        """
        Run a single wall approach test.
        
        Returns dict with test config and collected data.
        """
        self.get_logger().info(
            f"Running wall test: target={target_dist}m, speed={max_speed}m/s, "
            f"accel={accel}m/s², decel={decel}m/s², safety={safety_dist}m"
        )
        
        # Configure the test
        cmd = f"wall-test {target_dist} {max_speed} {accel} {decel} {safety_dist}"
        response = self.send_command(cmd)
        if response is None or "ERROR" in str(response):
            self.get_logger().error(f"Failed to configure test: {response}")
            return None
        self.get_logger().info(f"  Config response: {response}")
            
        # Clear data and start collecting
        self.current_test_data = []
        self.collecting = True
        self.test_start_time = time.time()
        
        # Trigger wall-test mode
        response = self.send_command("set-event wall-test")
        if response is None:
            self.get_logger().error("FAILED to trigger wall-test - no response!")
            time.sleep(0.5)
            response = self.send_command("set-event wall-test")
            if response is None:
                self.get_logger().error("Second attempt also failed!")
        else:
            self.get_logger().info(f"  Event response: {response}")
        
        # Wait for test to complete
        max_wait = 15.0
        start = time.time()
        robot_stopped = False
        
        while time.time() - start < max_wait:
            time.sleep(0.1)
            
            if len(self.current_test_data) > 10:
                recent = self.current_test_data[-5:]
                avg_speed = sum(abs(d['v_actual']) for d in recent) / len(recent)
                
                # Check if robot has stopped (arrived or emergency stop)
                if avg_speed < 0.03:
                    if not robot_stopped:
                        robot_stopped = True
                        # Continue collecting for 0.5s after stop for settling data
                        time.sleep(0.5)
                    else:
                        break
        
        # Stop collecting
        self.collecting = False
        
        # Return to hand mode
        self.send_command("set-event hand")
        time.sleep(0.5)
        self.send_command("set-event hand")
        
        # Rest before next test
        time.sleep(rest_time)
        
        # Compute statistics
        if self.current_test_data:
            distances = [d['tof_distance'] for d in self.current_test_data if not np.isnan(d['tof_distance'])]
            final_distance = distances[-1] if distances else float('nan')
            start_distance = distances[0] if distances else float('nan')
            speeds = [d['v_actual'] for d in self.current_test_data]
            peak_velocity = max(speeds) if speeds else 0.0
        else:
            final_distance = float('nan')
            start_distance = float('nan')
            peak_velocity = 0.0
        
        result = {
            'target_distance': target_dist,
            'max_speed': max_speed,
            'accel': accel,
            'decel': decel,
            'safety_distance': safety_dist,
            'data': self.current_test_data.copy(),
            'timestamp': datetime.now().isoformat(),
            'start_distance': start_distance,
            'final_distance': final_distance,
            'distance_error': final_distance - target_dist if not np.isnan(final_distance) else float('nan'),
            'peak_velocity': peak_velocity,
            'valid': len(self.current_test_data) > 20 and peak_velocity > 0.1,
        }
        
        self.get_logger().info(
            f"Test complete: {len(result['data'])} samples, peak={peak_velocity:.2f} m/s, "
            f"final_dist={final_distance:.3f}m, error={result['distance_error']*1000:.1f}mm"
        )
        return result


def plot_wall_approach_results(result, save_path=None):
    """Plot the wall approach test results"""
    data = result['data']
    if not data:
        print("No data to plot!")
        return
    
    times = np.array([d['time'] for d in data])
    distances = np.array([d['tof_distance'] for d in data])
    v_actual = np.array([d['v_actual'] for d in data])
    v_target = np.array([d['v_target'] for d in data])
    accel_target = np.array([d['accel_target'] for d in data])
    left_cmd = np.array([d['left_motor_command'] for d in data])
    right_cmd = np.array([d['right_motor_command'] for d in data])
    
    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    
    # Title with test parameters
    fig.suptitle(
        f"Wall Approach Test: target={result['target_distance']}m, "
        f"max_speed={result['max_speed']}m/s, accel={result['accel']}m/s², decel={result['decel']}m/s²\n"
        f"Final distance: {result['final_distance']:.3f}m, Error: {result['distance_error']*1000:.1f}mm",
        fontsize=12
    )
    
    # Plot 1: Distance to wall
    ax1 = axes[0]
    ax1.plot(times, distances * 1000, 'b-', linewidth=1.5, label='TOF Distance')
    ax1.axhline(y=result['target_distance'] * 1000, color='g', linestyle='--', label='Target')
    ax1.axhline(y=result['safety_distance'] * 1000, color='r', linestyle='--', label='Safety')
    ax1.set_ylabel('Distance (mm)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Distance to Wall')
    
    # Plot 2: Velocity (actual vs target)
    ax2 = axes[1]
    ax2.plot(times, v_actual, 'b-', linewidth=1.5, label='v_actual')
    ax2.plot(times, v_target, 'r--', linewidth=1.5, label='v_target')
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax2.set_ylabel('Velocity (m/s)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Velocity Profile')
    
    # Plot 3: Commanded acceleration
    ax3 = axes[2]
    ax3.plot(times, accel_target, 'g-', linewidth=1.5, label='accel_target')
    ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax3.axhline(y=result['accel'], color='b', linestyle=':', alpha=0.5, label=f"+accel ({result['accel']})")
    ax3.axhline(y=-result['decel'], color='r', linestyle=':', alpha=0.5, label=f"-decel ({-result['decel']})")
    ax3.set_ylabel('Acceleration (m/s²)')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Commanded Acceleration (feedforward)')
    
    # Plot 4: Motor commands
    ax4 = axes[3]
    ax4.plot(times, left_cmd, 'b-', linewidth=1, label='Left motor')
    ax4.plot(times, right_cmd, 'r-', linewidth=1, label='Right motor')
    ax4.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax4.set_ylabel('Motor PWM')
    ax4.set_xlabel('Time (s)')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    ax4.set_title('Motor Commands')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved plot to {save_path}")
        plt.close(fig)
    else:
        plt.show()


def plot_velocity_vs_distance(result, save_path=None):
    """Plot velocity vs distance (phase plot) with theoretical braking curve"""
    data = result['data']
    if not data:
        print("No data to plot!")
        return
    
    distances = np.array([d['tof_distance'] for d in data])
    v_actual = np.array([d['v_actual'] for d in data])
    v_target = np.array([d['v_target'] for d in data])
    
    # Filter out NaN values
    valid = ~np.isnan(distances) & ~np.isnan(v_actual)
    distances = distances[valid]
    v_actual = v_actual[valid]
    v_target_filtered = v_target[valid]
    
    # Compute theoretical braking curve: v = sqrt(2 * decel * (d - target))
    target_dist = result['target_distance']
    decel = result['decel']
    max_speed = result['max_speed']
    
    d_theory = np.linspace(target_dist, max(distances), 100)
    v_theory = np.sqrt(2 * decel * (d_theory - target_dist))
    v_theory = np.minimum(v_theory, max_speed)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    ax.scatter(distances * 1000, v_actual, c='blue', s=10, alpha=0.6, label='Actual velocity')
    ax.scatter(distances * 1000, v_target_filtered, c='red', s=10, alpha=0.4, label='Target velocity')
    ax.plot(d_theory * 1000, v_theory, 'g-', linewidth=2, label=f'Theoretical braking curve (decel={decel})')
    
    ax.axvline(x=target_dist * 1000, color='k', linestyle='--', label='Target distance')
    ax.axhline(y=max_speed, color='orange', linestyle=':', label=f'Max speed ({max_speed})')
    
    ax.set_xlabel('Distance from wall (mm)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title(
        f"Velocity vs Distance Phase Plot\n"
        f"Target={target_dist}m, Error={result['distance_error']*1000:.1f}mm"
    )
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved phase plot to {save_path}")
        plt.close(fig)
    else:
        plt.show()


def run_single_test(args):
    """Run a single wall approach test"""
    print(f"\n{'='*70}")
    print(f"Wall Approach Test")
    print(f"{'='*70}\n")
    
    rclpy.init()
    client = WallApproachTestClient()
    
    # Use executor for cleaner shutdown
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(client)
    
    # Spin ROS in background thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        print("Connecting to robot...")
        time.sleep(2.0)
        
        if not client.wait_for_robot_ready(timeout=5.0):
            print("Robot not responding, exiting")
            return
        
        print(f"\nStarting wall approach test...")
        print(f"  Target distance: {args.target}m")
        print(f"  Max speed: {args.speed}m/s")
        print(f"  Acceleration: {args.accel}m/s²")
        print(f"  Deceleration: {args.decel}m/s²")
        print(f"  Safety distance: {args.safety}m")
        
        input("\n>>> Position robot ~1m from wall, then press ENTER to start test...")
        
        result = client.run_wall_test(
            args.target, args.speed, args.accel, args.decel, args.safety
        )
        
        if result and result.get('valid', False):
            # Save data
            data_file = output_path("wall_approach_data.pkl")
            with open(data_file, 'wb') as f:
                pickle.dump(result, f)
            print(f"\nSaved data to {data_file}")
            
            # Plot results
            plot_wall_approach_results(result, output_path("wall_approach_time.png"))
            plot_velocity_vs_distance(result, output_path("wall_approach_phase.png"))
            
            print(f"\n{'='*70}")
            print(f"Test Summary:")
            print(f"  Start distance:  {result['start_distance']*1000:.1f}mm")
            print(f"  Target distance: {result['target_distance']*1000:.1f}mm")
            print(f"  Final distance:  {result['final_distance']*1000:.1f}mm")
            print(f"  Position error:  {result['distance_error']*1000:.1f}mm")
            print(f"  Peak velocity:   {result['peak_velocity']:.2f}m/s")
            print(f"{'='*70}")
        else:
            print("\nTest failed or invalid data collected")
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        executor.shutdown()
        client.destroy_node()
        rclpy.try_shutdown()
        import os
        os._exit(0)  # Force clean exit to avoid ROS2 cleanup issues


def run_parameter_sweep(args):
    """Run a sweep of parameters to find optimal settings"""
    print(f"\n{'='*70}")
    print(f"Wall Approach Parameter Sweep")
    print(f"{'='*70}\n")
    
    rclpy.init()
    client = WallApproachTestClient()
    
    # Use executor for cleaner shutdown
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(client)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # Parameter combinations to test
    test_configs = [
        # (target_dist, max_speed, accel, decel, safety_dist, description)
        (0.20, 0.5, 2.0, 1.5, 0.12, "Conservative baseline"),
        (0.20, 1.0, 2.0, 1.5, 0.12, "Faster approach"),
        (0.20, 1.0, 3.0, 2.0, 0.12, "Higher accel/decel"),
        (0.20, 1.5, 4.0, 2.5, 0.10, "Aggressive"),
        (0.20, 2.0, 5.0, 3.0, 0.10, "Very aggressive"),
        (0.15, 1.0, 3.0, 2.0, 0.08, "Closer target"),
    ]
    
    all_results = []
    
    try:
        print("Connecting to robot...")
        time.sleep(2.0)
        
        if not client.wait_for_robot_ready(timeout=5.0):
            print("Robot not responding, exiting")
            return
        
        for i, (target, speed, accel, decel, safety, desc) in enumerate(test_configs):
            print(f"\n[{i+1}/{len(test_configs)}] {desc}")
            print(f"  target={target}m, speed={speed}m/s, accel={accel}, decel={decel}")
            
            input(f"\n>>> Position robot ~1m from wall, then press ENTER...")
            
            result = client.run_wall_test(target, speed, accel, decel, safety)
            
            if result:
                result['description'] = desc
                all_results.append(result)
                
                # Save progress
                progress_file = output_path("sweep_progress.pkl")
                with open(progress_file, 'wb') as f:
                    pickle.dump(all_results, f)
        
        # Summary
        print(f"\n{'='*70}")
        print(f"Sweep Summary:")
        print(f"{'='*70}")
        print(f"{'Description':<25} {'Speed':>6} {'Accel':>6} {'Decel':>6} {'Error':>10} {'Peak v':>8}")
        print(f"{'-'*70}")
        
        for r in all_results:
            err_mm = r['distance_error'] * 1000 if not np.isnan(r['distance_error']) else float('nan')
            print(f"{r['description']:<25} {r['max_speed']:>6.1f} {r['accel']:>6.1f} {r['decel']:>6.1f} "
                  f"{err_mm:>9.1f}mm {r['peak_velocity']:>7.2f}")
        
        # Save final results
        final_file = output_path("sweep_results.pkl")
        with open(final_file, 'wb') as f:
            pickle.dump(all_results, f)
        print(f"\nSaved results to {final_file}")
        
        # Generate summary PDF
        generate_sweep_report(all_results)
        
    except KeyboardInterrupt:
        print("\n\nSweep interrupted by user")
    finally:
        executor.shutdown()
        client.destroy_node()
        rclpy.shutdown()


def generate_sweep_report(results):
    """Generate a PDF report of all sweep results"""
    pdf_path = output_path("sweep_report.pdf")
    
    with PdfPages(pdf_path) as pdf:
        for i, result in enumerate(results):
            if not result.get('data'):
                continue
                
            # Time-series plot
            fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
            
            data = result['data']
            times = np.array([d['time'] for d in data])
            distances = np.array([d['tof_distance'] for d in data])
            v_actual = np.array([d['v_actual'] for d in data])
            v_target = np.array([d['v_target'] for d in data])
            accel_target = np.array([d['accel_target'] for d in data])
            
            fig.suptitle(
                f"Test {i+1}: {result.get('description', 'N/A')}\n"
                f"speed={result['max_speed']}, accel={result['accel']}, decel={result['decel']}, "
                f"error={result['distance_error']*1000:.1f}mm",
                fontsize=11
            )
            
            axes[0].plot(times, distances * 1000, 'b-')
            axes[0].axhline(y=result['target_distance'] * 1000, color='g', linestyle='--')
            axes[0].set_ylabel('Distance (mm)')
            axes[0].grid(True, alpha=0.3)
            
            axes[1].plot(times, v_actual, 'b-', label='actual')
            axes[1].plot(times, v_target, 'r--', label='target')
            axes[1].set_ylabel('Velocity (m/s)')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)
            
            axes[2].plot(times, accel_target, 'g-')
            axes[2].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
            axes[2].set_ylabel('Accel cmd (m/s²)')
            axes[2].set_xlabel('Time (s)')
            axes[2].grid(True, alpha=0.3)
            
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)
    
    print(f"Generated report: {pdf_path}")


def main():
    parser = argparse.ArgumentParser(description='Wall Approach Test Script')
    
    # Test parameters
    parser.add_argument('--target', type=float, default=0.20,
                        help='Target distance from wall (m)')
    parser.add_argument('--speed', type=float, default=0.5,
                        help='Maximum approach speed (m/s)')
    parser.add_argument('--accel', type=float, default=2.0,
                        help='Acceleration rate (m/s²)')
    parser.add_argument('--decel', type=float, default=1.5,
                        help='Deceleration rate (m/s²)')
    parser.add_argument('--safety', type=float, default=0.12,
                        help='Safety stop distance (m)')
    
    # Mode selection
    parser.add_argument('--sweep', action='store_true',
                        help='Run parameter sweep instead of single test')
    parser.add_argument('--analyze', type=str, default=None,
                        help='Analyze existing data file instead of running test')
    
    args = parser.parse_args()
    
    if args.analyze:
        # Load and plot existing data
        print(f"Loading data from {args.analyze}")
        with open(args.analyze, 'rb') as f:
            result = pickle.load(f)
        
        if isinstance(result, list):
            # Sweep results
            generate_sweep_report(result)
            for i, r in enumerate(result):
                print(f"Test {i+1}: error={r['distance_error']*1000:.1f}mm")
        else:
            # Single test
            plot_wall_approach_results(result)
            plot_velocity_vs_distance(result)
    elif args.sweep:
        run_parameter_sweep(args)
    else:
        run_single_test(args)


if __name__ == '__main__':
    main()
