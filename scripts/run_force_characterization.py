#!/usr/bin/env python3
"""
Force Characterization Test & Analysis Pipeline (Python-driven)
================================================================

Python controls the test loop, robot runs one test at a time.

Usage:
    # Run all tests on rack (wheels off ground)
    python3 run_force_characterization.py --stage rack
    
    # Run single test
    python3 run_force_characterization.py --single coast 1.0 0.0
    
    # Analyze existing data file
    python3 run_force_characterization.py --analyze data_2026_01_20.pkl

Requirements:
    - Robot in hand mode
    - micro-ROS agent running
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from white_crash_msgs.msg import Update
import argparse
import time
import pickle
import numpy as np
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from datetime import datetime
import sys
import threading
import yaml


from rcl_interfaces.msg import Log


class ForceTestClient(Node):
    """Client for running individual force characterization tests"""
    
    def __init__(self, stage='rack'):
        super().__init__('force_test_client')
        self.stage = stage  # 'rack' or 'ground'
        
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
        
        # Subscriber for rosout (robot log messages)
        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            qos_profile
        )
        
        # Data for current test
        self.current_test_data = []
        self.collecting = False
        self.test_start_time = None
        
        self.last_response = None
        self.response_received = threading.Event()
        
    def rosout_callback(self, msg):
        """Print robot log messages"""
        # Filter to only show messages from white_crash
        if 'white_crash' in msg.name or msg.name == '':
            print(f"  [ROBOT] {msg.msg}")
        
    def response_callback(self, msg):
        """Handle command responses from robot"""
        self.last_response = msg.data
        self.response_received.set()
        
    def update_callback(self, msg):
        """Collect data from Update messages"""
        if self.collecting:
            current_time = time.time() - self.test_start_time
            self.current_test_data.append({
                'time': current_time,
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
        
        # Temporarily enable collecting to check for updates
        was_collecting = self.collecting
        self.collecting = True
        self.test_start_time = time.time()
        
        while time.time() - start < timeout:
            time.sleep(0.5)
            if len(self.current_test_data) > initial_count + 2:
                self.collecting = was_collecting
                self.current_test_data = []  # Clear test data
                self.get_logger().info("Robot is responsive")
                return True
        
        self.collecting = was_collecting
        self.current_test_data = []
        self.get_logger().error("Robot not responding!")
        return False
            
    def run_single_test(self, mode, velocity, param, rest_time=2.0):
        """
        Run a single force characterization test.
        
        Returns dict with test config and collected data.
        """
        self.get_logger().info(f"Running test: {mode} @ {velocity} m/s, param={param}")
        
        # Configure the test
        response = self.send_command(f"force-test {mode} {velocity} {param}")
        if response is None or "ERROR" in response:
            self.get_logger().error(f"Failed to configure test: {response}")
            return None
        self.get_logger().info(f"  Config response: {response}")
            
        # Clear data and start collecting
        self.current_test_data = []
        self.collecting = True
        self.test_start_time = time.time()
        
        # Trigger force-char mode (use stage-specific event)
        event_name = f"force-char-{self.stage}"
        response = self.send_command(f"set-event {event_name}")
        if response is None:
            self.get_logger().error(f"FAILED to trigger {event_name} - no response!")
            # Try again
            time.sleep(0.5)
            response = self.send_command(f"set-event {event_name}")
            if response is None:
                self.get_logger().error(f"Second attempt also failed!")
        else:
            self.get_logger().info(f"  Event response: {response}")
        
        # Wait for test to complete (accel + settle + test = ~9 seconds typical)
        # Add buffer for safety
        max_wait = 15.0
        start = time.time()
        got_velocity = False  # Track if we ever reached meaningful velocity
        
        while time.time() - start < max_wait:
            time.sleep(0.5)
            
            if len(self.current_test_data) > 10:
                recent = self.current_test_data[-10:]
                avg_speed = sum(d['left_speed'] + d['right_speed'] for d in recent) / (2 * len(recent))
                
                # Mark that we reached target velocity (within tolerance)
                if avg_speed > velocity * 0.7:
                    got_velocity = True
                
                # Only check for stop AFTER we've reached velocity
                if got_velocity and avg_speed < 0.05:
                    time.sleep(0.5)  # Capture a bit more
                    break
        
        # Stop collecting
        self.collecting = False
        
        # Return to hand mode - send twice with delay to ensure FSM processes it
        self.send_command("set-event hand")
        time.sleep(0.5)  # Wait for FSM state transition to complete
        self.send_command("set-event hand")  # Send again in case first was during force-char
        
        # Rest before next test
        time.sleep(rest_time)
        
        # Validate result - check if we actually got meaningful data
        peak_velocity = 0.0
        if self.current_test_data:
            speeds = [(d['left_speed'] + d['right_speed']) / 2 for d in self.current_test_data]
            peak_velocity = max(speeds) if speeds else 0.0
        
        result = {
            'mode': mode,
            'target_velocity': velocity,
            'param': param,
            'data': self.current_test_data.copy(),
            'timestamp': datetime.now().isoformat(),
            'peak_velocity': peak_velocity,
            'valid': peak_velocity > velocity * 0.5,  # Consider valid if reached 50% of target
        }
        
        self.get_logger().info(f"Test complete: {len(result['data'])} samples, peak={peak_velocity:.2f} m/s, valid={result['valid']}")
        return result


def build_test_sequence(stage='rack'):
    """Build the test sequence based on stage (rack or ground)"""
    tests = []
    
    if stage == 'ground':
        # GROUND TESTS: Minimal informative set (~10 tests)
        # Wall hits are acceptable at these speeds
        # Note: Pure brake may cause wheel lockup/sliding - detectable via TOF
        
        # Test 1-3: Coast at different velocities (friction model)
        for v in [0.5, 0.7, 1.0]:
            tests.append(('coast', v, 0.0))
        
        # Test 4-6: Pure brake at different velocities (max braking)
        for v in [0.5, 0.7, 1.0]:
            tests.append(('pure_brake', v, 1.0))
        
        # Test 7-9: Proportional brake 50% (verify brake linearity)
        for v in [0.5, 0.7, 1.0]:
            tests.append(('prop_brake', v, 0.5))
        
        # Test 10: Proportional brake 25% (one more data point)
        tests.append(('prop_brake', 0.7, 0.25))
        
    else:
        # RACK TESTS: Original higher velocities (wheels off ground)
        
        # Test 1: Reduced Forward PWM (20 trials)
        pwm_levels = [0.8, 0.6, 0.4, 0.2, 0.0]
        velocities_1 = [0.5, 1.0, 2.0, 3.0]
        for v in velocities_1:
            for pwm in pwm_levels:
                tests.append(('reduced_pwm', v, pwm))
        
        # Test 2: Coast (6 trials)
        velocities_2 = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
        for v in velocities_2:
            tests.append(('coast', v, 0.0))
        
        # Test 3: Proportional Braking (20 trials)
        brake_levels = [0.2, 0.4, 0.6, 0.8, 1.0]
        velocities_3 = [0.5, 1.0, 2.0, 3.0]
        for v in velocities_3:
            for brake in brake_levels:
                tests.append(('prop_brake', v, brake))
        
        # Test 4: Pure Brake (6 trials)
        for v in velocities_2:
            tests.append(('pure_brake', v, 1.0))
        
        # Test 5: Reverse Torque (6 trials) - CAREFUL, low speeds only!
        reverse_pwm = [0.1, 0.2, 0.3]
        velocities_5 = [0.2, 0.4]
        for v in velocities_5:
            for pwm in reverse_pwm:
                tests.append(('reverse', v, pwm))
    
    return tests


def run_test_suite(stage='rack'):
    """Run the full test suite with Python driving the loop"""
    print(f"\n{'='*70}")
    print(f"Force Characterization Test Suite - {stage.upper()}")
    print(f"{'='*70}\n")
    
    rclpy.init()
    client = ForceTestClient(stage=stage)
    
    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()
    
    # Build test sequence
    tests = build_test_sequence(stage=stage)
    print(f"Total tests to run: {len(tests)}")
    
    # Results storage
    all_results = []
    failed_tests = []  # Track failed tests for retry
    max_retries = 2
    
    # Ground tests require manual repositioning
    manual_reposition = (stage == 'ground')
    
    try:
        # Give ROS time to connect
        print("Connecting to robot...")
        time.sleep(2.0)
        
        # Run each test
        for i, (mode, velocity, param) in enumerate(tests):
            print(f"\n[{i+1}/{len(tests)}] Testing: {mode} @ {velocity} m/s, param={param}")
            
            # For ground tests, wait for user to reposition robot
            if manual_reposition:
                input("  >>> Position robot ~1m from wall, then press ENTER to run test...")
                
                # Verify robot is still responsive before running test
                if not client.wait_for_robot_ready(timeout=5.0):
                    print("  ⚠ Robot not responding, skipping test")
                    failed_tests.append((mode, velocity, param))
                    continue
            
            result = client.run_single_test(mode, velocity, param)
            
            if result and result.get('valid', False):
                all_results.append(result)
                # Save incrementally (in case of crash)
                temp_file = f"force_char_progress_{stage}.pkl"
                with open(temp_file, 'wb') as f:
                    pickle.dump({'stage': stage, 'results': all_results}, f)
                print(f"  ✓ Valid test, saved progress ({len(all_results)} good tests)")
            else:
                peak = result.get('peak_velocity', 0) if result else 0
                print(f"  ✗ Invalid test (peak={peak:.2f}), will retry later")
                failed_tests.append((mode, velocity, param))
        
        # Retry failed tests
        if failed_tests:
            print(f"\n{'='*70}")
            print(f"Retrying {len(failed_tests)} failed tests...")
            print(f"{'='*70}")
            
            for retry in range(max_retries):
                if not failed_tests:
                    break
                    
                print(f"\nRetry attempt {retry + 1}/{max_retries}")
                still_failed = []
                
                for mode, velocity, param in failed_tests:
                    print(f"  Retrying: {mode} @ {velocity} m/s, param={param}")
                    
                    # For ground tests, wait for user to reposition
                    if manual_reposition:
                        input("  >>> Position robot ~1m from wall, then press ENTER...")
                    else:
                        # Rack tests: just wait for stabilization
                        time.sleep(3.0)
                    
                    result = client.run_single_test(mode, velocity, param, rest_time=3.0)
                    
                    if result and result.get('valid', False):
                        all_results.append(result)
                        temp_file = f"force_char_progress_{stage}.pkl"
                        with open(temp_file, 'wb') as f:
                            pickle.dump({'stage': stage, 'results': all_results}, f)
                        print(f"    ✓ Retry succeeded!")
                    else:
                        still_failed.append((mode, velocity, param))
                        print(f"    ✗ Retry failed")
                
                failed_tests = still_failed
        
        print(f"\n{'='*70}")
        print(f"All tests complete! {len(all_results)} valid, {len(failed_tests)} failed")
        print(f"{'='*70}\n")
        
        # Save final results
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        data_file = f"force_char_data_{timestamp}.pkl"
        with open(data_file, 'wb') as f:
            pickle.dump({'stage': stage, 'results': all_results, 'timestamp': timestamp}, f)
        print(f"Data saved to: {data_file}")
        
        return data_file
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        data_file = f"force_char_data_{timestamp}_interrupted.pkl"
        with open(data_file, 'wb') as f:
            pickle.dump({'stage': stage, 'results': all_results, 'timestamp': timestamp}, f)
        print(f"Partial data saved to: {data_file}")
        return data_file
        
    finally:
        client.destroy_node()
        rclpy.shutdown()


def run_single_test(mode, velocity, param):
    """Run a single test for debugging"""
    print(f"\n{'='*70}")
    print(f"Single Force Test: {mode} @ {velocity} m/s, param={param}")
    print(f"{'='*70}\n")
    
    rclpy.init()
    client = ForceTestClient(stage='rack')  # Default to rack for single tests
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()
    
    try:
        print("Connecting to robot...")
        time.sleep(2.0)
        
        result = client.run_single_test(mode, velocity, param)
        
        if result:
            print(f"\nTest complete: {len(result['data'])} samples")
            
            # Quick plot
            if result['data']:
                times = [d['time'] for d in result['data']]
                speeds = [(d['left_speed'] + d['right_speed']) / 2 for d in result['data']]
                
                plt.figure(figsize=(10, 4))
                plt.plot(times, speeds, 'b-', linewidth=2)
                plt.xlabel('Time (s)')
                plt.ylabel('Velocity (m/s)')
                plt.title(f'{mode} @ {velocity} m/s, param={param}')
                plt.grid(True, alpha=0.3)
                plt.savefig(f'single_test_{mode}_{velocity}_{param}.png')
                plt.show()
                print(f"Plot saved to: single_test_{mode}_{velocity}_{param}.png")
        else:
            print("Test failed!")
            
    finally:
        client.destroy_node()
        rclpy.shutdown()


class ForceCharacterizationAnalyzer:
    """Analyzes collected force characterization data"""
    
    def __init__(self, data_file):
        """Load data from pickle file"""
        with open(data_file, 'rb') as f:
            self.data_dict = pickle.load(f)
        
        self.stage = self.data_dict.get('stage', 'unknown')
        self.results = self.data_dict.get('results', [])
        
        print(f"Loaded data from {data_file}")
        print(f"Stage: {self.stage}")
        print(f"Total tests: {len(self.results)}")
        
    def analyze_trial(self, result):
        """Analyze a single trial result"""
        data = result['data']
        if not data:
            return None
            
        t = np.array([d['time'] for d in data])
        left_speed = np.array([d['left_speed'] for d in data])
        right_speed = np.array([d['right_speed'] for d in data])
        avg_speed = (left_speed + right_speed) / 2.0
        
        # Find peak velocity
        peak_idx = np.argmax(avg_speed)
        peak_velocity = avg_speed[peak_idx]
        
        # Deceleration phase
        decel_data = avg_speed[peak_idx:]
        decel_t = t[peak_idx:] - t[peak_idx]
        
        # Compute deceleration if enough data
        if len(decel_data) > 10:
            try:
                v_smooth = savgol_filter(decel_data, min(11, len(decel_data)//2*2+1), 3)
                dt = np.diff(decel_t)
                dv = np.diff(v_smooth)
                deceleration = dv / np.where(dt > 0, dt, 1e-6)
            except:
                deceleration = np.array([])
        else:
            deceleration = np.array([])
        
        return {
            'peak_velocity': peak_velocity,
            'target_velocity': result['target_velocity'],
            'mode': result['mode'],
            'param': result['param'],
            'decel_t': decel_t,
            'decel_v': decel_data,
            'deceleration': deceleration,
        }
    
    def generate_report(self, output_pdf):
        """Generate comprehensive PDF report"""
        print(f"\nGenerating report: {output_pdf}")
        
        with PdfPages(output_pdf) as pdf:
            # Summary page
            fig, ax = plt.subplots(figsize=(11, 8.5))
            ax.text(0.5, 0.9, 'Force Characterization Report', 
                   ha='center', fontsize=20, fontweight='bold',
                   transform=ax.transAxes)
            ax.text(0.5, 0.7, f'Stage: {self.stage}', 
                   ha='center', fontsize=14, transform=ax.transAxes)
            ax.text(0.5, 0.6, f'Total tests: {len(self.results)}', 
                   ha='center', fontsize=14, transform=ax.transAxes)
            ax.text(0.5, 0.5, f'Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}', 
                   ha='center', fontsize=12, transform=ax.transAxes)
            ax.axis('off')
            pdf.savefig(fig)
            plt.close()
            
            # Group results by mode
            by_mode = {}
            for result in self.results:
                mode = result['mode']
                if mode not in by_mode:
                    by_mode[mode] = []
                by_mode[mode].append(result)
            
            # Plot each mode
            for mode, results in by_mode.items():
                fig, axes = plt.subplots(2, 1, figsize=(11, 8.5))
                fig.suptitle(f'Mode: {mode} ({len(results)} tests)', fontsize=14)
                
                for result in results:
                    data = result['data']
                    if not data:
                        continue
                    t = [d['time'] for d in data]
                    v = [(d['left_speed'] + d['right_speed']) / 2 for d in data]
                    cmd = [(d['left_motor_command'] + d['right_motor_command']) / 2 for d in data]
                    
                    label = f"v={result['target_velocity']}, p={result['param']}"
                    axes[0].plot(t, v, alpha=0.7, linewidth=1, label=label)
                    axes[1].plot(t, cmd, alpha=0.7, linewidth=1)
                
                axes[0].set_ylabel('Velocity (m/s)')
                axes[0].set_title('Velocity vs Time')
                axes[0].grid(True, alpha=0.3)
                axes[0].legend(fontsize=6, ncol=3)
                
                axes[1].set_ylabel('Motor Command')
                axes[1].set_xlabel('Time (s)')
                axes[1].set_title('Motor Command vs Time')
                axes[1].grid(True, alpha=0.3)
                
                plt.tight_layout()
                pdf.savefig(fig)
                plt.close()
        
        print(f"Report saved to {output_pdf}")
    
    def export_summary(self, output_yaml):
        """Export summary statistics to YAML"""
        summary = {
            'metadata': {
                'stage': self.stage,
                'num_tests': len(self.results),
                'timestamp': self.data_dict.get('timestamp', 'unknown'),
            },
            'tests_by_mode': {}
        }
        
        for result in self.results:
            mode = result['mode']
            if mode not in summary['tests_by_mode']:
                summary['tests_by_mode'][mode] = []
            
            analysis = self.analyze_trial(result)
            if analysis:
                summary['tests_by_mode'][mode].append({
                    'target_velocity': result['target_velocity'],
                    'param': result['param'],
                    'peak_velocity': float(analysis['peak_velocity']),
                    'num_samples': len(result['data']),
                })
        
        with open(output_yaml, 'w') as f:
            yaml.dump(summary, f, default_flow_style=False)
        
        print(f"Summary exported to {output_yaml}")


def analyze_data(data_file):
    """Analyze existing data file"""
    print(f"\n{'='*70}")
    print("Force Characterization Analysis")
    print(f"{'='*70}\n")
    
    analyzer = ForceCharacterizationAnalyzer(data_file)
    
    timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    
    # Generate report
    report_pdf = f"force_char_report_{timestamp}.pdf"
    analyzer.generate_report(report_pdf)
    
    # Export summary
    summary_yaml = f"force_char_summary_{timestamp}.yaml"
    analyzer.export_summary(summary_yaml)
    
    print(f"\n{'='*70}")
    print("Analysis complete!")
    print(f"  Report: {report_pdf}")
    print(f"  Summary: {summary_yaml}")
    print(f"{'='*70}\n")


def main():
    parser = argparse.ArgumentParser(
        description='Force characterization test and analysis (Python-driven)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--stage', choices=['rack', 'ground'],
                      help='Run full test suite on rack or ground')
    group.add_argument('--single', nargs=3, metavar=('MODE', 'VELOCITY', 'PARAM'),
                      help='Run single test: --single coast 1.0 0.0')
    group.add_argument('--analyze', metavar='FILE',
                      help='Analyze existing data file')
    
    args = parser.parse_args()
    
    if args.analyze:
        analyze_data(args.analyze)
    elif args.single:
        mode, velocity, param = args.single
        run_single_test(mode, float(velocity), float(param))
    else:
        data_file = run_test_suite(args.stage)
        if data_file:
            analyze_data(data_file)


if __name__ == '__main__':
    main()
