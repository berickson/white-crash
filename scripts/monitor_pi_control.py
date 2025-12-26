#!/usr/bin/env python3
"""
PI Control Monitor Script for Step 2
Triggers PI control test mode and monitors commanded vs actual velocities in real-time.

Usage:
    python3 monitor_pi_control.py [--plot]
    
    Without --plot: Terminal output only
    With --plot: Real-time matplotlib plot

Requirements:
    - ROS 2 Jazzy
    - Robot ready for testing (on racks or ground)
    - micro-ROS agent running
    - matplotlib (for plotting): pip install matplotlib
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from white_crash_msgs.msg import Update
import time
import argparse
import sys

# Optional matplotlib for plotting
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

class PIControlMonitor(Node):
    def __init__(self, enable_plot=False):
        super().__init__('pi_control_monitor')
        
        self.enable_plot = enable_plot and MATPLOTLIB_AVAILABLE
        
        # Publisher to trigger PI test mode
        self.mode_pub = self.create_publisher(String, '/fsm/event', 10)
        
        # Subscriber to collect data - use best effort QoS to match publisher
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.update_sub = self.create_subscription(
            Update,
            '/white_crash/update',
            self.update_callback,
            qos_profile
        )
        
        # Data buffers
        self.data = {
            'timestamp': [],
            'left_target_velocity': [],
            'left_actual_velocity': [],
            'right_target_velocity': [],
            'right_actual_velocity': [],
            'left_pwm': [],
            'right_pwm': [],
            'battery_voltage': []
        }
        
        # For computing target velocity from feedforward model inverse
        # V = 0.271 + 2.800 * v  =>  v = (V - 0.271) / 2.800
        self.ff_offset = 0.271
        self.ff_gain = 2.800
        
        self.collecting = False
        self.start_time = None
        
        # Stats tracking
        self.error_stats = {
            'left_errors': [],
            'right_errors': []
        }
        
    def voltage_to_velocity(self, voltage):
        """Convert voltage command to target velocity using inverse feedforward model"""
        return (voltage - self.ff_offset) / self.ff_gain
    
    def update_callback(self, msg):
        if self.collecting:
            current_time = time.time() - self.start_time
            
            # Calculate voltage commands from PWM
            left_voltage = msg.left_motor_command * msg.battery_voltage
            right_voltage = msg.right_motor_command * msg.battery_voltage
            
            # Estimate target velocity from voltage command (inverse of feedforward)
            # This is approximate - actual target depends on PI correction too
            # But gives us a rough sense of commanded velocity
            left_target_v = self.voltage_to_velocity(left_voltage)
            right_target_v = self.voltage_to_velocity(right_voltage)
            
            # Store data
            self.data['timestamp'].append(current_time)
            self.data['left_target_velocity'].append(left_target_v)
            self.data['left_actual_velocity'].append(msg.left_speed)
            self.data['right_target_velocity'].append(right_target_v)
            self.data['right_actual_velocity'].append(msg.right_speed)
            self.data['left_pwm'].append(msg.left_motor_command)
            self.data['right_pwm'].append(msg.right_motor_command)
            self.data['battery_voltage'].append(msg.battery_voltage)
            
            # Calculate errors
            left_error = left_target_v - msg.left_speed
            right_error = right_target_v - msg.right_speed
            self.error_stats['left_errors'].append(abs(left_error))
            self.error_stats['right_errors'].append(abs(right_error))
            
            # Terminal output every 0.5 seconds
            if len(self.data['timestamp']) % 50 == 0:
                left_error_pct = (abs(left_error) / max(abs(left_target_v), 0.01)) * 100
                right_error_pct = (abs(right_error) / max(abs(right_target_v), 0.01)) * 100
                
                self.get_logger().info(
                    f"t={current_time:5.1f}s | "
                    f"L: tgt={left_target_v:+.3f} act={msg.left_speed:+.3f} err={left_error:+.3f} ({left_error_pct:.1f}%) | "
                    f"R: tgt={right_target_v:+.3f} act={msg.right_speed:+.3f} err={right_error:+.3f} ({right_error_pct:.1f}%) | "
                    f"V_bat={msg.battery_voltage:.2f}V"
                )
    
    def trigger_pi_test_mode(self):
        """Publish event to start PI control test mode"""
        self.get_logger().info("Triggering PI control test mode...")
        msg = String()
        msg.data = "pi-test"
        self.mode_pub.publish(msg)
        time.sleep(0.5)  # Give FSM time to process
        
    def start_collection(self):
        """Start data collection"""
        self.get_logger().info("Starting data collection...")
        self.collecting = True
        self.start_time = time.time()
        
    def stop_collection(self):
        """Stop data collection and print summary"""
        self.collecting = False
        
        if len(self.error_stats['left_errors']) > 0:
            import numpy as np
            left_mean = np.mean(self.error_stats['left_errors'])
            left_max = np.max(self.error_stats['left_errors'])
            right_mean = np.mean(self.error_stats['right_errors'])
            right_max = np.max(self.error_stats['right_errors'])
            
            self.get_logger().info("="*60)
            self.get_logger().info("Test Complete - Error Statistics:")
            self.get_logger().info(f"  Left Wheel:  Mean Error = {left_mean:.4f} m/s, Max Error = {left_max:.4f} m/s")
            self.get_logger().info(f"  Right Wheel: Mean Error = {right_mean:.4f} m/s, Max Error = {right_max:.4f} m/s")
            self.get_logger().info("="*60)
            
    def plot_results(self):
        """Generate plots of the test results"""
        if not self.enable_plot or len(self.data['timestamp']) == 0:
            return
            
        import matplotlib.pyplot as plt
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('PI Control Test Results', fontsize=16)
        
        t = self.data['timestamp']
        
        # Left wheel velocity
        ax = axes[0]
        ax.plot(t, self.data['left_target_velocity'], 'b--', label='Left Target', linewidth=2)
        ax.plot(t, self.data['left_actual_velocity'], 'b-', label='Left Actual', linewidth=1)
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Left Wheel Velocity Tracking')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Right wheel velocity
        ax = axes[1]
        ax.plot(t, self.data['right_target_velocity'], 'r--', label='Right Target', linewidth=2)
        ax.plot(t, self.data['right_actual_velocity'], 'r-', label='Right Actual', linewidth=1)
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Right Wheel Velocity Tracking')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # PWM commands and battery voltage
        ax = axes[2]
        ax.plot(t, self.data['left_pwm'], 'b-', label='Left PWM', alpha=0.7)
        ax.plot(t, self.data['right_pwm'], 'r-', label='Right PWM', alpha=0.7)
        ax.set_ylabel('PWM Command')
        ax.set_xlabel('Time (s)')
        ax.set_title('Motor PWM Commands')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add battery voltage on secondary axis
        ax2 = ax.twinx()
        ax2.plot(t, self.data['battery_voltage'], 'g--', label='Battery Voltage', alpha=0.5)
        ax2.set_ylabel('Battery Voltage (V)', color='g')
        ax2.tick_params(axis='y', labelcolor='g')
        ax2.legend(loc='upper right')
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = f'/home/brian/projects/white-crash/scripts/pi_control_test_{int(time.time())}.png'
        plt.savefig(plot_filename, dpi=150)
        self.get_logger().info(f"Plot saved to: {plot_filename}")
        
        plt.show()


def main(args=None):
    parser = argparse.ArgumentParser(description='Monitor PI control test')
    parser.add_argument('--plot', action='store_true', help='Enable real-time plotting (requires matplotlib)')
    parser.add_argument('--duration', type=float, default=35.0, help='Test duration in seconds (default: 35s)')
    cli_args = parser.parse_args()
    
    if cli_args.plot and not MATPLOTLIB_AVAILABLE:
        print("ERROR: matplotlib not available. Install with: pip install matplotlib")
        print("Running in terminal-only mode...")
        cli_args.plot = False
    
    rclpy.init(args=args)
    monitor = PIControlMonitor(enable_plot=cli_args.plot)
    
    print("="*60)
    print("PI Control Test Monitor")
    print("="*60)
    print("This script will:")
    print("  1. Trigger PI control test mode via FSM event")
    print("  2. Monitor commanded vs actual velocities")
    print(f"  3. Run for ~{cli_args.duration:.0f} seconds")
    print("  4. Display error statistics")
    if cli_args.plot:
        print("  5. Generate plots of the results")
    print("="*60)
    print()
    
    # Wait for ROS to initialize
    print("Waiting for ROS connections...")
    time.sleep(2)
    
    # Trigger test mode
    monitor.trigger_pi_test_mode()
    time.sleep(1)
    
    # Start data collection
    monitor.start_collection()
    
    try:
        # Run for specified duration
        end_time = time.time() + cli_args.duration
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(monitor, timeout_sec=0.01)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        monitor.stop_collection()
        
        # Generate plots if requested
        if cli_args.plot:
            monitor.plot_results()
        
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
