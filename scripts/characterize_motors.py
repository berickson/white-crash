#!/usr/bin/env python3
"""
Motor Characterization Script for Step 1
Triggers open-loop test mode, collects data, and fits feedforward model.

Usage:
    python3 characterize_motors.py
    
Requirements:
    - ROS 2 Jazzy
    - Robot in hand mode
    - Robot on racks (wheels off ground)
    - micro-ROS agent running
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from white_crash_msgs.msg import Update
import time
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import sys

class MotorCharacterizer(Node):
    def __init__(self):
        super().__init__('motor_characterizer')
        
        # Publisher to trigger open-loop mode
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
        
        self.data = {
            'timestamp': [],
            'left_voltage': [],
            'left_velocity': [],
            'right_voltage': [],
            'right_velocity': [],
            'battery_voltage': []
        }
        
        self.collecting = False
        self.start_time = None
        
    def update_callback(self, msg):
        if self.collecting:
            current_time = time.time() - self.start_time
            self.data['timestamp'].append(current_time)
            # Calculate voltage from PWM and battery voltage
            self.data['left_voltage'].append(msg.left_motor_command * msg.battery_voltage)
            self.data['left_velocity'].append(msg.left_speed)
            self.data['right_voltage'].append(msg.right_motor_command * msg.battery_voltage)
            self.data['right_velocity'].append(msg.right_speed)
            self.data['battery_voltage'].append(msg.battery_voltage)
            
            # Print status
            if len(self.data['timestamp']) % 10 == 0:
                left_voltage = msg.left_motor_command * msg.battery_voltage
                right_voltage = msg.right_motor_command * msg.battery_voltage
                self.get_logger().info(
                    f"Collecting data... {current_time:.1f}s | "
                    f"V_L={left_voltage:.2f}V @ {msg.left_speed:.3f}m/s | "
                    f"V_R={right_voltage:.2f}V @ {msg.right_speed:.3f}m/s"
                )
    
    def trigger_open_loop_mode(self):
        """Publish event to start open-loop characterization"""
        msg = String()
        msg.data = 'open-loop'
        self.mode_pub.publish(msg)
        self.get_logger().info('Triggered open-loop characterization mode')
        
    def start_collection(self):
        """Start data collection"""
        self.collecting = True
        self.start_time = time.time()
        self.get_logger().info('Started data collection')
        
    def stop_collection(self):
        """Stop data collection"""
        self.collecting = False
        self.get_logger().info(f'Stopped data collection. Collected {len(self.data["timestamp"])} samples')
        
    def analyze_data(self):
        """Analyze collected data and fit feedforward model"""
        self.get_logger().info('Analyzing data...')
        
        # Convert to numpy arrays
        left_v = np.array(self.data['left_voltage'])
        left_vel = np.array(self.data['left_velocity'])
        right_v = np.array(self.data['right_voltage'])
        right_vel = np.array(self.data['right_velocity'])
        
        # Remove zero velocity points (startup transients)
        left_mask = left_vel > 0.01
        right_mask = right_vel > 0.01
        
        left_v_filtered = left_v[left_mask]
        left_vel_filtered = left_vel[left_mask]
        right_v_filtered = right_v[right_mask]
        right_vel_filtered = right_vel[right_mask]
        
        # Fit linear model: voltage = a + b*velocity
        # (Could also try polynomial if needed)
        def linear_model(velocity, a, b):
            return a + b * velocity
        
        try:
            # Fit left motor
            left_params, _ = curve_fit(linear_model, left_vel_filtered, left_v_filtered)
            left_offset, left_gain = left_params
            
            # Fit right motor  
            right_params, _ = curve_fit(linear_model, right_vel_filtered, right_v_filtered)
            right_offset, right_gain = right_params
            
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('FEEDFORWARD MODEL RESULTS:')
            self.get_logger().info('='*60)
            self.get_logger().info(f'Left Motor:  V = {left_offset:.3f} + {left_gain:.3f} * v')
            self.get_logger().info(f'Right Motor: V = {right_offset:.3f} + {right_gain:.3f} * v')
            self.get_logger().info('='*60)
            
            # Calculate initial PI gains
            # P gain should be 1-2 V/(m/s) since feedforward does most work
            # I gain should be 0.5-1.0 V/(m/s*s)
            avg_gain = (left_gain + right_gain) / 2
            recommended_p = min(2.0, avg_gain * 0.2)  # 20% of feedforward gain
            recommended_i = recommended_p * 0.5
            
            self.get_logger().info('\nRECOMMENDED INITIAL PI GAINS:')
            self.get_logger().info(f'P gain: {recommended_p:.2f} V/(m/s)')
            self.get_logger().info(f'I gain: {recommended_i:.2f} V/(m/s*s)')
            self.get_logger().info(f'D gain: 0.0 (not needed for velocity control)')
            self.get_logger().info('='*60)
            
            # Save results to file
            with open('motor_characterization_results.txt', 'w') as f:
                f.write('Motor Characterization Results\n')
                f.write('='*60 + '\n\n')
                f.write('Feedforward Model: voltage = offset + gain * velocity\n\n')
                f.write(f'Left Motor:\n')
                f.write(f'  offset = {left_offset:.3f} V\n')
                f.write(f'  gain   = {left_gain:.3f} V/(m/s)\n\n')
                f.write(f'Right Motor:\n')
                f.write(f'  offset = {right_offset:.3f} V\n')
                f.write(f'  gain   = {right_gain:.3f} V/(m/s)\n\n')
                f.write(f'Recommended Initial PI Gains:\n')
                f.write(f'  P = {recommended_p:.2f} V/(m/s)\n')
                f.write(f'  I = {recommended_i:.2f} V/(m/s*s)\n')
                f.write(f'  D = 0.0\n')
            
            self.get_logger().info('Results saved to motor_characterization_results.txt')
            
            # Plot results
            self.plot_results(
                left_vel_filtered, left_v_filtered, left_params,
                right_vel_filtered, right_v_filtered, right_params
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to fit model: {e}')
            
    def plot_results(self, left_vel, left_v, left_params, right_vel, right_v, right_params):
        """Create plots of data and fitted models"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Left motor
        ax1.scatter(left_vel, left_v, alpha=0.5, label='Measured')
        vel_range = np.linspace(0, max(left_vel), 100)
        ax1.plot(vel_range, left_params[0] + left_params[1] * vel_range, 
                'r-', linewidth=2, label='Fitted Model')
        ax1.set_xlabel('Velocity (m/s)')
        ax1.set_ylabel('Voltage (V)')
        ax1.set_title(f'Left Motor\nV = {left_params[0]:.2f} + {left_params[1]:.2f}*v')
        ax1.legend()
        ax1.grid(True)
        
        # Right motor
        ax2.scatter(right_vel, right_v, alpha=0.5, label='Measured')
        vel_range = np.linspace(0, max(right_vel), 100)
        ax2.plot(vel_range, right_params[0] + right_params[1] * vel_range,
                'r-', linewidth=2, label='Fitted Model')
        ax2.set_xlabel('Velocity (m/s)')
        ax2.set_ylabel('Voltage (V)')
        ax2.set_title(f'Right Motor\nV = {right_params[0]:.2f} + {right_params[1]:.2f}*v')
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        plt.savefig('motor_characterization_plot.png', dpi=150)
        self.get_logger().info('Plot saved to motor_characterization_plot.png')
        
        # Optionally show plot
        # plt.show()


def main():
    print('='*60)
    print('Motor Characterization Script')
    print('='*60)
    print('This script will:')
    print('1. Trigger open-loop characterization mode on the robot')
    print('2. Collect voltage and velocity data')
    print('3. Fit feedforward model: voltage = f(velocity)')
    print('4. Calculate recommended initial PI gains')
    print()
    print('Prerequisites:')
    print('- Robot must be in hand mode')
    print('- Robot must be on racks (wheels off ground)')
    print('- micro-ROS agent must be running')
    print('='*60)
    
    input('Press ENTER when ready to start...')
    
    rclpy.init()
    node = MotorCharacterizer()
    
    # Give time for connections to establish
    print('Waiting for ROS connections...')
    time.sleep(2)
    
    # Trigger open-loop mode
    node.trigger_open_loop_mode()
    time.sleep(1)
    
    # Start collecting data
    node.start_collection()
    
    # Run for ~20 seconds (5 voltages * ~3s each + margin)
    print('Collecting data for ~20 seconds...')
    start = time.time()
    while time.time() - start < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Stop collection
    node.stop_collection()
    
    # Analyze
    if len(node.data['timestamp']) > 0:
        node.analyze_data()
    else:
        print('ERROR: No data collected! Check that robot is running and publishing to /tank/update')
    
    node.destroy_node()
    rclpy.shutdown()
    
    print('\nCharacterization complete!')


if __name__ == '__main__':
    main()
