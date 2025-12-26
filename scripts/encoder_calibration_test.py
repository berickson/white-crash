#!/usr/bin/env python3
"""
Encoder Calibration Test Script

Drives robot forward toward a wall, using TOF sensor to measure actual distance traveled.
Compares encoder-reported distance to TOF-measured distance to detect calibration errors.

Usage:
1. Place robot 1-3 meters from a wall
2. Run: ros2 topic pub /white_crash/mode std_msgs/msg/String "{data: 'auto'}" --once
3. Run this script: python3 encoder_calibration_test.py
4. Robot will drive forward and stop near the wall
5. Script reports calibration factor (encoder_distance / actual_distance)

If encoders report 3x the actual distance, the script will show "Calibration factor: 3.0x"
and tell you to divide meters_per_odometer_tick by 3.0.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from white_crash_msgs.msg import Update
import time
import sys

class EncoderCalibrationTest(Node):
    def __init__(self):
        super().__init__('encoder_calibration_test')
        
        # QoS profile for subscription - match publisher's best-effort
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/white_crash/cmd_vel', 10)
        self.range_sub = self.create_subscription(
            Range,
            '/white_crash/center_distance',
            self.range_callback,
            qos_profile
        )
        self.update_sub = self.create_subscription(
            Update,
            '/white_crash/update',
            self.update_callback,
            qos_profile
        )
        
        # State variables
        self.tof_distance = None
        self.left_odo_ticks = None
        self.right_odo_ticks = None
        self.initial_left_odo_ticks = None
        self.initial_right_odo_ticks = None
        self.initial_tof_distance = None
        self.started = False
        self.stopped = False
        self.stop_time = None  # Track when encoders stopped moving
        self.last_left_odo_ticks = None
        self.last_right_odo_ticks = None
        
        # Encoder calibration (from main.cpp - meters_per_odometer_tick)
        # This is what we're trying to verify/calibrate
        self.meters_per_tick = 0.000653  # Current value from main.cpp
        
        # Control parameters
        self.drive_speed = 0.3  # m/s forward
        self.slow_distance = 0.5  # Start slowing down at 50cm
        self.stop_distance = 0.25  # Stop at 25cm
        self.slow_speed = 0.1  # m/s when close
        
        self.get_logger().info('Encoder Calibration Test initialized')
        self.get_logger().info('Waiting for first update message...')
        
    def range_callback(self, msg):
        """Process Range messages from TOF sensor"""
        self.tof_distance = msg.range
        
    def update_callback(self, msg):
        """Process update messages from robot - track odometer ticks"""
        self.left_odo_ticks = msg.left_odometer_ticks
        self.right_odo_ticks = msg.right_odometer_ticks
        
        # Record initial readings on first valid message
        if not self.started and self.tof_distance is not None and self.tof_distance < 10.0:
            self.initial_tof_distance = self.tof_distance
            self.initial_left_odo_ticks = self.left_odo_ticks
            self.initial_right_odo_ticks = self.right_odo_ticks
            self.started = True
            self.get_logger().info(f'Starting test.')
            self.get_logger().info(f'Initial TOF distance: {self.tof_distance:.3f}m')
            self.get_logger().info(f'Initial left odometer ticks: {self.initial_left_odo_ticks}')
            self.get_logger().info(f'Initial right odometer ticks: {self.initial_right_odo_ticks}')
    
    def calculate_distance_traveled(self):
        """Calculate average distance traveled based on encoders"""
        if self.initial_left_odo_ticks is None or self.left_odo_ticks is None:
            return None
        left_ticks = self.left_odo_ticks - self.initial_left_odo_ticks
        right_ticks = self.right_odo_ticks - self.initial_right_odo_ticks
        left_distance = left_ticks * self.meters_per_tick
        right_distance = right_ticks * self.meters_per_tick
        avg_distance = (left_distance + right_distance) / 2.0
        return avg_distance
        
    def run_test(self):
        """Main test loop"""
        loop_period = 0.05  # 50ms = 20 Hz - fast enough to avoid cmd_vel timeout
        
        self.get_logger().info('Starting in 2 seconds...')
        time.sleep(2.0)
        
        log_counter = 0  # Only log every Nth iteration to avoid spam
        
        self.get_logger().info('Entering main control loop...')
        stop = False
        
        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except Exception as e:
                self.get_logger().error(f'spin_once failed: {e}')
                break
            
            if not self.started:
                if log_counter % 20 == 0:
                    self.get_logger().info('Waiting for start conditions...')
                log_counter += 1
                time.sleep(loop_period)
                continue
                
            # Check TOF distance
            if self.tof_distance is None or self.tof_distance > 10.0:
                if log_counter % 20 == 0:  # Log every second
                    self.get_logger().warn(f'No valid TOF reading (tof={self.tof_distance}), continuing...')
                cmd = Twist()
                cmd.linear.x = self.drive_speed
                self.cmd_vel_pub.publish(cmd)
                log_counter += 1
                time.sleep(loop_period)
                continue
            
            distance_traveled = self.calculate_distance_traveled()
            
            # Create cmd_vel message that will be sent every loop iteration
            cmd = Twist()
            
            # Log state every iteration for now to debug
            if distance_traveled is not None:
                self.get_logger().info(f'Loop #{log_counter}: TOF={self.tof_distance:.3f}m, Encoder={distance_traveled:.3f}m, stopped={self.stopped}')
            else:
                self.get_logger().info(f'Loop #{log_counter}: TOF={self.tof_distance:.3f}m, Encoder=None, stopped={self.stopped}')
            
            # stop once and for all if we get close enough
            if self.tof_distance <= self.stop_distance:
                stop = True

            # Decision logic based on TOF distance
            if stop:
                # STOP - we're close to the wall
                cmd.linear.x = 0.0
                self.cmd_vel_pub.publish(cmd)
                
                # Check if encoders have stopped moving
                encoders_stopped = False
                if self.last_left_odo_ticks is not None and self.last_right_odo_ticks is not None:
                    encoders_stopped = (self.left_odo_ticks == self.last_left_odo_ticks and 
                                       self.right_odo_ticks == self.last_right_odo_ticks)
                
                # Update last encoder readings
                self.last_left_odo_ticks = self.left_odo_ticks
                self.last_right_odo_ticks = self.right_odo_ticks
                
                if not self.stopped:
                    if encoders_stopped:
                        # Encoders have stopped moving, start countdown
                        self.get_logger().info(f'Encoders stopped! Starting 2-second settle timer...')
                        self.stopped = True
                        self.stop_time = time.time()
                    else:
                        self.get_logger().info(f'Waiting for encoders to stop... (L:{self.left_odo_ticks}, R:{self.right_odo_ticks})')
                else:
                    # Check if encoders are still stopped
                    if not encoders_stopped:
                        # Encoders started moving again, reset
                        self.get_logger().info('Encoders moved! Resetting settle timer...')
                        self.stopped = False
                        self.stop_time = None
                    else:
                        # Encoders still stopped, check if enough time has passed
                        time_stopped = time.time() - self.stop_time
                        if time_stopped < 2.0:
                            # Still settling
                            self.get_logger().info(f'Settling... {time_stopped:.1f}s with encoders stable')
                        else:
                            # Fully stopped, print results
                            self.get_logger().info('Robot fully stopped for 2 seconds, calculating results...')
                            
                            # Print results if we have valid data
                            if distance_traveled is not None and self.initial_tof_distance is not None:
                                # Calculate actual distance traveled using TOF
                                actual_distance = self.initial_tof_distance - self.tof_distance
                                
                                # Calculate ticks traveled
                                left_ticks = self.left_odo_ticks - self.initial_left_odo_ticks
                                right_ticks = self.right_odo_ticks - self.initial_right_odo_ticks
                                avg_ticks = (left_ticks + right_ticks) / 2.0
                                
                                # Calculate calibration factor
                                if actual_distance > 0.01:  # Avoid division by zero
                                    calibration_factor = distance_traveled / actual_distance
                                    correct_meters_per_tick = actual_distance / avg_ticks if avg_ticks != 0 else 0
                                else:
                                    calibration_factor = None
                                    correct_meters_per_tick = None
                                
                                self.get_logger().info('=' * 70)
                                self.get_logger().info('ENCODER CALIBRATION TEST RESULTS')
                                self.get_logger().info('=' * 70)
                                self.get_logger().info(f'Initial TOF distance:      {self.initial_tof_distance:.3f} m')
                                self.get_logger().info(f'Final TOF distance:        {self.tof_distance:.3f} m')
                                self.get_logger().info(f'ACTUAL distance traveled:  {actual_distance:.3f} m  (from TOF)')
                                self.get_logger().info('')
                                self.get_logger().info(f'Left wheel ticks:          {left_ticks}')
                                self.get_logger().info(f'Right wheel ticks:         {right_ticks}')
                                self.get_logger().info(f'Average ticks:             {avg_ticks:.1f}')
                                self.get_logger().info('')
                                self.get_logger().info(f'Current meters_per_tick:   {self.meters_per_tick:.6f}')
                                self.get_logger().info(f'ENCODER distance traveled: {distance_traveled:.3f} m  (from ticks)')
                                self.get_logger().info('')
                                if calibration_factor is not None:
                                    self.get_logger().info(f'CALIBRATION FACTOR: {calibration_factor:.3f}x')
                                    self.get_logger().info('')
                                    if abs(calibration_factor - 1.0) > 0.05:
                                        self.get_logger().warn(f'Encoders are miscalibrated by {calibration_factor:.3f}x!')
                                        if correct_meters_per_tick is not None:
                                            self.get_logger().warn(f'Current value: meters_per_odometer_tick = {self.meters_per_tick:.6f}')
                                            self.get_logger().warn(f'Corrected value: meters_per_odometer_tick = {correct_meters_per_tick:.6f}')
                                            self.get_logger().warn(f'To fix: UPDATE meters_per_odometer_tick to {correct_meters_per_tick:.6f} in main.cpp')
                                    else:
                                        self.get_logger().info('Encoders are well calibrated! (within 5%)')
                                self.get_logger().info('=' * 70)
                                break
                
            elif self.tof_distance < self.slow_distance:
                # SLOW DOWN - getting close
                cmd.linear.x = self.slow_speed
                self.cmd_vel_pub.publish(cmd)
                    
            else:
                # NORMAL SPEED - far from wall
                cmd.linear.x = self.drive_speed
                self.cmd_vel_pub.publish(cmd)
            
            log_counter += 1
            
            try:
                time.sleep(loop_period)
            except Exception as e:
                self.get_logger().error(f'Sleep failed: {e}')
                break
        
        self.get_logger().info('Exited main loop')
        
        # Ensure stopped
        cmd = Twist()
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    test = EncoderCalibrationTest()
    
    try:
        test.run_test()
    except KeyboardInterrupt:
        test.get_logger().info('Test interrupted by user')
        # Stop the robot
        cmd = Twist()
        cmd.linear.x = 0.0
        test.cmd_vel_pub.publish(cmd)
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
