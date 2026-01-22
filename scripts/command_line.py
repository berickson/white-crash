#!/usr/bin/env python3
"""
Command-line tool for sending commands to white_crash robot over ROS topics.
Supports both command-line mode and interactive REPL mode.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import sys
import threading

# Enable command history with up/down arrows
try:
    import readline
    # Enable history
    readline.parse_and_bind('tab: complete')
    readline.parse_and_bind('set editing-mode emacs')
except ImportError:
    pass  # readline not available on all platforms


class ReplClient(Node):
    def __init__(self):
        super().__init__('command_line_client')
        
        # Create QoS profile with best-effort reliability to match robot
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
        
        self.last_response = None
        self.response_received = threading.Event()
        
    def response_callback(self, msg):
        """Callback for command responses from robot"""
        self.last_response = msg.data
        self.response_received.set()
        
    def send_command(self, command_text, wait_for_response=True):
        """Send a command and optionally wait for response"""
        msg = String()
        msg.data = command_text
        self.command_pub.publish(msg)
        
        if wait_for_response:
            # Wait up to 2 seconds for response
            self.response_received.clear()
            if self.response_received.wait(timeout=2.0):
                return self.last_response
            else:
                return "ERROR: No response from robot (timeout)"
        return None


def interactive_mode():
    """Run in interactive REPL mode"""
    rclpy.init()
    client = ReplClient()
    
    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()

    try:
        while True:
            try:
                command = input("white_crash> ").strip()
            except EOFError:
                break
                
            if not command:
                continue
                
            if command in ['quit', 'exit', 'q']:
                break
                
            response = client.send_command(command)
            if response:
                print(response)
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        client.destroy_node()
        rclpy.shutdown()


def command_line_mode(command_text):
    """Run a single command from command line"""
    rclpy.init()
    client = ReplClient()
    
    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()
    
    response = client.send_command(command_text)
    print(response)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) == 1:
        # No arguments - run in interactive mode
        interactive_mode()
    else:
        # Arguments provided - run single command
        command = ' '.join(sys.argv[1:])
        command_line_mode(command)
