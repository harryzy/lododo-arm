#!/usr/bin/env python3
"""
Test node - for testing RViz plugin result display functionality
Subscribe to /arm_command topic and publish simulated execution results to /arm_command_result
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class TestCommandProcessor(Node):
    def __init__(self):
        super().__init__('test_command_processor')
        
        # Subscribe to commands
        self.command_sub = self.create_subscription(
            String,
            '/arm_command',
            self.command_callback,
            10
        )
        
        # Publish results
        self.result_pub = self.create_publisher(
            String,
            '/arm_command_result',
            10
        )
        
        self.get_logger().info('Test command processor node started')
        self.get_logger().info('Subscribing: /arm_command')
        self.get_logger().info('Publishing: /arm_command_result')
    
    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Simulate processing delay
        time.sleep(0.5)
        
        # Generate different results based on command
        if command == 'scan_front':
            result = {
                'command': 'scan_front',
                'status': 'success',
                'message': 'Front scan complete',
                'objects_found': 2,
                'timestamp': time.time()
            }
        elif command == 'scan_all':
            result = {
                'command': 'scan_all',
                'status': 'success',
                'message': 'Full scan complete',
                'objects_found': 5,
                'views_scanned': 4,
                'timestamp': time.time()
            }
        elif command == 'scan_and_grasp':
            result = {
                'command': 'scan_and_grasp',
                'status': 'success',
                'message': 'Scan and grasp complete',
                'object_grasped': True,
                'grasp_position': {'x': 0.3, 'y': 0.2, 'z': 0.15},
                'timestamp': time.time()
            }
        else:
            result = {
                'command': command,
                'status': 'unknown',
                'message': f'Unknown command: {command}',
                'timestamp': time.time()
            }
        
        # Publish results
        result_msg = String()
        result_msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(result_msg)
        
        self.get_logger().info(f'Publishing result: {result["message"]}')


def main(args=None):
    rclpy.init(args=args)
    node = TestCommandProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
