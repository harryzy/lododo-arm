#!/usr/bin/env python3
"""
Simplified joint angle reader
Directly listens to /joint_states and displays arm joint angles in real-time
Suitable for quick viewing and recording
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import sys


class SimpleJointReader(Node):
    def __init__(self):
        super().__init__('simple_joint_reader')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.last_print_time = 0
        self.print_interval = 1.0  # Print once per second
        
        print('\n' + '='*80)
        print('🤖 Simplified Joint Angle Monitor')
        print('='*80)
        print('Real-time display of arm joint angles (updated every second)')
        print('Press Ctrl+C to exit')
        print('='*80 + '\n')
    
    def joint_callback(self, msg: JointState):
        """Receive and display joint state"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Limit print frequency
        if current_time - self.last_print_time < self.print_interval:
            return
        
        self.last_print_time = current_time
        
        # Clear screen effect (print separator line)
        print('\n' + '-'*80)
        print(f'Time: {self.get_clock().now().to_msg().sec}s')
        print('-'*80)
        
        # Extract arm joints
        arm_joints = []
        for i, name in enumerate(msg.name):
            if 'joint' in name.lower() and 'finger' not in name.lower():
                arm_joints.append((name, i))
        
        # Sort
        arm_joints.sort(key=lambda x: x[0])
        
        # Display table header
        print(f'{"Joint Name":<12} {"Radians":>10} {"Degrees":>10}')
        print('-'*80)
        
        # Display each joint
        positions_deg = []
        positions_rad = []
        for joint_name, idx in arm_joints:
            if idx < len(msg.position):
                rad = msg.position[idx]
                deg = math.degrees(rad)
                positions_rad.append(rad)
                positions_deg.append(deg)
                print(f'{joint_name:<12} {rad:>10.4f} {deg:>10.2f}°')
        
        # Print Python configuration format
        print('\n📋 Python config (degrees):')
        print('[', end='')
        print(', '.join([f'{d:.2f}' for d in positions_deg]), end='')
        print(']')
        
        print('\n📋 Python config (radians):')
        print('[', end='')
        print(', '.join([f'{r:.5f}' for r in positions_rad]), end='')
        print(']')
        
        sys.stdout.flush()


def main():
    rclpy.init()
    node = SimpleJointReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\n👋 Stopping monitoring')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
