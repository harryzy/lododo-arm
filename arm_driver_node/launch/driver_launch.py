"""
Launch file for real Feetech servo arm driver.

Starts arm_driver_node with hardware serial communication parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for hardware communication'
        ),
        
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='Serial baud rate (default: 1M for Feetech servos)'
        ),
        
        # Launch driver node
        Node(
            package='arm_driver_node',
            executable='arm_driver_node',
            name='arm_driver_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'serial_port': LaunchConfiguration('serial_port')},
                {'baud_rate': LaunchConfiguration('baud_rate')},
                {'update_rate': 20.0},  # Joint state publishing rate (Hz)
                {'fast_trajectory_execution': False},  # Use fast trajectory mode
                {'fast_trajectory_threshold': 10},  # Waypoint count threshold for fast mode
                {'auto_reset_home': True},  # Auto-reset to home position on startup
            ],
            output='screen'
        )
    ])