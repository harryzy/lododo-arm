#!/usr/bin/env python3
"""
Lightweight Robot Side Launch File for Raspberry Pi 3B (1GB RAM)

This is an optimized version for resource-constrained hardware.
Reduces camera resolution and frame rate for better performance.

Differences from robot_side.launch.py:
- Lower camera resolution (640x480 -> 320x240)
- Reduced frame rate (15fps -> 10fps)
- Lower robot_state_publisher frequency (30Hz -> 20Hz)
- Optimized for 1GB RAM devices

Launch on Raspberry Pi:
    ros2 launch arm_bringup robot_side_lite.launch.py

Launch Arguments:
    serial_port: Serial port for arm servos (default: from config file or /dev/ttyACM0)
    baud_rate: Baud rate for serial communication (default: from config file or 1000000)
    use_camera: Enable/disable USB camera (default: true)
    camera_device: Camera device path (default: /dev/video0)

Author: lododo
License: Apache 2.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml_config(package_name, config_file):
    """Load YAML configuration file and return as dict"""
    try:
        config_path = os.path.join(
            get_package_share_directory(package_name),
            'config',
            config_file
        )
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
        return {}


def generate_launch_description():
    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "arm", package_name="arm_moveit_config"
    ).to_moveit_configs()

    # Load default configuration from YAML
    default_config = load_yaml_config('arm_bringup', 'default_params.yaml')

    # Declare launch arguments with values from YAML config
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=str(default_config.get('serial_port', '/dev/ttyACM0')),
        description='Serial port for robot communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value=str(default_config.get('baud_rate', 1000000)),
        description='Baud rate for serial communication'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch USB camera'
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_camera = LaunchConfiguration('use_camera')
    camera_device = LaunchConfiguration('camera_device')

    # Package directories
    arm_bringup_pkg = FindPackageShare('arm_bringup')
    arm_driver_pkg = FindPackageShare('arm_driver_node')
    
    # 1. Static transform publisher - publishes world to base_link transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. Launch arm driver node (ensure joint_states topic is published)
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([arm_driver_pkg, 'launch', 'driver_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'serial_port': serial_port,
            'baud_rate': baud_rate,
        }.items(),
    )

    # 3. Delayed robot_state_publisher launch (ensure driver starts first)
    # LITE: Reduced frequency to 20Hz (vs 30Hz) to save CPU on Pi 3B
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 20.0},  # Reduced from 30Hz to save CPU on Pi 3B
            {"ignore_timestamp": False},
        ],
    )

    delayed_robot_state_publisher = TimerAction(
        period=3.0,
        actions=[robot_state_publisher]
    )

    # 4. Launch USB camera with LITE configuration
    # Optimized for Raspberry Pi 3B (1GB RAM):
    # - Reduced resolution: 320x240 (vs 640x480)
    # - Lower framerate: 10fps (vs 15fps)
    # - Saves CPU, memory, and USB bandwidth
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([arm_bringup_pkg, 'launch', 'camera.launch.py'])
        ]),
        launch_arguments={
            'camera_device': camera_device,
            'frame_id': 'camera_optical_frame',
            'image_width': '320',      # LITE: Reduced from 640
            'image_height': '240',     # LITE: Reduced from 480
            'framerate': '10',         # LITE: Reduced from 15
        }.items(),
        condition=IfCondition(use_camera)
    )

    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        baud_rate_arg,
        use_sim_time_arg,
        use_camera_arg,
        camera_device_arg,
        
        # Nodes and launch files
        static_tf,                      # Static TF publisher
        driver_launch,                  # Arm driver node
        delayed_robot_state_publisher,  # Robot state publisher (delayed, 20Hz)
        camera_launch,                  # USB camera (LITE mode: 320x240@10fps)
    ])

