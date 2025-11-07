#!/usr/bin/env python3
"""
Robot Side Launch File for Distributed Deployment

This launch file runs on the Raspberry Pi connected to the robotic arm.
It reuses the driver_view_launch.py approach but excludes:
- move_group (runs on PC side)
- RViz (runs on PC side)

Includes:
- Static TF publisher (world -> base_link)
- Arm driver node (serial communication with servos)
- Robot state publisher (publishes URDF and TF transforms)
- USB Camera driver (optional)

Hardware Requirements:
- Raspberry Pi 3B/4 (2GB+ RAM recommended)
- USB connection to arm servos (typically /dev/ttyUSB0 or /dev/ttyACM0)
- USB Camera (optional, e.g., Logitech C270, C920)

Network Requirements:
- Both robot and PC must be on the same network
- ROS_DOMAIN_ID must match on both sides
- Configure DDS for cross-machine communication if needed

Launch Arguments:
- serial_port: Serial port for arm servos (default: /dev/ttyUSB0)
- baud_rate: Baud rate for serial communication (default: 115200)
- use_camera: Enable/disable camera (default: true)
- camera_device: Camera device path (default: /dev/video0)

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


def generate_launch_description():
    # Build MoveIt configuration (reuse from driver_view_launch.py)
    moveit_config = MoveItConfigsBuilder(
        "arm", package_name="arm_moveit_config"
    ).to_moveit_configs()

    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for robot communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
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
    # (same as driver_view_launch.py)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. Launch arm driver node (ensure joint_states topic is published)
    # (same as driver_view_launch.py)
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
    # (same as driver_view_launch.py)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 20.0},
            {"ignore_timestamp": False},
        ],
    )

    delayed_robot_state_publisher = TimerAction(
        period=3.0,
        actions=[robot_state_publisher]
    )

    # 4. Launch USB camera (optional, reuse camera.launch.py)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([arm_bringup_pkg, 'launch', 'camera.launch.py'])
        ]),
        launch_arguments={
            'camera_device': camera_device,
            'frame_id': 'camera_optical_frame',  # Must match URDF
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
        delayed_robot_state_publisher,  # Robot state publisher (delayed)
        camera_launch,                  # USB camera (optional)
    ])

