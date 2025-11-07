#!/usr/bin/env python3
"""
Robot Side Launch File for Distributed Deployment

This launch file runs on the Raspberry Pi connected to the robotic arm.
It includes:
- Arm driver node (servo control)
- USB Camera driver (reuses camera.launch.py)
- Robot state publisher (publishes TF transforms)

Hardware Requirements:
- Raspberry Pi 3B/4 (2GB+ RAM recommended)
- USB connection to arm servos
- USB Camera (e.g., Logitech C270, C920)

Network Requirements:
- Both robot and PC must be on the same network
- ROS_DOMAIN_ID must match on both sides
- Proper DDS configuration for cross-machine communication

Launch Arguments:
- port: Serial port for arm servos (default: /dev/ttyUSB0)
- camera_device: Camera device path (default: /dev/video0)
- enable_camera: Enable/disable camera (default: true)
- ros_domain_id: ROS Domain ID for network isolation (default: 0)

Author: lododo
License: Apache 2.0
"""
from launch.substitutions import Command
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port for arm servos"
    )
    
    camera_type_arg = DeclareLaunchArgument(
        "camera_type",
        default_value="usb",
        description="Camera type: 'usb' for USB camera, 'realsense' for RealSense D435i"
    )
    
    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/video0",
        description="Camera device path for USB camera"
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value="true",
        description="Enable camera driver"
    )
    
    ros_domain_id_arg = DeclareLaunchArgument(
        "ros_domain_id",
        default_value="0",
        description="ROS_DOMAIN_ID for DDS communication (must match PC side)"
    )
    
    # Get launch configurations
    port = LaunchConfiguration("port")
    camera_type = LaunchConfiguration("camera_type")
    camera_device = LaunchConfiguration("camera_device")
    enable_camera = LaunchConfiguration("enable_camera")
    
    # Get package paths
    arm_moveit_config_share = FindPackageShare("arm_moveit_config")
    
    # Robot description
    urdf_file = PathJoinSubstitution([
        arm_moveit_config_share,
        "config",
        "arm.urdf.xacro"
    ])
    
    # Robot State Publisher (publishes TF transforms)
    robot_description_content = Command(['xacro ', urdf_file])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": False
        }],
        output="screen"
    )
    
    # Arm Driver Node (hardware interface)
    arm_driver = Node(
        package="arm_driver_node",
        executable="arm_driver_node",
        name="arm_driver_node",
        parameters=[{
            "port": port,
            "publish_rate": 50.0,
            "use_sim_time": False
        }],
        output="screen",
        respawn=True,
        respawn_delay=2.0
    )
    
    # Camera Launch (reuse existing camera.launch.py)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("arm_bringup"),
                "launch",
                "camera.launch.py"
            ])
        ]),
        launch_arguments={
            "camera_device": camera_device,
        }.items(),
        condition=IfCondition(enable_camera)
    )
    
    # Joint State Publisher GUI (optional, for manual testing)
    # Commented out by default, uncomment if needed for debugging
    # joint_state_publisher_gui = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui"
    # )
    
    return LaunchDescription([
        # Launch arguments
        port_arg,
        camera_type_arg,
        camera_device_arg,
        enable_camera_arg,
        ros_domain_id_arg,
        
        # Nodes
        robot_state_publisher,
        arm_driver,
        camera_launch,
    ])
