#!/usr/bin/env python3
"""
Robot Side Launch File for Distributed Deployment

This launch file runs on the Raspberry Pi connected to the robotic arm.
It includes:
- Arm driver node (servo control)
- Camera driver (RealSense D435i)
- Joint state publisher
- Robot state publisher

Hardware Requirements:
- Raspberry Pi 4 (4GB+ RAM recommended)
- USB connection to arm servos
- RealSense D435i camera

Network Requirements:
- Both robot and PC must be on the same network
- ROS_DOMAIN_ID must match on both sides
- Proper DDS configuration for cross-machine communication

Author: lododo
License: Apache 2.0
"""

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
    
    camera_serial_arg = DeclareLaunchArgument(
        "camera_serial_no",
        default_value="",
        description="RealSense camera serial number (empty for first available)"
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
    camera_serial_no = LaunchConfiguration("camera_serial_no")
    enable_camera = LaunchConfiguration("enable_camera")
    
    # Get package paths
    arm_description_share = FindPackageShare("arm_description")
    
    # Robot description
    urdf_file = PathJoinSubstitution([
        arm_description_share,
        "urdf",
        "arm.urdf.xacro"
    ])
    
    # Robot State Publisher (publishes TF transforms)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": urdf_file,
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
    
    # RealSense Camera Launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ])
        ]),
        launch_arguments={
            "serial_no": camera_serial_no,
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "camera_name": "camera",
            "camera_namespace": "",
            # Performance settings for Raspberry Pi
            "depth_module.profile": "640x480x30",
            "rgb_camera.profile": "640x480x30",
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
        camera_serial_arg,
        enable_camera_arg,
        ros_domain_id_arg,
        
        # Nodes
        robot_state_publisher,
        arm_driver,
        realsense_launch,
    ])
