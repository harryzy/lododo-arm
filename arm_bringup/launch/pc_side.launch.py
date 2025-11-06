#!/usr/bin/env python3
"""
PC Side Launch File for Distributed Deployment

This launch file runs on the PC (control station).
It includes:
- MoveIt2 motion planning
- RViz visualization
- Voice control interface (optional)
- Planning execution node

NOTE: YOLO perception node must be started separately using:
      ros2 run arm_bringup start_yolo_cube_detect.sh
      (YOLO requires isolated virtual environment)
      
      Available YOLO scripts (in order of recommendation):
      1. start_yolo_cube_detect.sh (recommended) - Optimized for cube detection
      2. start_yolo_dual_view.sh (advanced) - Dual-view triangulation
      3. start_yolo_node.sh (basic) - Single-view projection
      
      Note: Cube detection provides best results for grasping tasks.
      Other modes require manual camera calibration and lighting tuning.

Hardware Requirements:
- PC with network connection to robot
- GPU recommended for YOLO (launched separately)

Network Requirements:
- Both robot and PC must be on the same network
- ROS_DOMAIN_ID must match on both sides
- Robot side must be running robot_side.launch.py

Author: lododo
License: Apache 2.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz for visualization"
    )
    
    voice_control_arg = DeclareLaunchArgument(
        "voice_control",
        default_value="false",
        description="Enable voice control interface"
    )
    
    ros_domain_id_arg = DeclareLaunchArgument(
        "ros_domain_id",
        default_value="0",
        description="ROS_DOMAIN_ID for DDS communication (must match robot side)"
    )
    
    # Get launch configurations
    rviz = LaunchConfiguration("rviz")
    voice_control = LaunchConfiguration("voice_control")
    
    # Get package paths
    arm_moveit_config_share = FindPackageShare("arm_moveit_config")
    arm_description_share = FindPackageShare("arm_description")
    arm_bringup_share = FindPackageShare("arm_bringup")
    
    # MoveIt2 Configuration
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                arm_moveit_config_share,
                "launch",
                "move_group.launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": "false",
            "publish_monitored_planning_scene": "true"
        }.items()
    )
    
    # RViz with MoveIt plugin
    rviz_config_file = PathJoinSubstitution([
        arm_moveit_config_share,
        "config",
        "moveit.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{
            "use_sim_time": False
        }],
        condition=IfCondition(rviz)
    )
    
    # Planning Execution Node
    planning_node = Node(
        package="arm_planning_py",
        executable="arm_planning_py_node",
        name="arm_planning_py_node",
        parameters=[{
            "use_sim_time": False,
            "planning_time": 5.0,
            "planning_attempts": 10,
            "max_velocity_scaling_factor": 0.3,
            "max_acceleration_scaling_factor": 0.3
        }],
        output="screen",
        respawn=True,
        respawn_delay=3.0
    )
    
    # Voice Control Interface (optional)
    voice_node = Node(
        package="arm_voice_interface",
        executable="arm_voice_node",
        name="arm_voice_node",
        parameters=[{
            "use_sim_time": False,
            "language_model": "vosk-model-small-cn-0.22",  # Chinese model
            "confidence_threshold": 0.8
        }],
        output="screen",
        condition=IfCondition(voice_control)
    )
    
    # Delay planning node to ensure MoveIt is ready
    delayed_planning_node = TimerAction(
        period=3.0,
        actions=[planning_node]
    )
    
    return LaunchDescription([
        # Launch arguments
        rviz_arg,
        voice_control_arg,
        ros_domain_id_arg,
        
        # MoveIt2 (includes move_group and other planning components)
        moveit_launch,
        
        # Visualization
        rviz_node,
        
        # High-level nodes (with delays to ensure dependencies are ready)
        delayed_planning_node,
        
        # Optional: Voice control
        voice_node,
    ])
