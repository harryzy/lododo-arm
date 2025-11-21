#!/usr/bin/env python3
"""
Unified Bringup Launch File with Model Selection
Allows switching between arm_description and lekiwi_description models.

Usage:
  ros2 launch arm_bringup unified_bringup.launch.py use_lekiwi:=true
  ros2 launch arm_bringup unified_bringup.launch.py use_lekiwi:=false  # default
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    use_lekiwi_arg = DeclareLaunchArgument(
        'use_lekiwi',
        default_value='false',
        description='Use lekiwi_description model if true, otherwise use arm_description'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo) if true'
    )
    
    # Get launch configurations
    use_lekiwi = LaunchConfiguration('use_lekiwi')
    use_sim = LaunchConfiguration('use_sim')
    
    # Launch MoveIt with arm_description (original model)
    arm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ]),
        condition=UnlessCondition(use_lekiwi)
    )
    
    # Launch MoveIt with lekiwi_description (detailed model)
    lekiwi_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lekiwi_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ]),
        condition=IfCondition(use_lekiwi)
    )
    
    return LaunchDescription([
        use_lekiwi_arg,
        use_sim_arg,
        arm_moveit_launch,
        lekiwi_moveit_launch,
    ])
