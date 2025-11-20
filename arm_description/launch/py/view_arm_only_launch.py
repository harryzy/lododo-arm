#!/usr/bin/env python3
"""
Launch file to visualize only the LeKiwi robotic arm in RViz2.
This excludes the mobile base and only shows the 6-DOF arm.
"""

from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Generate launch description for arm-only visualization.
    
    Loads the lekiwi_arm.xacro file and launches:
    - joint_state_publisher_gui: For manual joint control
    - robot_state_publisher: Publishes robot state
    - rviz2: Visualization
    """
    
    # Get the arm xacro file path
    arm_xacro_file = os.path.join(
        get_package_share_directory('arm_description'),
        'urdf',
        'lekiwi_arm.xacro'
    )
    
    # Get RViz config file
    rviz_config_file = os.path.join(
        get_package_share_directory('arm_description'),
        'urdf',
        'lekiwi_arm.rviz'
    )

    # Process the xacro file to generate robot description
    robot_description = ParameterValue(
        Command(['xacro ', arm_xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        # Declare launch argument for GUI
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui for manual control'
        ),
        
        # Joint State Publisher with GUI for manual control
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'robot_description': robot_description
            }],
            condition=IfCondition(LaunchConfiguration('use_gui'))
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 30.0
            }]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
