#!/usr/bin/env python3
"""
Planar Perception Bringup Launch File

Launch the planar perception system on PC side.
This is a simple wrapper that launches the planar perception package.

Usage:
    ros2 launch arm_bringup planar_perception_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include the planar perception launch file from arm_perception_planar package
    planar_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_perception_planar'),
                'launch',
                'planar_perception.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        planar_perception_launch,
    ])

