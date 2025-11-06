#!/usr/bin/env python3
"""
Minimal Bringup Launch File
Starts only the essential components for basic arm control.

This launch file includes:
- Robot driver or simulation
- Robot state publisher
- Joint state publisher

Use this for:
- Basic testing
- Hardware diagnostics
- Minimal resource usage scenarios
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import yaml
from ament_index_python.packages import get_package_share_directory


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
    # Load default configuration from YAML
    default_config = load_yaml_config('arm_bringup', 'default_params.yaml')
    
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=str(default_config.get('serial_port', '/dev/ttyACM0')),
        description='Serial port for robot communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value=str(default_config.get('baud_rate', 115200)),
        description='Baud rate for serial communication'
    )

    # Get launch configurations
    use_sim = LaunchConfiguration('use_sim')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    # Package directories
    moveit_config_pkg = FindPackageShare('arm_moveit_config')

    # Launch Gazebo simulation (conditional)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'gazebo_launch.py'])
        ]),
        launch_arguments={
            'use_rviz': 'false',  # minimalmode does not startRViz
        }.items(),
        condition=IfCondition(use_sim)
    )

    # Launch robot driver (conditional)
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'driver_view_launch.py'])
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'use_rviz': 'false',  # minimalmode does not startRViz
        }.items(),
        condition=UnlessCondition(use_sim)
    )

    def print_startup_info(context):
        """Print startup information based on use_sim value"""
        use_sim_value = context.launch_configurations.get('use_sim', 'false')
        mode = 'Simulation' if use_sim_value == 'true' else 'Real Hardware'
        
        print('\n' + '='*60)
        print('Starting Minimal Arm System (No GUI)')
        print('='*60)
        print(f'Mode: {mode}')
        print('='*60 + '\n')
        return []

    return LaunchDescription([
        # Arguments
        use_sim_arg,
        serial_port_arg,
        baud_rate_arg,
        
        # Info
        OpaqueFunction(function=print_startup_info),
        
        # Launch files
        sim_launch,
        driver_launch,
    ])
