#!/usr/bin/env python3
"""
Real Robot Bringup Launch File
Starts the complete arm system with physical robot hardware, RViz and all functionalities.

This launch file includes:
- Real robot driver (serial communication)
- Robot state publisher
- MoveIt move_group
- RViz with custom control panel
- YOLO perception node
- Arm planning interface
- Voice control interface (optional)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
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
    
    # Declare launch arguments with values from YAML config
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
    
    use_voice_control_arg = DeclareLaunchArgument(
        'use_voice_control',
        default_value='true',
        description='Enable voice control interface'
    )
    
    use_perception_arg = DeclareLaunchArgument(
        'use_perception',
        default_value='true',
        description='Enable YOLO perception system'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch real USB camera (disable for simulation)'
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value=str(default_config.get('camera_device', '/dev/video0')),
        description='Camera device for perception'
    )
    
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value=str(default_config.get('camera_width', '640')),
        description='Camera image width'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value=str(default_config.get('camera_height', '480')),
        description='Camera image height'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value=str(default_config.get('camera_fps', '15.0')),
        description='Camera framerate (FPS)'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value=str(default_config.get('default_log_level', 'info')),
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_voice_control = LaunchConfiguration('use_voice_control')
    use_perception = LaunchConfiguration('use_perception')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera = LaunchConfiguration('use_camera')
    camera_device = LaunchConfiguration('camera_device')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')
    log_level = LaunchConfiguration('log_level')

    # Package directories
    moveit_config_pkg = FindPackageShare('arm_moveit_config')
    rviz_plugin_pkg = FindPackageShare('arm_rviz_plugin')
    arm_bringup_pkg = FindPackageShare('arm_bringup')

    # Launch robot driver with MoveIt (without RViz, we'll launch custom RViz with plugin instead)
    # Note: driver_view_launch.py already includes move_group with detailed configuration
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'driver_view_launch.py'])
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'use_rviz': 'false',  # Disable RViz here, we launch custom one below
        }.items()
    )

    # Note: move_group is already launched by driver_view_launch.py above
    # Do NOT launch it again to avoid duplicate nodes and resource waste

    # Launch RViz with custom control panel (delayed to ensure move_group is ready)
    # NOTE: Condition removed for debugging - RViz will ALWAYS launch
    rviz_launch = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[DEBUG] Starting RViz with custom control panel NOW!'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([rviz_plugin_pkg, 'launch', 'arm_control_rviz.launch.py'])
                ])
            )
        ]
    )
    
    # Add immediate log to confirm this code path is reached
    rviz_start_info = LogInfo(msg='[DEBUG] RViz launch configured - will start in 8 seconds...')
    
    # Launch real USB camera (matches Gazebo camera configuration)
    # Note: camera.launch.py uses fixed 640x480@15fps configuration
    # Because v4l2_camera image_size and time_per_frame parameters do not support LaunchConfiguration
    camera_launch = GroupAction(
        actions=[
            LogInfo(msg='[INFO] Starting USB camera node...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([arm_bringup_pkg, 'launch', 'camera.launch.py'])
                ]),
                launch_arguments={
                    'camera_device': camera_device,
                    'frame_id': 'camera_optical_frame',  # Must match URDF
                    'image_width': camera_width,
                    'image_height': camera_height,
                    'framerate': camera_fps,
                }.items()
            )
        ],
        condition=IfCondition(use_camera)
    )

    # Note: YOLO perception node is NOT launched here due to dependency conflicts
    # YOLO requires a separate virtual environment to avoid conflicts with MoveIt
    # To launch YOLO perception manually:
    #   ros2 run arm_bringup start_yolo_node.sh
    # Or in another terminal:
    #   cd ~/lododo-arm
    #   source ~/yolo_venv/bin/activate
    #   source install/setup.bash
    #   ros2 run arm_perception_yolo yolo_detection_node

    # Launch arm planning interface
    measurement_params_path = PathJoinSubstitution([
        FindPackageShare('arm_perception_yolo'),
        'config',
        'measurement_params.yaml'
    ])
    
    planning_node = Node(
        package='arm_planning_py',
        executable='arm_command_interface',
        name='arm_command_interface',
        output='screen',
        parameters=[measurement_params_path],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Note: arm_grasper is a class, not a standalone node
    # It's used by arm_command_interface internally

    # Launch voice control interface (conditional)
    voice_control_node = GroupAction(
        condition=IfCondition(use_voice_control),
        actions=[
            Node(
                package='arm_voice_interface',
                executable='arm_voice_node',
                name='arm_voice_node',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level]
            )
        ]
    )

    # Startup info
    startup_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            'Starting Real Arm System\n',
            '='*60, '\n',
            'Serial Port: ', serial_port, '\n',
            'Baud Rate: ', baud_rate, '\n',
            'Camera: ', use_camera, ' (', camera_device, ')\n',
            'Voice Control: ', use_voice_control, '\n',
            'Perception: ', use_perception, '\n',
            'RViz: ', use_rviz, '\n',
            '='*60, '\n'
        ]
    )

    return LaunchDescription([
        # Arguments
        serial_port_arg,
        baud_rate_arg,
        use_voice_control_arg,
        use_perception_arg,
        use_rviz_arg,
        use_camera_arg,  # New: control camera launch
        camera_device_arg,
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        log_level_arg,
        
        # Info
        startup_info,
        rviz_start_info,  # Debug info for RViz launch
        
        # Launch files
        driver_launch,  # Includes: driver, robot_state_publisher, move_group
        camera_launch,  # Real USB camera (replaces Gazebo camera)
        rviz_launch,    # Custom RViz with control panel plugin (unconditional for debug)
        
        # Nodes
        # Note: YOLO perception node must be started separately (see comments above)
        # Note: arm_grasper is used internally by arm_command_interface, not launched separately
        planning_node,
        voice_control_node,
    ])
