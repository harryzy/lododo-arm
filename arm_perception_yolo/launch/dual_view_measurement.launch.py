#!/usr/bin/env python3
"""
Dual-View Triangulation System Launch File

Start the following nodes:
1. YoloDetector - YOLO detection + Detection2D publishing
2. TriangulationNode - Triangulation
3. MeasurementCoordinator - Measurement coordinator
4. ScenePublisher - Scene publisher

Usage:
  ros2 launch arm_perception_yolo dual_view_measurement.launch.py

Parameters:
  enable_triangulation:=true   # Enable triangulation
  enable_coordinator:=true     # Enable coordinator
  enable_arm_movement:=false   # Enable automatic arm movement
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package path
    pkg_share = get_package_share_directory('arm_perception_yolo')
    
    # Configuration files
    measurement_params_file = os.path.join(pkg_share, 'config', 'measurement_params.yaml')
    
    # Declare parameters
    enable_triangulation_arg = DeclareLaunchArgument(
        'enable_triangulation',
        default_value='false',
        description='Enable triangulation (requires dual-view)'
    )
    
    enable_coordinator_arg = DeclareLaunchArgument(
        'enable_coordinator',
        default_value='true',
        description='Enable measurement coordinator'
    )
    
    enable_arm_movement_arg = DeclareLaunchArgument(
        'enable_arm_movement',
        default_value='false',
        description='Enable automatic arm movement (false=manual mode)'
    )
    
    baseline_arg = DeclareLaunchArgument(
        'baseline',
        default_value='0.20',
        description='Camera translation distance (meters)'
    )
    
    # Get parameters
    enable_triangulation = LaunchConfiguration('enable_triangulation')
    enable_coordinator = LaunchConfiguration('enable_coordinator')
    enable_arm_movement = LaunchConfiguration('enable_arm_movement')
    baseline = LaunchConfiguration('baseline')
    
    # 1. YOLO Detector (Use virtual environment launch script)
    # Note: Should actually be launched through yolo_perception_launch.py
    # This is only an example
    
    # 2. TriangulationNode
    triangulation_node = Node(
        package='arm_perception_yolo',
        executable='triangulation_node',
        name='triangulation_node',
        output='screen',
        parameters=[
            measurement_params_file,
            {
                'enable_triangulation': enable_triangulation,
                'baseline': baseline,
                'detect_cube_only': False,  # Do not enable cube detection mode by default
                'cube_aspect_ratio_tolerance': 0.3,  # bboxAspect ratio tolerance
                'cube_dimension_tolerance': 0.25,  # 3D dimension tolerance
            }
        ],
        remappings=[
            ('/yolo/detections', '/yolo/detections'),
            ('/measured_objects', '/measured_objects'),
        ]
    )
    
    # 3. MeasurementCoordinator
    coordinator_node = Node(
        package='arm_perception_yolo',
        executable='measurement_coordinator',
        name='measurement_coordinator',
        output='screen',
        parameters=[
            measurement_params_file,
            {
                'enable_arm_movement': enable_arm_movement,
                'baseline': baseline,
            }
        ],
        condition=lambda context: context.launch_configurations.get('enable_coordinator', 'true') == 'true'
    )
    
    # 4. ScenePublisher
    scene_publisher_node = Node(
        package='arm_perception_yolo',
        executable='scene_publisher',
        name='scene_publisher',
        output='screen',
        parameters=[
            measurement_params_file,
            {
                'publish_scene': True,
                'publish_json': True,
            }
        ]
    )
    
    # Startup information
    startup_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            '🔺 Dual-View Triangulation System Startup\n',
            '='*60, '\n',
            'Nodes:\n',
            '  • TriangulationNode - Triangulation\n',
            '  • MeasurementCoordinator - Measurement coordinator\n',
            '  • ScenePublisher - Scene publisher\n',
            '\n',
            'Parameters:\n',
            f'  • enable_triangulation: {enable_triangulation}\n',
            f'  • enable_coordinator: {enable_coordinator}\n',
            f'  • enable_arm_movement: {enable_arm_movement}\n',
            f'  • baseline: {baseline}m\n',
            '\n',
            'Topics:\n',
            '  Subscribe: /yolo/detections (Detection2DArray)\n',
            '  Publish: /measured_objects (MeasuredObject)\n',
            '       /collision_object (CollisionObject)\n',
            '       /detected_objects_json (String)\n',
            '\n',
            'Services:\n',
            '  /scan_and_measure (ScanAndMeasure)\n',
            '\n',
            'Actions:\n',
            '  /measure_object_action (MeasureObject)\n',
            '\n',
            '='*60, '\n',
        ]
    )
    
    return LaunchDescription([
        # Parameters
        enable_triangulation_arg,
        enable_coordinator_arg,
        enable_arm_movement_arg,
        baseline_arg,
        
        # Startup information
        startup_info,
        
        # Nodes
        triangulation_node,
        coordinator_node,
        scene_publisher_node,
    ])
