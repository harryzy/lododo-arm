#!/usr/bin/env python3
"""
YOLO Perception System Launch File (Simplified Version)

Started Nodes:
1. YoloDetector - YOLO Detection (running in virtual environment)
2. TriangulationNode - Triangulation (automatically enabled)
3. ProjectionNode - Result Publishing (single-view + dual-view)

Interfaces:
  Input: /detection_request (String) - Trigger detection
  Output: /detected_objects_json (String) - Unified result
       /collision_object (CollisionObject) - Scene object
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import sys, os


def generate_launch_description():
    # Package path
    pkg_share = get_package_share_directory('arm_perception_yolo')
    
    # Configuration files
    perception_cfg = PathJoinSubstitution(
        [FindPackageShare("arm_perception_yolo"), "config", "perception_params.yaml"]
    )
    measurement_cfg = os.path.join(pkg_share, 'config', 'measurement_params.yaml')
    
    # Virtual environment configuration
    venv = os.path.expanduser("~/yolo_venv")
    venv_bin = os.path.join(venv, "bin")
    env = os.environ.copy()
    env["VIRTUAL_ENV"] = venv
    env["PATH"] = venv_bin + os.pathsep + env.get("PATH", "")
    
    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8m.pt',  # 🚀 Use YOLOv8m model
        description='YOLO model path (yolov8n/s/m/l/x.pt)'
    )
    
    min_conf_arg = DeclareLaunchArgument(
        'min_conf',
        default_value='0.15',  # 🎯 Lower threshold to detect hard-to-recognize objects
        description='Minimum detection confidence (0.0-1.0)'
    )
    
    detection_mode_arg = DeclareLaunchArgument(
        'detection_mode',
        default_value='triggered',
        description='Detection mode: triggered or continuous'
    )
    
    # Note: baseline is now a GLOBAL parameter in measurement_params.yaml
    # This launch argument is kept for backward compatibility and override capability
    # Leave empty to use yaml value, or specify like: baseline:=0.20 to override
    baseline_arg = DeclareLaunchArgument(
        'baseline',
        default_value='',  # Empty = use value from measurement_params.yaml
        description='Override baseline angle (empty=use yaml, e.g., 0.20 for 20°)'
    )
    
    detect_cube_only_arg = DeclareLaunchArgument(
        'detect_cube_only',
        default_value='False',
        description='Whether to detect only cubes (True/False)'
    )
    
    cube_aspect_ratio_tolerance_arg = DeclareLaunchArgument(
        'cube_aspect_ratio_tolerance',
        default_value='0.3',
        description='Cube bbox aspect ratio tolerance'
    )
    
    cube_dimension_tolerance_arg = DeclareLaunchArgument(
        'cube_dimension_tolerance',
        default_value='0.25',
        description='Cube 3D dimension tolerance'
    )
    
    # Get parameter values
    model_path = LaunchConfiguration('model_path')
    min_conf = LaunchConfiguration('min_conf')
    detection_mode = LaunchConfiguration('detection_mode')
    baseline = LaunchConfiguration('baseline')
    detect_cube_only = LaunchConfiguration('detect_cube_only')
    cube_aspect_ratio_tolerance = LaunchConfiguration('cube_aspect_ratio_tolerance')
    cube_dimension_tolerance = LaunchConfiguration('cube_dimension_tolerance')
    
    # 1. YOLO Detector (Running in virtual environment)
    yolo_node = Node(
        package="arm_perception_yolo",
        executable=os.path.join(os.path.dirname(__file__), "..", "scripts", "run_yolo_venv.sh"),
        name="yolo_detector",
        parameters=[
            {
                "model_path": model_path,
                "min_conf": min_conf,
                "publish_label_mode": "both",
                "detection_mode": detection_mode,
            }
        ],
        output="screen",
        emulate_tty=True,
        env=env,
    )
    
    # 2. TriangulationNode (Triangulation, enabled by default)
    # Note: baseline is loaded from global parameters in measurement_cfg
    #       Only override here if launch argument is provided (non-empty)
    triangulation_node = Node(
        package='arm_perception_yolo',
        executable='triangulation_node',
        name='triangulation_node',
        output='screen',
        parameters=[
            measurement_cfg,  # Contains global baseline parameter
            {
                'baseline': baseline,  # Optional override (if baseline launch arg is non-empty)
                'enable_triangulation': True,  # Enabled by default
                'detect_cube_only': detect_cube_only,  # Cube detection mode
                'cube_aspect_ratio_tolerance': cube_aspect_ratio_tolerance,  # bbox aspect ratio tolerance
                'cube_dimension_tolerance': cube_dimension_tolerance,  # 3D dimension tolerance
            }
        ],
        remappings=[
            ('/yolo/detections', '/yolo/detections'),
            ('/measured_objects', '/measured_objects'),
        ]
    )
    
    # 3. ProjectionNode (Unified result publishing)
    projection_node = Node(
        package="arm_perception_yolo",
        executable="projection_node",
        name="projection_node",
        parameters=[
            {
                "config": perception_cfg,
                "detection_mode": detection_mode,
                "allowed_sites": ["front", "left", "right"],
                "grasp_z_offset_pct": 0.2,
            },
            measurement_cfg  # Includes position_correction parameters
        ],
        output="screen",
    )
    
    # Startup information
    startup_info = LogInfo(
        msg=[
            '\n',
            '='*70, '\n',
            '🤖 YOLO Perception System Startup (Simplified)\n',
            '='*70, '\n',
            'Nodes:\n',
            '  ✓ YoloDetector (Virtual environment)\n',
            '  ✓ TriangulationNode (Dual-view measurement)\n',
            '  ✓ ProjectionNode (Unified result publishing)\n',
            '\n',
            'Parameters:\n',
            '  • model_path: ', model_path, '\n',
            '  • min_conf: ', min_conf, '\n',
            '  • detection_mode: ', detection_mode, '\n',
            '  • baseline: from measurement_params.yaml (override: ', baseline, ')\n',
            '  • detect_cube_only: ', detect_cube_only, '\n',
            '\n',
            'Configuration:\n',
            '  📄 Global params: measurement_params.yaml (baseline, dual_view_timeout)\n',
            '  📄 Perception config: perception_params.yaml\n',
            '\n',
            'Interfaces:\n',
            '  Input: /detection_request (String)\n',
            '  Output: /detected_objects_json (String)\n',
            '       /collision_object (CollisionObject)\n',
            '       /measured_objects (MeasuredObject)\n',
            '\n',
            'Workflow:\n',
            '  Single-view: /detection_request → YOLO → ProjectionNode → JSON\n',
            '  Dual-view: /detection_request (view1) → YOLO\n',
            '         /detection_request (view2) → YOLO → Triangulation → JSON\n',
            '\n',
            '💡 To change baseline: Edit measurement_params.yaml or use baseline:=0.20\n',
            '='*70, '\n'
        ]
    )
    
    return LaunchDescription(
        [
            # Parameter declarations
            model_path_arg,
            min_conf_arg,
            detection_mode_arg,
            baseline_arg,
            detect_cube_only_arg,
            cube_aspect_ratio_tolerance_arg,
            cube_dimension_tolerance_arg,
            
            # Startup information
            startup_info,
            
            # Nodes
            yolo_node,
            triangulation_node,
            projection_node,
        ]
    )
