import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Launch RViz with the arm control panel plugin and full MoveIt configuration."""
    
    # Get the package share directories
    pkg_share = get_package_share_directory('arm_rviz_plugin')
    
    # Path to the RViz config file
    rviz_config_file = os.path.join(pkg_share, 'config', 'moveit_plugin.rviz')
    
    # Load MoveIt configuration (includes kinematics, SRDF, URDF, etc.)
    moveit_config = MoveItConfigsBuilder("arm", package_name="arm_moveit_config").to_moveit_configs()
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[
                # Load complete MoveIt configuration to make Interactive Markers work properly
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,  # ← Key! Used for IK and Interactive Markers
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )
    ])
