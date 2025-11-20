from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('arm_description'),
        'urdf',
        'LeKiwi.urdf'
    )

    # robot_description = Command(['xacro ', urdf_file])
    # robot_description = Command(['ros2 run xacro xacro ', 'src/arm_description/urdf/arm.xacro'])
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('arm_description'), 'urdf', 'lekiwi.rviz')],
            output='screen'
        )
    ])