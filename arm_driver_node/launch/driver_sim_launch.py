from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for hardware communication'
        ),
        
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate'
        ),
        
        # 启动驱动节点
        Node(
            package='arm_driver_node',
            executable='arm_driver_node_sim',
            name='arm_driver_node_sim',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'update_rate': 50.0
            }],
            output='screen'
        )
    ])