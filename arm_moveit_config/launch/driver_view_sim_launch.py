from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "arm", package_name="arm_moveit_config"
    ).to_moveit_configs()
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("arm_driver_node"),
                    "launch",
                    "driver_sim_launch.py",
                )
            ]
        ),
        launch_arguments={"serial_port": "/dev/ttyUSB0", "baud_rate": "115200"}.items(),
    )
    # return generate_move_group_launch(moveit_config)
    return LaunchDescription([driver_launch, generate_demo_launch(moveit_config)])
