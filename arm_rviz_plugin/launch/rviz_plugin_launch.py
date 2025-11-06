from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_name = "arm_moveit_config"
    robot_urdf_name = "arm.xacro"
    world_file_name = "empty.world"


 # Path setup
    pkg_name = "arm_moveit_config"  # Change to your package name
    pkg_share = get_package_share_directory(pkg_name)

    kinematics_file = os.path.join(pkg_share, "config", "kinematics.yaml")

    # Verify file exists
    if not os.path.exists(kinematics_file):
        raise FileNotFoundError(f"kinematics.yaml not found at {kinematics_file}")
    # Explicitly load kinematics configuration
    with open(kinematics_file, "r") as f:
        kinematics_config = yaml.safe_load(f)

    # 1. Declare parameters
    declare_world_arg = DeclareLaunchArgument(
        name="world",
        default_value=PathJoinSubstitution(
            [FindPackageShare(pkg_name), "worlds", world_file_name]
        ),
        description="Gazebo world file path",
    )

    # 2. Correct URDF loading method - use robot_state_publisher
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_description"),  # Note: use arm_description package
                    "urdf",
                    robot_urdf_name,  # Note: use correct file name
                ]
            ),
        ]
    )

    # 3. Launch robot_state_publisher (key! solves Gazebo plugin connection issues)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": True},
            {"publish_frequency": 30.0},
        ],
        output="screen",
        # Ensure correct topic mapping
        remappings=[
            ("robot_description", "/robot_description"),
        ],
    )

    # 4. LaunchGazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": "true",
        }.items(),
    )

    # 5. Spawn robot model to Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_robot", "-z", "0.1"],
        output="screen",
    )

    # 6. Delayed controller launch (depends on Gazebo and controller manager)
    delayed_controller_loader = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],  # Use correct controller names
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "hand_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
        ],
    )
    # 7. Launch MoveIt2 (disable built-in RViz)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare(pkg_name),
                        "launch",
                        "move_group_simple_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    # 8. Standalone RViz2 (ensure planning group loaded)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_gazebo_moveit",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_rviz_plugin"),
                    "config",
                    "moveit_plugin.rviz",  # Confirm arm_grp planning group defined in configuration
                ]
            ),
        ],
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description_content},
            kinematics_config,
        ],
        output="screen",
        # Ensure correct topic remapping
        remappings=[
            ("display_planned_path", "/move_group/display_planned_path"),
            ("robot_description", "/robot_description"),
            ("/move_group/monitored_planning_scene", "/monitored_planning_scene"),
        ],
    )

    return LaunchDescription(
        [
            declare_world_arg,
            robot_state_publisher_node,  # Launch first
            gazebo_launch,  # Then launch Gazebo
            TimerAction(period=3.0, actions=[spawn_robot]),  # Delayed robot spawning
            delayed_controller_loader,  # Delayed controller launch
            TimerAction(period=12.0, actions=[moveit_launch]),  # Delayed MoveIt launch
            TimerAction(period=16.0, actions=[rviz_node]),
        ]
    )
