import os
from launch import LaunchDescription
from launch_ros.actions import Node

VENV = os.path.expanduser("~/venv_yolo")


def generate_launch_description():
    # start from a copy of the current environment so we don't lose HOME, USER, etc.
    yolo_env = os.environ.copy()
    # ensure venv's bin is preferred
    yolo_env["PATH"] = f"{VENV}/bin:" + yolo_env.get("PATH", "")
    # let Python find venv site-packages first
    yolo_env["PYTHONPATH"] = f"{VENV}/lib/python3.10/site-packages:" + yolo_env.get(
        "PYTHONPATH", ""
    )
    yolo_env["VIRTUAL_ENV"] = VENV
    yolo_env["PYTHONNOUSERSITE"] = "1"
    yolo_env["PYTHONHOME"] = ""
    # Ensure ROS libs are found by the dynamic linker
    workspace_lib = os.path.join(
        os.path.expanduser("~"), "lododo_arm_yolo", "install", "lib"
    )
    yolo_env["LD_LIBRARY_PATH"] = (
        "/opt/ros/humble/lib:"
        + workspace_lib
        + ":"
        + yolo_env.get("LD_LIBRARY_PATH", "")
    )
    # Ensure HOME and ROS_HOME are set (rcutils needs these to expand logging dir)
    home_dir = os.path.expanduser("~")
    yolo_env.setdefault("HOME", home_dir)
    ros_home = os.environ.get("ROS_HOME", os.path.join(home_dir, ".ros"))
    yolo_env.setdefault("ROS_HOME", ros_home)

    # Create ROS home and log dirs so rcutils_expand_user can find them
    try:
        os.makedirs(os.path.join(ros_home, "log"), exist_ok=True)
    except Exception:
        # best-effort; leave error handling to rclpy if creation fails
        pass
    return LaunchDescription(
        [
            # Run the detector module using the venv python - launch will write a params file
            Node(
                package="arm_perception_yolo",
                executable=os.path.join(os.path.dirname(__file__), "..", "scripts", "run_yolo_venv.sh"),
                name="yolo_detector",
                parameters=[
                    {
                        "detection_mode": "triggered",
                        "publish_label_mode": "both",
                        "model_path": "yolov8n.pt",
                    }
                ],
                output="screen",
                emulate_tty=True,
                env=yolo_env,
            ),
            Node(
                package="arm_perception_yolo",
                executable="projection_node",
                name="projection_node",
                parameters=[
                    {
                        "detection_mode": "triggered",
                        "config": os.path.join(
                            os.path.expanduser("~"),
                            "lododo_arm_yolo/src/arm_perception_yolo/config/perception_params.yaml",
                        ),
                        "grasp_z_offset_pct": 0.2,
                    }
                ],
                output="screen",
                emulate_tty=True,
                env=yolo_env,
            ),
        ]
    )
