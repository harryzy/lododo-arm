from typing import List

MOVE_GROUP_ARM: str = "arm_grp"
MOVE_GROUP_HAND: str = "hand_grp"

# Gripper constants (updated for revolute single-joint gripper)
# Open angle (radians) and closed angle (radians). Open is -90deg, closed is 0rad.
GRIPPER_OPEN_ANGLE: float = 0.0
GRIPPER_CLOSED_ANGLE: float = -1.5708

# Backwards-compatible lists for MoveIt group states (single joint)
OPEN_HAND_JOINT_POSITIONS: List[float] = [GRIPPER_OPEN_ANGLE]
CLOSED_HAND_JOINT_POSITIONS: List[float] = [GRIPPER_CLOSED_ANGLE]

# If older code supplied gripper width in meters, use this as the historical max width
# to be mapped to the angular range by the adapter when needed.
GRIPPER_MAX_WIDTH_M: float = 0.04

GRIPPER_COMMAND_ACTION_NAME: str = "/hand_controller/follow_joint_trajectory"


def arm_joint_names() -> List[str]:
    return [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        # "joint6",
    ]


def arm_joint_home_positions() -> List[float]:
    return [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        # 0.0,
    ]


def base_link_name() -> str:
    return "base_link"


def end_effector_name() -> str:
    return "grasping_frame"
# "grasping_frame"


def hand_joint_names() -> List[str]:
    return ["finger_joint1"]

def init_scan_pose() -> List[float]:
    return [
        0.0,
        -65.0,# -25.0,
        85.0,#150.0,# 90.0,
        90.0,#40.0,# 80.0,
        0.0,
        # 0.0,
    ]