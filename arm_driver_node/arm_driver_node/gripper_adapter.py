"""
Gripper Adapter Module for Feetech Servo-Based Gripper.

Provides a compatibility layer to map various gripper command formats
to a single revolute joint angle and servo position for the physical gripper.

This adapter supports multiple input formats for maximum flexibility:
- Angle in radians (native format)
- Width in meters (legacy format from prismatic grippers)
- List/array formats (uses first element)

The gripper uses a revolute joint where:
- 0 rad = fully closed
- -π/2 rad (~-1.57 rad) = fully open
- Servo center position: 2048 (corresponds to 0 rad)

Author: lododo
License: Apache 2.0
"""
from typing import Optional, Sequence, Union
import math

# Servo configuration constants
GRIPPER_SERVO_ID = 6  # Physical servo ID for gripper
GRIPPER_CENTER_POSITION = 2048  # Servo center position (0 radians)
POSITION_PER_RAD = 2048 / math.pi  # Position units per radian


def width_to_angle(width_m: float, max_width_m: float = 0.04) -> float:
    """
    Convert legacy gripper width (meters) to gripper angle (radians).
    
    Maps linear width to angular position assuming:
    - width = 0.0m → fully closed (angle = 0 rad)
    - width = max_width_m → fully open (angle = -π/2 rad)
    
    Args:
        width_m: Gripper width in meters (0 to max_width_m)
        max_width_m: Maximum gripper width in meters (default: 0.04m = 40mm)
    
    Returns:
        Gripper angle in radians (0 to -π/2)
    """
    if width_m is None:
        return 0.0
    # Clamp fraction to [0, 1] range
    frac = max(0.0, min(1.0, width_m / max_width_m))
    # Map to angle: -π/2 = -1.5708 radians
    return -1.5708 * frac


def angle_to_servo_position(angle_rad: float) -> int:
    """
    Convert gripper angle (radians) to servo position units.
    
    Feetech servos use position units from 0 to 4095, with 2048 as center.
    
    Args:
        angle_rad: Gripper angle in radians
    
    Returns:
        Servo position in range [0, 4095]
    """
    pos = int(GRIPPER_CENTER_POSITION + angle_rad * POSITION_PER_RAD)
    # Clamp to valid servo range
    return max(0, min(4095, pos))


def normalize_gripper_input(value: Union[float, Sequence, None], max_width_m: float = 0.04) -> float:
    """
    Normalize various gripper input formats to a single angle in radians.
    
    Supported input formats:
    - float: Interpreted as angle in radians (if large) or width in meters (if small)
    - list/tuple/array: Uses the first element
    - None: Returns 0.0 (closed position)
    
    Heuristic for float values:
    - If 0 ≤ value ≤ 2*max_width_m: Treated as width in meters
    - Otherwise: Treated as angle in radians
    
    Args:
        value: Input value in various formats
        max_width_m: Maximum gripper width for width-to-angle conversion (default: 0.04m)
    
    Returns:
        Gripper angle in radians
    
    Raises:
        ValueError: If input type is not supported
    
    Examples:
        >>> normalize_gripper_input(0.02)  # 20mm width
        -0.7854  # About -45 degrees
        >>> normalize_gripper_input(-1.57)  # Direct angle
        -1.57  # -90 degrees (fully open)
        >>> normalize_gripper_input([0.03])  # List with width
        -1.1781  # About -67.5 degrees
    """
    if value is None:
        return 0.0

    # Extract first element if sequence
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        if len(value) == 0:
            return 0.0
        value = value[0]

    # Convert to numeric
    try:
        val = float(value)
    except Exception:
        raise ValueError(f"Unsupported gripper input type: {type(value)}")

    # Heuristic: small positive values are treated as width in meters
    if 0.0 <= val <= max_width_m * 2:
        return width_to_angle(val, max_width_m)

    # Otherwise treat as angle in radians
    return val
