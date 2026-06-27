"""Unit conversion and validation helpers used by the driver node."""

from math import degrees, isfinite, radians
from typing import Iterable, Mapping


ARM_JOINTS = (
    "left_arm_shoulder_pan",
    "left_arm_shoulder_lift",
    "left_arm_elbow_flex",
    "left_arm_wrist_flex",
    "left_arm_wrist_roll",
    "left_arm_gripper",
    "right_arm_shoulder_pan",
    "right_arm_shoulder_lift",
    "right_arm_elbow_flex",
    "right_arm_wrist_flex",
    "right_arm_wrist_roll",
    "right_arm_gripper",
)

HEAD_JOINTS = ("head_motor_1", "head_motor_2")
JOINT_NAMES = ARM_JOINTS + HEAD_JOINTS
GRIPPER_JOINTS = {"left_arm_gripper", "right_arm_gripper"}


def ros_to_hardware_position(name: str, value: float) -> float:
    """Convert ROS radians (or normalized gripper opening) to driver units."""
    if not isfinite(value):
        raise ValueError(f"Non-finite command for {name}: {value}")
    if name in GRIPPER_JOINTS:
        return min(100.0, max(0.0, value * 100.0))
    return degrees(value)


def hardware_to_ros_position(name: str, value: float) -> float:
    """Convert driver degrees (or gripper percent) to ROS units."""
    if name in GRIPPER_JOINTS:
        return min(1.0, max(0.0, value / 100.0))
    return radians(value)


def joint_message_to_action(
    names: Iterable[str], positions: Iterable[float]
) -> dict[str, float]:
    """Turn a name/position message into a partial LeRobot action dictionary."""
    names_list = list(names)
    positions_list = list(positions)
    if len(names_list) != len(positions_list):
        raise ValueError("Joint names and positions must have equal length")
    if len(set(names_list)) != len(names_list):
        raise ValueError("Joint command contains duplicate names")

    unknown = sorted(set(names_list) - set(JOINT_NAMES))
    if unknown:
        raise ValueError(f"Unknown XLeRobot joints: {', '.join(unknown)}")

    return {
        f"{name}.pos": ros_to_hardware_position(name, value)
        for name, value in zip(names_list, positions_list)
    }


def observation_to_joint_positions(
    observation: Mapping[str, object],
) -> tuple[list[str], list[float]]:
    """Extract known joints from a LeRobot observation in stable order."""
    names: list[str] = []
    positions: list[float] = []
    for name in JOINT_NAMES:
        key = f"{name}.pos"
        value = observation.get(key)
        if value is None:
            continue
        names.append(name)
        positions.append(hardware_to_ros_position(name, float(value)))
    return names, positions


def twist_to_action(linear_x: float, linear_y: float, angular_z: float) -> dict[str, float]:
    """Convert ROS Twist SI units to the XLeRobot body velocity convention."""
    values = (linear_x, linear_y, angular_z)
    if not all(isfinite(value) for value in values):
        raise ValueError("Velocity command contains a non-finite value")
    return {
        "x.vel": linear_x,
        "y.vel": linear_y,
        "theta.vel": degrees(angular_z),
    }
