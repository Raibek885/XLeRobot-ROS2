"""Interactive calibration entry point for XLeRobot hardware."""

import argparse
from importlib import import_module
from typing import Any


ROBOT_TYPES = {
    "xlerobot": (
        "xlerobot_driver.hardware.xlerobot",
        "XLerobotConfig",
        "XLerobot",
    ),
    "xlerobot_2wheels": (
        "xlerobot_driver.hardware.xlerobot_2wheels",
        "XLerobot2WheelsConfig",
        "XLerobot2Wheels",
    ),
    "xlerobot_mecanum": (
        "xlerobot_driver.hardware.xlerobot_mecanum",
        "XLerobotConfig",
        "XLerobot",
    ),
}


def make_robot(variant: str, robot_id: str, port1: str, port2: str) -> Any:
    """Construct one of the bundled XLeRobot hardware variants."""
    module_name, config_name, robot_name = ROBOT_TYPES[variant]
    module = import_module(module_name)
    config_class = getattr(module, config_name)
    robot_class = getattr(module, robot_name)
    config = config_class(
        id=robot_id,
        port1=port1,
        port2=port2,
        use_degrees=True,
        disable_torque_on_disconnect=True,
        max_relative_target=None,
    )
    return robot_class(config)


def main() -> None:
    """Run interactive arm/head/base calibration from a real terminal."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=sorted(ROBOT_TYPES), default="xlerobot")
    parser.add_argument("--robot-id", default="xlerobot_ros2")
    parser.add_argument("--port1", default="/dev/ttyACM0")
    parser.add_argument("--port2", default="/dev/ttyACM1")
    args = parser.parse_args()

    robot = make_robot(args.variant, args.robot_id, args.port1, args.port2)
    try:
        robot.connect(calibrate=True)
        print(f"Calibration completed: {robot.calibration_fpath}")
    finally:
        if robot.is_connected:
            robot.disconnect()
