"""Small in-memory replacement for hardware, intended for bringup testing."""

from time import monotonic
from typing import Any

from .conversions import JOINT_NAMES


class MockRobot:
    """Implement the subset of the LeRobot Robot API used by the ROS bridge."""

    def __init__(self) -> None:
        self.is_connected = False
        self._positions = {f"{name}.pos": 0.0 for name in JOINT_NAMES}
        self._velocity = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        self._last_update = monotonic()

    def connect(self, calibrate: bool = False) -> None:
        del calibrate
        self.is_connected = True

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("Mock robot is disconnected")
        self._last_update = monotonic()
        return {**self._positions, **self._velocity}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("Mock robot is disconnected")
        self._positions.update(
            {key: value for key, value in action.items() if key.endswith(".pos")}
        )
        self._velocity.update(
            {key: value for key, value in action.items() if key.endswith(".vel")}
        )
        return action

    def stop_base(self) -> None:
        self._velocity = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

    def disconnect(self) -> None:
        self.stop_base()
        self.is_connected = False
