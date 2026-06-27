"""Pure functions shared by teleoperation nodes and tests."""

from math import isfinite
from typing import Iterable, Sequence


def axis_value(axes: Sequence[float], index: int, deadzone: float) -> float:
    """Read a joystick axis safely and apply a symmetric deadzone."""
    if not 0.0 <= deadzone < 1.0:
        raise ValueError("deadzone must be in [0, 1)")
    if index < 0 or index >= len(axes):
        return 0.0
    value = float(axes[index])
    if not isfinite(value) or abs(value) <= deadzone:
        return 0.0
    magnitude = (abs(value) - deadzone) / (1.0 - deadzone)
    return (-1.0 if value < 0.0 else 1.0) * magnitude


def button_pressed(buttons: Sequence[int], index: int) -> bool:
    """Read a joystick button safely."""
    return 0 <= index < len(buttons) and bool(buttons[index])


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a scalar to an inclusive range."""
    if lower > upper:
        raise ValueError("Lower limit must not exceed upper limit")
    return min(upper, max(lower, value))


def extract_positions(
    message_names: Iterable[str],
    message_positions: Iterable[float],
    source_names: Sequence[str],
) -> list[float]:
    """Extract ordered source joints from a JointState-like pair of arrays."""
    names = list(message_names)
    positions = list(message_positions)
    if len(names) != len(positions):
        raise ValueError("JointState name and position arrays have different lengths")
    by_name = dict(zip(names, positions))
    missing = [name for name in source_names if name not in by_name]
    if missing:
        raise ValueError(f"Missing source joints: {', '.join(missing)}")
    values = [float(by_name[name]) for name in source_names]
    if not all(isfinite(value) for value in values):
        raise ValueError("Source joint state contains non-finite positions")
    return values


def filtered_step(
    previous: Sequence[float] | None,
    target: Sequence[float],
    alpha: float,
    max_delta: float,
) -> list[float]:
    """Apply a low-pass filter and per-cycle movement bound."""
    if not 0.0 < alpha <= 1.0:
        raise ValueError("filter_alpha must be in (0, 1]")
    if max_delta <= 0.0:
        raise ValueError("max_delta must be positive")
    if previous is None:
        return list(target)
    if len(previous) != len(target):
        raise ValueError("Previous and target vectors have different lengths")

    result = []
    for old, raw in zip(previous, target):
        filtered = alpha * raw + (1.0 - alpha) * old
        result.append(old + clamp(filtered - old, -max_delta, max_delta))
    return result


def mapped_positions(
    values: Sequence[float], scales: Sequence[float], offsets: Sequence[float]
) -> list[float]:
    """Apply per-joint affine transforms for leader-follower calibration."""
    if not (len(values) == len(scales) == len(offsets)):
        raise ValueError("values, scales and offsets must have equal length")
    return [value * scale + offset for value, scale, offset in zip(values, scales, offsets)]
