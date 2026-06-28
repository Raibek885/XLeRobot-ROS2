"""Serialization and interpolation helpers for recorded joint macros."""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import UTC, datetime
from math import isfinite
from pathlib import Path
from typing import Iterable


MACRO_VERSION = 1


@dataclass(frozen=True)
class JointMacro:
    """Uniformly sampled joint trajectory stored in ROS joint units."""

    joint_names: list[str]
    sample_rate: float
    frames: list[list[float]]
    created_at: str

    @classmethod
    def from_frames(
        cls,
        joint_names: Iterable[str],
        sample_rate: float,
        frames: Iterable[Iterable[float]],
    ) -> "JointMacro":
        macro = cls(
            joint_names=list(joint_names),
            sample_rate=float(sample_rate),
            frames=[[float(value) for value in frame] for frame in frames],
            created_at=datetime.now(UTC).isoformat(),
        )
        macro.validate()
        return macro

    @classmethod
    def load(cls, path: str | Path) -> "JointMacro":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if int(data.get("version", -1)) != MACRO_VERSION:
            raise ValueError(f"Unsupported macro version: {data.get('version')}")
        macro = cls(
            joint_names=[str(name) for name in data["joint_names"]],
            sample_rate=float(data["sample_rate"]),
            frames=[[float(value) for value in frame] for frame in data["frames"]],
            created_at=str(data.get("created_at", "")),
        )
        macro.validate()
        return macro

    def validate(self) -> None:
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be non-empty and unique")
        if not isfinite(self.sample_rate) or self.sample_rate <= 0.0:
            raise ValueError("sample_rate must be positive")
        if not self.frames:
            raise ValueError("macro must contain at least one frame")
        width = len(self.joint_names)
        for index, frame in enumerate(self.frames):
            if len(frame) != width:
                raise ValueError(f"frame {index} has {len(frame)} values; expected {width}")
            if not all(isfinite(value) for value in frame):
                raise ValueError(f"frame {index} contains a non-finite value")

    @property
    def duration(self) -> float:
        return max(0.0, (len(self.frames) - 1) / self.sample_rate)

    def save(self, path: str | Path) -> None:
        self.validate()
        target = Path(path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_text(
            json.dumps(
                {
                    "version": MACRO_VERSION,
                    "created_at": self.created_at,
                    "joint_names": self.joint_names,
                    "sample_rate": self.sample_rate,
                    "frames": self.frames,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        temporary.replace(target)


def interpolate_positions(
    start: Iterable[float], goal: Iterable[float], fraction: float
) -> list[float]:
    start_values = list(start)
    goal_values = list(goal)
    if len(start_values) != len(goal_values):
        raise ValueError("start and goal must have the same length")
    alpha = min(1.0, max(0.0, float(fraction)))
    return [
        start_value + (goal_value - start_value) * alpha
        for start_value, goal_value in zip(start_values, goal_values)
    ]
