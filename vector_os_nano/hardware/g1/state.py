# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Immutable command and state types shared by all G1 backends."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import Mapping, Sequence

from vector_os_nano.core.types import Pose3D


def _finite_tuple(values: Sequence[float], *, size: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size:
        raise ValueError(f"{label} must contain {size} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains a non-finite value")
    return result


class G1ControlMode(str, Enum):
    PASSIVE = "passive"
    STAND = "stand"
    LOCOMOTION = "locomotion"
    MANIPULATION = "manipulation"
    EMERGENCY_DAMPING = "emergency_damping"


@dataclass(frozen=True)
class G1VelocityCommand:
    vx: float
    vy: float
    vyaw: float
    sequence_id: int
    timestamp: float = field(default_factory=time.monotonic)
    ttl: float = 0.3

    def __post_init__(self) -> None:
        values = tuple(float(value) for value in (self.vx, self.vy, self.vyaw))
        object.__setattr__(self, "vx", values[0])
        object.__setattr__(self, "vy", values[1])
        object.__setattr__(self, "vyaw", values[2])
        object.__setattr__(self, "timestamp", float(self.timestamp))
        object.__setattr__(self, "ttl", float(self.ttl))
        if not all(math.isfinite(value) for value in (*values, self.timestamp, self.ttl)):
            raise ValueError("velocity command contains a non-finite value")
        if not isinstance(self.sequence_id, int) or self.sequence_id < 0:
            raise ValueError("sequence_id must be a non-negative integer")
        if self.ttl <= 0:
            raise ValueError("command TTL must be positive")


@dataclass(frozen=True)
class G1JointCommand:
    group: str
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    sequence_id: int
    duration: float = 0.0
    timestamp: float = field(default_factory=time.monotonic)
    ttl: float = 1.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "group", str(self.group))
        object.__setattr__(self, "joint_names", tuple(str(name) for name in self.joint_names))
        object.__setattr__(self, "positions", tuple(float(value) for value in self.positions))
        object.__setattr__(self, "duration", float(self.duration))
        object.__setattr__(self, "timestamp", float(self.timestamp))
        object.__setattr__(self, "ttl", float(self.ttl))
        if len(self.joint_names) != len(self.positions):
            raise ValueError("joint command names and positions must have equal length")
        if not self.group or not self.joint_names or any(not name for name in self.joint_names):
            raise ValueError("joint command cannot be empty")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint command contains duplicate names")
        if not isinstance(self.sequence_id, int) or self.sequence_id < 0:
            raise ValueError("sequence_id must be a non-negative integer")
        if (
            not math.isfinite(self.duration)
            or not math.isfinite(self.timestamp)
            or not math.isfinite(self.ttl)
            or self.duration < 0
            or self.ttl <= 0
        ):
            raise ValueError("joint command duration/TTL is invalid")
        if not all(math.isfinite(value) for value in self.positions):
            raise ValueError("joint command contains a non-finite position")


@dataclass(frozen=True)
class G1State:
    """One coherent state snapshot in semantic joint-name order."""

    timestamp: float
    joint_names: tuple[str, ...]
    joint_positions: tuple[float, ...]
    joint_velocities: tuple[float, ...]
    joint_efforts: tuple[float, ...]
    root_pose: Pose3D = field(default_factory=Pose3D)
    root_linear_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    root_angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_accelerometer: tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_gyroscope: tuple[float, float, float] = (0.0, 0.0, 0.0)
    foot_contacts: Mapping[str, bool] = field(
        default_factory=lambda: {"left": False, "right": False}
    )
    hand_holding: Mapping[str, bool] = field(
        default_factory=lambda: {"left": False, "right": False}
    )
    hand_forces: Mapping[str, float | None] = field(
        default_factory=lambda: {"left": None, "right": None}
    )
    control_mode: G1ControlMode = G1ControlMode.PASSIVE
    fallen: bool = False
    fault: str | None = None
    sequence_id: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp", float(self.timestamp))
        object.__setattr__(self, "joint_names", tuple(str(name) for name in self.joint_names))
        object.__setattr__(
            self,
            "joint_positions",
            tuple(float(value) for value in self.joint_positions),
        )
        object.__setattr__(
            self,
            "joint_velocities",
            tuple(float(value) for value in self.joint_velocities),
        )
        object.__setattr__(
            self,
            "joint_efforts",
            tuple(float(value) for value in self.joint_efforts),
        )
        object.__setattr__(
            self,
            "root_linear_velocity",
            _finite_tuple(self.root_linear_velocity, size=3, label="root linear velocity"),
        )
        object.__setattr__(
            self,
            "root_angular_velocity",
            _finite_tuple(self.root_angular_velocity, size=3, label="root angular velocity"),
        )
        object.__setattr__(
            self,
            "imu_accelerometer",
            _finite_tuple(self.imu_accelerometer, size=3, label="IMU accelerometer"),
        )
        object.__setattr__(
            self,
            "imu_gyroscope",
            _finite_tuple(self.imu_gyroscope, size=3, label="IMU gyroscope"),
        )
        object.__setattr__(self, "foot_contacts", dict(self.foot_contacts))
        object.__setattr__(self, "hand_holding", dict(self.hand_holding))
        object.__setattr__(self, "hand_forces", dict(self.hand_forces))
        object.__setattr__(self, "control_mode", G1ControlMode(self.control_mode))

        if not math.isfinite(self.timestamp):
            raise ValueError("G1State timestamp must be finite")
        if not isinstance(self.sequence_id, int) or self.sequence_id < 0:
            raise ValueError("G1State sequence_id must be a non-negative integer")
        size = len(self.joint_names)
        if not size or any(not name for name in self.joint_names):
            raise ValueError("G1State joint_names cannot be empty")
        if size != len(set(self.joint_names)):
            raise ValueError("G1State contains duplicate joint names")
        for label, values in (
            ("positions", self.joint_positions),
            ("velocities", self.joint_velocities),
            ("efforts", self.joint_efforts),
        ):
            if len(values) != size:
                raise ValueError(f"joint {label} length {len(values)} != joint_names length {size}")
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"joint {label} contains a non-finite value")
        pose_values = (
            self.root_pose.x,
            self.root_pose.y,
            self.root_pose.z,
            self.root_pose.qx,
            self.root_pose.qy,
            self.root_pose.qz,
            self.root_pose.qw,
        )
        if not all(math.isfinite(float(value)) for value in pose_values):
            raise ValueError("root pose contains a non-finite value")
        quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in pose_values[3:]))
        if quaternion_norm <= 1e-12:
            raise ValueError("root pose quaternion cannot be zero")
        for label, mapping in (
            ("foot_contacts", self.foot_contacts),
            ("hand_holding", self.hand_holding),
        ):
            unknown = set(mapping) - {"left", "right"}
            if unknown:
                raise ValueError(f"{label} contains unknown sides: {sorted(unknown)}")
        unknown_force_sides = set(self.hand_forces) - {"left", "right"}
        if unknown_force_sides:
            raise ValueError(
                f"hand_forces contains unknown sides: {sorted(unknown_force_sides)}"
            )
        for side, force in self.hand_forces.items():
            if force is not None and not math.isfinite(float(force)):
                raise ValueError(f"hand force for {side!r} is non-finite")

    @classmethod
    def zero(cls, joint_names: Sequence[str]) -> "G1State":
        names = tuple(joint_names)
        zeros = (0.0,) * len(names)
        return cls(
            timestamp=time.monotonic(),
            joint_names=names,
            joint_positions=zeros,
            joint_velocities=zeros,
            joint_efforts=zeros,
        )

    def positions_for(self, names: Sequence[str]) -> tuple[float, ...]:
        return self._values_for(names, self.joint_positions, label="positions")

    def velocities_for(self, names: Sequence[str]) -> tuple[float, ...]:
        return self._values_for(names, self.joint_velocities, label="velocities")

    def efforts_for(self, names: Sequence[str]) -> tuple[float, ...]:
        return self._values_for(names, self.joint_efforts, label="efforts")

    def _values_for(
        self,
        names: Sequence[str],
        values: tuple[float, ...],
        *,
        label: str,
    ) -> tuple[float, ...]:
        lookup = {name: index for index, name in enumerate(self.joint_names)}
        missing = [name for name in names if name not in lookup]
        if missing:
            raise KeyError(f"state {label} are missing joints: {missing}")
        return tuple(values[lookup[name]] for name in names)


__all__ = [
    "G1ControlMode",
    "G1VelocityCommand",
    "G1JointCommand",
    "G1State",
]
