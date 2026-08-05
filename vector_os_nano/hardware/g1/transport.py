# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Backend contract implemented by Isaac Sim, MuJoCo, and later real G1."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from vector_os_nano.hardware.g1.profile import G1Profile
from vector_os_nano.hardware.g1.state import (
    G1ControlMode,
    G1JointCommand,
    G1State,
    G1VelocityCommand,
)


class G1CapabilityError(NotImplementedError):
    """Raised when the selected backend cannot provide a requested capability."""


@dataclass(frozen=True)
class G1TransportCapabilities:
    """Capabilities actually implemented by one transport instance.

    Capability flags describe working backend paths, not theoretical robot
    abilities.  An Isaac transport without a locomotion controller must set
    ``locomotion=False`` even though the physical G1 can walk.
    """

    locomotion: bool = False
    holonomic: bool = False
    lidar: bool = False
    emergency_damping: bool = False
    joint_groups: frozenset[str] = field(default_factory=frozenset)

    def __post_init__(self) -> None:
        groups = frozenset(str(group) for group in self.joint_groups)
        valid = {"body", "left_arm", "right_arm", "left_hand", "right_hand"}
        unknown = groups - valid
        if unknown:
            raise ValueError(f"unknown G1 transport joint groups: {sorted(unknown)}")
        if self.holonomic and not self.locomotion:
            raise ValueError("holonomic requires locomotion support")
        object.__setattr__(self, "joint_groups", groups)

    def supports_group(self, group: str) -> bool:
        return group in self.joint_groups


@runtime_checkable
class G1Transport(Protocol):
    @property
    def name(self) -> str: ...

    @property
    def profile(self) -> G1Profile: ...

    @property
    def connected(self) -> bool: ...

    @property
    def capabilities(self) -> G1TransportCapabilities: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def read_state(self) -> G1State: ...

    def command_velocity(self, command: G1VelocityCommand) -> None: ...

    def command_joints(self, command: G1JointCommand) -> None: ...

    def set_mode(self, mode: G1ControlMode) -> None: ...

    def stop(self) -> None: ...

    def get_lidar_scan(self) -> Any: ...


__all__ = ["G1CapabilityError", "G1TransportCapabilities", "G1Transport"]
