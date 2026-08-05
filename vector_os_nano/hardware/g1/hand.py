# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Dex3-1 joint API and generic GripperProtocol compatibility view."""
from __future__ import annotations

import math
from typing import Mapping

from vector_os_nano.hardware.g1.coordinator import G1ControlCoordinator


# Presets are expressed in HAL semantic order: thumb0/1/2, index0/1,
# middle0/1.  Backends reorder by name; left/right flexion signs differ.
_OPEN: Mapping[str, tuple[float, ...]] = {
    "left": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "right": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
}
_POWER_GRASP: Mapping[str, tuple[float, ...]] = {
    "left": (0.35, 0.80, 1.40, -1.25, -1.40, -1.25, -1.40),
    "right": (-0.35, -0.80, -1.40, 1.25, 1.40, 1.25, 1.40),
}
_PINCH: Mapping[str, tuple[float, ...]] = {
    "left": (0.35, 0.75, 1.20, -1.00, -1.20, 0.0, 0.0),
    "right": (-0.35, -0.75, -1.20, 1.00, 1.20, 0.0, 0.0),
}


class G1Hand:
    """Full seven-motor Dex3-1 view for one side."""

    def __init__(self, coordinator: G1ControlCoordinator, side: str) -> None:
        if side not in {"left", "right"}:
            raise ValueError("G1 hand side must be 'left' or 'right'")
        if side not in coordinator.profile.hands:
            raise ValueError(f"G1 profile has no {side!r} Dex3-1 hand")
        self.coordinator = coordinator
        self.side = side
        self._owner = f"hand:{side}"
        self._group = f"{side}_hand"
        self._joint_names = coordinator.profile.joint_group(self._group)

    @property
    def name(self) -> str:
        return f"g1_{self.side}_dex3_1"

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    @property
    def dof(self) -> int:
        return len(self._joint_names)

    @property
    def connected(self) -> bool:
        return self.coordinator.connected

    @property
    def supports_joint_control(self) -> bool:
        return self.coordinator.capabilities.supports_group(self._group)

    def connect(self) -> None:
        self.coordinator.connect(owner=self._owner)

    def disconnect(self) -> None:
        self.coordinator.disconnect(owner=self._owner)

    def get_joint_positions(self) -> list[float]:
        return list(self.coordinator.read_state().positions_for(self._joint_names))

    def get_joint_velocities(self) -> list[float]:
        return list(self.coordinator.read_state().velocities_for(self._joint_names))

    def get_joint_efforts(self) -> list[float]:
        return list(self.coordinator.read_state().efforts_for(self._joint_names))

    def set_joint_positions(
        self,
        positions: list[float] | tuple[float, ...],
        *,
        duration: float = 0.0,
    ) -> bool:
        values = tuple(float(value) for value in positions)
        if len(values) != self.dof:
            raise ValueError(f"{self.name} requires {self.dof} joint positions")
        self.coordinator.command_group(
            self._group,
            self._joint_names,
            values,
            duration=duration,
        )
        return True

    def move_joints(
        self,
        positions: list[float] | tuple[float, ...],
        duration: float = 1.0,
    ) -> bool:
        return self.set_joint_positions(positions, duration=duration)

    def open(self, *, duration: float = 1.0) -> bool:
        return self.set_joint_positions(self._preset("open"), duration=duration)

    def release(self, *, duration: float = 1.0) -> bool:
        return self.open(duration=duration)

    def close(self, *, duration: float = 1.0) -> bool:
        """Send the operational power-grasp preset.

        ``True`` means the command was accepted.  Object possession must be
        checked separately with :meth:`is_holding`.
        """
        return self.power_grasp(duration=duration)

    def power_grasp(self, *, duration: float = 1.0) -> bool:
        return self.set_joint_positions(self._preset("power_grasp"), duration=duration)

    def pinch(self, *, duration: float = 1.0) -> bool:
        return self.set_joint_positions(self._preset("pinch"), duration=duration)

    def is_holding(self) -> bool:
        state = self.coordinator.read_state()
        return bool(state.hand_holding.get(self.side, False))

    def get_force(self) -> float | None:
        force = self.coordinator.read_state().hand_forces.get(self.side)
        return None if force is None else float(force)

    def get_position(self) -> float:
        """Return normalized operational openness (0 closed, 1 open)."""
        current = self.get_joint_positions()
        opened = self._preset("open")
        closed = self._preset("power_grasp")
        openness: list[float] = []
        for value, open_value, close_value in zip(current, opened, closed):
            span = open_value - close_value
            if abs(span) <= 1e-9:
                continue
            openness.append((value - close_value) / span)
        if not openness:
            return 1.0
        return max(0.0, min(1.0, sum(openness) / len(openness)))

    def stop(self) -> None:
        if not self.coordinator.connected:
            return
        try:
            self.coordinator.hold_group(self._group)
        except Exception:
            self.coordinator.stop(emergency=False)

    def _preset(self, kind: str) -> tuple[float, ...]:
        key = f"{self.side}_hand_{kind}"
        configured = self.coordinator.profile.poses.get(key)
        if configured is not None:
            values = tuple(float(value) for value in configured)
        elif kind == "open":
            values = _OPEN[self.side]
        elif kind == "power_grasp":
            values = _POWER_GRASP[self.side]
        elif kind == "pinch":
            values = _PINCH[self.side]
        else:  # pragma: no cover - private method guards all callers
            raise ValueError(f"unknown Dex3-1 preset {kind!r}")
        if len(values) != self.dof or not all(math.isfinite(value) for value in values):
            raise ValueError(f"G1 pose {key!r} must contain {self.dof} finite values")
        self.coordinator.profile.validate_joint_positions(self._joint_names, values)
        return values


class G1GripperAdapter:
    """Expose a Dex3-1 hand through the existing scalar GripperProtocol."""

    def __init__(self, hand: G1Hand) -> None:
        self.hand = hand

    @property
    def name(self) -> str:
        return f"{self.hand.name}_gripper"

    @property
    def side(self) -> str:
        return self.hand.side

    @property
    def supports_joint_control(self) -> bool:
        return self.hand.supports_joint_control

    def connect(self) -> None:
        self.hand.connect()

    def disconnect(self) -> None:
        self.hand.disconnect()

    def open(self) -> bool:
        return self.hand.open()

    def close(self) -> bool:
        return self.hand.close()

    def is_holding(self) -> bool:
        return self.hand.is_holding()

    def get_position(self) -> float:
        return self.hand.get_position()

    def get_force(self) -> float | None:
        return self.hand.get_force()


__all__ = ["G1Hand", "G1GripperAdapter"]
