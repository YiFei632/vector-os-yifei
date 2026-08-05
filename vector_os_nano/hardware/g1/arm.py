# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Named left/right ArmProtocol views over one G1 coordinator."""
from __future__ import annotations

import math
from typing import Sequence

from vector_os_nano.hardware.g1.coordinator import G1ControlCoordinator
from vector_os_nano.hardware.g1.ik import G1ArmKinematics


class G1Arm:
    def __init__(
        self,
        coordinator: G1ControlCoordinator,
        side: str,
        *,
        kinematics: G1ArmKinematics | None = None,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError("G1 arm side must be 'left' or 'right'")
        self.coordinator = coordinator
        self.side = side
        self._kinematics = kinematics
        self._owner = f"arm:{side}"
        self._group = f"{side}_arm"
        self._joint_names = coordinator.profile.joint_group(self._group)

    @property
    def name(self) -> str:
        return f"g1_{self.side}_arm"

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
    def tcp_frame(self) -> str:
        return self.coordinator.profile.tcp_frames[self.side]

    @property
    def supports_cartesian(self) -> bool:
        return self._kinematics is not None

    @property
    def supports_joint_control(self) -> bool:
        return self.coordinator.capabilities.supports_group(self._group)

    def connect(self) -> None:
        self.coordinator.connect(owner=self._owner)

    def disconnect(self) -> None:
        self.coordinator.disconnect(owner=self._owner)

    def get_joint_positions(self) -> list[float]:
        state = self.coordinator.read_state()
        return list(state.positions_for(self._joint_names))

    def get_joint_velocities(self) -> list[float]:
        state = self.coordinator.read_state()
        return list(state.velocities_for(self._joint_names))

    def get_joint_efforts(self) -> list[float]:
        state = self.coordinator.read_state()
        return list(state.efforts_for(self._joint_names))

    def move_joints(self, positions: list[float], duration: float = 3.0) -> bool:
        values = tuple(float(value) for value in positions)
        if len(values) != self.dof:
            raise ValueError(f"{self.name} requires {self.dof} joint positions")
        self.coordinator.command_group(
            self._group,
            self._joint_names,
            values,
            duration=duration,
        )
        timeout = max(3.0, float(duration) + 2.0)
        return self.coordinator.wait_group_target(
            self._group,
            values,
            tolerance=0.08,
            timeout=timeout,
        )

    def move_cartesian(
        self,
        target_xyz: tuple[float, float, float],
        duration: float = 3.0,
    ) -> bool:
        solution = self.ik(target_xyz)
        if solution is None:
            return False
        return self.move_joints(solution, duration=duration)

    def fk(
        self,
        joint_positions: list[float],
    ) -> tuple[list[float], list[list[float]]]:
        solver = self._require_kinematics()
        values = tuple(float(value) for value in joint_positions)
        if len(values) != self.dof:
            raise ValueError(f"{self.name} FK requires {self.dof} joint positions")
        self.coordinator.profile.validate_joint_positions(self._joint_names, values)
        position_raw, rotation_raw = solver.fk(self.side, values)
        position = [float(value) for value in position_raw]
        rotation = [[float(value) for value in row] for row in rotation_raw]
        if len(position) != 3 or len(rotation) != 3 or any(len(row) != 3 for row in rotation):
            raise ValueError("G1 kinematics backend returned an invalid FK shape")
        if not all(
            math.isfinite(value)
            for value in (*position, *(value for row in rotation for value in row))
        ):
            raise ValueError("G1 kinematics backend returned non-finite FK values")
        return position, rotation

    def ik(
        self,
        target_xyz: tuple[float, float, float],
        current_joints: list[float] | None = None,
    ) -> list[float] | None:
        solver = self._require_kinematics()
        target = tuple(float(value) for value in target_xyz)
        if len(target) != 3 or not all(math.isfinite(value) for value in target):
            raise ValueError("G1 IK target must contain three finite values")
        if current_joints is None:
            seed = tuple(self.get_joint_positions())
        else:
            seed = tuple(float(value) for value in current_joints)
            if len(seed) != self.dof:
                raise ValueError(f"{self.name} IK seed requires {self.dof} joint positions")
            self.coordinator.profile.validate_joint_positions(self._joint_names, seed)
        solution_raw = solver.ik(self.side, target, seed)
        if solution_raw is None:
            return None
        solution = tuple(float(value) for value in solution_raw)
        if len(solution) != self.dof:
            raise ValueError("G1 kinematics backend returned the wrong IK dimension")
        self.coordinator.profile.validate_joint_positions(self._joint_names, solution)
        return list(solution)

    def stop(self) -> None:
        if not self.coordinator.connected:
            return
        try:
            self.coordinator.hold_group(self._group)
        except Exception:
            # If a group hold cannot be sent, fall back to the whole-body safe
            # stop.  stop() is intentionally best effort and non-throwing.
            self.coordinator.stop(emergency=False)

    def _require_kinematics(self) -> G1ArmKinematics:
        if self._kinematics is None:
            raise NotImplementedError(
                f"{self.name} Cartesian control requires a matching G1 rev-1.0 "
                "kinematics backend; none was configured"
            )
        return self._kinematics


__all__ = ["G1Arm"]
