# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Optional kinematics contract for G1 arms.

This module intentionally contains no placeholder solver.  A backend must
inject a solver built from the matching rev-1.0 URDF before Cartesian methods
become available.
"""
from __future__ import annotations

from typing import Protocol, Sequence, runtime_checkable


@runtime_checkable
class G1ArmKinematics(Protocol):
    def fk(
        self,
        side: str,
        joint_positions: Sequence[float],
    ) -> tuple[Sequence[float], Sequence[Sequence[float]]]: ...

    def ik(
        self,
        side: str,
        target_xyz: tuple[float, float, float],
        current_joints: Sequence[float],
    ) -> Sequence[float] | None: ...


__all__ = ["G1ArmKinematics"]
