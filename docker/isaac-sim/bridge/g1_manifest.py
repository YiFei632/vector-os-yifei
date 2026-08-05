#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics
"""Canonical manifest for the Unitree G1 EDU 29-DOF + dual Dex3 model.

The hand order below is the *pinned Isaac task's* action/observation order.
It is not the physical right-hand Dex3 DDS motor order.  Keep this module
dependency-free: it is imported by both Isaac Sim's Python 3.11 process and
the ROS 2 Python process.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence


GO2_ROBOT_TYPE = "go2"
G1_ROBOT_TYPE = "g1_29dof_dex3"

_ROBOT_TYPE_ALIASES = {
    "go2": GO2_ROBOT_TYPE,
    "g1": G1_ROBOT_TYPE,
    "g129": G1_ROBOT_TYPE,
    "g1_29dof_dex3": G1_ROBOT_TYPE,
    "g1-29dof-dex3": G1_ROBOT_TYPE,
}


G1_BODY_JOINTS: tuple[str, ...] = (
    # Left leg (6)
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    # Right leg (6)
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    # Waist (3)
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    # Left arm (7)
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    # Right arm (7)
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

G1_LEFT_ARM_JOINTS: tuple[str, ...] = G1_BODY_JOINTS[15:22]
G1_RIGHT_ARM_JOINTS: tuple[str, ...] = G1_BODY_JOINTS[22:29]

G1_LEFT_HAND_ISAAC_TASK_JOINTS: tuple[str, ...] = (
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
)

G1_RIGHT_HAND_ISAAC_TASK_JOINTS: tuple[str, ...] = (
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
)

G1_ALL_JOINTS: tuple[str, ...] = (
    G1_BODY_JOINTS
    + G1_LEFT_HAND_ISAAC_TASK_JOINTS
    + G1_RIGHT_HAND_ISAAC_TASK_JOINTS
)

# Compatibility aliases for older bridge consumers.  New code must use the
# explicit ISAAC_TASK names so these vectors are never reused for real DDS.
G1_LEFT_HAND_JOINTS = G1_LEFT_HAND_ISAAC_TASK_JOINTS
G1_RIGHT_HAND_JOINTS = G1_RIGHT_HAND_ISAAC_TASK_JOINTS

G1_LEFT_ARM_BODY_INDICES: tuple[int, ...] = tuple(range(15, 22))
G1_RIGHT_ARM_BODY_INDICES: tuple[int, ...] = tuple(range(22, 29))

COMMAND_CHANNEL_SIZES: dict[str, int] = {
    "base": 4,  # ROS convention: vx, vy, yaw_rate, standing_height
    "body": 29,
    "left_arm": 7,
    "right_arm": 7,
    "left_hand": 7,
    "right_hand": 7,
}


def resolve_robot_type(value: str | None) -> str:
    """Return a canonical robot type or raise instead of guessing."""
    key = (value or GO2_ROBOT_TYPE).strip().lower()
    try:
        return _ROBOT_TYPE_ALIASES[key]
    except KeyError as exc:
        supported = ", ".join(sorted(_ROBOT_TYPE_ALIASES))
        raise ValueError(
            f"Unsupported ISAAC_ROBOT_TYPE={value!r}; supported values: {supported}"
        ) from exc


def validate_joint_vector(
    channel: str,
    values: Sequence[float] | Iterable[float],
) -> list[float]:
    """Validate and normalize one command vector."""
    import math

    if channel not in COMMAND_CHANNEL_SIZES:
        raise ValueError(f"Unknown G1 command channel: {channel!r}")
    result = [float(value) for value in values]
    expected = COMMAND_CHANNEL_SIZES[channel]
    if len(result) != expected:
        raise ValueError(
            f"G1 {channel} command requires {expected} values, got {len(result)}"
        )
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"G1 {channel} command contains NaN or infinity")
    return result


@dataclass(frozen=True)
class G1Manifest:
    """Stable capability description written to the shared-state directory."""

    schema_version: int = 1
    robot_type: str = G1_ROBOT_TYPE
    model: str = "Unitree G1 EDU 29-DOF + dual Dex3"
    body_dof: int = 29
    left_hand_dof: int = 7
    right_hand_dof: int = 7

    def as_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "robot_type": self.robot_type,
            "model": self.model,
            "body_dof": self.body_dof,
            "left_hand_dof": self.left_hand_dof,
            "right_hand_dof": self.right_hand_dof,
            "joint_names": list(G1_ALL_JOINTS),
            "command_channels": dict(COMMAND_CHANNEL_SIZES),
        }


def validate_manifest() -> None:
    """Fail loudly if an edit breaks the official 29 + 7 + 7 contract."""
    if len(G1_BODY_JOINTS) != 29:
        raise RuntimeError("G1 body manifest must contain exactly 29 joints")
    if (
        len(G1_LEFT_HAND_ISAAC_TASK_JOINTS) != 7
        or len(G1_RIGHT_HAND_ISAAC_TASK_JOINTS) != 7
    ):
        raise RuntimeError("Each Dex3 hand manifest must contain exactly 7 joints")
    if len(G1_ALL_JOINTS) != 43 or len(set(G1_ALL_JOINTS)) != 43:
        raise RuntimeError("G1 manifest must contain 43 unique actuated joints")


validate_manifest()
