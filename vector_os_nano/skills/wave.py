# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""WaveSkill — wave the arm to greet the user.

Raises the arm to an upright position, oscillates the base joint
left and right several times, then returns home.
"""
from __future__ import annotations

import logging
import time

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.motion_profile import (
    LIMB_PARAMETER,
    MotionCapabilityError,
    require_gripper_control,
    require_joint_control,
    resolve_joint_pose,
    select_limb,
)

logger = logging.getLogger(__name__)

# Raised position: shoulder up, elbow extended
_RAISED: list[float] = [0.0, -0.6, 0.3, 0.3, 0.0]

# Wave positions: rotate base left/right from raised pose
_WAVE_LEFT: list[float] = [-0.4, -0.6, 0.3, 0.3, 0.0]
_WAVE_RIGHT: list[float] = [0.4, -0.6, 0.3, 0.3, 0.0]

_WAVE_CYCLES: int = 3
_RAISE_DURATION: float = 2.0
_WAVE_DURATION: float = 0.8
_PAUSE: float = 0.15


@skill(
    aliases=["wave", "hello", "hi", "greet", "招手", "打招呼", "挥手", "你好"],
    direct=True,
)
class WaveSkill:
    """Wave the robot arm to greet the user."""

    name: str = "wave"
    description: str = "Wave the arm to greet the user"
    # Typical REAL-TIME (viewer-synced) duration: raise 2s + 3 wave cycles (0.8s each,
    # ×2 directions) + home 2s + pauses ≈ 12–13s; 15s gives margin.  GoalExecutor
    # floors the step timeout at this value (R2-2).
    typical_duration_sec: float = 15.0
    # No meaningful state predicate — the always-safe truthy literal.
    verify_hint: str = "True"
    parameters: dict = {"arm": LIMB_PARAMETER}
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects: dict = {"is_moving": False}
    failure_modes: list[str] = [
        "no_arm", "move_failed", "gripper_failed", "invalid_profile",
        "capability_unavailable",
    ]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        try:
            arm, gripper, arm_name = select_limb(context, params)
        except ValueError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "no_arm"},
            )
        if arm is None:
            return SkillResult(
                success=False,
                error_message="No arm connected",
                result_data={"diagnosis": "no_arm"},
            )

        try:
            require_joint_control(arm, label="arm")
            if gripper is not None:
                require_gripper_control(gripper)
        except MotionCapabilityError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "capability_unavailable"},
            )

        try:
            raised = resolve_joint_pose(
                context, "wave", _RAISED, arm=arm, arm_name=arm_name,
                pose_key="raised",
            )
            wave_left = resolve_joint_pose(
                context, "wave", _WAVE_LEFT, arm=arm, arm_name=arm_name,
                pose_key="wave_left",
            )
            wave_right = resolve_joint_pose(
                context, "wave", _WAVE_RIGHT, arm=arm, arm_name=arm_name,
                pose_key="wave_right",
            )
            home_joints = resolve_joint_pose(
                context, "home",
                [-0.014, -1.238, 0.562, 0.858, 0.311],
                arm=arm, arm_name=arm_name,
            )
        except ValueError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "invalid_profile"},
            )

        # 1. Raise arm
        logger.info("[WAVE] Raising arm")
        if not arm.move_joints(raised, duration=_RAISE_DURATION):
            return SkillResult(
                success=False,
                error_message="Failed to raise arm",
                result_data={"diagnosis": "move_failed"},
            )

        # 2. Open gripper (open hand)
        if gripper is not None and gripper.open() is False:
            return SkillResult(
                success=False,
                error_message="Failed to open hand for wave",
                result_data={"diagnosis": "gripper_failed", "phase": "open"},
            )

        time.sleep(_PAUSE)

        # 3. Wave left-right
        for i in range(_WAVE_CYCLES):
            logger.info("[WAVE] Cycle %d/%d", i + 1, _WAVE_CYCLES)
            if not arm.move_joints(wave_left, duration=_WAVE_DURATION):
                return SkillResult(
                    success=False,
                    error_message="Wave-left motion failed",
                    result_data={
                        "diagnosis": "move_failed", "phase": "wave_left", "cycle": i + 1,
                    },
                )
            time.sleep(_PAUSE)
            if not arm.move_joints(wave_right, duration=_WAVE_DURATION):
                return SkillResult(
                    success=False,
                    error_message="Wave-right motion failed",
                    result_data={
                        "diagnosis": "move_failed", "phase": "wave_right", "cycle": i + 1,
                    },
                )
            time.sleep(_PAUSE)

        # 4. Return to center, then home
        if not arm.move_joints(raised, duration=_WAVE_DURATION):
            return SkillResult(
                success=False,
                error_message="Failed to return wave to center",
                result_data={"diagnosis": "move_failed", "phase": "center"},
            )
        if not arm.move_joints(home_joints, duration=_RAISE_DURATION):
            return SkillResult(
                success=False,
                error_message="Failed to return home after wave",
                result_data={"diagnosis": "move_failed", "phase": "home"},
            )

        logger.info("[WAVE] Done")
        return SkillResult(
            success=True,
            result_data={"diagnosis": "ok", "arm": arm_name},
        )
