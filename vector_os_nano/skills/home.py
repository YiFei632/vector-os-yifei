# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""HomeSkill — move arm to home position and open gripper.

Ported from skill_node_v2._execute_home(). No ROS2 imports.
"""
from __future__ import annotations

import logging

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.motion_profile import (
    HeldObjectOwnershipError,
    LIMB_PARAMETER,
    MotionCapabilityError,
    require_gripper_control,
    require_joint_control,
    require_limb_owns_held_object,
    resolve_joint_pose,
    select_limb,
)

logger = logging.getLogger(__name__)

_DEFAULT_HOME_JOINTS: list[float] = [-0.014, -1.238, 0.562, 0.858, 0.311]
_HOME_DURATION: float = 3.0


@skill(
    aliases=["go home", "reset", "回家", "归位", "复位", "回到初始位置"],
    direct=True,
)
class HomeSkill:
    """Move arm to home position and open gripper.

    Always executable — no preconditions required. After execution the
    gripper is open and the arm is in the home configuration.
    """

    name: str = "home"
    description: str = "Move arm to home position and open gripper"
    # Typical REAL-TIME (viewer-synced) duration: 3s arm move + gripper + overhead.
    # GoalExecutor floors the step timeout at this value (R2-2) so a fast-emitted plan
    # (e.g. timeout_sec=5) does not falsely mark home as timed-out under a live viewer.
    typical_duration_sec: float = 12.0
    # Success predicate this skill is verified against (single-source for the planner).
    verify_hint: str = "arm_at_home()"
    parameters: dict = {"arm": LIMB_PARAMETER}
    preconditions: list[str] = []
    postconditions: list[str] = ["gripper_empty"]
    effects: dict = {
        "gripper_state": "open",
        "held_object": None,
        "is_moving": False,
    }
    failure_modes: list[str] = [
        "no_arm", "move_failed", "gripper_failed", "invalid_profile",
        "capability_unavailable", "wrong_limb",
    ]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        """Move to home joint configuration, then open gripper.

        Home joint values are read from context.config["skills"]["home"]["joint_values"]
        if present; otherwise the hard-coded default is used.

        Args:
            params: ignored (HomeSkill takes no parameters).
            context: SkillContext providing arm and gripper access.

        Returns:
            SkillResult(success=True) when arm reaches home and gripper opens.
            SkillResult(success=False) if the arm move fails.
        """
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
            require_limb_owns_held_object(context, arm_name)
        except HeldObjectOwnershipError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "wrong_limb"},
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
            home_joints = resolve_joint_pose(
                context, "home", _DEFAULT_HOME_JOINTS,
                arm=arm, arm_name=arm_name,
            )
        except ValueError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "invalid_profile"},
            )

        logger.info("[HOME] Moving %s to home pose: %s", arm_name or "arm", home_joints)
        success = arm.move_joints(home_joints, duration=_HOME_DURATION)

        if not success:
            logger.error("[HOME] Arm move failed")
            return SkillResult(
                success=False,
                error_message="Arm move to home failed",
                result_data={"diagnosis": "move_failed"},
            )

        if gripper is not None:
            logger.info("[HOME] Opening gripper")
            if gripper.open() is False:
                return SkillResult(
                    success=False,
                    error_message="Gripper failed to open at home",
                    result_data={"diagnosis": "gripper_failed"},
                )

        logger.info("[HOME] Done")
        return SkillResult(
            success=True,
            result_data={
                "joint_values": list(home_joints),
                "arm": arm_name,
                "diagnosis": "ok",
            },
        )
