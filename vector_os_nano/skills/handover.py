# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""HandoverSkill — hand an object to the user.

Rotates the arm 90 degrees toward the user and releases the gripper.
Used when the user says "给我" (give it to me) instead of placing
on the table.

Algorithm:
  1. From home position (holding object), rotate shoulder_pan +90deg
  2. Open gripper to release
  3. Close gripper
  4. Return to home

No ROS2 imports.
"""
from __future__ import annotations

import logging
import time

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

_HOME_DURATION: float = 3.0
_DEFAULT_HOME_JOINTS: list[float] = [-0.014, -1.238, 0.562, 0.858, 0.311]


@skill(
    aliases=["give", "hand over", "给我", "递给我", "给", "拿给我", "交给我"],
    direct=False,
)
class HandoverSkill:
    """Hand a held object to the user by rotating and releasing.

    Rotates shoulder_pan ~90 degrees from home position, opens gripper
    to release the object, then returns home. Similar to pick's "drop"
    mode but intended as a deliberate handover to the user.
    """

    name: str = "handover"
    description: str = "Hand the held object to the user. Rotates arm toward user and releases. Use when user says 'give me' or '给我'."
    parameters: dict = {
        "arm": LIMB_PARAMETER,
        "direction": {
            "type": "string",
            "required": False,
            "default": "right",
            "enum": ["left", "right"],
            "description": "Which side the user is on: 'right' (default) or 'left'",
        },
    }
    # Typical REAL-TIME (viewer-synced) duration: rotate 3s + open gripper + home 3s +
    # overhead ≈ 8–10s real-time; 25s gives margin.  GoalExecutor floors the step
    # timeout at this value (R2-2) so a tight LLM-emitted timeout_sec is not a false
    # failure under a live viewer.
    typical_duration_sec: float = 25.0
    preconditions: list[str] = ["gripper_holding_any"]
    postconditions: list[str] = ["gripper_empty"]
    effects: dict = {"gripper_state": "open", "held_object": None}
    failure_modes: list[str] = [
        "no_arm", "no_gripper", "move_failed", "gripper_failed",
        "invalid_parameters", "invalid_profile", "capability_unavailable",
        "wrong_limb",
    ]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        try:
            arm, gripper, arm_name = select_limb(context, params)
        except ValueError as exc:
            return SkillResult(success=False, error_message=str(exc),
                               result_data={"diagnosis": "no_arm"})
        if arm is None:
            return SkillResult(success=False, error_message="No arm connected",
                               result_data={"diagnosis": "no_arm"})
        if gripper is None:
            return SkillResult(
                success=False,
                error_message=f"No matching gripper/hand for arm {arm_name!r}",
                result_data={"diagnosis": "no_gripper"},
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
            return SkillResult(success=False, error_message=str(exc),
                               result_data={"diagnosis": "invalid_profile"})

        direction = params.get("direction", "right")
        if direction not in {"left", "right"}:
            return SkillResult(
                success=False,
                error_message=f"Unsupported handover direction: {direction!r}",
                result_data={"diagnosis": "invalid_parameters"},
            )
        # Rotate shoulder_pan: +90deg for right, -90deg for left
        rotation = 1.57 if direction == "right" else -1.57

        # Step 1: Move to handover position (rotate from home)
        default_handover = list(home_joints)
        default_handover[0] = default_handover[0] + rotation
        try:
            handover_joints = resolve_joint_pose(
                context, "handover", default_handover,
                arm=arm, arm_name=arm_name,
                pose_key=f"joint_values_{direction}",
            )
        except ValueError as exc:
            return SkillResult(success=False, error_message=str(exc),
                               result_data={"diagnosis": "invalid_profile"})

        logger.info("[HANDOVER] Rotating %s (%.2f rad) to hand over...", direction, rotation)
        if not arm.move_joints(handover_joints, duration=_HOME_DURATION):
            return SkillResult(success=False, error_message="Move to handover position failed",
                               result_data={"diagnosis": "move_failed", "phase": "rotate"})

        # Step 2: Open gripper to release
        logger.info("[HANDOVER] Opening gripper...")
        if gripper.open() is False:
            return SkillResult(
                success=False,
                error_message="Gripper failed to release during handover",
                result_data={"diagnosis": "gripper_failed", "phase": "release"},
            )
        time.sleep(0.5)
        if gripper.close() is False:
            return SkillResult(
                success=False,
                error_message="Gripper failed to close after handover",
                result_data={"diagnosis": "gripper_failed", "phase": "close"},
            )

        # Step 3: Return home
        logger.info("[HANDOVER] Returning home...")
        if not arm.move_joints(home_joints, duration=_HOME_DURATION):
            return SkillResult(success=False, error_message="Return home failed",
                               result_data={"diagnosis": "move_failed", "phase": "home"})

        logger.info("[HANDOVER] Done!")
        return SkillResult(
            success=True,
            result_data={"diagnosis": "ok", "direction": direction, "arm": arm_name},
        )
