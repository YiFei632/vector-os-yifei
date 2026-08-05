# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Gripper skills — open and close as proper @skill classes.

Replaces hard-coded gripper routing in Agent._try_direct().
"""
from __future__ import annotations

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.motion_profile import (
    HeldObjectOwnershipError,
    LIMB_PARAMETER,
    MotionCapabilityError,
    require_gripper_control,
    require_limb_owns_held_object,
    select_gripper,
)


@skill(
    aliases=[
        "open", "open grip", "open gripper", "open claw",
        "release", "let go",
        "张开", "松开", "打开",
    ],
    direct=True,
)
class GripperOpenSkill:
    """Open the gripper / release held object."""

    name: str = "gripper_open"
    description: str = "Open the gripper to release any held object"
    parameters: dict = {"hand": LIMB_PARAMETER}
    preconditions: list[str] = []
    postconditions: list[str] = ["gripper_empty"]
    effects: dict = {"gripper_state": "open", "held_object": None}
    failure_modes: list[str] = [
        "no_arm", "no_gripper", "gripper_failed", "capability_unavailable",
        "wrong_limb",
    ]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        try:
            gripper, name = select_gripper(context, params)
        except ValueError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "no_gripper"},
            )
        if gripper is None:
            return SkillResult(
                success=False,
                error_message="No gripper connected",
                result_data={"diagnosis": "no_gripper"},
            )
        try:
            require_limb_owns_held_object(context, name)
        except HeldObjectOwnershipError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "wrong_limb"},
            )
        try:
            require_gripper_control(gripper)
        except MotionCapabilityError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "capability_unavailable"},
            )
        if gripper.open() is False:
            return SkillResult(
                success=False,
                error_message="Gripper failed to open",
                result_data={"diagnosis": "gripper_failed"},
            )
        context.world_model.update_robot_state(
            gripper_state="open", held_object=None, held_by=None
        )
        return SkillResult(success=True, result_data={"diagnosis": "ok", "hand": name})


@skill(
    aliases=[
        "close", "close grip", "close gripper", "close claw",
        "grip", "clench",
        "夹紧", "合上", "关闭",
    ],
    direct=True,
)
class GripperCloseSkill:
    """Close the gripper."""

    name: str = "gripper_close"
    description: str = "Close the gripper"
    parameters: dict = {"hand": LIMB_PARAMETER}
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects: dict = {"gripper_state": "closed"}
    failure_modes: list[str] = [
        "no_arm", "no_gripper", "gripper_failed", "capability_unavailable",
    ]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        try:
            gripper, name = select_gripper(context, params)
        except ValueError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "no_gripper"},
            )
        if gripper is None:
            return SkillResult(
                success=False,
                error_message="No gripper connected",
                result_data={"diagnosis": "no_gripper"},
            )
        try:
            require_gripper_control(gripper)
        except MotionCapabilityError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "capability_unavailable"},
            )
        if gripper.close() is False:
            return SkillResult(
                success=False,
                error_message="Gripper failed to close",
                result_data={"diagnosis": "gripper_failed"},
            )
        context.world_model.update_robot_state(gripper_state="closed")
        return SkillResult(success=True, result_data={"diagnosis": "ok", "hand": name})
