# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""PlaceSkill — place a held object at a target position.

Full port of skill_node_v2._execute_place(). The target position is
specified via parameters (x, y, z in metres in the base frame).

Algorithm:
  1. Build target and above-target positions
  2. IK for above-target
  3. Move above-target
  4. IK for target (warm-started from above)
  5. Descend to target
  6. Open gripper
  7. Lift back above target

No ROS2 imports.
"""
from __future__ import annotations

import logging

import numpy as np

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.motion_profile import (
    HeldObjectOwnershipError,
    LIMB_PARAMETER,
    MotionCapabilityError,
    require_cartesian_control,
    require_gripper_control,
    require_limb_owns_held_object,
    resolve_joint_pose,
    select_limb,
)

logger = logging.getLogger(__name__)

# Named location map — from robot's perspective
# x+ = forward (away from base), y+ = left, y- = right
_LOCATION_MAP: dict[str, tuple[float, float]] = {
    "front":        (0.30, 0.00),
    "front_left":   (0.30, 0.12),
    "front_right":  (0.30, -0.12),
    "center":       (0.22, 0.00),
    "left":         (0.22, 0.12),
    "right":        (0.22, -0.12),
    "back":         (0.12, 0.00),
    "back_left":    (0.12, 0.12),
    "back_right":   (0.12, -0.12),
}

_DEFAULT_PLACE_Z: float = 0.04
_DEFAULT_PRE_GRASP_HEIGHT: float = 0.06
_APPROACH_DURATION: float = 3.0
_DESCEND_DURATION: float = 2.0
_LIFT_DURATION: float = 2.0
_HOME_DURATION: float = 3.0
_DEFAULT_HOME_JOINTS: list[float] = [-0.014, -1.238, 0.562, 0.858, 0.311]


@skill(
    aliases=["put", "put down", "放", "放下", "放到", "放置", "放在"],
    direct=False,
)
class PlaceSkill:
    """Place a held object at a named location or coordinates.

    Accepts either a named location (front, left, back_right, etc.)
    or explicit x, y, z coordinates in metres.

    Parameters:
        location (str, optional): Named position — front, front_left, front_right,
            center, left, right, back, back_left, back_right.
        x, y, z (float, optional): Explicit coordinates (override location).
    """

    name: str = "place"
    description: str = "Place held object at a location: front, left, right, center, back, front_left, front_right, back_left, back_right"
    # Typical REAL-TIME (viewer-synced) duration in seconds: approach 3s + descent 2s +
    # open gripper + lift 2s + home 3s + perception overhead ≈ 15–20s; 45s gives margin
    # for a retry or a slow IK solve. GoalExecutor floors the step timeout at this value
    # (R2-2) so a completed place is never falsely marked timeout under a live viewer.
    typical_duration_sec: float = 45.0
    # Success predicate this skill is verified against (single-source for the planner).
    # A place RELEASES the held object, so the gripper ends empty: not holding_object()
    # discriminates a real place (released -> False -> verify passes) from a failed one
    # (still holding). Preferred over placed_count() >= 1, which is trivially true in a
    # region-less robot world (all resting objects count) and so verifies nothing.
    verify_hint: str = "not holding_object()"
    failure_modes: list[str] = [
        "no_arm", "no_gripper", "ik_unreachable", "move_failed",
        "gripper_failed", "invalid_profile", "capability_unavailable",
        "wrong_limb",
    ]
    parameters: dict = {
        "arm": LIMB_PARAMETER,
        "location": {
            "type": "string",
            "required": False,
            "default": "front",
            "enum": list(_LOCATION_MAP.keys()),
            "description": "Named position: front, front_left, front_right, center, left, right, back, back_left, back_right",
            "source": "static",
        },
        "x": {
            "type": "float",
            "required": False,
            "description": "Target X in metres (overrides location)",
        },
        "y": {
            "type": "float",
            "required": False,
            "description": "Target Y in metres (overrides location)",
        },
        "z": {
            "type": "float",
            "required": False,
            "description": "Target Z in metres",
        },
    }
    preconditions: list[str] = ["gripper_holding_any"]
    postconditions: list[str] = ["gripper_empty"]
    effects: dict = {
        "gripper_state": "open",
        "held_object": None,
    }

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        """Execute place sequence.

        Falls back to context.config["skills"]["place"] defaults if params
        are missing; then falls back to the module-level defaults.

        Args:
            params: optional x, y, z keys.
            context: SkillContext providing arm and gripper access.

        Returns:
            SkillResult(success=True, result_data={"placed_at": [x, y, z]}) on success.
            SkillResult(success=False, error_message=...) on failure.
        """
        params = params or {}
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
            require_cartesian_control(arm)
            require_gripper_control(gripper)
        except MotionCapabilityError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "capability_unavailable"},
            )

        cfg_place = context.config.get("skills", {}).get("place", {})
        pre_grasp_h: float = (
            context.config.get("skills", {})
            .get("pick", {})
            .get("pre_grasp_height", _DEFAULT_PRE_GRASP_HEIGHT)
        )
        try:
            home_joints = resolve_joint_pose(
                context,
                "home",
                _DEFAULT_HOME_JOINTS,
                arm=arm,
                arm_name=arm_name,
            )
        except ValueError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                result_data={"diagnosis": "invalid_profile"},
            )

        # Resolve target coordinates from location name or explicit params
        if "x" in params and "y" in params:
            tx = float(params["x"])
            ty = float(params["y"])
        else:
            location = params.get("location", "front")
            loc_xy = _LOCATION_MAP.get(location, _LOCATION_MAP["front"])
            tx, ty = loc_xy
            logger.info("[PLACE] Location '%s' → (%.3f, %.3f)", location, tx, ty)
        tz = float(params.get("z", cfg_place.get("z", _DEFAULT_PLACE_Z)))

        logger.info("[PLACE] Target: (%.3f, %.3f, %.3f) m", tx, ty, tz)

        place_pos = np.array([tx, ty, tz], dtype=float)
        above_pos = place_pos.copy()
        above_pos[2] += pre_grasp_h

        current_joints = arm.get_joint_positions()

        # IK for above-place position
        q_above_result = arm.ik(
            (above_pos[0], above_pos[1], above_pos[2]),
            current_joints,
        )
        if q_above_result is None:
            return SkillResult(
                success=False,
                error_message="IK failed for above-place position",
                result_data={
                    "diagnosis": "ik_unreachable",
                    "target_cm": [round(tx * 100, 1), round(ty * 100, 1), round(tz * 100, 1)],
                    "above_target_cm": [
                        round(above_pos[0] * 100, 1),
                        round(above_pos[1] * 100, 1),
                        round(above_pos[2] * 100, 1),
                    ],
                    "hint": "IK solver could not reach above-place position.",
                },
            )
        q_above = list(q_above_result)

        # Move above target
        logger.info("[PLACE] Moving above target ...")
        if not arm.move_joints(q_above, duration=_APPROACH_DURATION):
            return SkillResult(
                success=False,
                error_message="Move to above-place failed",
                result_data={"diagnosis": "move_failed", "phase": "approach"},
            )

        # IK for place position (warm-started from above)
        q_place_result = arm.ik(
            (place_pos[0], place_pos[1], place_pos[2]),
            q_above,
        )
        if q_place_result is None:
            return SkillResult(
                success=False,
                error_message="IK failed for place position",
                result_data={
                    "diagnosis": "ik_unreachable",
                    "target_cm": [round(tx * 100, 1), round(ty * 100, 1), round(tz * 100, 1)],
                    "hint": "IK solver could not reach place position.",
                },
            )
        q_place = list(q_place_result)

        # Descend to place position
        logger.info("[PLACE] Descending ...")
        if not arm.move_joints(q_place, duration=_DESCEND_DURATION):
            return SkillResult(
                success=False,
                error_message="Place descent failed",
                result_data={"diagnosis": "move_failed", "phase": "descend"},
            )

        # Open gripper to release object
        logger.info("[PLACE] Opening gripper ...")
        if gripper.open() is False:
            return SkillResult(
                success=False,
                error_message="Gripper failed to release object",
                result_data={"diagnosis": "gripper_failed", "phase": "release"},
            )

        # Lift back to above position
        logger.info("[PLACE] Lifting ...")
        if not arm.move_joints(q_above, duration=_LIFT_DURATION):
            return SkillResult(
                success=False,
                error_message="Place lift failed",
                result_data={"diagnosis": "move_failed", "phase": "lift"},
            )

        # Close gripper and return home
        if gripper.close() is False:
            return SkillResult(
                success=False,
                error_message="Gripper failed to close after place",
                result_data={"diagnosis": "gripper_failed", "phase": "close"},
            )

        logger.info("[PLACE] Returning home ...")
        if arm.move_joints(home_joints, duration=_HOME_DURATION) is False:
            return SkillResult(
                success=False,
                error_message="Return home after place failed",
                result_data={"diagnosis": "move_failed", "phase": "home"},
            )

        logger.info("[PLACE] Place complete!")
        return SkillResult(
            success=True,
            result_data={
                "placed_at": [round(tx, 4), round(ty, 4), round(tz, 4)],
                "arm": arm_name,
                "diagnosis": "ok",
            },
        )
