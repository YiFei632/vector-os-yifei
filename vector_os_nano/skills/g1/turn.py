# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""G1 yaw turns through the generic blocking base interface."""
from __future__ import annotations

import math

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.g1._common import capability_failure, require_base_method
from vector_os_nano.skills.g1.walk import _positive_finite

_DEFAULT_DIRECTION = "left"
_DEFAULT_ANGLE_DEG = 90.0
_YAW_SPEED_RAD_S = 0.5


@skill(
    aliases=["turn", "rotate", "转", "转弯", "转向", "左转", "右转"],
    direct=False,
)
class TurnSkill:
    """Rotate G1 left or right by a positive angle in degrees."""

    name = "turn"
    description = "Rotate G1 left or right by a positive angle in degrees."
    typical_duration_sec = 10.0
    parameters = {
        "direction": {
            "type": "string",
            "required": False,
            "default": _DEFAULT_DIRECTION,
            "enum": ["left", "right"],
            "description": "Turn direction in the body frame.",
        },
        "angle": {
            "type": "number",
            "required": False,
            "default": _DEFAULT_ANGLE_DEG,
            "description": "Positive turn angle in degrees.",
        },
    }
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects = {"heading": "changed", "is_moving": False}
    failure_modes = [
        "no_base",
        "invalid_parameters",
        "capability_unavailable",
        "turn_failed",
    ]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        params = params or {}
        direction = params.get("direction", _DEFAULT_DIRECTION)
        if direction not in {"left", "right"}:
            return SkillResult(
                success=False,
                error_message=f"Unsupported turn direction: {direction!r}",
                diagnosis_code="invalid_parameters",
            )
        try:
            angle_deg = _positive_finite(
                params.get("angle", _DEFAULT_ANGLE_DEG), "angle"
            )
        except ValueError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                diagnosis_code="invalid_parameters",
            )

        _base, walk, failure = require_base_method(
            context, "walk", locomotion=True
        )
        if failure is not None:
            return failure

        angle_rad = math.radians(angle_deg)
        yaw_speed = _YAW_SPEED_RAD_S if direction == "left" else -_YAW_SPEED_RAD_S
        duration = angle_rad / _YAW_SPEED_RAD_S
        try:
            ok = walk(0.0, 0.0, yaw_speed, duration)
        except NotImplementedError as exc:
            return capability_failure(f"G1 turning is unavailable: {exc}")
        except Exception as exc:
            return SkillResult(
                success=False,
                error_message=f"Turn command failed: {exc}",
                diagnosis_code="turn_failed",
            )
        if not ok:
            return SkillResult(
                success=False,
                error_message="Turn command did not complete",
                diagnosis_code="turn_failed",
            )
        return SkillResult(
            success=True,
            result_data={
                "direction": direction,
                "angle_deg": angle_deg,
                "angle_rad": angle_rad,
                "duration": duration,
            },
        )


G1TurnSkill = TurnSkill

__all__ = ["G1TurnSkill", "TurnSkill"]
