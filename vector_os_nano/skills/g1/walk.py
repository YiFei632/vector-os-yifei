# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""G1 translational walking through the generic blocking base interface."""
from __future__ import annotations

import math

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.g1._common import (
    advertised_capability,
    capability_failure,
    require_base_method,
)

_DEFAULT_DIRECTION = "forward"
_DEFAULT_DISTANCE_M = 1.0
_DEFAULT_SPEED_MPS = 0.25
_MAX_FORWARD_SPEED_MPS = 0.4
_MAX_LATERAL_SPEED_MPS = 0.2
_DIRECTION_VECTORS: dict[str, tuple[float, float]] = {
    "forward": (1.0, 0.0),
    "backward": (-1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
}


def _positive_finite(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a positive finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a positive finite number") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be a positive finite number")
    return result


@skill(
    aliases=["walk", "go", "move", "走", "走路", "前进", "后退", "横移"],
    direct=False,
)
class WalkSkill:
    """Walk G1 forward, backward, or sideways for a requested distance."""

    name = "walk"
    description = (
        "Walk G1 forward, backward, left, or right by a distance using its "
        "base locomotion controller."
    )
    typical_duration_sec = 30.0
    parameters = {
        "direction": {
            "type": "string",
            "required": False,
            "default": _DEFAULT_DIRECTION,
            "enum": list(_DIRECTION_VECTORS),
            "description": "Body-frame travel direction.",
        },
        "distance": {
            "type": "number",
            "required": False,
            "default": _DEFAULT_DISTANCE_M,
            "description": "Positive travel distance in metres.",
        },
        "speed": {
            "type": "number",
            "required": False,
            "default": _DEFAULT_SPEED_MPS,
            "description": "Positive requested speed in m/s; conservatively clamped.",
        },
    }
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects = {"position": "changed", "is_moving": False}
    failure_modes = [
        "no_base",
        "invalid_parameters",
        "capability_unavailable",
        "walk_failed",
    ]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        params = params or {}
        direction = params.get("direction", _DEFAULT_DIRECTION)
        if direction not in _DIRECTION_VECTORS:
            return SkillResult(
                success=False,
                error_message=f"Unsupported walk direction: {direction!r}",
                diagnosis_code="invalid_parameters",
            )
        try:
            distance = _positive_finite(
                params.get("distance", _DEFAULT_DISTANCE_M), "distance"
            )
            requested_speed = _positive_finite(
                params.get("speed", _DEFAULT_SPEED_MPS), "speed"
            )
        except ValueError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                diagnosis_code="invalid_parameters",
            )

        base, walk, failure = require_base_method(
            context, "walk", locomotion=True
        )
        if failure is not None:
            return failure

        vx_sign, vy_sign = _DIRECTION_VECTORS[direction]
        lateral = vy_sign != 0.0
        if lateral and advertised_capability(base, "holonomic") is False:
            return capability_failure(
                "Connected base does not support lateral walking"
            )

        limit = _MAX_LATERAL_SPEED_MPS if lateral else _MAX_FORWARD_SPEED_MPS
        speed = min(requested_speed, limit)
        vx = vx_sign * speed
        vy = vy_sign * speed
        duration = distance / speed

        try:
            ok = walk(vx, vy, 0.0, duration)
        except NotImplementedError as exc:
            return capability_failure(f"G1 walking is unavailable: {exc}")
        except Exception as exc:  # Hardware failures are returned, never leaked.
            return SkillResult(
                success=False,
                error_message=f"Walk command failed: {exc}",
                diagnosis_code="walk_failed",
            )
        if not ok:
            return SkillResult(
                success=False,
                error_message="Walk command did not complete",
                diagnosis_code="walk_failed",
            )

        position = None
        get_position = getattr(base, "get_position", None)
        if callable(get_position):
            try:
                raw_position = get_position()
                position = list(raw_position)
            except Exception:
                position = None

        return SkillResult(
            success=True,
            result_data={
                "direction": direction,
                "distance": distance,
                "speed": speed,
                "duration": duration,
                "position": position,
            },
        )


G1WalkSkill = WalkSkill

__all__ = ["G1WalkSkill", "WalkSkill"]
