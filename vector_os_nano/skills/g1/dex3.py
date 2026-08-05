# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Dex3-1 hand pose skill for the left and right G1 hands."""
from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.g1._common import capability_failure

_SIDES = ("left", "right")
_PRESET_METHODS = {
    "open": "open",
    "power_grasp": "power_grasp",
    "pinch": "pinch",
}
_DEX3_DOF = 7
_DEFAULT_DURATION_S = 1.0
_MISSING = object()


def _nonnegative_finite(value: object, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite non-negative number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a finite non-negative number") from exc
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{label} must be a finite non-negative number")
    return result


def _joint_positions(value: object) -> list[float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError("positions must be a sequence of exactly 7 numbers")
    if len(value) != _DEX3_DOF:
        raise ValueError("positions must contain exactly 7 joint values")
    positions: list[float] = []
    for raw in value:
        if isinstance(raw, bool):
            raise ValueError("positions must contain exactly 7 finite numbers")
        try:
            joint = float(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "positions must contain exactly 7 finite numbers"
            ) from exc
        if not math.isfinite(joint):
            raise ValueError("positions must contain exactly 7 finite numbers")
        positions.append(joint)
    return positions


def _hand_capability_failure(hand: Any, side: str) -> SkillResult | None:
    """Reject an explicitly wrong side, DOF, or missing backend joint group."""
    advertised_side = getattr(hand, "side", _MISSING)
    if advertised_side is not _MISSING and advertised_side != side:
        return capability_failure(
            f"Hand registered as {side!r} advertises side {advertised_side!r}"
        )

    dof = getattr(hand, "dof", _MISSING)
    if dof is not _MISSING:
        try:
            dof = dof() if callable(dof) else dof
            if int(dof) != _DEX3_DOF:
                return capability_failure(
                    f"Dex3 hand requires 7 joints, but {side!r} hand has {dof}"
                )
        except Exception:
            return capability_failure(
                f"Could not verify the {side!r} hand joint count"
            )

    coordinator = getattr(hand, "coordinator", _MISSING)
    capabilities = (
        getattr(coordinator, "capabilities", _MISSING)
        if coordinator is not _MISSING
        else _MISSING
    )
    if capabilities is not _MISSING:
        group = f"{side}_hand"
        supports_group = getattr(capabilities, "supports_group", None)
        try:
            if callable(supports_group) and not bool(supports_group(group)):
                return capability_failure(
                    f"Selected backend does not support the {group!r} joint group"
                )
            groups = getattr(capabilities, "joint_groups", _MISSING)
            if (
                not callable(supports_group)
                and groups is not _MISSING
                and group not in groups
            ):
                return capability_failure(
                    f"Selected backend does not support the {group!r} joint group"
                )
        except Exception:
            return capability_failure(
                f"Could not verify backend support for the {group!r} joint group"
            )
    return None


@skill(
    aliases=[
        "dex3 pose",
        "hand pose",
        "open hand",
        "power grasp",
        "pinch",
        "手势",
        "张开手",
        "抓握",
        "捏取",
    ],
    direct=True,
)
class Dex3PoseSkill:
    """Apply one named Dex3 preset or one exact seven-joint pose."""

    name = "dex3_pose"
    description = (
        "Set the left or right G1 Dex3-1 hand to open, power_grasp, pinch, "
        "or an exact seven-joint position vector. Exactly one of preset and "
        "positions is required."
    )
    typical_duration_sec = 3.0
    parameters = {
        "side": {
            "type": "string",
            "required": True,
            "enum": list(_SIDES),
            "description": "Hand to command. Must be explicitly left or right.",
        },
        "preset": {
            "type": "string",
            "required": False,
            "enum": list(_PRESET_METHODS),
            "description": "Named pose; mutually exclusive with positions.",
        },
        "positions": {
            "type": "array",
            "required": False,
            "items": {"type": "number"},
            "minItems": _DEX3_DOF,
            "maxItems": _DEX3_DOF,
            "description": (
                "Exactly 7 positions in the selected hand's declared joint order; "
                "mutually exclusive with preset."
            ),
        },
        "duration": {
            "type": "number",
            "required": False,
            "default": _DEFAULT_DURATION_S,
            "description": "Non-negative interpolation duration in seconds.",
        },
    }
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects = {"hand_pose": "changed"}
    failure_modes = [
        "invalid_parameters",
        "no_hand",
        "capability_unavailable",
        "hand_command_failed",
    ]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        params = params or {}
        side = params.get("side")
        if side not in _SIDES:
            return SkillResult(
                success=False,
                error_message="side must be explicitly 'left' or 'right'",
                diagnosis_code="invalid_parameters",
            )

        has_preset = "preset" in params and params["preset"] is not None
        has_positions = "positions" in params and params["positions"] is not None
        if has_preset == has_positions:
            return SkillResult(
                success=False,
                error_message="Provide exactly one of preset or positions",
                diagnosis_code="invalid_parameters",
            )

        try:
            duration = _nonnegative_finite(
                params.get("duration", _DEFAULT_DURATION_S), "duration"
            )
            positions = (
                _joint_positions(params["positions"]) if has_positions else None
            )
        except ValueError as exc:
            return SkillResult(
                success=False,
                error_message=str(exc),
                diagnosis_code="invalid_parameters",
            )

        preset = params.get("preset") if has_preset else None
        if has_preset and preset not in _PRESET_METHODS:
            return SkillResult(
                success=False,
                error_message=f"Unsupported Dex3 preset: {preset!r}",
                diagnosis_code="invalid_parameters",
            )

        hand = context.get_hand(side)
        if hand is None:
            return SkillResult(
                success=False,
                error_message=f"No {side} Dex3 hand is registered",
                diagnosis_code="no_hand",
            )
        capability_error = _hand_capability_failure(hand, side)
        if capability_error is not None:
            return capability_error

        if has_preset:
            method_name = _PRESET_METHODS[preset]
            command = getattr(hand, method_name, None)
        else:
            method_name = "set_joint_positions"
            command = getattr(hand, method_name, None)
            if not callable(command):
                method_name = "move_joints"
                command = getattr(hand, method_name, None)
        if not callable(command):
            return capability_failure(
                f"The {side} hand does not provide {method_name}()"
            )

        try:
            if has_preset:
                ok = command(duration=duration)
            else:
                ok = command(positions, duration=duration)
        except NotImplementedError as exc:
            return capability_failure(f"Dex3 command is unavailable: {exc}")
        except ValueError as exc:
            # G1Hand validates profile joint limits before publishing a command.
            return SkillResult(
                success=False,
                error_message=f"Invalid Dex3 pose: {exc}",
                diagnosis_code="invalid_parameters",
            )
        except Exception as exc:
            return SkillResult(
                success=False,
                error_message=f"Dex3 command failed: {exc}",
                diagnosis_code="hand_command_failed",
            )
        if not ok:
            return SkillResult(
                success=False,
                error_message="Dex3 command was rejected",
                diagnosis_code="hand_command_failed",
            )

        result_data: dict[str, Any] = {"side": side, "duration": duration}
        if has_preset:
            result_data["preset"] = preset
        else:
            result_data["positions"] = positions
        return SkillResult(success=True, result_data=result_data)


__all__ = ["Dex3PoseSkill"]
