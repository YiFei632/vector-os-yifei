# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Helpers for robot-specific poses and named limb selection.

Skills remain hardware-independent: a deployment supplies poses by arm name,
while legacy single-arm configurations continue using ``joint_values``.
"""
from __future__ import annotations

import math
from numbers import Integral
from typing import Any, Mapping, Sequence

from vector_os_nano.core.skill import SkillContext


class MotionCapabilityError(RuntimeError):
    """The selected limb exists but cannot execute the requested controller."""


class HeldObjectOwnershipError(RuntimeError):
    """A skill selected a different limb from the one holding the object."""


def held_object_binding(context: SkillContext) -> tuple[str | None, str | None]:
    """Return the single-object binding without assuming every WorldModel version.

    Older saved worlds have no ``held_by`` field and lightweight test doubles
    may not implement ``get_robot``.  Those cases remain compatible and return
    an unknown owner.
    """
    world_model = getattr(context, "world_model", None)
    get_robot = getattr(world_model, "get_robot", None)
    if not callable(get_robot):
        return None, None
    try:
        state = get_robot()
    except Exception:
        return None, None
    held_object = getattr(state, "held_object", None)
    if not isinstance(held_object, str) or not held_object:
        return None, None
    owner = getattr(state, "held_by", None)
    if owner not in {"left", "right"}:
        owner = None
    return held_object, owner


def require_limb_owns_held_object(
    context: SkillContext,
    limb_name: str | None,
) -> None:
    """Reject release skills that target the non-owning side."""
    held_object, owner = held_object_binding(context)
    if held_object is not None and owner is not None and limb_name != owner:
        raise HeldObjectOwnershipError(
            f"{owner!r} holds {held_object!r}; select that limb instead of "
            f"{limb_name!r}"
        )


def require_no_held_object(context: SkillContext) -> None:
    """Enforce the phase-one single-held-object model for direct skill calls."""
    held_object, owner = held_object_binding(context)
    if held_object is not None:
        suffix = f" with {owner!r}" if owner is not None else ""
        raise HeldObjectOwnershipError(
            f"already holding {held_object!r}{suffix}; release it before picking"
        )


def _device_side(device: Any) -> str | None:
    side = getattr(device, "side", None)
    if side is None:
        side = getattr(getattr(device, "hand", None), "side", None)
    return side if side in {"left", "right"} else None


def _validate_named_pair(name: str, arm: Any, gripper: Any) -> None:
    if name not in {"left", "right"}:
        return
    arm_side = _device_side(arm)
    gripper_side = _device_side(gripper) if gripper is not None else None
    if arm_side is not None and arm_side != name:
        raise ValueError(
            f"arm registry entry {name!r} contains a {arm_side!r}-side device"
        )
    if gripper_side is not None and gripper_side != name:
        raise ValueError(
            f"gripper registry entry {name!r} contains a {gripper_side!r}-side device"
        )
    if arm_side is not None and gripper_side is not None and arm_side != gripper_side:
        raise ValueError(
            f"selected arm side {arm_side!r} does not match gripper side {gripper_side!r}"
        )


def require_joint_control(device: Any, *, label: str) -> None:
    """Fail before motion when a device explicitly denies joint control."""
    try:
        supported = getattr(device, "supports_joint_control", True)
    except Exception as exc:  # capability properties may query a backend
        raise MotionCapabilityError(f"cannot query {label} joint capability: {exc}") from exc
    if supported is False:
        raise MotionCapabilityError(f"selected {label} has no joint controller")
    if not callable(getattr(device, "move_joints", None)) and not callable(
        getattr(device, "set_joint_positions", None)
    ):
        raise MotionCapabilityError(f"selected {label} exposes no joint command method")


def require_cartesian_control(arm: Any) -> None:
    """Fail before any hand/arm command when Cartesian IK is unavailable."""
    require_joint_control(arm, label="arm")
    try:
        supported = getattr(arm, "supports_cartesian", True)
    except Exception as exc:
        raise MotionCapabilityError(f"cannot query arm Cartesian capability: {exc}") from exc
    if supported is False or not callable(getattr(arm, "ik", None)):
        raise MotionCapabilityError("selected arm has no Cartesian IK controller")


def require_gripper_control(gripper: Any) -> None:
    """Validate the operational open/close adapter without commanding it."""
    try:
        supported = getattr(gripper, "supports_joint_control", True)
    except Exception as exc:
        raise MotionCapabilityError(
            f"cannot query gripper joint capability: {exc}"
        ) from exc
    if supported is False:
        raise MotionCapabilityError("selected gripper has no joint controller")
    if not callable(getattr(gripper, "open", None)) or not callable(
        getattr(gripper, "close", None)
    ):
        raise MotionCapabilityError("selected gripper exposes no open/close controller")


def select_limb(
    context: SkillContext,
    params: Mapping[str, Any] | None = None,
) -> tuple[Any, Any, str | None]:
    """Select an arm and its same-named gripper.

    ``arm`` is the canonical parameter and ``side`` is accepted as a friendly
    alias.  With no explicit selection, SkillContext's configured defaults are
    used exactly as before.
    """
    params = params or {}
    requested = params.get("arm", params.get("side"))
    name = str(requested) if requested is not None else context.default_arm_name
    if name is not None:
        arm = context.get_arm(name)
        if arm is None:
            available = ", ".join(context.arms) or "none"
            raise ValueError(f"unknown arm {name!r}; available arms: {available}")
        gripper = context.get_gripper(name)
        if (
            gripper is None
            and requested is None
            and len(context.arms) <= 1
            and len(context.grippers) <= 1
        ):
            gripper = context.gripper
        _validate_named_pair(name, arm, gripper)
        return arm, gripper, name
    return context.arm, context.gripper, None


def select_gripper(
    context: SkillContext,
    params: Mapping[str, Any] | None = None,
) -> tuple[Any, str | None]:
    params = params or {}
    requested = params.get("hand", params.get("arm", params.get("side")))
    name = str(requested) if requested is not None else context.default_gripper_name
    if name is not None:
        gripper = context.get_gripper(name)
        if gripper is None:
            available = ", ".join(context.grippers) or "none"
            raise ValueError(f"unknown gripper {name!r}; available grippers: {available}")
        side = _device_side(gripper)
        if name in {"left", "right"} and side is not None and side != name:
            raise ValueError(
                f"gripper registry entry {name!r} contains a {side!r}-side device"
            )
        return gripper, name
    return context.gripper, None


def resolve_joint_pose(
    context: SkillContext,
    skill_name: str,
    default: Sequence[float],
    *,
    arm: Any,
    arm_name: str | None = None,
    pose_key: str = "joint_values",
) -> list[float]:
    """Resolve and validate a pose for the selected arm.

    Supported configuration forms, in decreasing priority::

        skills.<skill>.poses_by_arm.<arm>.<pose_key>
        skills.<skill>.<pose_key>_by_arm.<arm>
        skills.<skill>.<pose_key>

    The last form preserves all existing single-arm configuration files.
    """
    cfg = context.config.get("skills", {}).get(skill_name, {})
    raw: Any = None
    if arm_name is not None:
        poses_by_arm = cfg.get("poses_by_arm", {})
        if isinstance(poses_by_arm, Mapping):
            arm_poses = poses_by_arm.get(arm_name, {})
            if isinstance(arm_poses, Mapping):
                raw = arm_poses.get(pose_key)
        by_arm = cfg.get(f"{pose_key}_by_arm", {})
        if raw is None and isinstance(by_arm, Mapping):
            raw = by_arm.get(arm_name)
    if raw is None:
        raw = cfg.get(pose_key, default)

    try:
        values = [float(value) for value in raw]
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"skills.{skill_name}.{pose_key} must be a numeric joint vector"
        ) from exc

    declared_dof = getattr(arm, "dof", None)
    # Several existing integrations use light-weight mocks or old arm drivers
    # without a concrete integer ``dof`` attribute.  In that legacy case the
    # configured vector remains the contract; real named G1 arms report 7.
    dof = int(declared_dof) if isinstance(declared_dof, Integral) else len(values)
    if dof <= 0:
        dof = len(values)
    if len(values) != dof:
        selected = f" for arm {arm_name!r}" if arm_name is not None else ""
        raise ValueError(
            f"skills.{skill_name}.{pose_key}{selected} has {len(values)} values; "
            f"{getattr(arm, 'name', type(arm).__name__)} requires {dof}"
        )
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"skills.{skill_name}.{pose_key} contains a non-finite value")
    return values


LIMB_PARAMETER: dict[str, Any] = {
    "type": "string",
    "required": False,
    "enum": ["left", "right"],
    "description": "Named arm/hand side; uses the robot profile default when omitted",
}


__all__ = [
    "HeldObjectOwnershipError",
    "LIMB_PARAMETER",
    "MotionCapabilityError",
    "held_object_binding",
    "require_cartesian_control",
    "require_gripper_control",
    "require_joint_control",
    "require_limb_owns_held_object",
    "require_no_held_object",
    "resolve_joint_pose",
    "select_gripper",
    "select_limb",
]
