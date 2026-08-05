# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Shared capability checks for G1 skills.

The helpers deliberately use duck typing.  Importing this module must not pull
in a concrete G1 backend, ROS, or a robot-specific transport.
"""
from __future__ import annotations

from collections.abc import Callable
from typing import Any

from vector_os_nano.core.skill import SkillContext
from vector_os_nano.core.types import SkillResult

_MISSING = object()


def capability_failure(message: str) -> SkillResult:
    """Return the common result used for an unavailable hardware operation."""
    return SkillResult(
        success=False,
        error_message=message,
        diagnosis_code="capability_unavailable",
    )


def require_base_method(
    context: SkillContext,
    method_name: str,
    *,
    locomotion: bool = False,
) -> tuple[Any | None, Callable[..., Any] | None, SkillResult | None]:
    """Resolve a base method and reject an explicitly absent capability.

    ``BaseProtocol`` does not define a locomotion capability flag, while
    ``G1Base`` exposes it through its coordinator.  Both forms are accepted:
    a missing flag means that the callable protocol method is authoritative;
    an explicit ``False`` fails before the method is invoked.
    """
    base = context.base
    if base is None:
        return None, None, SkillResult(
            success=False,
            error_message="No base connected",
            diagnosis_code="no_base",
        )

    method = getattr(base, method_name, None)
    if not callable(method):
        return base, None, capability_failure(
            f"Connected base does not provide {method_name}()"
        )

    if locomotion and advertised_capability(base, "locomotion") is False:
        return base, None, capability_failure(
            "Connected base has no locomotion controller"
        )

    return base, method, None


def advertised_capability(base: Any, capability: str) -> bool | None:
    """Read an optional capability advertised by a duck-typed base.

    Supported layouts are ``supports_<name>`` on the base, ``capabilities`` on
    the base, and ``coordinator.capabilities`` as used by :class:`G1Base`.
    Any explicit false value wins, so contradictory adapters fail closed.
    """
    values: list[bool] = []

    found, value = _read_bool(base, f"supports_{capability}")
    if found:
        values.append(value)

    for owner in (base, _safe_getattr(base, "coordinator")):
        if owner is _MISSING:
            continue
        capabilities = _safe_getattr(owner, "capabilities")
        if capabilities is _MISSING:
            continue
        found, value = _read_bool(capabilities, capability)
        if found:
            values.append(value)

    if any(value is False for value in values):
        return False
    if values:
        return True
    return None


def _safe_getattr(owner: Any, name: str) -> Any:
    try:
        return getattr(owner, name, _MISSING)
    except Exception:  # A broken advertised property is not usable capability.
        return _MISSING


def _read_bool(owner: Any, name: str) -> tuple[bool, bool]:
    value = _safe_getattr(owner, name)
    if value is _MISSING:
        return False, False
    try:
        value = value() if callable(value) else value
        return True, bool(value)
    except Exception:
        # The adapter advertised the field but could not answer it.  Fail
        # closed rather than issuing a motion command under uncertainty.
        return True, False


__all__ = ["advertised_capability", "capability_failure", "require_base_method"]
