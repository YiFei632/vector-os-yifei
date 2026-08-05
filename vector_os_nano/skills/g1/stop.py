# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""G1-local stop skill with no filesystem or middleware side effects."""
from __future__ import annotations

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.g1._common import capability_failure, require_base_method


@skill(
    aliases=["stop", "halt", "freeze", "停", "停止", "别动"],
    direct=True,
)
class StopSkill:
    """Stop G1 through ``BaseProtocol.stop`` only."""

    name = "stop"
    description = "Immediately request that G1 stop moving."
    parameters: dict = {}
    preconditions: list[str] = []
    # The current HAL acknowledges command delivery but does not expose a
    # measured stationary predicate.  Do not claim a postcondition that the
    # WorldModel cannot verify.
    postconditions: list[str] = []
    effects = {"is_moving": False}
    failure_modes = ["no_base", "capability_unavailable", "stop_failed"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        _base, stop, failure = require_base_method(context, "stop")
        if failure is not None:
            return failure
        try:
            outcome = stop()
        except NotImplementedError as exc:
            return capability_failure(f"G1 stop is unavailable: {exc}")
        except Exception as exc:
            return SkillResult(
                success=False,
                error_message=f"Stop command failed: {exc}",
                diagnosis_code="stop_failed",
            )
        # BaseProtocol.stop returns None.  An explicit False is accepted as an
        # adapter-specific rejection; every other normal return means issued.
        if outcome is False:
            return SkillResult(
                success=False,
                error_message="Stop command was rejected",
                diagnosis_code="stop_failed",
            )
        return SkillResult(success=True, result_data={"stopped": True})


G1StopSkill = StopSkill


@skill(
    aliases=["emergency stop", "e-stop", "estop", "急停", "紧急停止"],
    direct=True,
)
class EmergencyStopSkill:
    """Latch commands and request the strongest stop a G1 backend exposes."""

    name = "emergency_stop"
    description = (
        "Latch G1 commands and request backend zero/hold or damping; manual "
        "recovery is required."
    )
    parameters: dict = {}
    preconditions: list[str] = []
    # Emergency damping is latched by the transport.  There is no independent
    # motion-state sensor in the generic WorldModel with which to verify it.
    postconditions: list[str] = []
    effects = {"is_moving": False, "emergency_latched": True}
    failure_modes = ["no_base", "capability_unavailable", "stop_failed"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        _base, emergency_stop, failure = require_base_method(
            context, "emergency_stop"
        )
        if failure is not None:
            return failure
        try:
            outcome = emergency_stop()
        except NotImplementedError as exc:
            return capability_failure(f"G1 emergency stop is unavailable: {exc}")
        except Exception as exc:
            return SkillResult(
                success=False,
                error_message=f"Emergency stop failed: {exc}",
                diagnosis_code="stop_failed",
            )
        if outcome is False:
            return SkillResult(
                success=False,
                error_message="Emergency stop was rejected",
                diagnosis_code="stop_failed",
            )
        return SkillResult(
            success=True,
            result_data={
                "stopped": True,
                "emergency_latched": True,
                "damping_supported": bool(
                    getattr(_base, "supports_emergency_damping", False)
                ),
            },
        )


G1EmergencyStopSkill = EmergencyStopSkill

__all__ = [
    "EmergencyStopSkill",
    "G1EmergencyStopSkill",
    "G1StopSkill",
    "StopSkill",
]
