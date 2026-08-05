# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""G1 standing skill."""
from __future__ import annotations

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.g1._common import capability_failure, require_base_method


@skill(aliases=["stand", "站", "站起来", "起立"], direct=True)
class StandSkill:
    """Request G1's backend-defined stable standing mode."""

    name = "stand"
    description = "Command G1 to enter its standing mode."
    parameters: dict = {}
    preconditions: list[str] = []
    # ``stand()`` reports whether the backend accepted the request.  The
    # generic WorldModel has no measured stance predicate yet.
    postconditions: list[str] = []
    effects = {"base_stance": "stand"}
    failure_modes = ["no_base", "capability_unavailable", "stand_failed"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        _base, stand, failure = require_base_method(context, "stand")
        if failure is not None:
            return failure
        try:
            ok = stand()
        except NotImplementedError as exc:
            return capability_failure(f"G1 standing is unavailable: {exc}")
        except Exception as exc:
            return SkillResult(
                success=False,
                error_message=f"Stand command failed: {exc}",
                diagnosis_code="stand_failed",
            )
        if not ok:
            return SkillResult(
                success=False,
                error_message="Stand command was rejected",
                diagnosis_code="stand_failed",
            )
        return SkillResult(success=True, result_data={"stance": "stand"})


G1StandSkill = StandSkill

__all__ = ["G1StandSkill", "StandSkill"]
