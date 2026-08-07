# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Unit tests for MolmoSpaces RBY1 skills."""
from __future__ import annotations

from vector_os_nano.core.skill import SkillContext
from vector_os_nano.skills import molmospaces_rby1
from vector_os_nano.skills.molmospaces_rby1 import RBY1PickObjectSkill


class _NoPickBridge:
    def __init__(self) -> None:
        self.execute_called = False

    def connect(self):
        return {"supported_task_types": ["nav_to_obj"]}

    def execute(self, *args, **kwargs):
        self.execute_called = True
        raise AssertionError("pick skill should not execute when manipulation is unsupported")


def test_rby1_pick_fails_fast_when_bridge_does_not_advertise_manipulation(monkeypatch) -> None:
    bridge = _NoPickBridge()

    def fake_bridge_for_context(context):
        return bridge, {"endpoint": {"timeout_s": 1.0}, "context": {}}

    monkeypatch.setattr(molmospaces_rby1, "_bridge_for_context", fake_bridge_for_context)

    result = RBY1PickObjectSkill().execute({"object": "mug"}, SkillContext())

    assert result.success is False
    assert result.diagnosis_code == "manipulation_unsupported"
    assert bridge.execute_called is False
