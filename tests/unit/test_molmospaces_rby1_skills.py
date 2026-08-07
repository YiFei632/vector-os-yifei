# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Unit tests for MolmoSpaces RBY1 skills."""
from __future__ import annotations

from vector_os_nano.core.skill import SkillContext
from vector_os_nano.skills import molmospaces_rby1
from vector_os_nano.skills.molmospaces_rby1 import RBY1NavigateToObjectSkill, RBY1PickObjectSkill


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


class _NavBridge:
    def __init__(self) -> None:
        self.execute_kwargs = None

    def execute(self, *args, **kwargs):
        self.execute_kwargs = kwargs
        return {"success": True, "terminal_reason": "task_done"}


def test_rby1_navigation_does_not_request_execute_reset(monkeypatch) -> None:
    bridge = _NavBridge()

    def fake_bridge_for_context(context):
        return bridge, {"endpoint": {"timeout_s": 1.0}, "context": {}}

    monkeypatch.setattr(molmospaces_rby1, "_bridge_for_context", fake_bridge_for_context)

    result = RBY1NavigateToObjectSkill().execute({"target": "table"}, SkillContext())

    assert result.success is True
    assert bridge.execute_kwargs is not None
    assert bridge.execute_kwargs["context"]["structured_action"] == "navigate_to_object"
    assert bridge.execute_kwargs["context"]["target_types"] == ["table"]
    assert bridge.execute_kwargs["context"]["allow_execute_reset"] is False
