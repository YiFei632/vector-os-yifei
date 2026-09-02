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


def test_scene_sync_skill_syncs_bridge_objects_into_context(monkeypatch) -> None:
    from vector_os_nano.skills.molmospaces_rby1 import RBY1SyncSceneSkill

    class Bridge:
        def execute(self, instruction, *, context, mode, timeout_s):
            assert context["structured_action"] == "list_scene_objects"
            return {"objects": [{"object_id": "fridge_1", "category": "refrigerator", "position": [1, 2, 0]}]}

    class Graph:
        def __init__(self): self.objects = []
        def add_object(self, obj): self.objects.append(obj)

    graph = Graph()
    monkeypatch.setattr(
        molmospaces_rby1,
        "_bridge_for_context",
        lambda _context: (Bridge(), {"endpoint": {"timeout_s": 1.0}, "context": {}}),
    )
    result = RBY1SyncSceneSkill().execute(
        {}, SkillContext(services={"spatial_memory": graph})
    )

    assert result.success is True
    assert result.result_data["synced_objects"] == 1
    assert graph.objects[0].category == "refrigerator"


def test_observe_skill_does_not_list_or_sync_scene_objects(monkeypatch) -> None:
    from vector_os_nano.skills.molmospaces_rby1 import RBY1ObserveSkill

    class Bridge:
        def observe(self): return {"loaded": True, "robot": "rby1"}
        def execute(self, *_args, **_kwargs):
            raise AssertionError("observe must not query list_scene_objects")

    monkeypatch.setattr(
        molmospaces_rby1, "_bridge_for_context",
        lambda _context: (Bridge(), {"endpoint": {"timeout_s": 1.0}}),
    )

    result = RBY1ObserveSkill().execute({}, SkillContext())

    assert result.success is True
    assert "objects" not in result.result_data
