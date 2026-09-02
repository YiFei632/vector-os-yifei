# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Unit tests for MolmoSpaces RBY1 perception and scene sync."""
from __future__ import annotations

from vector_os_nano.core.skill import SkillContext
from vector_os_nano.core.world_model import WorldModel
from vector_os_nano.integrations.molmospaces.perception import (
    MolmoSpacesRBY1Perception,
    sync_scene_to_context,
)


class _Bridge:
    def execute(self, instruction, *, context=None, mode="auto", timeout_s=None):
        del instruction, mode, timeout_s
        action = (context or {}).get("structured_action")
        if action == "observe_rgb":
            return {"success": False, "error": "rgb unavailable"}
        if action == "list_scene_objects":
            return {
                "objects": [
                    {
                        "object_id": "mug_1",
                        "name": "mug_1",
                        "label": "mug",
                        "category": "mug",
                        "natural_name": "mug",
                        "aliases": ["cup", "mug"],
                        "position": [1.0, 2.0, 0.8],
                        "confidence": 1.0,
                    }
                ]
            }
        return {}


def test_rby1_perception_does_not_fallback_to_scene_ground_truth() -> None:
    perception = MolmoSpacesRBY1Perception(bridge=_Bridge())

    detections = perception.detect("cup")
    assert detections == []


def test_sync_scene_to_context_updates_world_model() -> None:
    ctx = SkillContext(world_model=WorldModel())

    result = sync_scene_to_context(
        ctx,
        [
            {
                "object_id": "mug_1",
                "label": "mug",
                "category": "mug",
                "position": [1.0, 2.0, 0.8],
            }
        ],
    )

    assert result == {"synced_objects": 1}
    objs = ctx.world_model.get_objects()
    assert len(objs) == 1
    assert objs[0].object_id == "mug_1"
    assert objs[0].label == "mug"
