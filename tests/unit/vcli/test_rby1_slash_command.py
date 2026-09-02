# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Unit tests for the /rby1 CLI shortcut."""
from __future__ import annotations

from types import SimpleNamespace

from vector_os_nano.vcli.cli import (
    _configure_rby1_agent_services,
    _configure_rby1_vgg_agent,
    _handle_rby1_agent_text,
    _handle_rby1_direct_text,
    _handle_rby1_slash_command,
    _init_agent,
    _parse_rby1_agent_text,
)
from vector_os_nano.vcli.tools.base import ToolResult


class _FakeTool:
    def __init__(self) -> None:
        self.calls: list[dict] = []

    def execute(self, params, context):
        self.calls.append({"params": dict(params), "context": context})
        return ToolResult(content="ok", metadata=dict(params))


class _FakeRegistry:
    def __init__(self, tool):
        self._tool = tool

    def get(self, name: str):
        if isinstance(self._tool, dict):
            return self._tool.get(name)
        if name == "molmospaces_rby1":
            return self._tool
        return None


def test_rby1_slash_execute_uses_default_endpoint_and_scene() -> None:
    tool = _FakeTool()
    registry = _FakeRegistry(tool)
    app_state = {
        "agent": None,
        "engine": None,
        "molmospaces_rby1_endpoint": {"host": "10.0.0.2", "port": 9999, "timeout_s": 300.0},
        "molmospaces_rby1_scene": "lab_room",
        "molmospaces_rby1_viewer": True,
        "molmospaces_rby1_viewer_camera": "free",
    }

    ok = _handle_rby1_slash_command(
        ["go", "to", "the", "fridge"],
        registry,
        session=None,
        app_state=app_state,
    )

    assert ok is True
    assert len(tool.calls) == 1
    params = tool.calls[0]["params"]
    assert params["action"] == "execute"
    assert params["instruction"] == "go to the fridge"
    assert params["host"] == "10.0.0.2"
    assert params["port"] == 9999
    assert params["timeout_s"] == 300.0
    assert params["scene_name"] == "lab_room"
    assert params["context"] == {
        "viewer": True, "viewer_camera": "free",
        "camera_system": "gopro_d455", "navigation_camera": "head_camera",
        "rectify_gopro": True, "gopro_rectified_vfov": 94.0,
        "global_map_robot_radius_m": 0.05,
    }


def test_rby1_slash_reset_can_override_scene() -> None:
    tool = _FakeTool()
    registry = _FakeRegistry(tool)
    app_state = {
        "agent": None,
        "engine": None,
        "molmospaces_rby1_endpoint": {"host": "127.0.0.1", "port": 8765},
        "molmospaces_rby1_scene": "default_scene",
    }

    ok = _handle_rby1_slash_command(
        ["reset", "kitchen_scene"],
        registry,
        session=None,
        app_state=app_state,
    )

    assert ok is True
    assert len(tool.calls) == 1
    params = tool.calls[0]["params"]
    assert params["action"] == "reset"
    assert params["scene_name"] == "kitchen_scene"
    assert params["host"] == "127.0.0.1"
    assert params["port"] == 8765


def test_rby1_direct_text_routes_plain_input_when_enabled() -> None:
    tool = _FakeTool()
    registry = _FakeRegistry(tool)
    app_state = {
        "agent": None,
        "engine": None,
        "molmospaces_rby1_direct_text": True,
        "molmospaces_rby1_endpoint": {"host": "127.0.0.1", "port": 8765},
    }

    ok = _handle_rby1_direct_text(
        "go to the table and pick up the mug",
        registry,
        session=None,
        app_state=app_state,
    )

    assert ok is True
    assert len(tool.calls) == 1
    params = tool.calls[0]["params"]
    assert params["action"] == "execute"
    assert params["instruction"] == "go to the table and pick up the mug"


def test_rby1_agent_text_parser_splits_navigation_and_pick() -> None:
    assert _parse_rby1_agent_text("go to the table and pick up the mug") == [
        ("onering_navigation", {"target": "table"}),
        ("rby1_pick_object", {"object": "mug"}),
    ]


def test_rby1_agent_text_parser_accepts_chinese_navigation_and_pick() -> None:
    assert _parse_rby1_agent_text("走到桌子并拿起杯子") == [
        ("onering_navigation", {"target": "table"}),
        ("rby1_pick_object", {"object": "mug"}),
    ]


def test_rby1_agent_text_routes_structured_skill_sequence_when_enabled() -> None:
    nav_tool = _FakeTool()
    pick_tool = _FakeTool()
    registry = _FakeRegistry(
        {
            "onering_navigation": nav_tool,
            "rby1_pick_object": pick_tool,
        }
    )
    app_state = {
        "agent": None,
        "engine": None,
        "molmospaces_rby1_agent_text": True,
    }

    ok = _handle_rby1_agent_text(
        "go to the table and pick up the mug",
        registry,
        session=None,
        app_state=app_state,
    )

    assert ok is True
    assert nav_tool.calls[0]["params"] == {"target": "table"}
    assert pick_tool.calls[0]["params"] == {"object": "mug"}


def test_rby1_vgg_agent_registers_rby1_and_generic_navigation_skills() -> None:
    args = SimpleNamespace(
        sim=False,
        sim_go2=False,
        sim_g1=False,
        molmospaces_rby1_agent_text=False,
        molmospaces_rby1_vgg=True,
        molmospaces_rby1_host="127.0.0.1",
        molmospaces_rby1_port=8765,
        molmospaces_rby1_timeout=300.0,
        molmospaces_rby1_scene=None,
        molmospaces_rby1_viewer=False,
        molmospaces_rby1_viewer_camera="free",
    )

    agent = _init_agent(args)
    _configure_rby1_agent_services(agent, args)
    _configure_rby1_vgg_agent(agent, args)

    assert getattr(agent, "_molmospaces_rby1_vgg") is True
    assert set(agent._skill_registry.list_skills()) == {
        "rby1_observe",
        "rby1_sync_scene",
        "rby1_detect_object",
        "onering_navigation",
        "astar_plan",
        "grounding_dino_detect",
        "rby1_pick_object",
        "rby1_place_object",
        "rby1_stop",
    }
    assert agent._perception is not None
