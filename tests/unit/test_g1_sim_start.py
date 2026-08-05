# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

from __future__ import annotations

from unittest.mock import MagicMock, patch

from vector_os_nano.vcli.tools.sim_tool import SimStartTool


def _context() -> MagicMock:
    context = MagicMock()
    context.app_state = {"agent": None, "registry": None, "engine": None}
    context.cwd = "/tmp"
    return context


def _agent() -> MagicMock:
    agent = MagicMock()
    agent._arm = MagicMock()
    agent._base = MagicMock()
    agent._spatial_memory = None
    agent._skill_registry.list_skills.return_value = []
    return agent


def test_schema_exposes_exact_g1_startup_contract() -> None:
    properties = SimStartTool.input_schema["properties"]
    assert "g1" in properties["sim_type"]["enum"]
    assert properties["profile_path"]["type"] == "string"
    assert properties["controller_mode"]["enum"] == ["whole_body"]


def test_execute_routes_g1_mujoco_with_profile_and_gui() -> None:
    tool = SimStartTool()
    context = _context()
    with patch.object(SimStartTool, "_start_g1", return_value=_agent()) as start:
        result = tool.execute(
            {
                "sim_type": "g1",
                "backend": "mujoco",
                "gui": False,
                "profile_path": "/tmp/g1.yaml",
            },
            context,
        )

    assert not result.is_error
    start.assert_called_once_with(
        backend="mujoco", gui=False, profile_path="/tmp/g1.yaml"
    )


def test_execute_routes_g1_isaac() -> None:
    tool = SimStartTool()
    context = _context()
    with patch.object(SimStartTool, "_start_g1", return_value=_agent()) as start:
        result = tool.execute(
            {"sim_type": "g1", "backend": "isaac", "gui": True}, context
        )

    assert not result.is_error
    start.assert_called_once_with(backend="isaac", gui=True, profile_path=None)


def test_g1_gazebo_and_unknown_controller_fail_before_start() -> None:
    tool = SimStartTool()
    context = _context()
    with patch.object(SimStartTool, "_start_g1") as start:
        gazebo = tool.execute(
            {"sim_type": "g1", "backend": "gazebo"}, context
        )
        controller = tool.execute(
            {
                "sim_type": "g1",
                "backend": "mujoco",
                "controller_mode": "joint_only",
            },
            context,
        )

    assert gazebo.is_error and controller.is_error
    start.assert_not_called()
