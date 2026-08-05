# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Behavioural regression tests for VGG SkillContext construction."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from vector_os_nano.core.agent import Agent
from vector_os_nano.core.skill import SkillRegistry
from vector_os_nano.vcli.engine import VectorEngine
from vector_os_nano.vcli.intent_router import IntentRouter


class _Arm:
    def __init__(self, name: str):
        self.name = name

    def get_joint_positions(self):
        return [0.0] * 5


class _Base:
    name = "base"

    def get_position(self):
        return [0.0, 0.0, 0.0]

    def get_heading(self):
        return 0.0


def _engine_context(agent, skill_registry=None):
    """Run the real init_vgg wiring and invoke GoalExecutor's context factory."""
    registry = skill_registry or getattr(agent, "_skill_registry", None)
    engine = VectorEngine(backend=MagicMock(), intent_router=IntentRouter())
    engine.init_vgg(agent=agent, skill_registry=registry)
    assert engine._goal_executor is not None
    assert engine._goal_executor._build_context is not None
    return engine._goal_executor._build_context()


def test_real_agent_builder_is_the_single_context_source() -> None:
    left_arm, right_arm = _Arm("left"), _Arm("right")
    left_hand, right_hand = object(), object()
    left_gripper, right_gripper = object(), object()
    base, nav = _Base(), object()
    agent = Agent(
        config={},
        arms={"left": left_arm, "right": right_arm},
        hands={"left": left_hand, "right": right_hand},
        grippers={"left": left_gripper, "right": right_gripper},
        bases={"g1": base},
        services={"nav": nav},
        default_arm_name="right",
        default_base_name="g1",
    )
    agent._calibration = object()
    real_builder = agent.build_context
    agent.build_context = MagicMock(side_effect=real_builder)

    ctx = _engine_context(agent)

    agent.build_context.assert_called_once_with()
    assert ctx.arms == {"left": left_arm, "right": right_arm}
    assert ctx.hands == {"left": left_hand, "right": right_hand}
    assert ctx.grippers == {"left": left_gripper, "right": right_gripper}
    assert ctx.bases == {"g1": base}
    assert ctx.arm is right_arm
    assert ctx.hand is right_hand
    assert ctx.gripper is right_gripper
    assert ctx.base is base
    assert ctx.services["nav"] is nav
    assert ctx.services["skill_registry"] is agent._skill_registry


def test_legacy_agent_like_object_keeps_singleton_fallback() -> None:
    arm = _Arm("arm")
    gripper = SimpleNamespace(name="gripper")
    base = _Base()
    world_model = object()
    registry = SkillRegistry()
    agent = SimpleNamespace(
        _base=base,
        _arm=arm,
        _gripper=gripper,
        _perception=None,
        _spatial_memory=None,
        _vlm=None,
        _world_model=world_model,
        _config={"x": 1},
        _calibration=None,
        _skill_registry=registry,
    )

    ctx = _engine_context(agent, registry)

    assert ctx.arm is arm
    assert ctx.gripper is gripper
    assert ctx.base is base
    assert ctx.arms == {"default": arm}
    assert ctx.grippers == {"default": gripper}
    assert ctx.bases == {"default": base}
    assert ctx.world_model is world_model
    assert ctx.config == {"x": 1}


def test_agent_like_plural_fallback_preserves_names_hands_and_services() -> None:
    left, right = _Arm("left"), _Arm("right")
    left_hand, right_hand = object(), object()
    base, nav = _Base(), object()
    registry = SkillRegistry()
    agent = SimpleNamespace(
        _base=base,
        _arm=right,
        _gripper=None,
        _arms={"left": left, "right": right},
        _grippers={},
        _hands={"left": left_hand, "right": right_hand},
        _bases={"g1": base},
        _services={"nav": nav},
        _default_arm_name="right",
        _default_gripper_name=None,
        _default_hand_name="right",
        _default_base_name="g1",
        _perception=None,
        _spatial_memory=None,
        _vlm=None,
        _world_model=None,
        _config={},
        _calibration=None,
        _skill_registry=registry,
    )

    ctx = _engine_context(agent, registry)

    assert ctx.arms == {"left": left, "right": right}
    assert ctx.hands == {"left": left_hand, "right": right_hand}
    assert ctx.bases == {"g1": base}
    assert ctx.arm is right
    assert ctx.hand is right_hand
    assert ctx.base is base
    assert ctx.services["nav"] is nav
    assert ctx.services["skill_registry"] is registry


def test_agent_like_empty_hardware_context_is_graceful() -> None:
    registry = SkillRegistry()
    agent = SimpleNamespace(
        _base=None,
        _arm=None,
        _gripper=None,
        _perception=None,
        _spatial_memory=None,
        _vlm=None,
        _world_model=None,
        _config=None,
        _calibration=None,
        _skill_registry=registry,
    )

    ctx = _engine_context(agent, registry)

    assert ctx.arm is None
    assert ctx.gripper is None
    assert ctx.hand is None
    assert ctx.base is None
