# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Focused tests for Agent's named hardware and context compatibility seam."""
from __future__ import annotations

import pytest

from vector_os_nano.core.agent import Agent
from vector_os_nano.core.types import ExecutionResult


class _Device:
    def __init__(self, name: str, *, fail_connect: bool = False, fail_stop: bool = False):
        self.name = name
        self.fail_connect = fail_connect
        self.fail_stop = fail_stop
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.stop_calls = 0

    def connect(self) -> None:
        self.connect_calls += 1
        if self.fail_connect:
            raise ConnectionError(self.name)

    def disconnect(self) -> None:
        self.disconnect_calls += 1

    def stop(self) -> None:
        self.stop_calls += 1
        if self.fail_stop:
            raise RuntimeError(self.name)

    # State methods let one lightweight fake stand in for either an arm or base.
    def get_joint_positions(self):
        return [0.0] * 5

    def get_position(self):
        return [0.0, 0.0, 0.0]

    def get_heading(self):
        return 0.0


class _DuckTypedIKArm(_Device):
    def __init__(self):
        super().__init__("duck_ik_arm")
        self.injected_solver = None

    def set_ik_solver(self, solver) -> None:
        self.injected_solver = solver


class _PlanCaptureExecutor:
    def __init__(self) -> None:
        self.plan = None

    def execute(self, plan, registry, context, **kwargs):
        self.plan = plan
        return ExecutionResult(success=True, status="completed")


def _agent(**kwargs) -> Agent:
    config = kwargs.pop("config", {})
    agent = Agent(config=config, **kwargs)
    # Context tests exercise injection, not calibration file loading.
    agent._calibration = object()
    return agent


def test_legacy_singular_hardware_becomes_default_registries() -> None:
    arm, gripper, base = _Device("arm"), _Device("gripper"), _Device("base")
    agent = _agent(arm=arm, gripper=gripper, base=base)

    ctx = agent.build_context()

    assert agent._arm is arm and ctx.arm is arm
    assert agent._gripper is gripper and ctx.gripper is gripper
    assert agent._base is base and ctx.base is base
    assert ctx.arms == {"default": arm}
    assert ctx.grippers == {"default": gripper}
    assert ctx.bases == {"default": base}
    assert ctx.default_arm_name == "default"
    assert ctx.default_gripper_name == "default"
    assert ctx.default_base_name == "default"


def test_named_registries_defaults_services_and_hands_reach_context() -> None:
    left_arm, right_arm = _Device("left_arm"), _Device("right_arm")
    left_hand, right_hand = _Device("left_hand"), _Device("right_hand")
    left_gripper, right_gripper = _Device("left_gripper"), _Device("right_gripper")
    base = _Device("base")
    nav = object()
    agent = _agent(
        arms={"left": left_arm, "right": right_arm},
        hands={"left": left_hand, "right": right_hand},
        grippers={"left": left_gripper, "right": right_gripper},
        bases={"g1": base},
        services={"nav": nav},
        default_arm_name="right",
        default_base_name="g1",
    )

    ctx = agent.build_context()

    assert ctx.arms == {"left": left_arm, "right": right_arm}
    assert ctx.hands == {"left": left_hand, "right": right_hand}
    assert ctx.grippers == {"left": left_gripper, "right": right_gripper}
    assert ctx.bases == {"g1": base}
    assert ctx.arm is right_arm
    # Matching side defaults are inferred when only default_arm_name is explicit.
    assert ctx.hand is right_hand
    assert ctx.gripper is right_gripper
    assert ctx.base is base
    assert ctx.services["nav"] is nav
    assert ctx.services["skill_registry"] is agent._skill_registry


def test_agent_copies_input_and_each_built_context_registry() -> None:
    arm = _Device("left")
    arms = {"left": arm}
    services = {"nav": object()}
    agent = _agent(arms=arms, services=services)

    arms["external"] = _Device("external")
    services["external"] = object()
    first = agent.build_context()
    first.arms["context_only"] = _Device("context_only")
    first.services["context_only"] = object()
    second = agent.build_context()

    assert set(agent._arms) == {"left"}
    assert set(agent._services) == {"nav"}
    assert set(second.arms) == {"left"}
    assert "context_only" not in second.services


def test_legacy_singular_alias_replacement_remains_live() -> None:
    original_arm, replacement_arm = _Device("original"), _Device("replacement")
    original_base, replacement_base = _Device("base1"), _Device("base2")
    agent = _agent(arm=original_arm, base=original_base)
    agent._arm = replacement_arm
    agent._base = replacement_base

    ctx = agent.build_context()

    assert ctx.arm is replacement_arm
    assert ctx.base is replacement_base
    assert ctx.arms == {"default": replacement_arm}
    assert ctx.bases == {"default": replacement_base}


def test_live_agent_services_override_reserved_injected_names() -> None:
    stale_registry = object()
    stale_vlm = object()
    live_vlm = object()
    nav = object()
    agent = _agent(
        services={
            "nav": nav,
            "skill_registry": stale_registry,
            "vlm": stale_vlm,
        }
    )
    agent._vlm = live_vlm

    ctx = agent.build_context()

    assert ctx.services["nav"] is nav
    assert ctx.services["vlm"] is live_vlm
    assert ctx.services["skill_registry"] is agent._skill_registry


def test_private_build_context_alias_remains_compatible() -> None:
    arm = _Device("arm")
    agent = _agent(arm=arm)

    assert agent._build_context().arm is arm


def test_duck_typed_non_so101_arm_never_gets_so101_ik_injected() -> None:
    arm = _DuckTypedIKArm()
    agent = _agent(arm=arm)

    assert agent.build_context().arm is arm
    assert agent._ik_solver is None
    assert arm.injected_solver is None


def test_singular_and_named_registry_must_refer_to_same_device() -> None:
    left, other = _Device("left"), _Device("other")

    with pytest.raises(ValueError, match="Conflicting arm"):
        _agent(arm=other, arms={"left": left})

    agent = _agent(arm=left, arms={"left": left})
    assert agent._default_arm_name == "left"
    assert agent._arm is left


def test_unknown_agent_default_fails_fast() -> None:
    with pytest.raises(ValueError, match="Unknown default arm"):
        _agent(arms={"left": _Device("left")}, default_arm_name="right")


def test_lifecycle_covers_all_devices_and_deduplicates_by_identity() -> None:
    base = _Device("base")
    left_arm, right_arm = _Device("left_arm"), _Device("right_arm")
    left_hand, right_hand = _Device("left_hand"), _Device("right_hand")
    left_gripper, right_gripper = _Device("left_gripper"), _Device("right_gripper")
    perception = _Device("perception")
    agent = _agent(
        arms={"left": left_arm, "left_alias": left_arm, "right": right_arm},
        hands={"left": left_hand, "right": right_hand},
        grippers={"left": left_gripper, "right": right_gripper},
        bases={"g1": base},
        perception=perception,
        default_arm_name="right",
    )
    devices = [
        base,
        left_arm,
        right_arm,
        left_hand,
        right_hand,
        left_gripper,
        right_gripper,
        perception,
    ]

    agent.connect()
    agent.stop()
    agent.disconnect()

    assert all(device.connect_calls == 1 for device in devices)
    assert all(device.disconnect_calls == 1 for device in devices)
    assert all(device.stop_calls == 1 for device in devices if device is not perception)
    assert perception.stop_calls == 0


def test_stop_and_disconnect_continue_after_one_device_fails() -> None:
    broken = _Device("broken", fail_stop=True)
    healthy = _Device("healthy")
    agent = _agent(arms={"broken": broken, "healthy": healthy})

    agent.stop()
    agent.disconnect()

    assert broken.stop_calls == 1
    assert healthy.stop_calls == 1
    assert broken.disconnect_calls == 1
    assert healthy.disconnect_calls == 1


def test_connect_failure_rolls_back_already_connected_devices() -> None:
    base = _Device("base")
    broken_arm = _Device("broken", fail_connect=True)
    agent = _agent(bases={"base": base}, arms={"broken": broken_arm})

    with pytest.raises(ConnectionError):
        agent.connect()

    assert base.connect_calls == 1
    assert base.disconnect_calls == 1
    assert broken_arm.connect_calls == 1


def test_named_pick_propagates_side_to_scan_and_respects_configured_hold() -> None:
    left, right = _Device("left"), _Device("right")
    agent = _agent(
        arms={"left": left, "right": right},
        default_arm_name="right",
        config={"skills": {"pick": {"default_mode": "hold"}}},
    )
    capture = _PlanCaptureExecutor()
    agent._executor = capture

    result = agent.execute_skill("pick", {"arm": "left", "object_label": "cup"})

    assert result.success
    assert capture.plan is not None
    steps = capture.plan.steps
    assert [step.skill_name for step in steps] == ["scan", "detect", "pick"]
    assert steps[0].parameters == {"arm": "left"}
    assert steps[1].parameters == {"query": "cup"}
    assert steps[2].parameters["arm"] == "left"
    assert steps[2].parameters["mode"] == "hold"


def test_named_pick_explicit_drop_appends_same_side_home() -> None:
    left, right = _Device("left"), _Device("right")
    agent = _agent(
        arms={"left": left, "right": right},
        default_arm_name="right",
        config={"skills": {"pick": {"default_mode": "hold"}}},
    )
    capture = _PlanCaptureExecutor()
    agent._executor = capture

    result = agent.execute_skill("pick", {"arm": "left", "mode": "drop"})

    assert result.success
    assert capture.plan is not None
    assert capture.plan.steps[-1].skill_name == "home"
    assert capture.plan.steps[-1].parameters == {"arm": "left"}


def test_grounded_world_object_without_camera_runs_pick_directly() -> None:
    from vector_os_nano.core.world_model import ObjectState

    agent = _agent(
        arms={"left": _Device("left")},
        default_arm_name="left",
        config={"skills": {"pick": {"default_mode": "hold"}}},
    )
    agent.world.add_object(
        ObjectState(object_id="cup_0", label="cup", x=0.3, y=0.15, z=0.1)
    )
    capture = _PlanCaptureExecutor()
    agent._executor = capture

    result = agent.execute_skill(
        "pick", {"arm": "left", "object_id": "cup_0"}
    )

    assert result.success
    assert capture.plan is not None
    assert [step.skill_name for step in capture.plan.steps] == ["pick"]
    assert capture.plan.steps[0].parameters == {
        "arm": "left",
        "object_id": "cup_0",
        "mode": "hold",
    }


def test_g1_hand_base_navigation_and_stop_skills_never_append_home() -> None:
    from vector_os_nano.skills.g1 import get_g1_skills

    agent = _agent(skills=get_g1_skills())
    capture = _PlanCaptureExecutor()
    agent._executor = capture

    for skill_name in (
        "dex3_pose", "walk", "turn", "stand", "navigate", "where_am_i", "stop",
        "emergency_stop",
    ):
        result = agent.execute_skill(skill_name, {})
        assert result.success
        assert capture.plan is not None
        assert [step.skill_name for step in capture.plan.steps] == [skill_name]
