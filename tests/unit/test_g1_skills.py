# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Focused tests for the G1-specific skill package."""
from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from vector_os_nano.core.skill import SkillContext
from vector_os_nano.skills.g1 import (
    Dex3PoseSkill,
    EmergencyStopSkill,
    StandSkill,
    StopSkill,
    TurnSkill,
    WalkSkill,
    get_g1_skills,
)
from vector_os_nano.skills.go2.where_am_i import WhereAmISkill
from vector_os_nano.skills.navigate import NavigateSkill


class RecordingBase:
    name = "g1"
    supports_locomotion = True
    supports_holonomic = True

    def __init__(self) -> None:
        self.walk_calls: list[tuple[float, float, float, float]] = []
        self.stand_calls = 0
        self.stop_calls = 0
        self.emergency_stop_calls = 0

    def walk(self, vx: float, vy: float, vyaw: float, duration: float) -> bool:
        self.walk_calls.append((vx, vy, vyaw, duration))
        return True

    def stand(self) -> bool:
        self.stand_calls += 1
        return True

    def stop(self) -> None:
        self.stop_calls += 1

    def emergency_stop(self) -> None:
        self.emergency_stop_calls += 1

    def get_position(self) -> list[float]:
        return [1.0, 2.0, 0.8]


class RecordingHand:
    dof = 7

    def __init__(self, side: str) -> None:
        self.side = side
        self.calls: list[tuple[str, object, float]] = []

    def open(self, *, duration: float = 1.0) -> bool:
        self.calls.append(("open", None, duration))
        return True

    def power_grasp(self, *, duration: float = 1.0) -> bool:
        self.calls.append(("power_grasp", None, duration))
        return True

    def pinch(self, *, duration: float = 1.0) -> bool:
        self.calls.append(("pinch", None, duration))
        return True

    def set_joint_positions(
        self, positions: list[float], *, duration: float = 0.0
    ) -> bool:
        self.calls.append(("positions", list(positions), duration))
        return True


def _base_context(base: object | None = None) -> SkillContext:
    return SkillContext(bases={"g1": base or RecordingBase()})


def _hands_context() -> tuple[SkillContext, RecordingHand, RecordingHand]:
    left = RecordingHand("left")
    right = RecordingHand("right")
    return SkillContext(hands={"left": left, "right": right}), left, right


def test_get_g1_skills_reuses_generic_navigation_and_location() -> None:
    skills = get_g1_skills()
    assert [item.name for item in skills] == [
        "walk",
        "turn",
        "stand",
        "navigate",
        "where_am_i",
        "stop",
        "emergency_stop",
        "dex3_pose",
    ]
    assert isinstance(skills[3], NavigateSkill)
    assert isinstance(skills[4], WhereAmISkill)
    assert len({item.name for item in skills}) == len(skills)


@pytest.mark.parametrize(
    "direction,expected_signs",
    [
        ("forward", (1, 0)),
        ("backward", (-1, 0)),
        ("left", (0, 1)),
        ("right", (0, -1)),
    ],
)
def test_walk_maps_body_directions(
    direction: str, expected_signs: tuple[int, int]
) -> None:
    base = RecordingBase()
    result = WalkSkill().execute(
        {"direction": direction, "distance": 1.0, "speed": 0.3},
        _base_context(base),
    )
    assert result.success
    vx, vy, vyaw, duration = base.walk_calls[-1]
    assert (int(math.copysign(1, vx)) if vx else 0) == expected_signs[0]
    assert (int(math.copysign(1, vy)) if vy else 0) == expected_signs[1]
    assert vyaw == 0.0
    expected_speed = 0.2 if direction in {"left", "right"} else 0.3
    assert duration == pytest.approx(1.0 / expected_speed)


@pytest.mark.parametrize(
    "params",
    [
        {"direction": "diagonal"},
        {"distance": 0.0},
        {"distance": float("nan")},
        {"speed": -0.1},
    ],
)
def test_walk_rejects_invalid_parameters_before_motion(params: dict) -> None:
    base = RecordingBase()
    result = WalkSkill().execute(params, _base_context(base))
    assert not result.success
    assert result.diagnosis_code == "invalid_parameters"
    assert base.walk_calls == []


def test_walk_fails_before_call_when_locomotion_is_advertised_missing() -> None:
    base = RecordingBase()
    base.supports_locomotion = False
    result = WalkSkill().execute({}, _base_context(base))
    assert not result.success
    assert result.diagnosis_code == "capability_unavailable"
    assert base.walk_calls == []


def test_walk_reads_g1base_coordinator_capability_before_call() -> None:
    class CoordinatorBackedBase:
        name = "g1"

        def __init__(self) -> None:
            self.walk_calls: list[tuple[float, float, float, float]] = []
            self.coordinator = SimpleNamespace(
                capabilities=SimpleNamespace(locomotion=False, holonomic=False)
            )

        def walk(
            self, vx: float, vy: float, vyaw: float, duration: float
        ) -> bool:
            self.walk_calls.append((vx, vy, vyaw, duration))
            return True

    base = CoordinatorBackedBase()
    result = WalkSkill().execute({}, _base_context(base))
    assert not result.success
    assert result.diagnosis_code == "capability_unavailable"
    assert base.walk_calls == []


def test_lateral_walk_fails_before_call_for_non_holonomic_base() -> None:
    base = RecordingBase()
    base.supports_holonomic = False
    result = WalkSkill().execute({"direction": "left"}, _base_context(base))
    assert not result.success
    assert result.diagnosis_code == "capability_unavailable"
    assert base.walk_calls == []


def test_walk_requires_callable_protocol_method() -> None:
    base = SimpleNamespace(name="g1", supports_locomotion=True)
    result = WalkSkill().execute({}, _base_context(base))
    assert not result.success
    assert result.diagnosis_code == "capability_unavailable"


@pytest.mark.parametrize("direction,sign", [("left", 1), ("right", -1)])
def test_turn_uses_signed_yaw_and_angle_duration(direction: str, sign: int) -> None:
    base = RecordingBase()
    result = TurnSkill().execute(
        {"direction": direction, "angle": 90}, _base_context(base)
    )
    assert result.success
    vx, vy, vyaw, duration = base.walk_calls[-1]
    assert (vx, vy) == (0.0, 0.0)
    assert math.copysign(1.0, vyaw) == sign
    assert duration == pytest.approx(math.pi)


def test_turn_converts_not_implemented_to_capability_failure() -> None:
    class UnsupportedBase(RecordingBase):
        def walk(self, vx: float, vy: float, vyaw: float, duration: float) -> bool:
            raise NotImplementedError("controller unavailable")

    result = TurnSkill().execute({}, _base_context(UnsupportedBase()))
    assert not result.success
    assert result.diagnosis_code == "capability_unavailable"


def test_stand_uses_duck_typed_g1base_method() -> None:
    base = RecordingBase()
    result = StandSkill().execute({}, _base_context(base))
    assert result.success
    assert base.stand_calls == 1


def test_stand_fails_fast_when_method_is_missing() -> None:
    base = SimpleNamespace(name="g1")
    result = StandSkill().execute({}, _base_context(base))
    assert not result.success
    assert result.diagnosis_code == "capability_unavailable"


def test_stop_calls_only_base_stop() -> None:
    class StopOnlyBase:
        name = "g1"

        def __init__(self) -> None:
            self.called = False

        def stop(self) -> None:
            self.called = True

        def set_velocity(self, *_args: object) -> None:
            raise AssertionError("G1 StopSkill must not publish a velocity topic")

    base = StopOnlyBase()
    result = StopSkill().execute({}, _base_context(base))
    assert result.success
    assert base.called


def test_emergency_stop_uses_latched_base_path() -> None:
    base = RecordingBase()
    result = EmergencyStopSkill().execute({}, _base_context(base))
    assert result.success
    assert result.result_data["emergency_latched"] is True
    assert base.emergency_stop_calls == 1
    assert base.stop_calls == 0


def test_emergency_stop_reports_adapter_rejection() -> None:
    base = RecordingBase()
    base.emergency_stop = lambda: False

    result = EmergencyStopSkill().execute({}, _base_context(base))

    assert not result.success
    assert result.diagnosis_code == "stop_failed"


def test_all_base_skills_report_no_base() -> None:
    context = SkillContext()
    for skill in (
        WalkSkill(), TurnSkill(), StandSkill(), StopSkill(), EmergencyStopSkill()
    ):
        result = skill.execute({}, context)
        assert not result.success
        assert result.diagnosis_code == "no_base"


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("preset", ["open", "power_grasp", "pinch"])
def test_dex3_named_presets_select_exact_hand(side: str, preset: str) -> None:
    context, left, right = _hands_context()
    result = Dex3PoseSkill().execute(
        {"side": side, "preset": preset, "duration": 0.4}, context
    )
    assert result.success
    selected = left if side == "left" else right
    other = right if side == "left" else left
    assert selected.calls == [(preset, None, 0.4)]
    assert other.calls == []


def test_dex3_exact_seven_joint_positions() -> None:
    context, left, _right = _hands_context()
    positions = [0.1 * index for index in range(7)]
    result = Dex3PoseSkill().execute(
        {"side": "left", "positions": positions, "duration": 0.25}, context
    )
    assert result.success
    assert left.calls == [("positions", positions, 0.25)]
    assert result.result_data["positions"] == positions


@pytest.mark.parametrize(
    "params",
    [
        {"preset": "open"},
        {"side": "middle", "preset": "open"},
        {"side": "left"},
        {"side": "left", "preset": "open", "positions": [0.0] * 7},
        {"side": "left", "preset": "close"},
        {"side": "left", "positions": [0.0] * 6},
        {"side": "left", "positions": [0.0] * 6 + [float("inf")]},
        {"side": "left", "preset": "open", "duration": -1.0},
    ],
)
def test_dex3_rejects_invalid_parameters_before_command(params: dict) -> None:
    context, left, right = _hands_context()
    result = Dex3PoseSkill().execute(params, context)
    assert not result.success
    assert result.diagnosis_code == "invalid_parameters"
    assert left.calls == []
    assert right.calls == []


def test_dex3_reports_missing_selected_hand_without_using_other_side() -> None:
    right = RecordingHand("right")
    context = SkillContext(hands={"right": right})
    result = Dex3PoseSkill().execute(
        {"side": "left", "preset": "open"}, context
    )
    assert not result.success
    assert result.diagnosis_code == "no_hand"
    assert right.calls == []


def test_dex3_rejects_non_seven_dof_hand_before_command() -> None:
    hand = RecordingHand("left")
    hand.dof = 6
    context = SkillContext(hands={"left": hand})
    result = Dex3PoseSkill().execute(
        {"side": "left", "preset": "open"}, context
    )
    assert not result.success
    assert result.diagnosis_code == "capability_unavailable"
    assert hand.calls == []


def test_dex3_rejects_backend_without_selected_joint_group() -> None:
    hand = RecordingHand("left")
    hand.coordinator = SimpleNamespace(
        capabilities=SimpleNamespace(
            supports_group=lambda group: group == "right_hand"
        )
    )
    context = SkillContext(hands={"left": hand})
    result = Dex3PoseSkill().execute(
        {"side": "left", "preset": "pinch"}, context
    )
    assert not result.success
    assert result.diagnosis_code == "capability_unavailable"
    assert hand.calls == []


def test_dex3_requires_exact_preset_capability() -> None:
    hand = SimpleNamespace(side="right", dof=7, open=lambda **_kwargs: True)
    context = SkillContext(hands={"right": hand})
    result = Dex3PoseSkill().execute(
        {"side": "right", "preset": "pinch"}, context
    )
    assert not result.success
    assert result.diagnosis_code == "capability_unavailable"


def test_g1_motion_sources_have_no_legacy_flags_or_ros_topics() -> None:
    package = Path(__file__).parents[2] / "vector_os_nano" / "skills" / "g1"
    source = "\n".join(
        (package / name).read_text(encoding="utf-8")
        for name in ("_common.py", "walk.py", "turn.py", "stance.py", "stop.py")
    )
    for forbidden in (
        "/tmp/vector_nav_active",
        "cmd_vel",
        "geometry_msgs",
        "rclpy",
        "skills.go2",
    ):
        assert forbidden not in source
