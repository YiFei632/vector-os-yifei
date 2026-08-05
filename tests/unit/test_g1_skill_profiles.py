# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

import yaml

from vector_os_nano.core.skill import SkillContext
from vector_os_nano.core.world_model import ObjectState, WorldModel
from vector_os_nano.hardware.g1 import G1Profile
from vector_os_nano.skills.gripper import GripperOpenSkill
from vector_os_nano.skills.handover import HandoverSkill
from vector_os_nano.skills.home import HomeSkill
from vector_os_nano.skills.pick import PickSkill
from vector_os_nano.skills.place import PlaceSkill
from vector_os_nano.skills.scan import ScanSkill
from vector_os_nano.skills.wave import WaveSkill


_PROFILE = (
    Path(__file__).parents[2] / "config" / "robots" / "g1_edu_flagship_a.yaml"
)


def _context() -> SkillContext:
    config = yaml.safe_load(_PROFILE.read_text(encoding="utf-8"))
    left = MagicMock(name="left_arm")
    left.name = "g1_left_arm"
    left.dof = 7
    left.move_joints.return_value = True
    left.get_joint_positions.return_value = [0.0] * 7
    left.ik.return_value = [0.1] * 7
    right = MagicMock(name="right_arm")
    right.name = "g1_right_arm"
    right.dof = 7
    right.move_joints.return_value = True
    right.get_joint_positions.return_value = [0.0] * 7
    right.ik.return_value = [0.1] * 7
    left_gripper = MagicMock(name="left_dex3")
    right_gripper = MagicMock(name="right_dex3")
    return SkillContext(
        arms={"left": left, "right": right},
        grippers={"left": left_gripper, "right": right_gripper},
        default_arm_name="right",
        default_gripper_name="right",
        world_model=WorldModel(),
        config=config,
    )


def test_all_configured_g1_arm_skill_poses_are_finite_and_within_limits() -> None:
    config = yaml.safe_load(_PROFILE.read_text(encoding="utf-8"))
    profile = G1Profile.from_yaml(_PROFILE)
    for skill_name, skill_config in config["skills"].items():
        for key, by_arm in skill_config.items():
            if not key.endswith("_by_arm") or not isinstance(by_arm, dict):
                continue
            for side, values in by_arm.items():
                if side not in {"left", "right"}:
                    continue
                assert len(values) == 7, (skill_name, key, side)
                profile.validate_joint_positions(
                    profile.arm_joints[side], tuple(float(value) for value in values)
                )


def test_home_selects_named_arm_and_matching_dex3() -> None:
    context = _context()
    result = HomeSkill().execute({"arm": "left"}, context)
    assert result.success
    target = context.arms["left"].move_joints.call_args.args[0]
    assert len(target) == 7
    assert target == [0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0]
    context.grippers["left"].open.assert_called_once()
    context.grippers["right"].open.assert_not_called()


def test_scan_uses_default_right_profile() -> None:
    context = _context()
    result = ScanSkill().execute({}, context)
    assert result.success
    target = context.arms["right"].move_joints.call_args.args[0]
    assert target == [0.10, -0.25, 0.0, 1.10, 0.0, 0.0, 0.0]


def test_wave_profile_is_seven_dof(monkeypatch) -> None:
    monkeypatch.setattr("vector_os_nano.skills.wave.time.sleep", lambda _: None)
    context = _context()
    result = WaveSkill().execute({"arm": "right"}, context)
    assert result.success
    calls = context.arms["right"].move_joints.call_args_list
    assert calls
    assert all(len(call.args[0]) == 7 for call in calls)
    assert calls[0].args[0] == [-0.20, -1.00, 0.0, 1.30, 0.0, 0.0, 0.0]


def test_gripper_skill_selects_left_hand_adapter() -> None:
    context = _context()
    result = GripperOpenSkill().execute({"hand": "left"}, context)
    assert result.success
    context.grippers["left"].open.assert_called_once()
    context.grippers["right"].open.assert_not_called()


def test_missing_g1_pose_fails_before_sending_five_dof_default() -> None:
    arm = MagicMock()
    arm.name = "g1_right_arm"
    arm.dof = 7
    arm.move_joints.return_value = True
    context = SkillContext(
        arms={"right": arm},
        default_arm_name="right",
        world_model=WorldModel(),
        config={},
    )
    result = HomeSkill().execute({}, context)
    assert not result.success
    assert result.result_data["diagnosis"] == "invalid_profile"
    arm.move_joints.assert_not_called()


def test_pick_uses_selected_g1_arm_matching_dex3_and_seven_dof_home(
    monkeypatch,
) -> None:
    monkeypatch.setattr("vector_os_nano.skills.pick.time.sleep", lambda _: None)
    context = _context()
    context.world_model.add_object(
        ObjectState(
            object_id="test_cube",
            label="cube",
            x=0.30,
            y=0.10,
            z=0.10,
        )
    )

    result = PickSkill().execute(
        {"arm": "left", "object_label": "cube", "mode": "hold"},
        context,
    )

    assert result.success
    assert result.result_data["arm"] == "left"
    assert context.arms["left"].move_joints.call_args_list
    assert all(
        len(call.args[0]) == 7
        for call in context.arms["left"].move_joints.call_args_list
    )
    context.arms["right"].move_joints.assert_not_called()
    assert context.grippers["left"].open.called
    assert context.grippers["left"].close.called
    context.grippers["right"].open.assert_not_called()


def test_place_uses_selected_g1_arm_and_profile_home() -> None:
    context = _context()

    result = PlaceSkill().execute(
        {"arm": "left", "x": 0.30, "y": 0.10, "z": 0.10},
        context,
    )

    assert result.success
    assert result.result_data["arm"] == "left"
    calls = context.arms["left"].move_joints.call_args_list
    assert calls
    assert all(len(call.args[0]) == 7 for call in calls)
    assert calls[-1].args[0] == [0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0]
    context.arms["right"].move_joints.assert_not_called()
    context.grippers["left"].open.assert_called_once()
    context.grippers["right"].open.assert_not_called()


def test_handover_direction_uses_distinct_seven_dof_g1_poses(monkeypatch) -> None:
    monkeypatch.setattr("vector_os_nano.skills.handover.time.sleep", lambda _: None)
    left_context = _context()
    right_context = _context()

    left_result = HandoverSkill().execute(
        {"arm": "right", "direction": "left"}, left_context
    )
    right_result = HandoverSkill().execute(
        {"arm": "right", "direction": "right"}, right_context
    )

    assert left_result.success and right_result.success
    left_target = left_context.arms["right"].move_joints.call_args_list[0].args[0]
    right_target = right_context.arms["right"].move_joints.call_args_list[0].args[0]
    assert len(left_target) == len(right_target) == 7
    assert left_target != right_target
    assert left_target[2] == -0.35
    assert right_target[2] == 0.35


def test_pick_without_same_side_hand_fails_before_motion(monkeypatch) -> None:
    monkeypatch.setattr("vector_os_nano.skills.pick.time.sleep", lambda _: None)
    context = _context()
    del context.grippers["left"]
    context.world_model.add_object(
        ObjectState(object_id="cube", label="cube", x=0.30, y=0.10, z=0.10)
    )

    result = PickSkill().execute(
        {"arm": "left", "object_label": "cube", "mode": "hold"}, context
    )

    assert not result.success
    assert result.result_data["diagnosis"] == "no_gripper"
    context.arms["left"].move_joints.assert_not_called()
    context.grippers["right"].open.assert_not_called()


def test_pick_rejects_missing_cartesian_capability_before_hand_command() -> None:
    context = _context()
    context.arms["left"].supports_cartesian = False
    context.world_model.add_object(
        ObjectState(object_id="cube", label="cube", x=0.30, y=0.10, z=0.10)
    )

    result = PickSkill().execute(
        {"arm": "left", "object_label": "cube", "mode": "hold"}, context
    )

    assert not result.success
    assert result.result_data["diagnosis"] == "capability_unavailable"
    context.arms["left"].move_joints.assert_not_called()
    context.grippers["left"].open.assert_not_called()


def test_named_registry_side_mismatch_fails_before_home_motion() -> None:
    context = _context()
    context.arms["left"].side = "right"

    result = HomeSkill().execute({"arm": "left"}, context)

    assert not result.success
    assert result.result_data["diagnosis"] == "no_arm"
    context.arms["left"].move_joints.assert_not_called()


def test_release_skill_rejects_non_owning_arm_before_motion() -> None:
    context = _context()
    context.world_model.update_robot_state(
        held_object="cup", held_by="right", gripper_state="holding"
    )

    result = HomeSkill().execute({"arm": "left"}, context)

    assert not result.success
    assert result.result_data["diagnosis"] == "wrong_limb"
    context.arms["left"].move_joints.assert_not_called()
    context.grippers["left"].open.assert_not_called()


def test_pick_rejects_second_object_in_single_ownership_model() -> None:
    context = _context()
    context.world_model.update_robot_state(
        held_object="cup", held_by="right", gripper_state="holding"
    )

    result = PickSkill().execute(
        {"arm": "left", "object_label": "cube", "mode": "hold"}, context
    )

    assert not result.success
    assert result.result_data["diagnosis"] == "already_holding"
    context.arms["left"].move_joints.assert_not_called()
    context.grippers["left"].open.assert_not_called()


def test_wave_intermediate_motion_failure_is_not_reported_as_success(monkeypatch) -> None:
    monkeypatch.setattr("vector_os_nano.skills.wave.time.sleep", lambda _: None)
    context = _context()
    context.arms["right"].move_joints.side_effect = [True, False]

    result = WaveSkill().execute({"arm": "right"}, context)

    assert not result.success
    assert result.result_data["diagnosis"] == "move_failed"
    assert result.result_data["phase"] == "wave_left"
