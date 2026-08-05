# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

from __future__ import annotations

from pathlib import Path
import time

import pytest

pytest.importorskip("mujoco")

from vector_os_nano.hardware.g1.profile import (
    G1Profile,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
)
from vector_os_nano.hardware.g1.robot import G1Robot
from vector_os_nano.hardware.g1.state import (
    G1ControlMode,
    G1JointCommand,
    G1VelocityCommand,
)
from vector_os_nano.hardware.sim.g1_mujoco_transport import MuJoCoG1Transport


_ROOT = Path(
    "/media/fishyu/fish-14tb-11/YiFei/unitree_ros/robots/g1_description"
)
_MJCF = _ROOT / "g1_29dof_with_hand_rev_1_0.xml"
pytestmark = pytest.mark.skipif(not _MJCF.exists(), reason="local G1 vendor assets absent")


def _profile() -> G1Profile:
    return G1Profile(
        profile_id="g1_test",
        model_name="g1_29dof_with_hand_rev_1_0",
        mode_machine=5,
        urdf_path=_ROOT / "g1_29dof_with_hand_rev_1_0.urdf",
        mjcf_path=_MJCF,
        mesh_dir=_ROOT / "meshes",
    )


def test_loads_vendor_model_and_reports_semantic_state() -> None:
    transport = MuJoCoG1Transport(_profile(), realtime=True)
    try:
        transport.connect()
        time.sleep(0.04)
        state = transport.read_state()
        assert len(state.joint_names) == 43
        assert len(state.joint_positions) == 43
        assert state.root_pose.z == pytest.approx(0.8)
        assert not state.fallen
        assert transport.supports_locomotion
    finally:
        transport.disconnect()
    assert not transport.connected


def test_accepts_locomotion_and_advances_root_pose() -> None:
    transport = MuJoCoG1Transport(_profile())
    try:
        transport.connect()
        before = transport.read_state().root_pose
        transport.command_velocity(G1VelocityCommand(0.12, 0.0, 0.0, 1))
        time.sleep(0.20)
        after = transport.read_state().root_pose
        assert after.x > before.x + 0.01
        assert transport.read_state().control_mode is G1ControlMode.LOCOMOTION
    finally:
        transport.disconnect()


def test_right_arm_joint_command_uses_exact_named_group() -> None:
    transport = MuJoCoG1Transport(_profile(), realtime=True)
    target = (0.45, -0.30, 0.0, 0.90, 0.0, 0.0, 0.20)
    try:
        transport.connect()
        transport.command_joints(
            G1JointCommand(
                group="right_arm",
                joint_names=RIGHT_ARM_JOINTS,
                positions=target,
                sequence_id=1,
                duration=0.05,
            )
        )
        time.sleep(0.25)
        actual = transport.read_state().positions_for(RIGHT_ARM_JOINTS)
        assert actual == pytest.approx(target, abs=0.16)

        with pytest.raises(ValueError, match="exact semantic order"):
            transport.command_joints(
                G1JointCommand(
                    group="right_arm",
                    joint_names=tuple(reversed(RIGHT_ARM_JOINTS)),
                    positions=target,
                    sequence_id=2,
                )
            )
    finally:
        transport.disconnect()


def test_left_and_right_trajectories_do_not_cancel_each_other() -> None:
    transport = MuJoCoG1Transport(_profile(), realtime=True)
    left_target = (0.40, 0.25, 0.0, 0.85, 0.0, 0.0, -0.15)
    right_target = (0.40, -0.25, 0.0, 0.85, 0.0, 0.0, 0.15)
    try:
        transport.connect()
        transport.command_joints(
            G1JointCommand(
                group="left_arm",
                joint_names=LEFT_ARM_JOINTS,
                positions=left_target,
                sequence_id=1,
                duration=0.10,
            )
        )
        transport.command_joints(
            G1JointCommand(
                group="right_arm",
                joint_names=RIGHT_ARM_JOINTS,
                positions=right_target,
                sequence_id=2,
                duration=0.10,
            )
        )
        time.sleep(0.30)
        state = transport.read_state()
        assert state.positions_for(LEFT_ARM_JOINTS) == pytest.approx(
            left_target, abs=0.16
        )
        assert state.positions_for(RIGHT_ARM_JOINTS) == pytest.approx(
            right_target, abs=0.16
        )
    finally:
        transport.disconnect()


def test_g1_arm_adapter_blocks_until_joint_state_converges() -> None:
    transport = MuJoCoG1Transport(_profile(), realtime=True)
    robot = G1Robot(transport.profile, transport)
    target = [0.45, -0.30, 0.0, 0.90, 0.0, 0.0, 0.20]
    try:
        robot.connect()
        assert robot.right_arm.move_joints(target, duration=0.20)
        actual = robot.right_arm.get_joint_positions()
        assert max(abs(value - goal) for value, goal in zip(actual, target)) <= 0.08
    finally:
        robot.disconnect()


def test_emergency_mode_remains_latched_until_explicit_clear() -> None:
    transport = MuJoCoG1Transport(_profile(), realtime=True)
    robot = G1Robot(transport.profile, transport)
    try:
        robot.connect()
        robot.emergency_stop()
        robot.base.stop()
        assert robot.emergency_latched
        assert transport.read_state().control_mode is G1ControlMode.EMERGENCY_DAMPING

        robot.clear_emergency()
        assert not robot.emergency_latched
        assert transport.read_state().control_mode is G1ControlMode.PASSIVE
    finally:
        robot.disconnect()
