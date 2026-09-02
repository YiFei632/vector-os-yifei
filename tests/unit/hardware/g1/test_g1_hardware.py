# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

from __future__ import annotations

from dataclasses import replace
import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys
import textwrap
from typing import Any

import pytest

from vector_os_nano.core.types import Pose3D
from vector_os_nano.hardware.arm import ArmProtocol
from vector_os_nano.hardware.base import BaseProtocol
from vector_os_nano.hardware.g1 import (
    G1CapabilityError,
    G1ControlMode,
    G1JointMap,
    G1Profile,
    G1Robot,
    G1State,
    G1TransportCapabilities,
    PinocchioG1Kinematics,
)
from vector_os_nano.hardware.g1.profile import (
    LEFT_ARM_JOINTS,
    LEFT_HAND_REV_1_0_XML_JOINTS,
    LEFT_HAND_SEMANTIC_JOINTS,
    RIGHT_ARM_JOINTS,
    RIGHT_HAND_REV_1_0_XML_JOINTS,
    RIGHT_HAND_SEMANTIC_JOINTS,
)
from vector_os_nano.hardware.g1.state import G1JointCommand, G1VelocityCommand
from vector_os_nano.hardware.gripper import GripperProtocol


def make_profile(tmp_path: Path) -> G1Profile:
    return G1Profile(
        profile_id="test-g1-mode-5",
        model_name="g1_29dof_with_hand_rev_1_0",
        mode_machine=5,
        urdf_path=tmp_path / "g1.urdf",
        mjcf_path=tmp_path / "g1.xml",
        mesh_dir=tmp_path / "meshes",
    )


class MockG1Transport:
    name = "mock-g1"

    def __init__(
        self,
        profile: G1Profile,
        *,
        capabilities: G1TransportCapabilities | None = None,
    ) -> None:
        self.profile = profile
        self.capabilities = capabilities or G1TransportCapabilities(
            locomotion=True,
            holonomic=True,
            lidar=True,
            emergency_damping=True,
            joint_groups=frozenset(
                {"body", "left_arm", "right_arm", "left_hand", "right_hand"}
            ),
        )
        self._connected = False
        self.connect_count = 0
        self.disconnect_count = 0
        self.stop_count = 0
        self.velocity_commands: list[G1VelocityCommand] = []
        self.joint_commands: list[G1JointCommand] = []
        self.mode_commands: list[G1ControlMode] = []
        self.lidar = object()
        self.state = G1State.zero(profile.all_joint_names)

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self.connect_count += 1
        self._connected = True

    def disconnect(self) -> None:
        self.disconnect_count += 1
        self._connected = False

    def read_state(self) -> G1State:
        if not self.connected:
            raise RuntimeError("mock disconnected")
        return self.state

    def command_velocity(self, command: G1VelocityCommand) -> None:
        self.velocity_commands.append(command)
        self.state = replace(
            self.state,
            root_linear_velocity=(command.vx, command.vy, 0.0),
            root_angular_velocity=(0.0, 0.0, command.vyaw),
            sequence_id=command.sequence_id,
        )

    def command_joints(self, command: G1JointCommand) -> None:
        self.joint_commands.append(command)
        positions = dict(zip(self.state.joint_names, self.state.joint_positions))
        positions.update(zip(command.joint_names, command.positions))
        self.state = replace(
            self.state,
            joint_positions=tuple(positions[name] for name in self.state.joint_names),
            sequence_id=command.sequence_id,
        )

    def set_mode(self, mode: G1ControlMode) -> None:
        self.mode_commands.append(mode)
        self.state = replace(self.state, control_mode=mode)

    def stop(self) -> None:
        self.stop_count += 1

    def get_lidar_scan(self) -> Any:
        return self.lidar


@pytest.fixture
def robot(tmp_path: Path) -> G1Robot:
    profile = make_profile(tmp_path)
    return G1Robot(profile, MockG1Transport(profile))


def test_named_adapters_share_one_coordinator_and_satisfy_protocols(robot: G1Robot) -> None:
    assert set(robot.arms) == {"left", "right"}
    assert set(robot.hands) == {"left", "right"}
    assert set(robot.grippers) == {"left", "right"}
    assert robot.left_arm.name == "g1_left_arm"
    assert robot.right_arm.name == "g1_right_arm"
    assert robot.left_hand.name == "g1_left_dex3_1"
    assert robot.base.coordinator is robot.left_arm.coordinator
    assert robot.base.coordinator is robot.right_hand.coordinator
    assert isinstance(robot.base, BaseProtocol)
    assert isinstance(robot.left_arm, ArmProtocol)
    assert isinstance(robot.grippers["right"], GripperProtocol)


def test_shared_lifecycle_is_idempotent_and_reference_counted(robot: G1Robot) -> None:
    transport = robot.transport
    robot.base.connect()
    robot.base.connect()
    robot.left_arm.connect()
    assert transport.connect_count == 1
    assert robot.coordinator.lifecycle_owners == frozenset({"base", "arm:left"})

    robot.base.disconnect()
    robot.base.disconnect()
    assert robot.connected
    assert transport.disconnect_count == 0

    robot.left_arm.disconnect()
    assert not robot.connected
    assert transport.disconnect_count == 1
    robot.left_arm.disconnect()
    assert transport.disconnect_count == 1


def test_robot_context_manager_connects_and_disconnects_once(robot: G1Robot) -> None:
    transport = robot.transport
    with robot as connected:
        assert connected is robot
        robot.connect()
        assert robot.connected
    assert transport.connect_count == 1
    assert transport.disconnect_count == 1


def test_joint_map_matches_rev_1_0_actuator_permutations() -> None:
    joint_map = G1JointMap()
    assert joint_map.body_xml_indices == (*range(22), *range(29, 36))
    assert joint_map.left_hand_xml_indices == (22, 23, 24, 27, 28, 25, 26)
    assert joint_map.right_hand_xml_indices == tuple(range(36, 43))

    xml = tuple(float(index) for index in range(43))
    body, left, right = joint_map.split_xml(xml)
    assert body == (*map(float, range(22)), *map(float, range(29, 36)))
    assert left == (22.0, 23.0, 24.0, 27.0, 28.0, 25.0, 26.0)
    assert right == tuple(map(float, range(36, 43)))
    assert joint_map.semantic_to_xml(
        body,
        left_hand_values=left,
        right_hand_values=right,
    ) == xml

    # Both hands use thumb/middle/index in the pinned Isaac task.  The vendor
    # XML uses that order on the left, but index/middle on the right.
    assert joint_map.hand_semantic_to_isaaclab("left", tuple(range(7))) == (
        0.0, 1.0, 2.0, 5.0, 6.0, 3.0, 4.0,
    )
    assert joint_map.hand_semantic_to_isaaclab("right", tuple(range(7))) == (
        0.0, 1.0, 2.0, 5.0, 6.0, 3.0, 4.0,
    )
    assert LEFT_HAND_REV_1_0_XML_JOINTS != LEFT_HAND_SEMANTIC_JOINTS
    assert RIGHT_HAND_REV_1_0_XML_JOINTS == RIGHT_HAND_SEMANTIC_JOINTS


def test_joint_map_rejects_bad_dimensions_and_non_finite_values() -> None:
    joint_map = G1JointMap()
    with pytest.raises(ValueError, match="length"):
        joint_map.xml_to_body_dds([0.0] * 42)
    with pytest.raises(ValueError, match="non-finite"):
        joint_map.xml_to_body_dds([0.0] * 42 + [float("nan")])


def test_base_clips_velocity_and_returns_generic_odometry(robot: G1Robot) -> None:
    transport = robot.transport
    robot.base.connect()
    transport.state = replace(
        transport.state,
        root_pose=Pose3D(x=1.0, y=2.0, z=0.8, qz=0.5, qw=0.8660254038),
    )
    robot.base.set_velocity(9.0, -9.0, 9.0)
    command = transport.velocity_commands[-1]
    assert (command.vx, command.vy, command.vyaw) == (0.6, -0.4, 1.0)
    assert transport.mode_commands[-1] is G1ControlMode.LOCOMOTION
    odometry = robot.base.get_odometry()
    assert (odometry.x, odometry.y, odometry.z) == (1.0, 2.0, 0.8)
    assert (odometry.vx, odometry.vy, odometry.vyaw) == (0.6, -0.4, 1.0)
    assert robot.base.get_heading() == pytest.approx(1.0471975512)
    assert robot.base.get_lidar_scan() is transport.lidar


def test_base_walk_refreshes_velocity_before_gateway_watchdog(robot: G1Robot) -> None:
    transport = robot.transport
    robot.base.connect()

    assert robot.base.walk(0.2, 0.0, 0.0, duration=0.45)

    moving = [command for command in transport.velocity_commands if command.vx > 0]
    assert len(moving) >= 2
    assert transport.velocity_commands[-1].vx == 0.0


def test_base_stand_and_emergency_stop_are_explicit(robot: G1Robot) -> None:
    robot.base.connect()
    assert robot.base.stand()
    assert robot.coordinator.mode is G1ControlMode.STAND
    assert robot.base.stand(wait=True, stable_duration=0.0, timeout=0.1)
    robot.base.emergency_stop()
    assert robot.coordinator.mode is G1ControlMode.EMERGENCY_DAMPING
    assert robot.base.emergency_latched
    assert robot.emergency_latched
    robot.base.stop()
    assert robot.emergency_latched
    assert robot.coordinator.mode is G1ControlMode.EMERGENCY_DAMPING
    robot.clear_emergency()
    assert not robot.emergency_latched
    assert robot.coordinator.mode is G1ControlMode.STAND


def test_fixed_base_emergency_recovery_returns_to_passive(tmp_path: Path) -> None:
    profile = make_profile(tmp_path)
    transport = MockG1Transport(
        profile,
        capabilities=G1TransportCapabilities(
            emergency_damping=True,
            joint_groups=frozenset(
                {"left_arm", "right_arm", "left_hand", "right_hand"}
            ),
        ),
    )
    robot = G1Robot(profile, transport)
    robot.connect()
    robot.emergency_stop()
    assert robot.emergency_latched

    robot.base.clear_emergency()

    assert not robot.emergency_latched
    assert robot.coordinator.mode is G1ControlMode.PASSIVE
    assert transport.mode_commands[-1] is G1ControlMode.PASSIVE
    assert robot.left_arm.move_joints([0.0] * 7, duration=0.0)


def test_emergency_delivery_failure_is_surfaced_but_latch_remains(tmp_path: Path) -> None:
    profile = make_profile(tmp_path)

    class FailingEmergencyTransport(MockG1Transport):
        def set_mode(self, mode: G1ControlMode) -> None:
            if mode is G1ControlMode.EMERGENCY_DAMPING:
                raise RuntimeError("mode channel unavailable")
            super().set_mode(mode)

        def stop(self) -> None:
            raise RuntimeError("stop channel unavailable")

    robot = G1Robot(profile, FailingEmergencyTransport(profile))
    robot.connect()

    with pytest.raises(RuntimeError, match="backend stop contract was not satisfied"):
        robot.emergency_stop()

    assert robot.emergency_latched
    assert robot.coordinator.mode is G1ControlMode.EMERGENCY_DAMPING


def test_walk_requires_a_real_backend_capability(tmp_path: Path) -> None:
    profile = make_profile(tmp_path)
    transport = MockG1Transport(profile, capabilities=G1TransportCapabilities())
    robot = G1Robot(profile, transport)
    robot.connect()
    assert not robot.base.supports_holonomic
    assert not robot.base.supports_lidar
    with pytest.raises(G1CapabilityError, match="no locomotion controller"):
        robot.base.set_velocity(0.1, 0.0, 0.0)


def test_arm_joint_api_is_strict_and_cartesian_is_not_faked(robot: G1Robot) -> None:
    robot.left_arm.connect()
    assert robot.left_arm.joint_names == list(LEFT_ARM_JOINTS)
    assert robot.right_arm.joint_names == list(RIGHT_ARM_JOINTS)
    assert robot.left_arm.dof == 7
    assert robot.left_arm.move_joints([0.0] * 7, duration=0.25)
    command = robot.transport.joint_commands[-1]
    assert command.group == "left_arm"
    assert command.joint_names == LEFT_ARM_JOINTS
    assert command.duration == 0.25

    with pytest.raises(ValueError, match="requires 7"):
        robot.left_arm.move_joints([0.0] * 6)
    with pytest.raises(ValueError, match="outside"):
        robot.left_arm.move_joints([9.0] + [0.0] * 6)
    with pytest.raises(NotImplementedError, match="kinematics backend"):
        robot.left_arm.ik((0.4, 0.2, 0.8))
    with pytest.raises(NotImplementedError, match="kinematics backend"):
        robot.left_arm.fk([0.0] * 7)
    with pytest.raises(NotImplementedError, match="kinematics backend"):
        robot.left_arm.move_cartesian((0.4, 0.2, 0.8))


def test_arm_group_capability_failure_is_explicit(tmp_path: Path) -> None:
    profile = make_profile(tmp_path)
    transport = MockG1Transport(
        profile,
        capabilities=G1TransportCapabilities(joint_groups=frozenset({"right_arm"})),
    )
    robot = G1Robot(profile, transport)
    robot.left_arm.connect()
    with pytest.raises(G1CapabilityError, match="left_arm"):
        robot.left_arm.move_joints([0.0] * 7)


def test_dex3_full_joint_api_and_gripper_view(robot: G1Robot) -> None:
    hand = robot.right_hand
    hand.connect()
    assert hand.joint_names == list(RIGHT_HAND_SEMANTIC_JOINTS)
    assert hand.dof == 7
    assert hand.get_position() == pytest.approx(1.0)
    assert hand.close(duration=0.2)
    command = robot.transport.joint_commands[-1]
    assert command.group == "right_hand"
    assert command.joint_names == RIGHT_HAND_SEMANTIC_JOINTS
    assert command.positions == (-0.35, -0.8, -1.4, 1.25, 1.4, 1.25, 1.4)
    assert hand.get_joint_positions() == list(command.positions)
    assert hand.get_position() == pytest.approx(0.0)

    robot.transport.state = replace(
        robot.transport.state,
        hand_holding={"left": False, "right": True},
        hand_forces={"left": None, "right": 4.5},
    )
    gripper = robot.grippers["right"]
    assert gripper.is_holding()
    assert gripper.get_force() == 4.5
    assert gripper.open()

    with pytest.raises(ValueError, match="requires 7"):
        hand.set_joint_positions([0.0] * 6)
    with pytest.raises(ValueError, match="outside"):
        hand.set_joint_positions([9.0] + [0.0] * 6)


def test_left_and_right_dex3_close_signs_are_mirrored(robot: G1Robot) -> None:
    robot.left_hand.connect()
    robot.left_hand.power_grasp()
    left = robot.transport.joint_commands[-1]
    robot.right_hand.power_grasp()
    right = robot.transport.joint_commands[-1]
    assert left.joint_names == LEFT_HAND_SEMANTIC_JOINTS
    assert left.positions == tuple(-value for value in right.positions)


def test_transport_state_order_is_strictly_validated(robot: G1Robot) -> None:
    robot.connect()
    state = robot.transport.state
    robot.transport.state = replace(
        state,
        joint_names=tuple(reversed(state.joint_names)),
        joint_positions=tuple(reversed(state.joint_positions)),
        joint_velocities=tuple(reversed(state.joint_velocities)),
        joint_efforts=tuple(reversed(state.joint_efforts)),
    )
    with pytest.raises(ValueError, match="joint order mismatch"):
        robot.read_state()


def test_profile_root_env_and_hash_validation(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    fallback = tmp_path / "fallback"
    override = tmp_path / "override"
    for root in (fallback, override):
        root.mkdir()
        (root / "meshes").mkdir()
        (root / "robot.urdf").write_text(f"urdf:{root.name}", encoding="utf-8")
        (root / "robot.xml").write_text(f"xml:{root.name}", encoding="utf-8")
    monkeypatch.setenv("VECTOR_G1_TEST_ROOT", str(override))
    urdf_hash = hashlib.sha256((override / "robot.urdf").read_bytes()).hexdigest()
    mjcf_hash = hashlib.sha256((override / "robot.xml").read_bytes()).hexdigest()
    profile = G1Profile.from_mapping(
        {
            "robot": {"profile_id": "hash-test", "mode_machine": 5},
            "assets": {
                "root": str(fallback),
                "root_env": "VECTOR_G1_TEST_ROOT",
                "urdf": "robot.urdf",
                "mjcf": "robot.xml",
                "meshes": "meshes",
                "sha256": {"urdf": urdf_hash, "mjcf": mjcf_hash},
            },
        }
    )
    assert profile.urdf_path == (override / "robot.urdf").resolve()
    assert profile.mjcf_path == (override / "robot.xml").resolve()
    assert profile.asset_root_env == "VECTOR_G1_TEST_ROOT"

    with pytest.raises(ValueError, match="SHA256 mismatch"):
        G1Profile.from_mapping(
            {
                "assets": {
                    "root": str(override),
                    "root_env": None,
                    "urdf": "robot.urdf",
                    "mjcf": "robot.xml",
                    "meshes": "meshes",
                    "urdf_sha256": "0" * 64,
                }
            }
        )


def test_state_rejects_bad_dimensions_and_non_finite_values(robot: G1Robot) -> None:
    names = robot.profile.all_joint_names
    with pytest.raises(ValueError, match="length"):
        G1State(
            timestamp=0.0,
            joint_names=names,
            joint_positions=(0.0,) * (len(names) - 1),
            joint_velocities=(0.0,) * len(names),
            joint_efforts=(0.0,) * len(names),
        )
    with pytest.raises(ValueError, match="non-finite"):
        G1State(
            timestamp=0.0,
            joint_names=names,
            joint_positions=(0.0,) * (len(names) - 1) + (float("nan"),),
            joint_velocities=(0.0,) * len(names),
            joint_efforts=(0.0,) * len(names),
        )


def test_pinocchio_fk_and_position_ik_against_supplied_rev_1_0_assets() -> None:
    # Some pytest/plugin combinations load incompatible native libraries into
    # the process before Pinocchio.  Exercise the optional native dependency in
    # a clean interpreter so a plugin cannot make this unit test segfault.
    if importlib.util.find_spec("pinocchio") is None:
        pytest.skip("optional Pinocchio dependency is not installed")
    root = Path(
        "/media/fishyu/fish-14tb-12/YiFei/unitree_ros/robots/g1_description"
    )
    urdf = root / "g1_29dof_with_hand_rev_1_0.urdf"
    if not urdf.is_file():
        pytest.skip("developer G1 rev-1.0 assets are not available")
    script = textwrap.dedent(
        f"""
        import numpy as np
        from pathlib import Path
        from vector_os_nano.hardware.g1 import G1Profile, PinocchioG1Kinematics
        root = Path({str(root)!r})
        profile = G1Profile(
            profile_id="actual-g1-rev-1.0",
            model_name="g1_29dof_with_hand_rev_1_0",
            mode_machine=5,
            urdf_path=root / "g1_29dof_with_hand_rev_1_0.urdf",
            mjcf_path=root / "g1_29dof_with_hand_rev_1_0.xml",
            mesh_dir=root / "meshes",
        )
        solver = PinocchioG1Kinematics(profile)
        for side in ("left", "right"):
            position, rotation = solver.fk(side, [0.0] * 7)
            assert len(position) == 3
            assert len(rotation) == 3 and all(len(row) == 3 for row in rotation)
            target = (position[0], position[1], position[2] + 0.01)
            solution = solver.ik(side, target, [0.0] * 7)
            assert solution is not None
            reached, _ = solver.fk(side, solution)
            assert np.allclose(reached, target, atol=2e-4)
            assert solver.ik(side, (100.0, 100.0, 100.0), [0.0] * 7) is None
        """
    )
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=Path(__file__).resolve().parents[4],
        capture_output=True,
        text=True,
        check=False,
    )
    if "compiled using NumPy 1.x" in completed.stderr:
        pytest.skip("installed Pinocchio binary is incompatible with this pytest Python")
    assert completed.returncode == 0, completed.stderr
