# SPDX-License-Identifier: Apache-2.0
"""Pure unit/contract tests for the Isaac G1 29-DOF + dual Dex3 plugin."""
from __future__ import annotations

import importlib
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
ISAAC_DIR = REPO_ROOT / "docker" / "isaac-sim"
BRIDGE_DIR = ISAAC_DIR / "bridge"
if str(BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(BRIDGE_DIR))

g1_manifest = importlib.import_module("g1_manifest")
g1_protocol = importlib.import_module("g1_file_protocol")
g1_runtime = importlib.import_module("g1_isaac_runtime")
g1_dds_bridge = importlib.import_module("g1_dds_bridge")

from vector_os_nano.hardware.g1.profile import (
    G1Profile,
    LEFT_HAND_SEMANTIC_JOINTS,
)
from vector_os_nano.hardware.g1.state import (
    G1ControlMode,
    G1JointCommand,
    G1VelocityCommand,
)
from vector_os_nano.hardware.g1.transport import G1CapabilityError, G1Transport
from vector_os_nano.hardware.sim.g1_isaac_transport import G1IsaacTransport


class TestG1Manifest:
    def test_exact_29_plus_dual_7_mapping(self) -> None:
        assert len(g1_manifest.G1_BODY_JOINTS) == 29
        assert len(g1_manifest.G1_LEFT_ARM_JOINTS) == 7
        assert len(g1_manifest.G1_RIGHT_ARM_JOINTS) == 7
        assert len(g1_manifest.G1_LEFT_HAND_ISAAC_TASK_JOINTS) == 7
        assert len(g1_manifest.G1_RIGHT_HAND_ISAAC_TASK_JOINTS) == 7
        assert len(g1_manifest.G1_ALL_JOINTS) == 43
        assert len(set(g1_manifest.G1_ALL_JOINTS)) == 43

    def test_official_body_wire_order_boundaries(self) -> None:
        assert g1_manifest.G1_BODY_JOINTS[:3] == (
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
        )
        assert g1_manifest.G1_BODY_JOINTS[15:22] == g1_manifest.G1_LEFT_ARM_JOINTS
        assert g1_manifest.G1_BODY_JOINTS[22:29] == g1_manifest.G1_RIGHT_ARM_JOINTS
        assert g1_manifest.G1_BODY_JOINTS[-1] == "right_wrist_yaw_joint"

    @pytest.mark.parametrize("alias", ["g1", "g129", "g1-29dof-dex3", "g1_29dof_dex3"])
    def test_g1_aliases_are_explicitly_normalized(self, alias: str) -> None:
        assert g1_manifest.resolve_robot_type(alias) == "g1_29dof_dex3"

    def test_go2_remains_default(self) -> None:
        assert g1_manifest.resolve_robot_type(None) == "go2"
        assert g1_manifest.resolve_robot_type("go2") == "go2"

    def test_unknown_robot_type_fails(self) -> None:
        with pytest.raises(ValueError, match="Unsupported ISAAC_ROBOT_TYPE"):
            g1_manifest.resolve_robot_type("some_robot")

    def test_wrong_command_size_fails(self) -> None:
        with pytest.raises(ValueError, match="requires 7"):
            g1_manifest.validate_joint_vector("left_hand", [0.0] * 6)


class TestG1FileProtocol:
    def test_channel_roundtrip_and_expiry(self) -> None:
        sent = 1_000_000_000
        command = g1_protocol.update_command_channel(
            None,
            "left_arm",
            [float(index) for index in range(7)],
            ttl_ms=100,
            timestamp_ns=sent,
        )
        assert g1_protocol.fresh_channel_values(
            command, "left_arm", timestamp_ns=sent + 99_000_000
        ) == [float(index) for index in range(7)]
        assert (
            g1_protocol.fresh_channel_values(
                command, "left_arm", timestamp_ns=sent + 101_000_000
            )
            is None
        )

    def test_channels_have_independent_timestamps(self) -> None:
        command = g1_protocol.update_command_channel(
            None, "base", [0.1, 0.0, 0.0, 0.8], timestamp_ns=100, ttl_ms=100
        )
        command = g1_protocol.update_command_channel(
            command, "right_hand", [0.2] * 7, timestamp_ns=200, ttl_ms=100
        )
        assert command["channels"]["base"]["monotonic_ns"] == 100
        assert command["channels"]["right_hand"]["monotonic_ns"] == 200
        assert command["channels"]["base"]["sequence"] == 1
        assert command["channels"]["right_hand"]["sequence"] == 2

    def test_atomic_json_roundtrip(self, tmp_path: Path) -> None:
        path = tmp_path / "state" / "command.json"
        document = g1_protocol.new_command_document()
        g1_protocol.atomic_write_json(path, document)
        assert g1_protocol.read_json(path) == document
        assert not list(path.parent.glob("*.tmp"))


class TestG1DDSMappings:
    def test_side_commands_only_touch_their_seven_body_indices(self) -> None:
        current = [float(index) for index in range(29)]
        merged = g1_dds_bridge.merge_body_targets(
            current, left_arm=[100.0 + index for index in range(7)]
        )
        assert merged[:15] == current[:15]
        assert merged[15:22] == [100.0 + index for index in range(7)]
        assert merged[22:] == current[22:]

    def test_ros_to_unitree_base_sign_and_safety_limits(self) -> None:
        assert g1_dds_bridge.unitree_base_command([0.2, 0.1, 0.3, 0.8]) == [
            0.2,
            -0.1,
            -0.3,
            0.8,
        ]
        assert g1_dds_bridge.unitree_base_command([9.0, -9.0, 9.0, 0.0]) == [
            1.0,
            0.5,
            -1.57,
            0.3,
        ]
        assert g1_dds_bridge.unitree_base_command(None) == [0.0, 0.0, 0.0, 0.8]

    def test_extract_odom_converts_wxyz_to_xyzw(self) -> None:
        message = {
            "init_state": {
                "articulation": {
                    "robot": {
                        "root_pose": [[1, 2, 3, 0.9, 0.1, 0.2, 0.3]],
                        "root_velocity": [[4, 5, 6, 7, 8, 9]],
                    }
                }
            }
        }
        assert g1_dds_bridge.extract_odom(message) == [
            1.0,
            2.0,
            3.0,
            0.1,
            0.2,
            0.3,
            0.9,
            4.0,
            5.0,
            6.0,
            7.0,
            8.0,
            9.0,
        ]

    def test_non_finite_motor_state_is_rejected(self) -> None:
        motors = [
            SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0) for _ in range(7)
        ]
        motors[3].q = float("nan")
        with pytest.raises(ValueError, match="non-finite"):
            g1_dds_bridge.G1DDSBridge._motor_vectors(
                SimpleNamespace(motor_state=motors), 7
            )

    def test_ready_requires_every_state_stream_to_be_fresh(self) -> None:
        updates = {
            "body": 900,
            "left_hand": 901,
            "right_hand": 902,
            "odom": 903,
        }
        assert g1_dds_bridge.state_streams_fresh(
            updates, timestamp_ns=1000, timeout_ns=100
        )
        updates["odom"] = 899
        assert not g1_dds_bridge.state_streams_fresh(
            updates, timestamp_ns=1000, timeout_ns=100
        )
        updates["odom"] = 1001
        assert not g1_dds_bridge.state_streams_fresh(
            updates, timestamp_ns=1000, timeout_ns=100
        )

    def test_stale_dds_stream_revokes_and_can_restore_ready(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        bridge = object.__new__(g1_dds_bridge.G1DDSBridge)
        bridge._state_dir = tmp_path
        bridge._state_path = tmp_path / g1_protocol.STATE_FILE
        bridge._ready_path = tmp_path / "ready"
        bridge._lock = threading.Lock()
        bridge._sequence = 0
        bridge._body_received = True
        bridge._left_hand_received = True
        bridge._right_hand_received = True
        bridge._ready_written = True
        bridge._state_timeout_ns = 100
        bridge._last_state_update_ns = {
            "body": 1,
            "left_hand": 1,
            "right_hand": 1,
            "odom": 1,
        }
        bridge._body_q = bridge._body_dq = bridge._body_tau = [0.0] * 29
        bridge._left_hand_q = bridge._left_hand_dq = bridge._left_hand_tau = [0.0] * 7
        bridge._right_hand_q = bridge._right_hand_dq = bridge._right_hand_tau = [0.0] * 7
        bridge._imu = {}
        bridge._odom = [0.0] * 13
        bridge._ready_path.write_text("g1\n", encoding="utf-8")
        g1_protocol.write_manifest(tmp_path, {"status": "ready"})
        monkeypatch.setattr(g1_dds_bridge, "monotonic_ns", lambda: 1000)

        bridge._write_state()

        assert not bridge._ready_path.exists()
        assert not g1_protocol.read_json(bridge._state_path)["ready"]
        assert g1_protocol.read_json(tmp_path / g1_protocol.MANIFEST_FILE)["status"] == "stale"

        bridge._last_state_update_ns = {
            stream: 1000 for stream in ("body", "left_hand", "right_hand", "odom")
        }
        bridge._write_state()
        assert bridge._ready_path.is_file()
        assert g1_protocol.read_json(bridge._state_path)["ready"]


def _runtime_config(tmp_path: Path) -> g1_runtime.G1RuntimeConfig:
    root = tmp_path / "unitree_sim_isaaclab"
    return g1_runtime.G1RuntimeConfig(
        unitree_root=root,
        state_dir=tmp_path / "state",
        python_executable=tmp_path / "python.sh",
        task=g1_runtime.DEFAULT_TASK,
        policy_relative_path=Path("assets/model/policy.onnx"),
        device="cuda:0",
        headless=True,
        enable_cameras=True,
        no_render=False,
    )


def _create_required_runtime_files(config: g1_runtime.G1RuntimeConfig) -> None:
    for path in config.required_files():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("test", encoding="utf-8")
    config.python_executable.write_text("#!/bin/sh\n", encoding="utf-8")


class TestG1RuntimePreflight:
    def test_missing_policy_fails_before_simulation(self, tmp_path: Path) -> None:
        config = _runtime_config(tmp_path)
        _create_required_runtime_files(config)
        config.policy_path.unlink()
        with pytest.raises(RuntimeError) as error:
            config.validate(check_python_modules=False)
        message = str(error.value)
        assert "policy.onnx" in message
        assert "No fallback controller" in message

    def test_complete_layout_passes_and_builds_official_command(
        self, tmp_path: Path
    ) -> None:
        config = _runtime_config(tmp_path)
        assert (
            config.unitree_root / "teleimager/src/teleimager/image_server.py"
        ) in config.required_files()
        _create_required_runtime_files(config)
        config.validate(check_python_modules=False)
        command = config.command()
        assert str(config.unitree_root / "sim_main.py") in command
        assert "Isaac-Move-Cylinder-G129-Dex3-Wholebody" in command
        assert "--enable_dex3_dds" in command
        assert "--enable_wholebody_dds" in command
        assert "--model_path" in command
        assert "assets/model/policy.onnx" in command

    def test_absolute_policy_path_is_rejected(self, tmp_path: Path) -> None:
        config = _runtime_config(tmp_path)
        config = g1_runtime.G1RuntimeConfig(
            **{
                **config.__dict__,
                "policy_relative_path": Path("/tmp/policy.onnx"),
            }
        )
        with pytest.raises(RuntimeError, match="must be relative"):
            config.validate(check_python_modules=False)


class TestG1ContainerContracts:
    def test_entrypoint_dispatches_without_changing_go2_default(self) -> None:
        content = (ISAAC_DIR / "docker-entrypoint.sh").read_text(encoding="utf-8")
        assert 'case "${ISAAC_ROBOT_TYPE:-go2}"' in content
        assert "isaac_sim_physics.py" in content
        assert "g1_isaac_runtime.py" in content
        assert "g1_dds_bridge.py" in content
        assert "rm -f" in content and '"${ISAAC_STATE_DIR}/ready"' in content

    def test_host_launcher_preflights_complete_official_g1_assets(self) -> None:
        content = (REPO_ROOT / "scripts/launch_isaac.sh").read_text(
            encoding="utf-8"
        )
        for required in (
            "g1_29dof_with_dex3_rev_1_0.usd",
            "small_warehouse_digital_twin.usd",
            "PackingTable_2/PackingTable.usd",
            "PackingTable/PackingTable.usd",
            "assets/model/policy.onnx",
        ):
            assert required in content
        assert 'export G1_ASSET_DIR="${G1_ASSET_DIR_VALUE}"' in content

    def test_compose_declares_plugin_assets_and_file_health(self) -> None:
        compose = yaml.safe_load(
            (ISAAC_DIR / "docker-compose.yaml").read_text(encoding="utf-8")
        )
        service = compose["services"]["isaac-sim"]
        environment = str(service["environment"])
        volumes = str(service["volumes"])
        health = str(service["healthcheck"]["test"])
        assert "ISAAC_ROBOT_TYPE=${ISAAC_ROBOT_TYPE:-go2}" in environment
        assert "G1_POLICY_PATH" in environment
        assert "G1_ASSET_DIR" in volumes
        assert "/tmp/isaac_state/ready" in health
        assert "/opt/ros/humble" not in health

    def test_unitree_dds_is_loopback_only(self) -> None:
        root = ET.parse(ISAAC_DIR / "unitree-cyclonedds.xml").getroot()
        interface = root.find(".//NetworkInterface")
        multicast = root.find(".//AllowMulticast")
        assert interface is not None and interface.get("name") == "lo"
        assert multicast is not None and multicast.text.strip().lower() == "false"

        # unitree_sdk2_python passes inline XML to CycloneDDS, so the XML alone
        # is insufficient. Both Unitree processes must explicitly bind ``lo``.
        dockerfile = (ISAAC_DIR / "Dockerfile").read_text(encoding="utf-8")
        gateway = (BRIDGE_DIR / "g1_dds_bridge.py").read_text(encoding="utf-8")
        compose = (ISAAC_DIR / "docker-compose.yaml").read_text(encoding="utf-8")
        assert 'ChannelFactoryInitialize(1, "lo")' in dockerfile
        assert "ChannelFactoryInitialize(domain, interface)" in gateway
        assert 'interface != "lo"' in gateway
        assert "UNITREE_DDS_INTERFACE=lo" in compose

    def test_dockerfile_pins_official_components_and_does_not_mask_install(self) -> None:
        content = (ISAAC_DIR / "Dockerfile").read_text(encoding="utf-8")
        assert "v2.3.2" in content
        assert "e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc" in content
        assert "65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5" in content
        assert "ln -s /isaac-sim /opt/IsaacLab/_isaac_sim" in content
        assert "test -x /opt/IsaacLab/_isaac_sim/python.sh" in content
        assert "|| true" not in content
        assert "| tail" not in content

    def test_ros_bridge_exposes_full_dual_arm_and_hand_topics(self) -> None:
        content = (BRIDGE_DIR / "ros2_publisher.py").read_text(encoding="utf-8")
        for topic in (
            "/g1/body/joint_commands",
            "/g1/left_arm/joint_commands",
            "/g1/right_arm/joint_commands",
            "/g1/left_hand/joint_commands",
            "/g1/right_hand/joint_commands",
            "/arm/joint_commands",
        ):
            assert topic in content

    def test_g1_ready_requires_root_odometry(self) -> None:
        content = (BRIDGE_DIR / "g1_dds_bridge.py").read_text(encoding="utf-8")
        assert "and self._odom is not None" in content


def _host_profile(tmp_path: Path) -> G1Profile:
    return G1Profile(
        profile_id="test-g1-isaac",
        model_name="g1_29dof_with_dex3_rev_1_0",
        mode_machine=5,
        urdf_path=tmp_path / "g1.urdf",
        mjcf_path=tmp_path / "g1.xml",
        mesh_dir=tmp_path / "meshes",
    )


class TestG1IsaacHostTransport:
    def test_structurally_implements_g1_transport(self, tmp_path: Path) -> None:
        transport = G1IsaacTransport(_host_profile(tmp_path))
        assert isinstance(transport, G1Transport)
        assert transport.capabilities.locomotion
        assert transport.capabilities.holonomic
        assert not transport.capabilities.lidar
        assert not transport.capabilities.emergency_damping
        assert transport.capabilities.joint_groups == frozenset(
            {"left_arm", "right_arm", "left_hand", "right_hand"}
        )

    def test_dex3_command_is_reordered_by_name_for_isaac(self, tmp_path: Path) -> None:
        profile = _host_profile(tmp_path)
        transport = G1IsaacTransport(profile)
        transport._connected = True
        transport._publish_joint_group = MagicMock()
        command = G1JointCommand(
            group="left_hand",
            joint_names=LEFT_HAND_SEMANTIC_JOINTS,
            positions=tuple(float(index) for index in range(7)),
            sequence_id=1,
        )
        transport.command_joints(command)
        named = dict(zip(LEFT_HAND_SEMANTIC_JOINTS, command.positions))
        expected = tuple(
            named[name]
            for name in g1_manifest.G1_LEFT_HAND_ISAAC_TASK_JOINTS
        )
        transport._publish_joint_group.assert_called_once_with(
            "left_hand", expected
        )

    def test_velocity_command_and_stale_sequence_guard(self, tmp_path: Path) -> None:
        transport = G1IsaacTransport(_host_profile(tmp_path))
        transport._connected = True
        transport._publish_velocity = MagicMock()
        command = G1VelocityCommand(0.2, 0.1, 0.3, sequence_id=4)
        transport.command_velocity(command)
        transport._publish_velocity.assert_called_once_with(0.2, 0.1, 0.3)
        with pytest.raises(ValueError, match="increase monotonically"):
            transport.command_velocity(command)

    def test_isaac_rejects_damping_but_soft_stop_zeroes_and_holds(
        self, tmp_path: Path
    ) -> None:
        transport = G1IsaacTransport(_host_profile(tmp_path))
        transport._connected = True
        transport._publish_velocity = MagicMock()
        transport._publish_joint_group = MagicMock()

        with pytest.raises(G1CapabilityError, match="no damping command channel"):
            transport.set_mode(G1ControlMode.EMERGENCY_DAMPING)

        transport.stop()
        transport._publish_velocity.assert_called_once_with(0.0, 0.0, 0.0)
        assert transport._publish_joint_group.call_count == 4
        assert transport._mode is G1ControlMode.STAND

    def test_isaac_stop_publish_failure_is_not_swallowed(self, tmp_path: Path) -> None:
        transport = G1IsaacTransport(_host_profile(tmp_path))
        transport._connected = True
        transport._publish_velocity = MagicMock(
            side_effect=RuntimeError("publisher unavailable")
        )

        with pytest.raises(RuntimeError, match="publisher unavailable"):
            transport.stop()

    def test_state_is_reordered_to_profile_semantic_order(self, tmp_path: Path) -> None:
        profile = _host_profile(tmp_path)
        transport = G1IsaacTransport(profile)
        source_names = tuple(g1_manifest.G1_ALL_JOINTS)
        source_positions = tuple(float(index) for index in range(43))
        joint_message = SimpleNamespace(
            name=source_names,
            position=source_positions,
            velocity=[0.0] * 43,
            effort=[0.0] * 43,
        )
        vector = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
        odom_message = SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=vector(1.0, 2.0, 0.8),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=vector(0.2, 0.0, 0.0),
                    angular=vector(0.0, 0.0, 0.1),
                )
            ),
        )
        transport._joint_state_cb(joint_message)
        transport._odom_cb(odom_message)
        transport._connected = True
        state = transport.read_state()
        source = dict(zip(source_names, source_positions))
        assert state.joint_positions == tuple(source[name] for name in profile.all_joint_names)
        assert state.root_pose.position == (1.0, 2.0, 0.8)
        assert state.root_linear_velocity == (0.2, 0.0, 0.0)

    def test_joint_and_odometry_freshness_are_checked_independently(
        self, tmp_path: Path
    ) -> None:
        transport = G1IsaacTransport(_host_profile(tmp_path), state_timeout=0.1)
        now = time.monotonic()
        transport._connected = True
        transport._last_joint_state_monotonic = now
        transport._last_odom_monotonic = now
        transport.read_state()

        transport._last_odom_monotonic = now - 1.0
        with pytest.raises(RuntimeError, match="odom_age"):
            transport.read_state()
