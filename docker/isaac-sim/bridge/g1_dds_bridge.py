#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics
"""Bridge Vector's file/ROS command contract to Unitree's official G1 DDS.

The official simulator deliberately uses the same DDS topics as a physical G1.
This process is launched with a loopback-only CycloneDDS configuration so those
commands cannot escape the container onto a real-robot network.
"""
from __future__ import annotations

import json
import logging
import math
import os
import signal
import threading
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

from g1_file_protocol import (
    COMMAND_FILE,
    MANIFEST_FILE,
    SCHEMA_VERSION,
    STATE_FILE,
    atomic_write_json,
    fresh_channel_values,
    monotonic_ns,
    read_json,
)
from g1_manifest import (
    G1_ALL_JOINTS,
    G1_BODY_JOINTS,
    G1_LEFT_ARM_BODY_INDICES,
    G1_LEFT_HAND_ISAAC_TASK_JOINTS,
    G1_RIGHT_ARM_BODY_INDICES,
    G1_RIGHT_HAND_ISAAC_TASK_JOINTS,
    G1_ROBOT_TYPE,
)


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
)
logger = logging.getLogger("g1_dds_bridge")

_DEFAULT_HEIGHT = 0.8
_PUBLISH_HZ = 50.0
_REQUIRED_STATE_STREAMS = ("body", "left_hand", "right_hand", "odom")


def state_streams_fresh(
    last_updates_ns: Mapping[str, int],
    *,
    timestamp_ns: int,
    timeout_ns: int,
) -> bool:
    """Return true only when every required DDS state stream is recent."""
    now_ns = int(timestamp_ns)
    limit_ns = int(timeout_ns)
    if limit_ns <= 0:
        raise ValueError("G1 DDS state timeout must be positive")
    for stream in _REQUIRED_STATE_STREAMS:
        received_ns = int(last_updates_ns.get(stream, 0))
        age_ns = now_ns - received_ns
        if received_ns <= 0 or age_ns < 0 or age_ns > limit_ns:
            return False
    return True


def _first_vector(value: Any, minimum: int) -> list[float] | None:
    """Normalize Isaac Lab's single-env nested vector representation."""
    if isinstance(value, list) and value and isinstance(value[0], list):
        value = value[0]
    if not isinstance(value, list) or len(value) < minimum:
        return None
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(item) for item in result):
        return None
    return result


def extract_odom(sim_state_message: str | Mapping[str, Any]) -> list[float] | None:
    """Extract Vector's 13-float odometry vector from ``rt/sim_state``.

    Isaac Lab stores root quaternions as ``w, x, y, z``; Vector's ROS bridge
    stores/publishes them as ``x, y, z, w``.
    """
    try:
        outer = (
            json.loads(sim_state_message)
            if isinstance(sim_state_message, str)
            else dict(sim_state_message)
        )
        state: Any = outer.get("init_state", outer)
        if isinstance(state, str):
            state = json.loads(state)
        articulation = state.get("articulation", state.get("articulations", {}))
        robot = articulation.get("robot", {})
        pose = _first_vector(robot.get("root_pose"), 7)
        velocity = _first_vector(
            robot.get("root_velocity", robot.get("root_vel")), 6
        )
        if pose is None or velocity is None:
            return None
        x, y, z, qw, qx, qy, qz = pose[:7]
        vx, vy, vz, wx, wy, wz = velocity[:6]
        return [x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz]
    except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
        return None


def unitree_base_command(values: Sequence[float] | None) -> list[float]:
    """Convert REP-103 base velocity to the official policy command convention."""
    if values is None:
        return [0.0, 0.0, 0.0, _DEFAULT_HEIGHT]
    vx, vy, yaw_rate, height = (float(value) for value in values)
    # Official policy limits from send_commands_keyboard.py.  Clamp again here
    # as a final safety boundary even though the ROS side validates inputs.
    vx = min(1.0, max(-0.6, vx))
    vy = min(0.5, max(-0.5, vy))
    yaw_rate = min(1.57, max(-1.57, yaw_rate))
    height = min(0.8, max(0.3, height))
    return [vx, -vy, -yaw_rate, height]


def merge_body_targets(
    current: Sequence[float],
    *,
    body: Sequence[float] | None = None,
    left_arm: Sequence[float] | None = None,
    right_arm: Sequence[float] | None = None,
) -> list[float]:
    """Merge full-body and side-specific commands by canonical joint index."""
    if len(current) != 29:
        raise ValueError(f"Expected 29 current body positions, got {len(current)}")
    result = [float(value) for value in (body if body is not None else current)]
    if len(result) != 29:
        raise ValueError(f"Expected 29 body target positions, got {len(result)}")
    if left_arm is not None:
        if len(left_arm) != 7:
            raise ValueError("Expected 7 left arm positions")
        for index, value in zip(G1_LEFT_ARM_BODY_INDICES, left_arm):
            result[index] = float(value)
    if right_arm is not None:
        if len(right_arm) != 7:
            raise ValueError("Expected 7 right arm positions")
        for index, value in zip(G1_RIGHT_ARM_BODY_INDICES, right_arm):
            result[index] = float(value)
    return result


class G1DDSBridge:
    """DDS adapter with independently timestamped base/arm/hand channels."""

    def __init__(self, state_dir: Path) -> None:
        self._state_dir = state_dir
        self._command_path = state_dir / COMMAND_FILE
        self._state_path = state_dir / STATE_FILE
        self._ready_path = state_dir / "ready"
        self._lock = threading.Lock()
        self._running = True
        self._sequence = 0
        self._last_command_sequence: dict[str, int] = {}
        self._body_received = False
        self._left_hand_received = False
        self._right_hand_received = False
        self._ready_written = False
        timeout_sec = float(os.environ.get("G1_DDS_STATE_TIMEOUT_SEC", "1.0"))
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError("G1_DDS_STATE_TIMEOUT_SEC must be finite and positive")
        self._state_timeout_ns = int(timeout_sec * 1_000_000_000)
        self._last_state_update_ns = {
            stream: 0 for stream in _REQUIRED_STATE_STREAMS
        }
        self._body_q = [0.0] * 29
        self._body_dq = [0.0] * 29
        self._body_tau = [0.0] * 29
        self._body_target: list[float] | None = None
        self._left_hand_q = [0.0] * 7
        self._left_hand_dq = [0.0] * 7
        self._left_hand_tau = [0.0] * 7
        self._right_hand_q = [0.0] * 7
        self._right_hand_dq = [0.0] * 7
        self._right_hand_tau = [0.0] * 7
        self._left_hand_target: list[float] | None = None
        self._right_hand_target: list[float] | None = None
        self._imu: dict[str, list[float]] = {}
        self._odom: list[float] | None = None
        self._setup_dds()

    def _setup_dds(self) -> None:
        domain = int(os.environ.get("UNITREE_DDS_DOMAIN_ID", "1"))
        if domain != 1:
            raise RuntimeError(
                "Pinned unitree_sim_isaaclab initializes DDS domain 1 internally; "
                "UNITREE_DDS_DOMAIN_ID must remain 1"
            )
        interface = os.environ.get("UNITREE_DDS_INTERFACE", "lo").strip()
        if interface != "lo":
            raise RuntimeError(
                "The G1 simulation DDS interface must remain 'lo'; Unitree's "
                "simulation topics are identical to physical-robot topics"
            )
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import (
            unitree_hg_msg_dds__HandCmd_,
            unitree_hg_msg_dds__LowCmd_,
        )
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
            HandCmd_,
            HandState_,
            LowCmd_,
            LowState_,
        )
        from unitree_sdk2py.utils.crc import CRC

        # The SDK supplies its own inline CycloneDDS XML, which overrides the
        # CYCLONEDDS_URI environment variable.  Passing ``lo`` is therefore the
        # actual containment boundary; the XML remains defense in depth.
        ChannelFactoryInitialize(domain, interface)
        self._String = String_
        self._crc = CRC()
        self._low_command = unitree_hg_msg_dds__LowCmd_()
        self._left_hand_command = unitree_hg_msg_dds__HandCmd_()
        self._right_hand_command = unitree_hg_msg_dds__HandCmd_()

        self._run_publisher = ChannelPublisher("rt/run_command/cmd", String_)
        self._low_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._left_hand_publisher = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
        self._right_hand_publisher = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
        for publisher in (
            self._run_publisher,
            self._low_publisher,
            self._left_hand_publisher,
            self._right_hand_publisher,
        ):
            publisher.Init()

        self._low_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._left_hand_subscriber = ChannelSubscriber(
            "rt/dex3/left/state", HandState_
        )
        self._right_hand_subscriber = ChannelSubscriber(
            "rt/dex3/right/state", HandState_
        )
        self._sim_state_subscriber = ChannelSubscriber("rt/sim_state", String_)
        self._low_subscriber.Init(self._on_low_state, 32)
        self._left_hand_subscriber.Init(self._on_left_hand_state, 32)
        self._right_hand_subscriber.Init(self._on_right_hand_state, 32)
        self._sim_state_subscriber.Init(self._on_sim_state, 4)
        logger.info(
            "G1 DDS bridge initialized on isolated domain %d, interface %s",
            domain,
            interface,
        )

    @staticmethod
    def _motor_vectors(message: Any, count: int) -> tuple[list[float], ...]:
        motors = list(message.motor_state)
        if len(motors) < count:
            raise ValueError(f"DDS state has {len(motors)} motors, expected {count}")
        vectors = (
            [float(motor.q) for motor in motors[:count]],
            [float(motor.dq) for motor in motors[:count]],
            [float(motor.tau_est) for motor in motors[:count]],
        )
        if not all(math.isfinite(value) for vector in vectors for value in vector):
            raise ValueError("DDS motor state contains a non-finite value")
        return vectors

    def _on_low_state(self, message: Any) -> None:
        try:
            q, dq, tau = self._motor_vectors(message, 29)
            imu = message.imu_state
            with self._lock:
                self._body_q, self._body_dq, self._body_tau = q, dq, tau
                if self._body_target is None:
                    self._body_target = list(q)
                self._imu = {
                    "quaternion_xyzw": [float(v) for v in imu.quaternion],
                    "gyroscope": [float(v) for v in imu.gyroscope],
                    "accelerometer": [float(v) for v in imu.accelerometer],
                }
                self._body_received = True
                self._last_state_update_ns["body"] = monotonic_ns()
        except Exception as exc:
            logger.warning("Rejected malformed rt/lowstate: %s", exc)

    def _on_left_hand_state(self, message: Any) -> None:
        try:
            q, dq, tau = self._motor_vectors(message, 7)
            with self._lock:
                self._left_hand_q, self._left_hand_dq, self._left_hand_tau = q, dq, tau
                if self._left_hand_target is None:
                    self._left_hand_target = list(q)
                self._left_hand_received = True
                self._last_state_update_ns["left_hand"] = monotonic_ns()
        except Exception as exc:
            logger.warning("Rejected malformed left Dex3 state: %s", exc)

    def _on_right_hand_state(self, message: Any) -> None:
        try:
            q, dq, tau = self._motor_vectors(message, 7)
            with self._lock:
                self._right_hand_q, self._right_hand_dq, self._right_hand_tau = q, dq, tau
                if self._right_hand_target is None:
                    self._right_hand_target = list(q)
                self._right_hand_received = True
                self._last_state_update_ns["right_hand"] = monotonic_ns()
        except Exception as exc:
            logger.warning("Rejected malformed right Dex3 state: %s", exc)

    def _on_sim_state(self, message: Any) -> None:
        odom = extract_odom(message.data)
        if odom is not None:
            with self._lock:
                self._odom = odom
                self._last_state_update_ns["odom"] = monotonic_ns()

    def _accept_new_joint_commands(
        self, document: Mapping[str, Any] | None, now_ns: int
    ) -> None:
        if not document:
            return
        entries = document.get("channels") or {}
        updates: list[tuple[int, str, list[float]]] = []
        for channel in ("body", "left_arm", "right_arm", "left_hand", "right_hand"):
            entry = entries.get(channel)
            if not isinstance(entry, dict):
                continue
            try:
                sequence = int(entry["sequence"])
            except (KeyError, TypeError, ValueError):
                continue
            if sequence <= self._last_command_sequence.get(channel, -1):
                continue
            # Mark the sequence consumed even if it is already stale: an old
            # command must never become applicable again after a clock/file race.
            self._last_command_sequence[channel] = sequence
            values = fresh_channel_values(document, channel, timestamp_ns=now_ns)
            if values is not None:
                updates.append((sequence, channel, values))

        with self._lock:
            for _, channel, values in sorted(updates):
                if channel == "body" and self._body_target is not None:
                    self._body_target = merge_body_targets(self._body_target, body=values)
                elif channel == "left_arm" and self._body_target is not None:
                    self._body_target = merge_body_targets(
                        self._body_target, left_arm=values
                    )
                elif channel == "right_arm" and self._body_target is not None:
                    self._body_target = merge_body_targets(
                        self._body_target, right_arm=values
                    )
                elif channel == "left_hand" and self._left_hand_target is not None:
                    self._left_hand_target = list(values)
                elif channel == "right_hand" and self._right_hand_target is not None:
                    self._right_hand_target = list(values)

    def _publish_commands(self, document: Mapping[str, Any] | None, now_ns: int) -> None:
        base = fresh_channel_values(document, "base", timestamp_ns=now_ns)
        self._run_publisher.Write(self._String(data=str(unitree_base_command(base))))
        with self._lock:
            body_target = None if self._body_target is None else list(self._body_target)
            left_target = (
                None if self._left_hand_target is None else list(self._left_hand_target)
            )
            right_target = (
                None if self._right_hand_target is None else list(self._right_hand_target)
            )
        if body_target is not None:
            for index, target in enumerate(body_target):
                self._low_command.motor_cmd[index].q = target
            self._low_command.crc = self._crc.Crc(self._low_command)
            self._low_publisher.Write(self._low_command)
        if left_target is not None:
            for index, target in enumerate(left_target):
                self._left_hand_command.motor_cmd[index].q = target
            self._left_hand_publisher.Write(self._left_hand_command)
        if right_target is not None:
            for index, target in enumerate(right_target):
                self._right_hand_command.motor_cmd[index].q = target
            self._right_hand_publisher.Write(self._right_hand_command)

    def _write_state(self) -> None:
        now_ns = monotonic_ns()
        with self._lock:
            self._sequence += 1
            ready = (
                self._body_received
                and self._left_hand_received
                and self._right_hand_received
                and self._odom is not None
                and state_streams_fresh(
                    self._last_state_update_ns,
                    timestamp_ns=now_ns,
                    timeout_ns=self._state_timeout_ns,
                )
            )
            document = {
                "schema_version": SCHEMA_VERSION,
                "robot_type": G1_ROBOT_TYPE,
                "sequence": self._sequence,
                # Timestamp the newest validated snapshot decision, not a claim
                # that each copied DDS vector arrived at this instant.
                "monotonic_ns": now_ns,
                "ready": ready,
                "body": {
                    "names": list(G1_BODY_JOINTS),
                    "position": list(self._body_q),
                    "velocity": list(self._body_dq),
                    "effort": list(self._body_tau),
                },
                "left_hand": {
                    "names": list(G1_LEFT_HAND_ISAAC_TASK_JOINTS),
                    "position": list(self._left_hand_q),
                    "velocity": list(self._left_hand_dq),
                    "effort": list(self._left_hand_tau),
                },
                "right_hand": {
                    "names": list(G1_RIGHT_HAND_ISAAC_TASK_JOINTS),
                    "position": list(self._right_hand_q),
                    "velocity": list(self._right_hand_dq),
                    "effort": list(self._right_hand_tau),
                },
                "joint_names": list(G1_ALL_JOINTS),
                "imu": dict(self._imu),
                "odom": None if self._odom is None else list(self._odom),
            }
        atomic_write_json(self._state_path, document)
        if ready and not self._ready_written:
            self._ready_path.write_text("g1_29dof_dex3\n", encoding="utf-8")
            manifest = read_json(self._state_dir / MANIFEST_FILE) or {}
            manifest["status"] = "ready"
            atomic_write_json(self._state_dir / MANIFEST_FILE, manifest)
            self._ready_written = True
            logger.info(
                "G1 ready: received 29 body + 7 left + 7 right joint states "
                "and root odometry"
            )
        elif not ready and self._ready_written:
            self._ready_path.unlink(missing_ok=True)
            manifest = read_json(self._state_dir / MANIFEST_FILE) or {}
            manifest["status"] = "stale"
            atomic_write_json(self._state_dir / MANIFEST_FILE, manifest)
            self._ready_written = False
            logger.error(
                "G1 state became stale; revoked ready until body, both Dex3 "
                "hands, and root odometry are all live again"
            )

    def run(self) -> None:
        period = 1.0 / _PUBLISH_HZ
        try:
            while self._running:
                started = time.monotonic()
                now_ns = monotonic_ns()
                command = read_json(self._command_path)
                self._accept_new_joint_commands(command, now_ns)
                self._publish_commands(command, now_ns)
                self._write_state()
                remaining = period - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            self._ready_path.unlink(missing_ok=True)
            manifest = read_json(self._state_dir / MANIFEST_FILE) or {}
            manifest["status"] = "stopped"
            atomic_write_json(self._state_dir / MANIFEST_FILE, manifest)

    def stop(self, *_: object) -> None:
        self._running = False


def main() -> None:
    state_dir = Path(os.environ.get("ISAAC_STATE_DIR", "/tmp/isaac_state"))
    state_dir.mkdir(parents=True, exist_ok=True)
    bridge = G1DDSBridge(state_dir)
    signal.signal(signal.SIGTERM, bridge.stop)
    signal.signal(signal.SIGINT, bridge.stop)
    bridge.run()


if __name__ == "__main__":
    main()
