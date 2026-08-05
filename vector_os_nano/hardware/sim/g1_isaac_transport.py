# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""ROS 2 transport for the G1 Isaac Sim 29-DoF + dual Dex3 plugin.

The container owns Isaac Lab and Unitree DDS.  This host-side transport only
uses stable ROS topics, so it does not require Isaac Sim or ``unitree_sdk2py``
in the Vector process.  All vectors are reordered by joint *name* at the
boundary; no Isaac/USD numeric index leaks into the hardware abstraction.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Mapping, Sequence

from vector_os_nano.core.types import Pose3D
from vector_os_nano.hardware.g1.profile import (
    BODY_DDS_JOINTS,
    LEFT_ARM_JOINTS,
    LEFT_HAND_SEMANTIC_JOINTS,
    RIGHT_ARM_JOINTS,
    RIGHT_HAND_SEMANTIC_JOINTS,
    G1Profile,
)
from vector_os_nano.hardware.g1.state import (
    G1ControlMode,
    G1JointCommand,
    G1State,
    G1VelocityCommand,
)
from vector_os_nano.hardware.g1.transport import (
    G1CapabilityError,
    G1TransportCapabilities,
)


logger = logging.getLogger(__name__)

# Current unitreerobotics/unitree_sim_isaaclab Dex3 observation/action order.
# It differs from the profile's HAL semantic order for index/middle, hence
# the explicit name-based conversion in both command and state directions.
_ISAAC_LEFT_HAND_JOINTS: tuple[str, ...] = (
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
)
_ISAAC_RIGHT_HAND_JOINTS: tuple[str, ...] = (
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
)
_ISAAC_STATE_JOINTS = BODY_DDS_JOINTS + _ISAAC_LEFT_HAND_JOINTS + _ISAAC_RIGHT_HAND_JOINTS

_COMMAND_TOPICS: Mapping[str, str] = {
    "body": "/g1/body/joint_commands",
    "left_arm": "/g1/left_arm/joint_commands",
    "right_arm": "/g1/right_arm/joint_commands",
    "left_hand": "/g1/left_hand/joint_commands",
    "right_hand": "/g1/right_hand/joint_commands",
}

_ISAAC_COMMAND_NAMES: Mapping[str, tuple[str, ...]] = {
    "body": BODY_DDS_JOINTS,
    "left_arm": LEFT_ARM_JOINTS,
    "right_arm": RIGHT_ARM_JOINTS,
    "left_hand": _ISAAC_LEFT_HAND_JOINTS,
    "right_hand": _ISAAC_RIGHT_HAND_JOINTS,
}


def _reorder_named(
    values: Sequence[float],
    source_names: Sequence[str],
    target_names: Sequence[str],
) -> tuple[float, ...]:
    if len(values) != len(source_names):
        raise ValueError("joint values and source names must have equal length")
    if len(set(source_names)) != len(source_names):
        raise ValueError("source joint names contain duplicates")
    lookup = {name: float(value) for name, value in zip(source_names, values)}
    missing = [name for name in target_names if name not in lookup]
    if missing:
        raise ValueError(f"joint state/command is missing names: {missing}")
    result = tuple(lookup[name] for name in target_names)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("joint state/command contains a non-finite value")
    return result


class G1IsaacTransport:
    """Implement ``G1Transport`` over the container's ROS 2 bridge."""

    def __init__(
        self,
        profile: G1Profile,
        *,
        connect_timeout: float = 10.0,
        state_timeout: float = 1.0,
        node_name: str = "g1_isaac_transport",
    ) -> None:
        if connect_timeout <= 0 or not math.isfinite(float(connect_timeout)):
            raise ValueError("connect_timeout must be finite and positive")
        if state_timeout <= 0 or not math.isfinite(float(state_timeout)):
            raise ValueError("state_timeout must be finite and positive")
        self._profile = profile
        self._connect_timeout = float(connect_timeout)
        self._state_timeout = float(state_timeout)
        self._node_name = str(node_name)
        self._connected = False
        self._node: Any = None
        self._spin_thread: threading.Thread | None = None
        self._velocity_publisher: Any = None
        self._joint_publishers: dict[str, Any] = {}
        self._lock = threading.RLock()
        self._ready_event = threading.Event()
        self._joint_state_received = False
        self._odom_received = False
        self._state_sequence = 0
        self._last_joint_state_monotonic = 0.0
        self._last_odom_monotonic = 0.0
        self._last_velocity_sequence = -1
        self._last_joint_sequence: dict[str, int] = {}
        self._mode = G1ControlMode.PASSIVE
        self._root_pose = Pose3D()
        self._root_linear_velocity = (0.0, 0.0, 0.0)
        self._root_angular_velocity = (0.0, 0.0, 0.0)
        self._state = G1State.zero(profile.all_joint_names)

    @property
    def name(self) -> str:
        return "isaac_g1_29dof_dex3"

    @property
    def profile(self) -> G1Profile:
        return self._profile

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def capabilities(self) -> G1TransportCapabilities:
        # The official Wholebody policy drives legs/waist and accepts velocity;
        # direct LowCmd joint targets are consumed for arms only.  Therefore
        # "body" is intentionally not advertised as a directly controlled group.
        return G1TransportCapabilities(
            locomotion=True,
            holonomic=True,
            lidar=False,
            emergency_damping=False,
            joint_groups=frozenset(
                {"left_arm", "right_arm", "left_hand", "right_hand"}
            ),
        )

    @property
    def supports_locomotion(self) -> bool:
        return True

    @property
    def supports_joint_groups(self) -> bool:
        return True

    @property
    def supports_root_state(self) -> bool:
        return True

    @property
    def supports_lidar(self) -> bool:
        return False

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._ready_event.clear()
            self._joint_state_received = False
            self._odom_received = False
            self._last_joint_state_monotonic = 0.0
            self._last_odom_monotonic = 0.0
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64MultiArray

            if not rclpy.ok():
                rclpy.init()
            node = Node(self._node_name)
            velocity_publisher = node.create_publisher(Twist, "/cmd_vel_nav", 10)
            joint_publishers = {
                group: node.create_publisher(Float64MultiArray, topic, 10)
                for group, topic in _COMMAND_TOPICS.items()
            }
            node.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
            node.create_subscription(Odometry, "/state_estimation", self._odom_cb, 10)
            with self._lock:
                self._node = node
                self._velocity_publisher = velocity_publisher
                self._joint_publishers = joint_publishers
            self._spin_thread = threading.Thread(
                target=self._spin,
                args=(rclpy,),
                name="g1_isaac_ros2",
                daemon=True,
            )
            self._spin_thread.start()
            if not self._ready_event.wait(self._connect_timeout):
                raise ConnectionError(
                    "G1 Isaac bridge did not provide a complete 43-joint state and "
                    f"root odometry within {self._connect_timeout:.1f}s"
                )
            with self._lock:
                self._connected = True
                self._mode = G1ControlMode.STAND
            logger.info("G1 Isaac transport connected")
        except ImportError as exc:
            self.disconnect()
            raise ConnectionError(
                "G1IsaacTransport requires ROS2 Jazzy/rclpy to be sourced"
            ) from exc
        except Exception:
            self.disconnect()
            raise

    def _spin(self, rclpy: Any) -> None:
        try:
            rclpy.spin(self._node)
        except Exception as exc:  # noqa: BLE001
            logger.error("G1 Isaac ROS spin stopped: %s", exc)

    def disconnect(self) -> None:
        with self._lock:
            node = self._node
            thread = self._spin_thread
            self._node = None
            self._spin_thread = None
            self._velocity_publisher = None
            self._joint_publishers = {}
            self._connected = False
            self._mode = G1ControlMode.PASSIVE
            self._ready_event.clear()
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def _maybe_mark_ready_locked(self) -> None:
        if self._joint_state_received and self._odom_received:
            self._ready_event.set()

    @staticmethod
    def _message_vector(
        values: Sequence[float],
        names: Sequence[str],
        target_names: Sequence[str],
        *,
        default: float = 0.0,
    ) -> tuple[float, ...]:
        if not values:
            return (float(default),) * len(target_names)
        return _reorder_named(values, names, target_names)

    def _joint_state_cb(self, message: Any) -> None:
        try:
            names = tuple(str(name) for name in message.name)
            positions = _reorder_named(
                message.position, names, self._profile.all_joint_names
            )
            velocities = self._message_vector(
                message.velocity, names, self._profile.all_joint_names
            )
            efforts = self._message_vector(
                message.effort, names, self._profile.all_joint_names
            )
            now = time.monotonic()
            with self._lock:
                self._state_sequence += 1
                self._last_joint_state_monotonic = now
                self._joint_state_received = True
                self._state = G1State(
                    timestamp=now,
                    joint_names=self._profile.all_joint_names,
                    joint_positions=positions,
                    joint_velocities=velocities,
                    joint_efforts=efforts,
                    root_pose=self._root_pose,
                    root_linear_velocity=self._root_linear_velocity,
                    root_angular_velocity=self._root_angular_velocity,
                    control_mode=self._mode,
                    sequence_id=self._state_sequence,
                )
                self._maybe_mark_ready_locked()
        except Exception as exc:  # noqa: BLE001
            logger.warning("Rejected malformed G1 /joint_states: %s", exc)

    def _odom_cb(self, message: Any) -> None:
        try:
            pose = message.pose.pose
            twist = message.twist.twist
            root_pose = Pose3D(
                x=float(pose.position.x),
                y=float(pose.position.y),
                z=float(pose.position.z),
                qx=float(pose.orientation.x),
                qy=float(pose.orientation.y),
                qz=float(pose.orientation.z),
                qw=float(pose.orientation.w),
            )
            linear = (
                float(twist.linear.x),
                float(twist.linear.y),
                float(twist.linear.z),
            )
            angular = (
                float(twist.angular.x),
                float(twist.angular.y),
                float(twist.angular.z),
            )
            if not all(
                math.isfinite(value)
                for value in (
                    *root_pose.position,
                    *root_pose.orientation,
                    *linear,
                    *angular,
                )
            ):
                raise ValueError("odometry contains a non-finite value")
            now = time.monotonic()
            with self._lock:
                self._root_pose = root_pose
                self._root_linear_velocity = linear
                self._root_angular_velocity = angular
                self._odom_received = True
                self._last_odom_monotonic = now
                self._state = G1State(
                    **{
                        **self._state.__dict__,
                        "timestamp": now,
                        "root_pose": root_pose,
                        "root_linear_velocity": linear,
                        "root_angular_velocity": angular,
                        "control_mode": self._mode,
                    }
                )
                self._maybe_mark_ready_locked()
        except Exception as exc:  # noqa: BLE001
            logger.warning("Rejected malformed G1 /state_estimation: %s", exc)

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("G1 Isaac transport is not connected")

    @staticmethod
    def _require_fresh_command(timestamp: float, ttl: float) -> None:
        age = time.monotonic() - float(timestamp)
        if age < -0.05 or age > float(ttl):
            raise ValueError(f"refusing stale G1 command (age={age:.3f}s, ttl={ttl:.3f}s)")

    def read_state(self) -> G1State:
        self._require_connected()
        with self._lock:
            now = time.monotonic()
            joint_age = now - self._last_joint_state_monotonic
            odom_age = now - self._last_odom_monotonic
            if joint_age > self._state_timeout or odom_age > self._state_timeout:
                raise RuntimeError(
                    "G1 Isaac state is stale "
                    f"(joint_age={joint_age:.3f}s, odom_age={odom_age:.3f}s, "
                    f"timeout={self._state_timeout:.3f}s)"
                )
            return self._state

    def command_velocity(self, command: G1VelocityCommand) -> None:
        self._require_connected()
        self._require_fresh_command(command.timestamp, command.ttl)
        if command.sequence_id <= self._last_velocity_sequence:
            raise ValueError("G1 velocity sequence_id must increase monotonically")
        self._publish_velocity(command.vx, command.vy, command.vyaw)
        self._last_velocity_sequence = command.sequence_id

    def _publish_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        from geometry_msgs.msg import Twist

        message = Twist()
        message.linear.x = float(vx)
        message.linear.y = float(vy)
        message.angular.z = float(vyaw)
        self._velocity_publisher.publish(message)

    def command_joints(self, command: G1JointCommand) -> None:
        self._require_connected()
        self._require_fresh_command(command.timestamp, command.ttl)
        if command.group not in _COMMAND_TOPICS:
            raise ValueError(f"unknown G1 Isaac joint group {command.group!r}")
        if not self.capabilities.supports_group(command.group):
            raise G1CapabilityError(
                f"official G1 Wholebody runtime does not directly control {command.group!r}"
            )
        previous = self._last_joint_sequence.get(command.group, -1)
        if command.sequence_id <= previous:
            raise ValueError(
                f"G1 {command.group} sequence_id must increase monotonically"
            )
        expected = self._profile.joint_group(command.group)
        if tuple(command.joint_names) != expected:
            raise ValueError(
                f"{command.group} requires exact semantic order {expected}; "
                f"received {command.joint_names}"
            )
        values = _reorder_named(
            command.positions,
            command.joint_names,
            _ISAAC_COMMAND_NAMES[command.group],
        )
        self._publish_joint_group(command.group, values)
        self._last_joint_sequence[command.group] = command.sequence_id

    def _publish_joint_group(self, group: str, values: Sequence[float]) -> None:
        from std_msgs.msg import Float64MultiArray

        message = Float64MultiArray()
        message.data = [float(value) for value in values]
        self._joint_publishers[group].publish(message)

    def set_mode(self, mode: G1ControlMode) -> None:
        self._require_connected()
        mode = G1ControlMode(mode)
        if mode is G1ControlMode.EMERGENCY_DAMPING:
            raise G1CapabilityError(
                "The pinned Isaac/Unitree bridge has no damping command channel; "
                "use emergency_stop() for a software command latch plus zero/hold"
            )
        if mode in {
            G1ControlMode.PASSIVE,
            G1ControlMode.STAND,
            G1ControlMode.MANIPULATION,
        }:
            self._publish_velocity(0.0, 0.0, 0.0)
        with self._lock:
            self._mode = mode

    def stop(self) -> None:
        self._require_connected()
        # Stop locomotion and replace every arm/hand target with the latest
        # observed pose.  The pinned Unitree task has no physical damping
        # channel, so this is an explicit zero/hold request and exceptions must
        # propagate to the emergency-stop caller.
        self._publish_velocity(0.0, 0.0, 0.0)
        with self._lock:
            state = self._state
        for group in ("left_arm", "right_arm", "left_hand", "right_hand"):
            semantic_names = self._profile.joint_group(group)
            positions = state.positions_for(semantic_names)
            values = _reorder_named(
                positions,
                semantic_names,
                _ISAAC_COMMAND_NAMES[group],
            )
            self._publish_joint_group(group, values)
        with self._lock:
            self._mode = G1ControlMode.STAND

    def get_lidar_scan(self) -> Any:
        self._require_connected()
        return None


__all__ = ["G1IsaacTransport"]
