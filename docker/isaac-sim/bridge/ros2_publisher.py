#!/usr/bin/env python3
"""ROS2 publisher process — runs under system Python 3.12.

Reads Isaac Sim state from shared files and publishes ROS2 topics.
Subscribes to velocity/joint commands and writes them back for the selected
robot runtime.  Go2 keeps its binary protocol; G1 uses a versioned JSON
protocol with independent command timestamps.

Topic names match the MuJoCo bridge exactly for drop-in compatibility.
"""
import os
import sys
import math
import time
import struct
import logging
import numpy as np

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
logger = logging.getLogger("ros2_pub")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, Image, Joy, JointState
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, Quaternion, Vector3
from std_msgs.msg import Float32, Float64MultiArray, Header
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time

_STATE_DIR = os.environ.get("ISAAC_STATE_DIR", "/tmp/isaac_state")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from g1_file_protocol import (
    COMMAND_FILE as _G1_COMMAND_FILE,
    SCHEMA_VERSION as _G1_SCHEMA_VERSION,
    STATE_FILE as _G1_STATE_FILE,
    atomic_write_json as _atomic_write_json,
    new_command_document as _new_g1_command_document,
    read_json as _read_json,
    update_command_channel as _update_g1_command_channel,
)
from g1_manifest import (
    G1_ALL_JOINTS,
    G1_BODY_JOINTS,
    G1_LEFT_ARM_JOINTS,
    G1_LEFT_HAND_ISAAC_TASK_JOINTS,
    G1_RIGHT_ARM_JOINTS,
    G1_RIGHT_HAND_ISAAC_TASK_JOINTS,
    G1_ROBOT_TYPE,
    resolve_robot_type,
)

_ROBOT_TYPE = resolve_robot_type(os.environ.get("ISAAC_ROBOT_TYPE", "go2"))
_G1_STANDING_HEIGHT = float(os.environ.get("G1_STANDING_HEIGHT", "0.8"))
_G1_LEGACY_ARM_SIDE = os.environ.get("G1_LEGACY_ARM_SIDE", "right").strip().lower()
if _G1_LEGACY_ARM_SIDE not in {"left", "right"}:
    raise ValueError("G1_LEGACY_ARM_SIDE must be 'left' or 'right'")

# Go2 joint names (12 DOF)
_GO2_JOINTS = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

# Names expected by the existing six-axis IsaacSimArmProxy.  In G1 mode this
# compatibility surface maps to the first six joints of the explicitly chosen
# arm; the seventh G1 wrist-yaw target is held at its current position.
_LEGACY_ARM_JOINTS = [
    "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3",
]


def _reliable_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class IsaacROS2Bridge(Node):
    """ROS2 node that reads Isaac Sim state files and publishes topics."""

    def __init__(self) -> None:
        super().__init__("isaac_sim_bridge")

        reliable = _reliable_qos()
        self._last_odom = None
        self._latest_g1_state = None
        self._g1_command = _read_json(
            os.path.join(_STATE_DIR, _G1_COMMAND_FILE)
        ) or _new_g1_command_document()

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, "/state_estimation", reliable)
        self._tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._joy_pub = self.create_publisher(Joy, "/joy", reliable)
        self._speed_pub = self.create_publisher(Float32, "/speed", 10)
        self._scan_pub = self.create_publisher(PointCloud2, "/registered_scan", 10)
        self._rgb_pub = self.create_publisher(Image, "/camera/image", 10)
        self._depth_pub = self.create_publisher(Image, "/camera/depth", 10)

        # Subscribers
        self.create_subscription(Twist, "/cmd_vel_nav", self._cmd_vel_nav_cb, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self._cmd_vel_cb, 10)

        if _ROBOT_TYPE == G1_ROBOT_TYPE:
            self._g1_body_pub = self.create_publisher(
                JointState, "/g1/body/joint_states", reliable
            )
            self._g1_left_arm_pub = self.create_publisher(
                JointState, "/g1/left_arm/joint_states", reliable
            )
            self._g1_right_arm_pub = self.create_publisher(
                JointState, "/g1/right_arm/joint_states", reliable
            )
            self._g1_left_hand_pub = self.create_publisher(
                JointState, "/g1/left_hand/joint_states", reliable
            )
            self._g1_right_hand_pub = self.create_publisher(
                JointState, "/g1/right_hand/joint_states", reliable
            )
            self._legacy_arm_pub = self.create_publisher(
                JointState, "/arm/joint_states", reliable
            )
            self.create_subscription(
                Float64MultiArray,
                "/g1/body/joint_commands",
                lambda msg: self._g1_joint_command_cb("body", msg),
                reliable,
            )
            self.create_subscription(
                Float64MultiArray,
                "/g1/left_arm/joint_commands",
                lambda msg: self._g1_joint_command_cb("left_arm", msg),
                reliable,
            )
            self.create_subscription(
                Float64MultiArray,
                "/g1/right_arm/joint_commands",
                lambda msg: self._g1_joint_command_cb("right_arm", msg),
                reliable,
            )
            self.create_subscription(
                Float64MultiArray,
                "/g1/left_hand/joint_commands",
                lambda msg: self._g1_joint_command_cb("left_hand", msg),
                reliable,
            )
            self.create_subscription(
                Float64MultiArray,
                "/g1/right_hand/joint_commands",
                lambda msg: self._g1_joint_command_cb("right_hand", msg),
                reliable,
            )
            self.create_subscription(
                Float64MultiArray,
                "/arm/joint_commands",
                self._legacy_arm_command_cb,
                reliable,
            )

        # Timers
        self.create_timer(1.0 / 50, self._publish_odom)
        self.create_timer(1.0 / 50, self._publish_tf)
        self.create_timer(1.0 / 50, self._publish_joints)
        self.create_timer(0.5, self._publish_joy)
        self.create_timer(0.5, self._publish_speed)

        self.get_logger().info(
            "IsaacROS2Bridge started (robot_type=%s)" % _ROBOT_TYPE
        )

    def _make_header(self, frame_id: str = "map") -> Header:
        now = self.get_clock().now().to_msg()
        h = Header()
        h.stamp = now
        h.frame_id = frame_id
        return h

    def _read_odom(self) -> tuple | None:
        """Read odom state: (x,y,z, qx,qy,qz,qw, vx,vy,vz, wx,wy,wz)."""
        if _ROBOT_TYPE == G1_ROBOT_TYPE:
            state = self._read_g1_state()
            odom = state.get("odom") if state else None
            if isinstance(odom, list) and len(odom) == 13:
                try:
                    return tuple(float(value) for value in odom)
                except (TypeError, ValueError):
                    return None
            return None
        path = os.path.join(_STATE_DIR, "odom.bin")
        try:
            with open(path, "rb") as f:
                data = f.read(52)  # 13 * 4 bytes
            if len(data) == 52:
                return struct.unpack("13f", data)
        except (FileNotFoundError, OSError):
            pass
        return None

    def _read_joints(self) -> list[float] | None:
        """Read joint positions."""
        path = os.path.join(_STATE_DIR, "joints.bin")
        try:
            with open(path, "rb") as f:
                data = f.read()
            n = len(data) // 4
            if n >= 12:
                return list(struct.unpack(f"{n}f", data))
        except (FileNotFoundError, OSError):
            pass
        return None

    def _read_g1_state(self) -> dict | None:
        state = _read_json(os.path.join(_STATE_DIR, _G1_STATE_FILE))
        if not state:
            self._latest_g1_state = None
            return None
        if (
            state.get("schema_version") != _G1_SCHEMA_VERSION
            or state.get("robot_type") != G1_ROBOT_TYPE
            or not state.get("ready")
        ):
            self._latest_g1_state = None
            return None
        try:
            body = state["body"]
            left = state["left_hand"]
            right = state["right_hand"]
            if body["names"] != list(G1_BODY_JOINTS):
                self._latest_g1_state = None
                return None
            if left["names"] != list(G1_LEFT_HAND_ISAAC_TASK_JOINTS):
                self._latest_g1_state = None
                return None
            if right["names"] != list(G1_RIGHT_HAND_ISAAC_TASK_JOINTS):
                self._latest_g1_state = None
                return None
            if not (
                len(body["position"]) == 29
                and len(left["position"]) == 7
                and len(right["position"]) == 7
            ):
                self._latest_g1_state = None
                return None
        except (KeyError, TypeError):
            self._latest_g1_state = None
            return None
        self._latest_g1_state = state
        return state

    def _write_cmd_vel(self, vx: float, vy: float, vyaw: float) -> None:
        """Write velocity command for physics process."""
        if _ROBOT_TYPE == G1_ROBOT_TYPE:
            self._write_g1_command(
                "base", [vx, vy, vyaw, _G1_STANDING_HEIGHT], ttl_ms=500
            )
            return
        path = os.path.join(_STATE_DIR, "cmd_vel.bin")
        tmp = path + ".tmp"
        with open(tmp, "wb") as f:
            f.write(struct.pack("fff", vx, vy, vyaw))
        os.replace(tmp, path)

    def _write_g1_command(
        self, channel: str, values: list[float], *, ttl_ms: int = 1000
    ) -> None:
        try:
            self._g1_command = _update_g1_command_channel(
                self._g1_command, channel, values, ttl_ms=ttl_ms
            )
            _atomic_write_json(
                os.path.join(_STATE_DIR, _G1_COMMAND_FILE), self._g1_command
            )
        except (OSError, TypeError, ValueError) as exc:
            self.get_logger().error(
                "Rejected G1 %s command: %s" % (channel, exc)
            )

    # -- Subscribers --

    def _cmd_vel_nav_cb(self, msg: Twist) -> None:
        self._write_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)
        self.get_logger().info("cmd_vel_nav: vx=%.2f vy=%.2f vyaw=%.2f" % (msg.linear.x, msg.linear.y, msg.angular.z), throttle_duration_sec=2.0)

    def _cmd_vel_cb(self, msg: TwistStamped) -> None:
        self._write_cmd_vel(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def _g1_joint_command_cb(self, channel: str, msg: Float64MultiArray) -> None:
        self._write_g1_command(channel, list(msg.data), ttl_ms=2000)

    def _legacy_arm_command_cb(self, msg: Float64MultiArray) -> None:
        values = [float(value) for value in msg.data]
        if len(values) != 6:
            self.get_logger().error(
                "Legacy /arm/joint_commands requires 6 values; got %d" % len(values)
            )
            return
        state = self._latest_g1_state or self._read_g1_state()
        seventh = 0.0
        if state:
            body = state["body"]["position"]
            seventh = float(body[21] if _G1_LEGACY_ARM_SIDE == "left" else body[28])
        self._write_g1_command(
            f"{_G1_LEGACY_ARM_SIDE}_arm", values + [seventh], ttl_ms=2000
        )

    # -- Publishers --

    def _publish_odom(self) -> None:
        odom = self._read_odom()
        if odom is None:
            if _ROBOT_TYPE == G1_ROBOT_TYPE:
                # Do not keep publishing a fresh TF header around a stale G1
                # pose after the DDS bridge has revoked readiness.
                self._last_odom = None
            return
        self._last_odom = odom
        x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz = odom

        msg = Odometry()
        msg.header = self._make_header("map")
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)
        msg.twist.twist.angular = Vector3(x=wx, y=wy, z=wz)
        self._odom_pub.publish(msg)

    def _publish_tf(self) -> None:
        if self._last_odom is None:
            return
        x, y, z, qx, qy, qz, qw = self._last_odom[:7]

        # map -> sensor (nav stack convention)
        tf = TransformStamped()
        tf.header = self._make_header("map")
        tf.child_frame_id = "sensor"
        tf.transform.translation = Vector3(x=x + 0.3, y=y, z=z + 0.2)  # sensor offset
        tf.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # map -> vehicle
        tf2 = TransformStamped()
        tf2.header = self._make_header("map")
        tf2.child_frame_id = "vehicle"
        tf2.transform.translation = Vector3(x=x, y=y, z=z)
        tf2.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        msg = TFMessage()
        msg.transforms = [tf, tf2]
        self._tf_pub.publish(msg)

    def _publish_joints(self) -> None:
        if _ROBOT_TYPE == G1_ROBOT_TYPE:
            self._publish_g1_joints()
            return
        joints = self._read_joints()
        if joints is None:
            return
        msg = JointState()
        msg.header = self._make_header("base_link")
        msg.name = _GO2_JOINTS[:len(joints)]
        msg.position = [float(j) for j in joints[:12]]
        self._joint_pub.publish(msg)

    def _joint_state_message(
        self,
        names: list[str],
        positions: list[float],
        velocities: list[float] | None = None,
        efforts: list[float] | None = None,
    ) -> JointState:
        msg = JointState()
        msg.header = self._make_header("base_link")
        msg.name = list(names)
        msg.position = [float(value) for value in positions]
        if velocities is not None:
            msg.velocity = [float(value) for value in velocities]
        if efforts is not None:
            msg.effort = [float(value) for value in efforts]
        return msg

    def _publish_g1_joints(self) -> None:
        state = self._read_g1_state()
        if state is None:
            return
        body = state["body"]
        left_hand = state["left_hand"]
        right_hand = state["right_hand"]
        body_q = list(body["position"])
        body_dq = list(body.get("velocity", []))
        body_tau = list(body.get("effort", []))
        left_q = list(left_hand["position"])
        right_q = list(right_hand["position"])
        all_q = body_q + left_q + right_q
        all_dq = body_dq + list(left_hand.get("velocity", [])) + list(
            right_hand.get("velocity", [])
        )
        all_tau = body_tau + list(left_hand.get("effort", [])) + list(
            right_hand.get("effort", [])
        )
        self._joint_pub.publish(
            self._joint_state_message(
                list(G1_ALL_JOINTS), all_q, all_dq, all_tau
            )
        )
        self._g1_body_pub.publish(
            self._joint_state_message(list(G1_BODY_JOINTS), body_q, body_dq, body_tau)
        )
        self._g1_left_arm_pub.publish(
            self._joint_state_message(
                list(G1_LEFT_ARM_JOINTS), body_q[15:22], body_dq[15:22], body_tau[15:22]
            )
        )
        self._g1_right_arm_pub.publish(
            self._joint_state_message(
                list(G1_RIGHT_ARM_JOINTS), body_q[22:29], body_dq[22:29], body_tau[22:29]
            )
        )
        self._g1_left_hand_pub.publish(
            self._joint_state_message(
                list(G1_LEFT_HAND_ISAAC_TASK_JOINTS),
                left_q,
                list(left_hand.get("velocity", [])),
                list(left_hand.get("effort", [])),
            )
        )
        self._g1_right_hand_pub.publish(
            self._joint_state_message(
                list(G1_RIGHT_HAND_ISAAC_TASK_JOINTS),
                right_q,
                list(right_hand.get("velocity", [])),
                list(right_hand.get("effort", [])),
            )
        )
        arm_q = body_q[15:22] if _G1_LEGACY_ARM_SIDE == "left" else body_q[22:29]
        self._legacy_arm_pub.publish(
            self._joint_state_message(_LEGACY_ARM_JOINTS, arm_q[:6])
        )

    def _publish_joy(self) -> None:
        msg = Joy()
        msg.header = self._make_header("base_link")
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 11
        msg.buttons[4] = 1  # autonomous mode for pathFollower
        self._joy_pub.publish(msg)

    def _publish_speed(self) -> None:
        if self._last_odom is None:
            return
        vx, vy = self._last_odom[7], self._last_odom[8]
        msg = Float32()
        msg.data = math.sqrt(vx * vx + vy * vy)
        self._speed_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = IsaacROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
