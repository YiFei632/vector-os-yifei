#!/usr/bin/env python3
"""ROS2 adapter for the MolmoSpaces RBY1 simulation.

This node is deliberately a transport adapter: MolmoSpaces remains in its
own process and is reached through the existing TCP bridge, while Vector and
navigation consumers use standard ROS2 topics.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np


def _pose(value: Any) -> tuple[float, float, float]:
    p = np.asarray(value, dtype=float).reshape(-1)
    if p.size >= 3 and p.size < 7:
        return float(p[0]), float(p[1]), float(p[2])
    if p.size < 7:
        raise RuntimeError("RBY1 state has no usable base pose")
    qw, qx, qy, qz = map(float, p[3:7])
    return float(p[0]), float(p[1]), math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


class RBY1MolmoSpacesROS2Bridge:
    def __init__(self, bridge: Any, *, camera: str = "head_camera", frame_id: str = "base_link") -> None:
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import OccupancyGrid, Odometry
            from sensor_msgs.msg import CameraInfo, Image
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
        except ImportError as exc:
            # Ubuntu ROS debs install rclpy under ``/opt/ros/<distro>/local``;
            # Conda's Python does not include that path automatically.
            ros_paths = []
            for distro in Path("/opt/ros").glob("*"):
                ros_paths.extend(distro.glob("local/lib/python*/dist-packages"))
                ros_paths.extend(distro.glob("lib/python*/site-packages"))
            for candidate in sorted(ros_paths):
                if str(candidate) not in sys.path:
                    sys.path.insert(0, str(candidate))
            try:
                import rclpy
                from geometry_msgs.msg import Twist
                from nav_msgs.msg import OccupancyGrid, Odometry
                from sensor_msgs.msg import CameraInfo, Image
                from std_msgs.msg import String
                from std_srvs.srv import Trigger
            except ImportError:
                raise RuntimeError("ROS2 Python packages are required; source /opt/ros/humble/setup.bash") from exc
        self.rclpy = rclpy
        if not rclpy.ok():
            os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros2_logs")
            Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
            rclpy.init(args=None)
        self.bridge = bridge
        self._Twist = Twist
        self._Odometry = Odometry
        self._OccupancyGrid = OccupancyGrid
        self._Image = Image
        self._CameraInfo = CameraInfo
        self._String = String
        self._Trigger = Trigger
        self.node = rclpy.create_node("rby1_molmospaces_ros2_bridge")
        self.camera, self.frame_id = camera, frame_id
        self._lock = threading.Lock(); self._pose = (0.0, 0.0, 0.0); self._last = time.monotonic()
        self.odom_pub = self.node.create_publisher(Odometry, "/rby1/odom", 10)
        self.rgb_pub = self.node.create_publisher(Image, "/rby1/rgb/image_raw", 10)
        self.depth_pub = self.node.create_publisher(Image, "/rby1/depth/image_raw", 10)
        self.info_pub = self.node.create_publisher(CameraInfo, "/rby1/camera_info", 10)
        self.map_pub = self.node.create_publisher(OccupancyGrid, "/rby1/map", 1)
        self.cmd_sub = self.node.create_subscription(Twist, "/rby1/cmd_vel", self._cmd_vel, 10)
        self.target_sub = self.node.create_subscription(String, "/rby1/navigation_target", self._target, 10)
        self.reset_srv = self.node.create_service(Trigger, "/rby1/reset", self._reset)
        self.timer = self.node.create_timer(0.1, self._publish_state)

    def _target(self, msg: Any) -> None:
        target = str(msg.data).strip()
        if target:
            self.bridge.execute(f"navigate to the {target}", context={"structured_action": "prepare_navigation", "target_types": [target]}, mode="structured", timeout_s=30.0)

    def _cmd_vel(self, msg: Any) -> None:
        with self._lock:
            x, y, yaw = self._pose
        dt = min(0.5, max(0.02, time.monotonic() - self._last)); self._last = time.monotonic()
        vx, vy, w = float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)
        if abs(w) > 1e-6:
            end_yaw = yaw + w * dt; r = 1.0 / w
            dx, dy = r * math.sin(w * dt) * vx + r * (1 - math.cos(w * dt)) * vy, r * (-(1 - math.cos(w * dt))) * vx + r * math.sin(w * dt) * vy
        else:
            end_yaw, dx, dy = yaw, vx * dt, vy * dt
        waypoint = [x + math.cos(yaw) * dx - math.sin(yaw) * dy, y + math.sin(yaw) * dx + math.cos(yaw) * dy, end_yaw]
        try:
            result = self.bridge.execute("ROS2 RBY1 velocity command", context={"structured_action": "step_waypoint", "waypoint": waypoint, "control_steps": 1, "max_translation_m": 0.5, "max_yaw_step_rad": 0.52}, mode="structured", timeout_s=10.0)
            if result.get("success"):
                with self._lock: self._pose = _pose(((result.get("state") or {}).get("robot") or {}).get("base_pose"))
        except Exception as exc:
            self.node.get_logger().warning(f"cmd_vel bridge failed: {exc}")

    def _reset(self, _request: Any, response: Any) -> Any:
        try:
            self.bridge.reset(); response.success = True; response.message = "MolmoSpaces RBY1 reset"
        except Exception as exc:
            response.success = False; response.message = str(exc)
        return response

    def _publish_state(self) -> None:
        try:
            result = self.bridge.execute("ROS2 RBY1 RGB-D observation", context={"structured_action": "observe_rgbd", "camera": self.camera}, mode="structured", timeout_s=10.0)
            from vector_os_nano.integrations.online_navigation import decode_rgbd_result
            obs = decode_rgbd_result(result); self._pose = _pose(result.get("base_pose", obs.base_pose))
            stamp = self.node.get_clock().now().to_msg(); x, y, yaw = self._pose
            odom = self._Odometry(); odom.header.stamp = stamp; odom.header.frame_id = "map"; odom.child_frame_id = self.frame_id
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y; odom.pose.pose.orientation.z = math.sin(yaw / 2); odom.pose.pose.orientation.w = math.cos(yaw / 2); self.odom_pub.publish(odom)
            rgb = self._Image(); rgb.header.stamp = stamp; rgb.header.frame_id = self.camera; rgb.height, rgb.width = obs.rgb.shape[:2]; rgb.encoding = "rgb8"; rgb.step = rgb.width * 3; rgb.data = obs.rgb.tobytes(); self.rgb_pub.publish(rgb)
            depth = self._Image(); depth.header.stamp = stamp; depth.header.frame_id = self.camera; depth.height, depth.width = obs.depth.shape; depth.encoding = "32FC1"; depth.is_bigendian = False; depth.step = depth.width * 4; depth.data = np.asarray(obs.depth, dtype=np.float32).tobytes(); self.depth_pub.publish(depth)
            info = self._CameraInfo(); info.header = rgb.header; info.width, info.height = rgb.width, rgb.height; info.k = np.asarray(obs.intrinsics, dtype=float).reshape(9).tolist(); self.info_pub.publish(info)
        except Exception as exc:
            self.node.get_logger().warning(f"observation publish failed: {exc}")

    def spin(self) -> None:
        self.rclpy.spin(self.node)

    def close(self) -> None:
        self.node.destroy_node(); self.rclpy.shutdown()


def main() -> None:
    import argparse
    from vector_os_nano.integrations.molmospaces import MolmoSpacesRBY1Bridge
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1"); parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--timeout", type=float, default=300.0); parser.add_argument("--camera", default="head_camera")
    parser.add_argument("--viewer", action="store_true", help="open the MolmoSpaces MuJoCo viewer")
    parser.add_argument("--camera-system", default="gopro_d455")
    args = parser.parse_args()
    from vector_os_nano.integrations.molmospaces.protocol import MolmoSpacesRBY1Endpoint
    bridge = MolmoSpacesRBY1Bridge(endpoint=MolmoSpacesRBY1Endpoint(host=args.host, port=args.port, timeout_s=args.timeout))
    bridge.connect()
    # The bridge starts targetless and unloaded.  Create the simulation task
    # before the ROS2 timer begins requesting RGB-D observations.
    bridge.reset(metadata={"viewer": bool(args.viewer), "viewer_camera": "free",
                           "camera_system": args.camera_system, "rectify_gopro": True})
    node = RBY1MolmoSpacesROS2Bridge(bridge, camera=args.camera)
    try: node.spin()
    finally: node.close(); bridge.close()


if __name__ == "__main__":
    main()
