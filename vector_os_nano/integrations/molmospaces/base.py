"""Vector robot-layer base backed by the MolmoSpaces RBY1 bridge."""
from __future__ import annotations
import math
import os
import sys
from pathlib import Path
from typing import Any

class MolmoSpacesRBY1Base:
    """Expose basic ``walk`` semantics through the bridge's waypoint primitive."""
    name = "molmospaces_rby1_base"
    def __init__(self, bridge: Any) -> None:
        self.bridge = bridge
    @staticmethod
    def _pose(state: dict[str, Any]) -> tuple[float, float, float]:
        values = (((state.get("state") or {}).get("robot") or {}).get("base_pose"))
        if not isinstance(values, (list, tuple)) or len(values) < 7:
            values = ((state.get("robot") or {}).get("base_pose"))
        if not isinstance(values, (list, tuple)) or len(values) < 7:
            raise RuntimeError("MolmoSpaces state has no RBY1 base pose")
        x, y = float(values[0]), float(values[1]); qw, qx, qy, qz = map(float, values[3:7])
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return x, y, yaw
    def _state(self) -> dict[str, Any]: return dict(self.bridge.observe())
    def get_position(self) -> list[float]:
        x, y, _ = self._pose(self._state()); return [x, y, 0.0]
    def get_heading(self) -> float: return self._pose(self._state())[2]
    def walk(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0, duration: float = 1.0) -> bool:
        duration = max(0.0, float(duration)); x, y, yaw = self._pose(self._state()); w = float(vyaw)
        if abs(w) > 1e-6:
            end_yaw = yaw + w * duration; radius = 1.0 / w
            dx = radius * math.sin(w * duration) * float(vx) + radius * (1.0 - math.cos(w * duration)) * float(vy)
            dy = radius * (-(1.0 - math.cos(w * duration))) * float(vx) + radius * math.sin(w * duration) * float(vy)
        else:
            end_yaw = yaw; dx, dy = float(vx) * duration, float(vy) * duration
        x += math.cos(yaw) * dx - math.sin(yaw) * dy; y += math.sin(yaw) * dx + math.cos(yaw) * dy
        result = self.bridge.execute("execute Vector RBY1 base velocity", context={
            "structured_action": "step_waypoint", "waypoint": [x, y, end_yaw], "control_steps": 1,
            "max_translation_m": max(0.5, abs(float(vx) * duration) + abs(float(vy) * duration) + 0.05),
            "max_yaw_step_rad": max(0.35, abs(w * duration) + 0.05)}, mode="structured", timeout_s=30.0)
        return bool(result.get("success", False))
    def stop(self) -> None: self.bridge.stop()


class ROS2RBY1Base:
    """Vector robot layer publishing standard ROS2 velocity commands."""
    name = "rby1_ros2_base"
    def __init__(self, *, cmd_topic: str = "/rby1/cmd_vel", odom_topic: str = "/rby1/odom") -> None:
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
        except ImportError:
            ros_paths = []
            for distro in Path("/opt/ros").glob("*"):
                ros_paths.extend(distro.glob("local/lib/python*/dist-packages"))
                ros_paths.extend(distro.glob("lib/python*/site-packages"))
            for candidate in sorted(ros_paths):
                if str(candidate) not in sys.path:
                    sys.path.insert(0, str(candidate))
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        import threading
        self._rclpy, self._lock = rclpy, threading.Lock()
        if not rclpy.ok():
            os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros2_logs")
            Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
            rclpy.init(args=None)
        self._node = rclpy.create_node("vector_rby1_robot_layer")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._pub = self._node.create_publisher(Twist, cmd_topic, 10)
        self._pose = [0.0, 0.0, 0.0]
        self._node.create_subscription(Odometry, odom_topic, self._odom, 10)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
    def _odom(self, msg: Any) -> None:
        import math
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock: self._pose = [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw]
    def get_position(self) -> list[float]:
        with self._lock: return [self._pose[0], self._pose[1], 0.0]
    def get_heading(self) -> float:
        with self._lock: return self._pose[2]
    def walk(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0, duration: float = 0.2) -> bool:
        from geometry_msgs.msg import Twist
        import time
        msg = Twist(); msg.linear.x = float(vx); msg.linear.y = float(vy); msg.angular.z = float(vyaw)
        end = time.monotonic() + max(0.05, float(duration))
        while time.monotonic() < end:
            self._pub.publish(msg); time.sleep(0.05)
        self.stop(); return True
    def stop(self) -> None:
        from geometry_msgs.msg import Twist
        self._pub.publish(Twist())
    def close(self) -> None:
        self.stop(); self._executor.shutdown(); self._node.destroy_node()
