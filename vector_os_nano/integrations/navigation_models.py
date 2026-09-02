# SPDX-License-Identifier: Apache-2.0
"""Small HTTP client shared by the online OneRING navigation skill.

The heavy navigation models live in their own conda environments.  Keeping the
Vector process to a JSON client makes the robot/agent environment import-safe
and prevents model dependencies from leaking into the skill registry.
"""
from __future__ import annotations

import base64
import io
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any
from pathlib import Path

import numpy as np
from PIL import Image


def _png_rgb(rgb: np.ndarray) -> str:
    image = Image.fromarray(np.asarray(rgb, dtype=np.uint8))
    buf = io.BytesIO()
    image.save(buf, format="PNG", optimize=True)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def _post_json(url: str, payload: dict[str, Any], timeout_s: float) -> dict[str, Any]:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            body = response.read().decode("utf-8")
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise RuntimeError(f"navigation model service unavailable at {url}: {exc}") from exc
    try:
        result = json.loads(body)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"navigation model returned invalid JSON: {body[:200]!r}") from exc
    if not isinstance(result, dict):
        raise RuntimeError("navigation model response must be a JSON object")
    if result.get("error"):
        raise RuntimeError(str(result["error"]))
    return result


@dataclass
class OneRINGClient:
    """Stateful client for the out-of-process OneRING discrete policy."""

    endpoint: str = "http://127.0.0.1:7801"
    timeout_s: float = 60.0

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "OneRINGClient":
        return cls(
            endpoint=str(config.get("endpoint") or os.environ.get("ONERING_URL", cls.endpoint)),
            timeout_s=float(config.get("timeout_s", os.environ.get("ONERING_TIMEOUT", 60.0))),
        )

    def reset(self, instruction: str) -> dict[str, Any]:
        return _post_json(
            f"{self.endpoint.rstrip('/')}/reset",
            {"instruction": str(instruction)},
            self.timeout_s,
        )

    def step(
        self,
        *,
        navigation_rgb: np.ndarray,
        manipulation_rgb: np.ndarray | None,
        instruction: str,
    ) -> dict[str, Any]:
        navigation_png = _png_rgb(navigation_rgb)
        payload = {
            "navigation_rgb_png": navigation_png,
            "instruction": str(instruction),
        }
        # RBY-1 currently exposes one navigation camera to OneRING. Avoid
        # compressing and transmitting the exact same frame twice; the server
        # aliases a missing manipulation image to the navigation image.
        if manipulation_rgb is not None and manipulation_rgb is not navigation_rgb:
            payload["manipulation_rgb_png"] = _png_rgb(manipulation_rgb)
        return _post_json(f"{self.endpoint.rstrip('/')}/step", payload, self.timeout_s)


class OneRINGROS2Client:
    """ROS2 transport implementing the same reset/step client contract."""
    def __init__(self, *, timeout_s: float = 60.0) -> None:
        import json, threading, uuid
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
        from std_msgs.msg import String
        if not rclpy.ok():
            os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros2_logs")
            Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
            rclpy.init(args=None)
        self._json, self._lock, self._pending = json, threading.Lock(), {}
        self._node = rclpy.create_node("vector_rby1_onering_client")
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._pub = self._node.create_publisher(String, "/rby1/onering/request", 10)
        self._sub = self._node.create_subscription(String, "/rby1/onering/action", self._on_action, 10)
        self._reset_pub = self._node.create_publisher(String, "/rby1/onering/reset", 10)
        self._timeout_s = float(timeout_s)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True); self._thread.start()
    def _on_action(self, msg):
        data = self._json.loads(msg.data); event = self._pending.get(data.get("request_id"))
        if event is not None: event["result"] = data; event["done"].set()
    def reset(self, instruction: str) -> dict[str, Any]:
        from std_msgs.msg import String
        msg = String(); msg.data = str(instruction)
        deadline = time.monotonic() + min(5.0, self._timeout_s)
        while self._node.count_subscribers("/rby1/onering/reset") == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        self._reset_pub.publish(msg); return {"ok": True}
    def step(self, *, navigation_rgb: np.ndarray, manipulation_rgb: np.ndarray | None, instruction: str) -> dict[str, Any]:
        import time, uuid
        from std_msgs.msg import String
        request_id = uuid.uuid4().hex; event = {"done": threading.Event(), "result": None}; self._pending[request_id] = event
        msg = String(); msg.data = self._json.dumps({"request_id": request_id, "instruction": str(instruction), "navigation_rgb_png": _png_rgb(navigation_rgb)})
        deadline = time.monotonic() + min(5.0, self._timeout_s)
        while self._node.count_subscribers("/rby1/onering/request") == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        self._pub.publish(msg)
        if not event["done"].wait(self._timeout_s): raise TimeoutError("OneRING ROS2 action timed out")
        self._pending.pop(request_id, None); result = event["result"] or {}; 
        if result.get("error"): raise RuntimeError(str(result["error"]))
        return result

    def close(self) -> None:
        self._executor.shutdown(); self._node.destroy_node()


__all__ = ["OneRINGClient", "OneRINGROS2Client"]
