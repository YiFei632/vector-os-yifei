#!/usr/bin/env python3
"""ROS2 request/response adapter for the stateful OneRING HTTP service."""
from __future__ import annotations
import base64, json
import sys
from pathlib import Path
from typing import Any

class OneRINGROS2Bridge:
    def __init__(self, endpoint: str = "http://127.0.0.1:7801") -> None:
        try:
            import rclpy
        except ImportError:
            ros_paths = []
            for distro in Path("/opt/ros").glob("*"):
                ros_paths.extend(distro.glob("local/lib/python*/dist-packages"))
                ros_paths.extend(distro.glob("lib/python*/site-packages"))
            for candidate in sorted(ros_paths):
                sys.path.insert(0, str(candidate))
            import rclpy
        from std_msgs.msg import String
        self.rclpy = rclpy
        if not rclpy.ok(): rclpy.init(args=None)
        self.node = rclpy.create_node("rby1_onering_ros2_bridge")
        self.endpoint = endpoint
        self._String = String
        self.pub = self.node.create_publisher(String, "/rby1/onering/action", 10)
        self.sub = self.node.create_subscription(String, "/rby1/onering/request", self._request, 10)
        self.reset_sub = self.node.create_subscription(String, "/rby1/onering/reset", self._reset, 10)

    def _client(self):
        from vector_os_nano.integrations.navigation_models import OneRINGClient
        return OneRINGClient(endpoint=self.endpoint, timeout_s=60.0)

    def _reset(self, msg: Any) -> None:
        try: self._client().reset(str(msg.data))
        except Exception as exc: self.node.get_logger().error(f"OneRING reset failed: {exc}")

    def _request(self, msg: Any) -> None:
        try:
            payload = json.loads(str(msg.data)); rgb = base64.b64decode(payload["navigation_rgb_png"])
            result = self._client().step(navigation_rgb=_decode_png(rgb), manipulation_rgb=None, instruction=str(payload["instruction"]))
            result["request_id"] = payload.get("request_id"); out = self._String(); out.data = json.dumps(result); self.pub.publish(out)
        except Exception as exc:
            out = self._String(); out.data = json.dumps({"request_id": None, "error": f"{type(exc).__name__}: {exc}"}); self.pub.publish(out)

def _decode_png(raw: bytes):
    import io, numpy as np
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"), dtype=np.uint8)

def main() -> None:
    import argparse
    p = argparse.ArgumentParser(); p.add_argument("--endpoint", default="http://127.0.0.1:7801"); a = p.parse_args()
    n = OneRINGROS2Bridge(a.endpoint)
    try: n.rclpy.spin(n.node)
    finally: n.node.destroy_node(); n.rclpy.shutdown()

if __name__ == "__main__": main()
