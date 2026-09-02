# SPDX-License-Identifier: Apache-2.0
"""Online semantic-topology updates for RBY1 navigation."""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Any, Iterable

import numpy as np

from vector_os_nano.core.scene_graph import SceneGraph

log = logging.getLogger(__name__)


@dataclass
class RGBDObservation:
    rgb: np.ndarray
    depth: np.ndarray
    intrinsics: np.ndarray
    camera_pose: np.ndarray
    base_pose: np.ndarray  # x, y, yaw in world coordinates
    timestamp: float = 0.0
    state: dict[str, Any] | None = None


def _depth_at_box(depth: np.ndarray, box: Iterable[float]) -> float | None:
    x1, y1, x2, y2 = [int(round(float(v))) for v in box]
    h, w = depth.shape[:2]
    x1, x2 = max(0, min(w - 1, x1)), max(0, min(w, x2))
    y1, y2 = max(0, min(h - 1, y1)), max(0, min(h, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    patch = np.asarray(depth[y1:y2, x1:x2], dtype=np.float32)
    valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 10.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def project_pixel_to_world(u: float, v: float, depth_m: float, intrinsics: np.ndarray, camera_pose: np.ndarray) -> np.ndarray:
    """Project a standard CV pixel/depth sample through camera-to-world pose."""
    k = np.asarray(intrinsics, dtype=float)
    z = float(depth_m)
    point_camera = np.array(
        [(float(u) - k[0, 2]) * z / max(k[0, 0], 1e-6),
         (float(v) - k[1, 2]) * z / max(k[1, 1], 1e-6), z, 1.0],
        dtype=float,
    )
    return (np.asarray(camera_pose, dtype=float) @ point_camera)[:3]


class OnlineSemanticMapper:
    """Record only current-view observations into the existing SceneGraph.

    ``vlm`` is intentionally duck-typed so a cloud VLM, local VLM, or a test
    double can be supplied. ``detector`` is similarly optional; when unavailable
    the room/viewpoint is still recorded and navigation remains functional.
    """

    def __init__(
        self,
        scene_graph: SceneGraph,
        *,
        vlm: Any = None,
        detector: Any = None,
        persist_every: int = 3,
        min_heading_delta: float = math.radians(30.0),
    ) -> None:
        self.scene_graph = scene_graph
        self.vlm = vlm
        self.detector = detector
        self.persist_every = max(1, int(persist_every))
        self.min_heading_delta = float(min_heading_delta)
        self._updates = 0
        self._last_pose: np.ndarray | None = None
        self._last_room: str | None = None

    def update(self, observation: RGBDObservation, *, target: str | None = None) -> dict[str, Any]:
        pose = np.asarray(observation.base_pose, dtype=float)
        if pose.size < 3:
            return {"recorded": False, "reason": "invalid_base_pose"}

        if self._last_pose is not None:
            distance = float(np.linalg.norm(pose[:2] - self._last_pose[:2]))
            heading_delta = abs(math.atan2(math.sin(pose[2] - self._last_pose[2]), math.cos(pose[2] - self._last_pose[2])))
            if distance < 1.5 and heading_delta < self.min_heading_delta:
                return {"recorded": False, "reason": "not_a_keyframe"}

        scene = None
        room = None
        objects: list[Any] = []
        description = ""
        if self.vlm is not None:
            try:
                scene = self.vlm.describe_scene(observation.rgb)
                description = str(getattr(scene, "summary", "") or "")
                objects = list(getattr(scene, "objects", []) or [])
                room_result = self.vlm.identify_room(observation.rgb)
                room = str(getattr(room_result, "room", "") or "")
            except Exception as exc:  # semantic logging must not stop control
                log.warning("online VLM observation failed: %s", exc)

        room = self._canonical_room(room, pose)
        detected_objects: list[tuple[str, float, float]] = []
        for item in objects:
            name = str(getattr(item, "name", "") or getattr(item, "label", "") or "").strip()
            if not name:
                continue
            confidence = float(getattr(item, "confidence", 0.5) or 0.5)
            world = self._locate_object(observation, name)
            if world is not None:
                detected_objects.append((name, float(world[0]), float(world[1])))
            # Do not invent a world coordinate when detection/depth is absent.
            # A later keyframe can localize the object; using the robot position
            # here would turn an unlocalized mention into a false navigation goal.

        self.scene_graph.observe_with_viewpoint(
            room,
            float(pose[0]),
            float(pose[1]),
            float(pose[2]),
            [name for name, _, _ in detected_objects],
            description,
            detected_objects=detected_objects or None,
        )
        if self._last_room and self._last_room != room:
            midpoint = (pose[:2] + self._last_pose[:2]) / 2.0 if self._last_pose is not None else pose[:2]
            self.scene_graph.add_door(self._last_room, room, float(midpoint[0]), float(midpoint[1]))

        self._last_pose = pose.copy()
        self._last_room = room
        self._updates += 1
        if self._updates % self.persist_every == 0:
            try:
                self.scene_graph.save()
            except Exception as exc:
                log.warning("online SceneGraph save failed: %s", exc)
        return {
            "recorded": True,
            "room": room,
            "objects": [name for name, _, _ in detected_objects],
            "timestamp": observation.timestamp or time.time(),
        }

    def close(self) -> None:
        try:
            self.scene_graph.save()
        except Exception as exc:
            log.warning("final online SceneGraph save failed: %s", exc)

    def ground_target(
        self, observation: RGBDObservation, target: str
    ) -> dict[str, Any]:
        """Run target-specific open-vocabulary grounding for navigation fallback."""
        name = str(target or "").strip()
        if not name:
            return {"recorded": False, "reason": "missing_target"}
        world = self._locate_object(observation, name)
        if world is None:
            return {"recorded": False, "reason": "target_not_detected", "target": name}
        pose = np.asarray(observation.base_pose, dtype=float)
        room = self._canonical_room(None, pose)
        self.scene_graph.observe_with_viewpoint(
            room,
            float(pose[0]), float(pose[1]), float(pose[2]),
            [name], f"Target-specific grounding: {name}",
            detected_objects=[(name, float(world[0]), float(world[1]))],
        )
        return {
            "recorded": True,
            "room": room,
            "objects": [name],
            "target_grounding": True,
            "position": [float(world[0]), float(world[1]), float(world[2])],
        }

    def _canonical_room(self, room: str | None, pose: np.ndarray) -> str:
        text = (room or "").strip().lower().replace(" ", "_")
        if text and text not in {"unknown", "none", "n/a"}:
            return text
        nearest = self.scene_graph.nearest_room(float(pose[0]), float(pose[1]))
        return nearest or "unknown_room"

    def _locate_object(self, observation: RGBDObservation, name: str) -> np.ndarray | None:
        if self.detector is None:
            return None
        try:
            detections = self.detector.detect(observation.rgb, name)
        except Exception as exc:
            log.warning("object detector failed for %r: %s", name, exc)
            return None
        if not detections:
            return None
        detection = detections[0]
        box = getattr(detection, "bbox", None)
        if box is None and isinstance(detection, dict):
            box = detection.get("bbox")
        if box is None:
            return None
        x1, y1, x2, y2 = [float(v) for v in box]
        depth_m = _depth_at_box(observation.depth, box)
        if depth_m is None:
            return None
        return project_pixel_to_world((x1 + x2) / 2.0, (y1 + y2) / 2.0, depth_m, observation.intrinsics, observation.camera_pose)


__all__ = ["OnlineSemanticMapper", "RGBDObservation", "project_pixel_to_world"]
