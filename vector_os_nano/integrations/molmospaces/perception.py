# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Perception and scene-sync helpers for the MolmoSpaces RBY1 bridge."""
from __future__ import annotations

import base64
import io
import time
from typing import Any

import numpy as np
from PIL import Image

from vector_os_nano.core.scene_graph import ObjectNode
from vector_os_nano.core.types import BBox3D, CameraIntrinsics, Detection, Pose3D, TrackedObject
from vector_os_nano.core.world_model import ObjectState
from vector_os_nano.integrations.molmospaces import MolmoSpacesRBY1Bridge
from vector_os_nano.integrations.molmospaces.protocol import MolmoSpacesRBY1Endpoint


def _endpoint_from_config(config: dict[str, Any]) -> MolmoSpacesRBY1Endpoint:
    endpoint = config.get("endpoint")
    endpoint = endpoint if isinstance(endpoint, dict) else {}
    return MolmoSpacesRBY1Endpoint(
        host=str(endpoint.get("host", "127.0.0.1")),
        port=int(endpoint.get("port", 8765)),
        timeout_s=float(endpoint.get("timeout_s", 300.0)),
    )


def _scene_context(config: dict[str, Any]) -> dict[str, Any]:
    context = dict(config.get("context", {}) or {})
    scene_name = config.get("scene_name")
    if scene_name:
        context.setdefault("scene_name", scene_name)
    return context


def _object_matches(query: str, obj: dict[str, Any]) -> bool:
    q = query.lower().strip()
    if q in {"all", "all objects", "objects", "everything", "*"}:
        return True
    candidates = [
        obj.get("object_id"),
        obj.get("name"),
        obj.get("label"),
        obj.get("category"),
        obj.get("natural_name"),
        obj.get("synset"),
    ]
    for alias in obj.get("aliases", []) or []:
        candidates.append(alias)
    for value in candidates:
        text = str(value or "").lower().replace("_", " ").strip()
        if text and (q == text or q in text or text in q):
            return True
    return False


def _objects_from_result(result: dict[str, Any]) -> list[dict[str, Any]]:
    objects = result.get("objects", [])
    return [dict(obj) for obj in objects if isinstance(obj, dict)]


def sync_scene_to_context(context: Any, objects: list[dict[str, Any]]) -> dict[str, Any]:
    """Synchronize MolmoSpaces objects into Vector world_model and scene_graph."""
    synced = 0
    now = time.time()
    world_model = getattr(context, "world_model", None)
    scene_graph = getattr(context, "services", {}).get("spatial_memory")

    for obj in objects:
        object_id = str(obj.get("object_id") or obj.get("name") or "")
        if not object_id:
            continue
        label = str(obj.get("label") or obj.get("category") or obj.get("natural_name") or object_id)
        pos = obj.get("position") if isinstance(obj.get("position"), list) else [0.0, 0.0, 0.0]
        x, y, z = (float(pos[0]), float(pos[1]), float(pos[2])) if len(pos) >= 3 else (0.0, 0.0, 0.0)

        if world_model is not None and hasattr(world_model, "add_object"):
            world_model.add_object(
                ObjectState(
                    object_id=object_id,
                    label=label,
                    x=x,
                    y=y,
                    z=z,
                    confidence=float(obj.get("confidence", 1.0)),
                    state=str(obj.get("state", "visible")),
                    last_seen=now,
                    properties={
                        "source": "molmospaces",
                        "category": obj.get("category"),
                        "natural_name": obj.get("natural_name"),
                        "pickup_candidate": obj.get("pickup_candidate"),
                        "receptacle": obj.get("receptacle"),
                    },
                )
            )

        if scene_graph is not None and hasattr(scene_graph, "add_object"):
            scene_graph.add_object(
                ObjectNode(
                    object_id=object_id,
                    category=str(obj.get("category") or label),
                    description=str(obj.get("natural_name") or label),
                    confidence=float(obj.get("confidence", 1.0)),
                    room_id=str(obj.get("room_id") or "molmospaces_scene"),
                    x=x,
                    y=y,
                    z=z,
                    attributes={
                        "source": "molmospaces",
                        "name": obj.get("name"),
                        "pickup_candidate": obj.get("pickup_candidate"),
                        "receptacle": obj.get("receptacle"),
                    },
                )
            )
        synced += 1
    return {"synced_objects": synced}


class MolmoSpacesRBY1Perception:
    """Perception backend backed by the MolmoSpaces RBY1 bridge.

    RGB frames are used with Grounding-DINO when available. If offscreen rendering
    is unavailable because a GUI viewer owns the GL context, detection falls back to
    MolmoSpaces ground-truth scene objects.
    """

    def __init__(
        self,
        config: dict[str, Any] | None = None,
        bridge: MolmoSpacesRBY1Bridge | None = None,
    ) -> None:
        self._config = dict(config or {})
        self._bridge = bridge or MolmoSpacesRBY1Bridge(endpoint=_endpoint_from_config(self._config))
        self._last_objects: list[dict[str, Any]] = []
        self._last_frame: np.ndarray | None = None

    @property
    def bridge(self) -> MolmoSpacesRBY1Bridge:
        return self._bridge

    def list_scene_objects(self) -> list[dict[str, Any]]:
        result = self._bridge.execute(
            "list scene objects",
            context={**_scene_context(self._config), "structured_action": "list_scene_objects"},
            mode="structured",
            timeout_s=_endpoint_from_config(self._config).timeout_s,
        )
        self._last_objects = _objects_from_result(result)
        return list(self._last_objects)

    def sync_scene(self, context: Any) -> dict[str, Any]:
        objects = self.list_scene_objects()
        return sync_scene_to_context(context, objects)

    def get_color_frame(self) -> np.ndarray:
        result = self._bridge.execute(
            "observe rgb",
            context={
                **_scene_context(self._config),
                "structured_action": "observe_rgb",
                "camera": self._config.get("camera", "head_camera"),
            },
            mode="structured",
            timeout_s=_endpoint_from_config(self._config).timeout_s,
        )
        image = result.get("image", {})
        if not isinstance(image, dict) or image.get("encoding") != "png_base64":
            raise RuntimeError(str(result.get("error") or "MolmoSpaces RGB frame unavailable"))
        data = base64.b64decode(str(image.get("data", "")))
        frame = np.asarray(Image.open(io.BytesIO(data)).convert("RGB"), dtype=np.uint8)
        self._last_frame = frame
        return frame

    def get_depth_frame(self) -> np.ndarray:
        raise NotImplementedError("MolmoSpaces RBY1 depth frames are not exposed yet")

    def get_intrinsics(self) -> CameraIntrinsics:
        if self._last_frame is not None:
            h, w = self._last_frame.shape[:2]
        else:
            w, h = 640, 480
        return CameraIntrinsics(fx=float(w), fy=float(w), cx=w / 2.0, cy=h / 2.0, width=w, height=h)

    def caption(self, length: str = "normal") -> str:
        del length
        objects = self.list_scene_objects()
        labels = [str(o.get("label") or o.get("category") or o.get("name")) for o in objects[:12]]
        return "MolmoSpaces scene with: " + ", ".join(labels)

    def visual_query(self, question: str) -> str:
        objects = self.list_scene_objects()
        matches = [o for o in objects if _object_matches(question, o)]
        if matches:
            return f"Found {len(matches)} matching object(s): " + ", ".join(
                str(o.get("label") or o.get("name")) for o in matches[:10]
            )
        return "No matching MolmoSpaces object found."

    def detect(self, query: str) -> list[Detection]:
        try:
            frame = self.get_color_frame()
            from vector_os_nano.perception.grounding_dino import get_shared_detector

            detections = get_shared_detector().detect(frame, query)
            if detections:
                return detections
        except Exception:
            pass

        objects = self.list_scene_objects()
        matches = [obj for obj in objects if _object_matches(query, obj)]
        detections: list[Detection] = []
        for idx, obj in enumerate(matches):
            # Ground-truth fallback has no pixel box. Return a stable dummy box so
            # downstream code sees a detection and can recover 3D pose via track().
            x1 = 10.0 + idx * 5.0
            detections.append(
                Detection(
                    label=str(obj.get("label") or obj.get("category") or obj.get("name")),
                    bbox=(x1, 10.0, x1 + 1.0, 11.0),
                    confidence=float(obj.get("confidence", 1.0)),
                )
            )
        return detections

    def track(self, detections: list[Detection]) -> list[TrackedObject]:
        if not self._last_objects:
            self._last_objects = self.list_scene_objects()
        tracked: list[TrackedObject] = []
        for idx, det in enumerate(detections):
            match = next((obj for obj in self._last_objects if _object_matches(det.label, obj)), None)
            pose = None
            bbox_3d = None
            if match is not None and isinstance(match.get("position"), list):
                pos = match["position"]
                if len(pos) >= 3:
                    pose = Pose3D(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
                    bbox_3d = BBox3D(center=pose, size_x=0.05, size_y=0.05, size_z=0.05)
            tracked.append(
                TrackedObject(
                    track_id=idx,
                    label=det.label,
                    bbox_2d=det.bbox,
                    pose=pose,
                    bbox_3d=bbox_3d,
                    confidence=det.confidence,
                )
            )
        return tracked

    def get_point_cloud(self, mask: np.ndarray | None = None) -> np.ndarray:
        del mask
        return np.empty((0, 3), dtype=np.float64)


__all__ = ["MolmoSpacesRBY1Perception", "sync_scene_to_context"]
