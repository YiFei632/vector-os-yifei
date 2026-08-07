# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""MolmoSpaces RBY1 skills.

These skills expose the external MolmoSpaces RBY1 bridge as structured
Vector OS Nano skills, so the planner can call named actions instead of
forwarding one opaque natural-language command.
"""
from __future__ import annotations

import os
import re
from typing import Any

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult
from vector_os_nano.integrations.molmospaces import MolmoSpacesRBY1Bridge
from vector_os_nano.integrations.molmospaces.client import MolmoSpacesRBY1Error
from vector_os_nano.integrations.molmospaces.perception import sync_scene_to_context
from vector_os_nano.integrations.molmospaces.protocol import MolmoSpacesRBY1Endpoint


_BRIDGE_CACHE: dict[tuple[str, int, float], MolmoSpacesRBY1Bridge] = {}


def _normalize_target(value: Any) -> str:
    text = str(value or "").strip().lower()
    text = re.sub(r"^(?:the|a|an)\s+", "", text)
    text = text.strip()
    aliases = {
        "桌子": "table",
        "餐桌": "table",
        "桌边": "table",
        "杯子": "mug",
        "马克杯": "mug",
        "水杯": "mug",
        "椅子": "chair",
        "沙发": "sofa",
        "冰箱": "refrigerator",
        "瓶子": "bottle",
        "苹果": "apple",
        "碗": "bowl",
    }
    return aliases.get(text, text)


def _rby1_config(context: SkillContext) -> dict[str, Any]:
    raw = context.services.get("molmospaces_rby1")
    config = dict(raw) if isinstance(raw, dict) else {}
    endpoint = config.get("endpoint")
    if not isinstance(endpoint, dict):
        endpoint = {}
    config["endpoint"] = {
        "host": str(
            endpoint.get("host")
            or os.environ.get("MOLMOSPACES_RBY1_HOST")
            or "127.0.0.1"
        ),
        "port": int(
            endpoint.get("port")
            or os.environ.get("MOLMOSPACES_RBY1_PORT")
            or 8765
        ),
        "timeout_s": float(
            endpoint.get("timeout_s")
            or os.environ.get("MOLMOSPACES_RBY1_TIMEOUT")
            or 300.0
        ),
    }
    return config


def _bridge_for_context(context: SkillContext) -> tuple[MolmoSpacesRBY1Bridge, dict[str, Any]]:
    config = _rby1_config(context)
    endpoint_config = config["endpoint"]
    key = (
        str(endpoint_config["host"]),
        int(endpoint_config["port"]),
        float(endpoint_config["timeout_s"]),
    )
    bridge = _BRIDGE_CACHE.get(key)
    if bridge is None:
        bridge = MolmoSpacesRBY1Bridge(
            endpoint=MolmoSpacesRBY1Endpoint(
                host=key[0],
                port=key[1],
                timeout_s=key[2],
            )
        )
        _BRIDGE_CACHE[key] = bridge
    return bridge, config


def _runtime_context(config: dict[str, Any]) -> dict[str, Any]:
    runtime_context = dict(config.get("context", {}) or {})
    scene_name = config.get("scene_name")
    if scene_name:
        runtime_context.setdefault("scene_name", scene_name)
    return runtime_context


def _bridge_error(exc: Exception) -> SkillResult:
    return SkillResult(
        success=False,
        error_message=f"MolmoSpaces RBY1 bridge error: {exc}",
        diagnosis_code="molmospaces_rby1_bridge_error",
    )


def _supported_bridge_items(metadata: dict[str, Any]) -> set[str]:
    supported: set[str] = set()
    for key in ("supported_actions", "supported_task_types", "capabilities"):
        values = metadata.get(key, [])
        if isinstance(values, list):
            supported.update(str(item).lower() for item in values)
    return supported


def _list_scene_objects(bridge: MolmoSpacesRBY1Bridge, config: dict[str, Any]) -> list[dict[str, Any]]:
    result = bridge.execute(
        "list scene objects",
        context={**_runtime_context(config), "structured_action": "list_scene_objects"},
        mode="structured",
        timeout_s=float(config["endpoint"]["timeout_s"]),
    )
    objects = result.get("objects", [])
    return [dict(obj) for obj in objects if isinstance(obj, dict)]


def _object_matches(query: str, obj: dict[str, Any]) -> bool:
    q = query.lower().strip()
    if q in {"all", "all objects", "objects", "everything", "*"}:
        return True
    values = [
        obj.get("name"),
        obj.get("object_id"),
        obj.get("label"),
        obj.get("category"),
        obj.get("natural_name"),
    ]
    values.extend(obj.get("aliases", []) or [])
    for value in values:
        text = str(value or "").lower().replace("_", " ").strip()
        if text and (q == text or q in text or text in q):
            return True
    return False


@skill(
    aliases=["rby1 observe", "observe rby1", "观察rby1", "查看rby1"],
    direct=False,
)
class RBY1ObserveSkill:
    name = "rby1_observe"
    description = "Observe the current MolmoSpaces RBY1 scene state."
    parameters: dict[str, Any] = {}
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects: dict[str, Any] = {}
    failure_modes: list[str] = ["molmospaces_rby1_bridge_error"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        del params
        try:
            bridge, config = _bridge_for_context(context)
            state = bridge.observe()
            try:
                objects = _list_scene_objects(bridge, config)
                state["objects"] = objects
                state["scene_sync"] = sync_scene_to_context(context, objects)
            except Exception as exc:  # noqa: BLE001
                state["scene_sync_error"] = f"{type(exc).__name__}: {exc}"
            return SkillResult(success=True, result_data=state)
        except (MolmoSpacesRBY1Error, TimeoutError, OSError) as exc:
            return _bridge_error(exc)


@skill(
    aliases=["rby1 sync scene", "sync rby1 scene", "同步rby1场景", "同步场景"],
    direct=False,
)
class RBY1SyncSceneSkill:
    name = "rby1_sync_scene"
    description = "Synchronize MolmoSpaces scene objects into Vector world model and scene graph."
    parameters: dict[str, Any] = {}
    preconditions: list[str] = ["MolmoSpaces RBY1 bridge server is running"]
    postconditions: list[str] = ["MolmoSpaces scene objects are available in Vector world state"]
    effects: dict[str, Any] = {}
    failure_modes: list[str] = ["molmospaces_rby1_bridge_error"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        del params
        try:
            bridge, config = _bridge_for_context(context)
            objects = _list_scene_objects(bridge, config)
            sync = sync_scene_to_context(context, objects)
            return SkillResult(success=True, result_data={"objects": objects, "scene_sync": sync})
        except (MolmoSpacesRBY1Error, TimeoutError, OSError) as exc:
            return _bridge_error(exc)


@skill(
    aliases=["rby1 detect", "detect rby1", "rby1 find", "rby1 找", "rby1 检测"],
    direct=False,
)
class RBY1DetectObjectSkill:
    name = "rby1_detect_object"
    description = "Detect or look up objects in the MolmoSpaces RBY1 scene."
    parameters: dict[str, Any] = {
        "query": {
            "type": "string",
            "description": "Object query to detect or look up in the MolmoSpaces scene.",
        }
    }
    preconditions: list[str] = ["MolmoSpaces RBY1 bridge server is running"]
    postconditions: list[str] = ["Matching objects are returned and synchronized to Vector world state"]
    effects: dict[str, Any] = {}
    failure_modes: list[str] = ["missing_query", "no_detections", "molmospaces_rby1_bridge_error"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        query = _normalize_target(params.get("query") or params.get("object") or params.get("target"))
        if not query:
            return SkillResult(
                success=False,
                error_message="Missing RBY1 detection query",
                diagnosis_code="missing_query",
            )
        try:
            bridge, config = _bridge_for_context(context)
            objects = _list_scene_objects(bridge, config)
            sync = sync_scene_to_context(context, objects)
            matches = [obj for obj in objects if _object_matches(query, obj)]
            if not matches:
                return SkillResult(
                    success=False,
                    result_data={"objects": objects, "scene_sync": sync, "query": query},
                    error_message=f"No MolmoSpaces objects matched {query!r}",
                    diagnosis_code="no_detections",
                )
            return SkillResult(
                success=True,
                result_data={"detections": matches, "objects": objects, "scene_sync": sync},
            )
        except (MolmoSpacesRBY1Error, TimeoutError, OSError) as exc:
            return _bridge_error(exc)


@skill(
    aliases=[
        "rby1 navigate",
        "rby1 go to",
        "navigate rby1",
        "rby1 走到",
        "rby1 导航到",
    ],
    direct=False,
)
class RBY1NavigateToObjectSkill:
    name = "rby1_navigate_to_object"
    description = "Navigate the MolmoSpaces RBY1 robot to an object or fixture in the active scene."
    typical_duration_sec = 120.0
    parameters: dict[str, Any] = {
        "target": {
            "type": "string",
            "description": "Object or fixture category to navigate to, such as table, mug, chair, or refrigerator.",
        }
    }
    preconditions: list[str] = ["MolmoSpaces RBY1 bridge server is running"]
    postconditions: list[str] = ["RBY1 is near the requested object when the nav task succeeds"]
    effects: dict[str, Any] = {"base": "move", "navigate": True}
    failure_modes: list[str] = ["missing_target", "navigation_failed", "molmospaces_rby1_bridge_error"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        target = _normalize_target(
            params.get("target")
            or params.get("object")
            or params.get("destination")
            or params.get("query")
        )
        if not target:
            return SkillResult(
                success=False,
                error_message="Missing RBY1 navigation target",
                diagnosis_code="missing_target",
            )

        try:
            bridge, config = _bridge_for_context(context)
            runtime_context = _runtime_context(config)
            runtime_context.update(
                {
                    "structured_action": "navigate_to_object",
                    "target": target,
                    "target_types": [target],
                    "allow_execute_reset": False,
                }
            )
            result = bridge.execute(
                f"go to the {target}",
                context=runtime_context,
                mode="structured",
                timeout_s=float(config["endpoint"]["timeout_s"]),
            )
        except (MolmoSpacesRBY1Error, TimeoutError, OSError) as exc:
            return _bridge_error(exc)

        terminal_reason = str(result.get("terminal_reason", ""))
        success = bool(result.get("success")) or terminal_reason == "task_done"
        if not success:
            return SkillResult(
                success=False,
                result_data=result,
                error_message=(
                    f"RBY1 navigation to {target!r} did not complete "
                    f"(terminal_reason={terminal_reason or 'unknown'})"
                ),
                diagnosis_code="navigation_failed",
            )
        return SkillResult(success=True, result_data=result)


@skill(
    aliases=["rby1 pick", "rby1 pick up", "pick rby1", "rby1 拿起", "rby1 抓取"],
    direct=False,
)
class RBY1PickObjectSkill:
    name = "rby1_pick_object"
    description = "Pick up an object with the MolmoSpaces RBY1 robot when the runtime supports manipulation."
    typical_duration_sec = 120.0
    parameters: dict[str, Any] = {
        "object": {
            "type": "string",
            "description": "Object category to pick up, such as mug, cup, bottle, apple, or bowl.",
        }
    }
    preconditions: list[str] = ["Object is reachable by RBY1", "MolmoSpaces manipulation policy is available"]
    postconditions: list[str] = ["RBY1 is holding the requested object"]
    effects: dict[str, Any] = {"arm": "move", "gripper": "close"}
    failure_modes: list[str] = ["missing_object", "manipulation_unsupported", "molmospaces_rby1_bridge_error"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        obj = _normalize_target(params.get("object") or params.get("target") or params.get("query"))
        if not obj:
            return SkillResult(
                success=False,
                error_message="Missing RBY1 pick object",
                diagnosis_code="missing_object",
            )

        try:
            bridge, config = _bridge_for_context(context)
            metadata = bridge.connect()
            supported = _supported_bridge_items(metadata)
            if not any(item in supported for item in ("pick", "pick_object", "manipulation")):
                return SkillResult(
                    success=False,
                    result_data={"bridge": metadata, "object": obj},
                    error_message=(
                        "MolmoSpaces RBY1 bridge is reachable, but this adapter "
                        "does not advertise pick/manipulation support yet."
                    ),
                    diagnosis_code="manipulation_unsupported",
                )
            runtime_context = _runtime_context(config)
            runtime_context.update(
                {
                    "structured_action": "pick_object",
                    "object": obj,
                    "target_types": [obj],
                    "allow_execute_reset": False,
                }
            )
            result = bridge.execute(
                f"pick up the {obj}",
                context=runtime_context,
                mode="structured",
                timeout_s=float(config["endpoint"]["timeout_s"]),
            )
        except (MolmoSpacesRBY1Error, TimeoutError, OSError) as exc:
            return _bridge_error(exc)

        if not bool(result.get("manipulation_supported", False)):
            return SkillResult(
                success=False,
                result_data=result,
                error_message=(
                    "MolmoSpaces RBY1 adapter accepted the structured pick request, "
                    "but the current adapter does not support manipulation yet."
                ),
                diagnosis_code="manipulation_unsupported",
            )
        success = bool(result.get("success"))
        return SkillResult(success=success, result_data=result)


@skill(
    aliases=["rby1 place", "rby1 put", "place rby1", "rby1 放置", "rby1 放到"],
    direct=False,
)
class RBY1PlaceObjectSkill:
    name = "rby1_place_object"
    description = "Place a held object onto or near a target in the MolmoSpaces RBY1 scene when supported."
    typical_duration_sec = 120.0
    parameters: dict[str, Any] = {
        "object": {
            "type": "string",
            "required": False,
            "description": "Held object to place.",
        },
        "target": {
            "type": "string",
            "description": "Placement target, such as table, counter, bowl, or receptacle.",
        },
    }
    preconditions: list[str] = ["RBY1 is holding an object", "MolmoSpaces manipulation policy is available"]
    postconditions: list[str] = ["The object is placed at the requested target"]
    effects: dict[str, Any] = {"arm": "move", "gripper": "open"}
    failure_modes: list[str] = ["missing_target", "manipulation_unsupported", "molmospaces_rby1_bridge_error"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        target = _normalize_target(params.get("target") or params.get("destination") or params.get("receptacle"))
        obj = _normalize_target(params.get("object") or "")
        if not target:
            return SkillResult(
                success=False,
                error_message="Missing RBY1 placement target",
                diagnosis_code="missing_target",
            )
        try:
            bridge, config = _bridge_for_context(context)
            metadata = bridge.connect()
            supported = _supported_bridge_items(metadata)
            if not any(item in supported for item in ("place", "place_object", "manipulation")):
                return SkillResult(
                    success=False,
                    result_data={"bridge": metadata, "object": obj, "target": target},
                    error_message=(
                        "MolmoSpaces RBY1 bridge is reachable, but this adapter "
                        "does not advertise place/manipulation support yet."
                    ),
                    diagnosis_code="manipulation_unsupported",
                )
            runtime_context = _runtime_context(config)
            runtime_context.update(
                {
                    "structured_action": "place_object",
                    "object": obj,
                    "target": target,
                    "target_types": [target],
                    "allow_execute_reset": False,
                }
            )
            result = bridge.execute(
                f"place {obj or 'the object'} on the {target}",
                context=runtime_context,
                mode="structured",
                timeout_s=float(config["endpoint"]["timeout_s"]),
            )
            success = bool(result.get("success"))
            return SkillResult(success=success, result_data=result)
        except (MolmoSpacesRBY1Error, TimeoutError, OSError) as exc:
            return _bridge_error(exc)


@skill(aliases=["rby1 stop", "stop rby1", "停止rby1"], direct=True)
class RBY1StopSkill:
    name = "rby1_stop"
    description = "Stop the MolmoSpaces RBY1 robot."
    parameters: dict[str, Any] = {}
    preconditions: list[str] = []
    postconditions: list[str] = ["RBY1 is stationary"]
    effects: dict[str, Any] = {"base": "stop", "arm": "stop"}
    failure_modes: list[str] = ["molmospaces_rby1_bridge_error"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        del params
        try:
            bridge, _config = _bridge_for_context(context)
            result = bridge.stop()
            return SkillResult(success=True, result_data=result)
        except (MolmoSpacesRBY1Error, TimeoutError, OSError) as exc:
            return _bridge_error(exc)


__all__ = [
    "RBY1DetectObjectSkill",
    "RBY1NavigateToObjectSkill",
    "RBY1ObserveSkill",
    "RBY1PickObjectSkill",
    "RBY1PlaceObjectSkill",
    "RBY1SyncSceneSkill",
    "RBY1StopSkill",
]
