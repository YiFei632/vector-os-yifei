# SPDX-License-Identifier: Apache-2.0
"""Generic VLN, trajectory-planning, and grounding skills."""
from __future__ import annotations

import os
import re
from typing import Any

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult


def _navigation_config(context: SkillContext) -> dict[str, Any]:
    """Merge generic navigation service settings with RBY1 compatibility config."""
    config: dict[str, Any] = {}
    rby1 = context.services.get("molmospaces_rby1")
    if isinstance(rby1, dict):
        config.update(rby1)
    generic = context.services.get("navigation")
    if isinstance(generic, dict):
        for key, value in generic.items():
            if isinstance(value, dict) and isinstance(config.get(key), dict):
                config[key] = {**config[key], **value}
            else:
                config[key] = value
    config.setdefault("onering", {
        "endpoint": os.environ.get("ONERING_URL", "http://127.0.0.1:7801"),
        "timeout_s": float(os.environ.get("ONERING_TIMEOUT", "60")),
        "enabled": os.environ.get("ONERING_ENABLED", "1").strip().lower()
        not in {"0", "false", "no", "off"},
    })
    config.setdefault("online", {})
    config["online"].setdefault(
        "max_unverified_stops",
        int(os.environ.get("RBY1_VLN_MAX_UNVERIFIED_STOPS", "3")),
    )
    config["online"].setdefault(
        "onering_control_steps",
        int(os.environ.get("RBY1_VLN_ONERING_CONTROL_STEPS", "3")),
    )
    config["online"].setdefault(
        "onering_fallback_policy",
        os.environ.get("ONERING_FALLBACK_POLICY", "astar"),
    )
    config.setdefault("grounding_dino", {})
    config.setdefault("astar", {})
    is_rby1 = isinstance(context.services.get("molmospaces_rby1"), dict)
    radius_env = "RBY1_ASTAR_ROBOT_RADIUS" if is_rby1 else "VECTOR_ASTAR_ROBOT_RADIUS"
    config["astar"].setdefault(
        "robot_radius_m", float(os.environ.get(radius_env, "0.05" if is_rby1 else "0.35"))
    )
    return config


def _onering_instruction(value: str) -> str:
    """Normalize a target into OneRING's ObjectNav-style language format."""
    text = " ".join(str(value or "").strip().split())
    if not text:
        return text
    if re.match(r"^(?:go|navigate|walk|move)\s+to\b", text, flags=re.IGNORECASE):
        return text.rstrip(".!?") + "."
    if re.match(r"^(?:find|locate|search\s+for)\b", text, flags=re.IGNORECASE):
        return text.rstrip(".!?") + "."
    target = re.sub(r"^(?:the|a|an)\s+", "", text, flags=re.IGNORECASE)
    return f"Go to the {target.rstrip('.!?')}."


def _normalize_navigation_target(value: str) -> str:
    """Normalize VGG/CLI object-target phrasing before OneRING inference."""
    text = " ".join(str(value or "").strip().split())
    text = re.sub(
        r"^(?:请(?:你)?\s*)?(?:导航去|导航到|走到|移动到|前往|走向|去到|去)\s*",
        "", text, flags=re.IGNORECASE,
    )
    text = re.sub(r"(?:那边|旁边|那里|附近|边上)\s*$", "", text).strip(" ,，。.!?")
    aliases = {
        "冰箱": "refrigerator", "电冰箱": "refrigerator",
        "垃圾桶": "trash can", "绿色垃圾桶": "green trash can",
        "桌子": "table", "椅子": "chair", "沙发": "sofa",
        "电视": "television", "厕所": "toilet", "床": "bed", "门": "door",
    }
    return aliases.get(text, text)


def _embodiment(context: SkillContext, config: dict[str, Any]):
    from vector_os_nano.navigation.embodiment import embodiment_from_context
    return embodiment_from_context(context, config)


def _planner(context: SkillContext, config: dict[str, Any]):
    injected = context.services.get("trajectory_planner")
    if injected is not None:
        return injected(context) if callable(injected) else injected
    from vector_os_nano.navigation.trajectory_planners import planners_from_config
    return planners_from_config(config)


def _scene_graph(context: SkillContext):
    graph = context.services.get("spatial_memory")
    if graph is not None:
        return graph
    from vector_os_nano.core.scene_graph import SceneGraph
    return SceneGraph()


def _detector(context: SkillContext, config: dict[str, Any]):
    injected = context.services.get("grounding_dino")
    if injected is not None and not isinstance(injected, dict):
        return injected
    detector_config = dict(config.get("grounding_dino", {}) or {})
    if isinstance(injected, dict):
        detector_config.update(injected)
    from vector_os_nano.perception.grounding_dino_native import NativeGroundingDinoDetector
    return NativeGroundingDinoDetector(**detector_config)


@skill(
    aliases=[
        "onering navigation", "onering navigate", "ring navigation", "OneRING导航",
        "视觉语言导航", "导航去", "导航到", "前往", "走向", "走到",
        "go to", "navigate to",
    ],
    direct=False,
)
class OneRINGNavigationSkill:
    """OneRING navigation skill backed only by local RGB-D A*."""

    name = "onering_navigation"
    description = (
        "Navigate to a language target with OneRING discrete actions. Known or "
        "grounded targets use Vector's local RGB-D A* planner."
    )
    typical_duration_sec = 120.0
    parameters = {
        "target": {"type": "string", "description": "Navigation target or natural-language instruction."}
    }
    preconditions = ["An RGB-D mobile base or navigation embodiment is available"]
    postconditions = ["The robot reaches the target or the VLN policy stops"]
    effects = {"base": "move", "navigate": True, "mapping": "incremental"}
    failure_modes = ["missing_target", "missing_navigation_embodiment", "navigation_failed"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        target = _normalize_navigation_target(str(
            params.get("target") or params.get("instruction")
            or params.get("query") or params.get("room") or ""
        ))
        if not target:
            return SkillResult(False, error_message="Missing navigation target", diagnosis_code="missing_target")
        config = _navigation_config(context)
        onering_config = config.setdefault("onering", {})
        onering_config.setdefault("enabled", True)
        try:
            embodiment = _embodiment(context, config)
            planner = _planner(context, config)
            graph = _scene_graph(context)
            detector = _detector(context, config)
            from vector_os_nano.integrations.navigation_models import OneRINGClient, OneRINGROS2Client
            from vector_os_nano.integrations.online_mapper import OnlineSemanticMapper
            from vector_os_nano.integrations.online_navigation import OnlineNavigationConfig, OnlineNavigator

            mapper = OnlineSemanticMapper(
                graph, vlm=context.services.get("vlm"), detector=detector,
                persist_every=int(config.get("map_persist_every", 3)),
            )
            online = OnlineNavigationConfig.from_dict(config.get("online", {}) or {})
            navigator = OnlineNavigator(
                embodiment=embodiment,
                planner=planner,
                onering=(
                    (OneRINGROS2Client(timeout_s=float((config.get("onering", {}) or {}).get("timeout_s", 60.0)))
                     if os.environ.get("ONERING_ROS2_TRANSPORT", "1").strip().lower() not in {"0", "false", "no", "off"}
                     else OneRINGClient.from_config(config.get("onering", {}) or {}))
                    if bool((config.get("onering", {}) or {}).get("enabled", True))
                    else None
                ),
                mapper=mapper,
                scene_graph=graph,
                config=online,
                timeout_s=float(config.get("timeout_s", 300.0)),
            )
            result = navigator.navigate(
                target,
                instruction=_onering_instruction(str(params.get("instruction") or target)),
            )
        except Exception as exc:  # hardware/model boundaries report structured failures
            return SkillResult(
                False,
                error_message=f"OneRING navigation failed: {type(exc).__name__}: {exc}",
                diagnosis_code="navigation_failed",
            )
        if not result.get("success"):
            return SkillResult(
                False, result_data=result,
                error_message=f"Navigation did not complete ({result.get('terminal_reason', 'unknown')})",
                diagnosis_code="navigation_failed",
            )
        result["embodiment"] = embodiment.name
        return SkillResult(True, result_data=result)
class _PlanSkillBase:
    planner_name = ""
    effects: dict[str, Any] = {}
    postconditions = ["A local trajectory is returned without moving the robot"]
    preconditions = ["An RGB-D navigation embodiment is available"]
    failure_modes = ["planning_failed"]
    parameters = {
        "goal_forward": {"type": "number", "description": "Forward goal in metres."},
        "goal_lateral": {"type": "number", "description": "Left-positive lateral goal in metres."},
    }

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        config = _navigation_config(context)
        config["planner"] = {"primary": self.planner_name, "fallback": None}
        try:
            embodiment = _embodiment(context, config)
            embodiment.ensure_ready()
            observation = embodiment.observe_rgbd()
            planner = _planner(context, config)
            planner.reset(observation.intrinsics)
            goal = [float(params["goal_forward"]), float(params["goal_lateral"])]
            result = planner.plan(observation, mode="pointgoal", goal=goal)
        except Exception as exc:
            return SkillResult(False, error_message=f"{self.planner_name} planning failed: {exc}", diagnosis_code="planning_failed")
        return SkillResult(bool(result.get("trajectory")), result_data=result,
                           error_message="" if result.get("trajectory") else "Planner returned no trajectory",
                           diagnosis_code="" if result.get("trajectory") else "planning_failed")


@skill(aliases=["astar plan", "a star plan", "a*规划", "astar规划"], direct=False)
class AStarPlanSkill(_PlanSkillBase):
    name = "astar_plan"
    planner_name = "astar"
    description = "Plan a local occupancy-grid A* trajectory without executing motion."


@skill(aliases=["grounding dino", "grounding-dino", "ground object", "目标检测"], direct=False)
class GroundingDINODetectSkill:
    name = "grounding_dino_detect"
    description = "Run local GroundingDINO open-vocabulary detection on the active robot camera."
    parameters = {"query": {"type": "string", "description": "Object category or text prompt."}}
    preconditions = ["An RGB camera is available"]
    postconditions = ["Grounded image bounding boxes are returned"]
    effects: dict[str, Any] = {}
    failure_modes = ["missing_query", "detection_failed"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        query = str(params.get("query") or params.get("target") or "").strip()
        if not query:
            return SkillResult(False, error_message="Missing grounding query", diagnosis_code="missing_query")
        config = _navigation_config(context)
        try:
            embodiment = _embodiment(context, config)
            embodiment.ensure_ready()
            observation = embodiment.observe_rgbd()
            detections = _detector(context, config).detect(observation.rgb, query)
            values = [item.to_dict() if hasattr(item, "to_dict") else {
                "label": getattr(item, "label", query),
                "bbox": list(getattr(item, "bbox", ())),
                "confidence": float(getattr(item, "confidence", 0.0)),
            } for item in detections]
        except Exception as exc:
            return SkillResult(False, error_message=f"GroundingDINO detection failed: {exc}", diagnosis_code="detection_failed")
        return SkillResult(True, result_data={"query": query, "detections": values})


__all__ = [
    "AStarPlanSkill", "GroundingDINODetectSkill", "OneRINGNavigationSkill",
]
