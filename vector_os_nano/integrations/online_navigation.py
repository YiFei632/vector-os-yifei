# SPDX-License-Identifier: Apache-2.0
"""OneRING-first closed-loop navigation with local RGB-D A*."""
from __future__ import annotations

import base64, io, logging, math, time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np
from PIL import Image

from vector_os_nano.integrations.online_mapper import OnlineSemanticMapper, RGBDObservation
from vector_os_nano.navigation.embodiment import NavigationEmbodiment
from vector_os_nano.navigation.trajectory_planners import TrajectoryPlanner
from vector_os_nano.navigation.trajectory_tracking import TrajectoryTracker, TrajectoryTrackingConfig


log = logging.getLogger(__name__)


def _decode_png(payload: dict[str, Any], *, depth: bool = False) -> np.ndarray:
    raw = base64.b64decode(str(payload.get("data", "")), validate=True)
    value = np.asarray(Image.open(io.BytesIO(raw)))
    if depth: return value.astype(np.float32) / float(payload.get("scale", 10000.0))
    if value.ndim == 2: value = np.repeat(value[..., None], 3, axis=2)
    return value[..., :3].astype(np.uint8)


def decode_rgbd_result(result: dict[str, Any]) -> RGBDObservation:
    if not result.get("success", True): raise RuntimeError(str(result.get("error", "RGB-D observation failed")))
    return RGBDObservation(rgb=_decode_png(dict(result["rgb"])), depth=_decode_png(dict(result["depth"]), depth=True), intrinsics=np.asarray(result["intrinsic"], dtype=np.float32), camera_pose=np.asarray(result["camera_pose"], dtype=np.float32), base_pose=np.asarray(result["base_pose"], dtype=np.float32), timestamp=float(result.get("timestamp", time.time())), state=dict(result.get("state", {})))


def world_goal_to_local(goal_xy: np.ndarray, base_pose: np.ndarray) -> list[float]:
    delta = np.asarray(goal_xy, dtype=float)[:2] - np.asarray(base_pose, dtype=float)[:2]
    yaw = float(base_pose[2])
    return [math.cos(yaw) * delta[0] + math.sin(yaw) * delta[1], -math.sin(yaw) * delta[0] + math.cos(yaw) * delta[1]]


def local_waypoint_to_world(point: list[float], base_pose: np.ndarray) -> list[float]:
    local = np.asarray(point, dtype=float).reshape(-1)
    if local.size < 2: raise ValueError("waypoint needs at least x and y")
    yaw = float(base_pose[2]); c, s = math.cos(yaw), math.sin(yaw)
    x = float(base_pose[0] + c * local[0] - s * local[1]); y = float(base_pose[1] + s * local[0] + c * local[1])
    heading = math.atan2(y - float(base_pose[1]), x - float(base_pose[0]))
    if math.hypot(local[0], local[1]) < 1e-4:
        heading = yaw + (float(local[2]) if local.size > 2 else 0.0)
    return [x, y, math.atan2(math.sin(heading), math.cos(heading))]


@dataclass
class OnlineNavigationConfig:
    max_steps: int = 100; goal_tolerance_m: float = 2.0; control_steps: int = 1; max_unverified_stops: int = 3; onering_control_steps: int = 3; max_result_history: int = 256; waypoint_tolerance_m: float = 0.16; lookahead_distance_m: float = 0.35; max_command_distance_m: float = 0.45; target_filter_alpha: float = 0.45; smoothing_window: int = 3; replan_interval_steps: int = 3

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "OnlineNavigationConfig":
        return cls(max_steps=int(raw.get("max_steps", 100)), goal_tolerance_m=float(raw.get("goal_tolerance_m", 2.0)), control_steps=int(raw.get("control_steps", 1)), max_unverified_stops=max(1, int(raw.get("max_unverified_stops", 3))), onering_control_steps=max(1, int(raw.get("onering_control_steps", 1))), max_result_history=max(16, int(raw.get("max_result_history", 256))), waypoint_tolerance_m=float(raw.get("waypoint_tolerance_m", 0.16)), lookahead_distance_m=float(raw.get("lookahead_distance_m", 0.35)), max_command_distance_m=float(raw.get("max_command_distance_m", 0.45)), target_filter_alpha=float(raw.get("target_filter_alpha", 0.45)), smoothing_window=max(1, int(raw.get("smoothing_window", 3))), replan_interval_steps=max(1, int(raw.get("replan_interval_steps", 3))))


class OnlineNavigator:
    """Run OneRING direct control and local A* target navigation."""
    def __init__(self, *, embodiment: NavigationEmbodiment, planner: TrajectoryPlanner, onering: Any, mapper: OnlineSemanticMapper, scene_graph: Any, config: OnlineNavigationConfig | None = None, timeout_s: float = 300.0) -> None:
        self.embodiment, self.planner, self.onering, self.mapper, self.scene_graph = embodiment, planner, onering, mapper, scene_graph
        self.config, self.timeout_s = config or OnlineNavigationConfig(), float(timeout_s)
        self.tracker = TrajectoryTracker(TrajectoryTrackingConfig(waypoint_tolerance_m=self.config.waypoint_tolerance_m, lookahead_distance_m=self.config.lookahead_distance_m, max_command_distance_m=self.config.max_command_distance_m, target_filter_alpha=self.config.target_filter_alpha, smoothing_window=self.config.smoothing_window, replan_interval_steps=self.config.replan_interval_steps))

    def navigate(self, target: str, *, instruction: str | None = None) -> dict[str, Any]:
        instruction = instruction or f"Navigate to the {target}."; updates = deque(maxlen=self.config.max_result_history); events = deque(maxlen=self.config.max_result_history); sources = {"semantic_map": 0, "onering": 0}; planners: dict[str, int] = {}; rejected = 0
        try:
            self.embodiment.ensure_ready(); prepare = getattr(self.embodiment, "prepare_navigation", None)
            if callable(prepare):
                prepared = prepare(target)
                if isinstance(prepared, dict) and prepared.get("prepared") is False and prepared.get("error"): return self._result(False, "target_prepare_failed", 0, sources, planners, updates, events, prepared, rejected)
            observation = self.embodiment.observe_rgbd()
        except Exception: self.mapper.close(); raise
        ring_ready = False; ring_disabled = False; ring_failures = 0; global_fallback_done = False; started = time.monotonic()
        try:
            self.planner.reset(observation.intrinsics)
            for step in range(self.config.max_steps):
                if time.monotonic() - started >= self.timeout_s: return self._result(False, "timeout", step, sources, planners, updates, events, None, rejected)
                # Decide from knowledge available before this frame. This
                # guarantees a fresh visual-only session sends its first RGB
                # observation to OneRING instead of grounding the whole scene
                # and immediately switching to A*.
                known = self._known_goal(target, observation.base_pose)
                # Semantic VLM mapping used to run synchronously here. A single
                # keyframe performed two remote VLM calls plus one
                # Grounding-DINO forward per described object, blocking the
                # visual control loop for 15-25 seconds. OneRING must always
                # consume the latest frame immediately. The legacy global
                # scene/map synchronization remains available only after three
                # consecutive OneRING failures.
                updates.append({
                    "recorded": False,
                    "reason": "deferred_during_navigation_control",
                    "timestamp": observation.timestamp,
                })
                if known is not None and float(np.linalg.norm(known - observation.base_pose[:2])) <= self.config.goal_tolerance_m: return self._result(True, "semantic_goal_reached", step, sources, planners, updates, events, None, rejected)
                if known is None and self.onering is not None and not ring_ready and not ring_disabled:
                    try: self.onering.reset(instruction); ring_ready = True
                    except Exception as exc: ring_failures += 1; events.append({"policy": "onering", "fallback": "astar", "failure_count": ring_failures, "error": f"{type(exc).__name__}: {exc}"}); ring_disabled = ring_failures >= 3
                if known is None and ring_ready:
                    try:
                        inference_started = time.perf_counter()
                        response = self.onering.step(navigation_rgb=observation.rgb, manipulation_rgb=None, instruction=instruction)
                        inference_ms = (time.perf_counter() - inference_started) * 1000.0
                        action = str(response.get("action", "")).strip().lower(); sources["onering"] += 1
                        events.append({"policy": "onering", "action": action, "action_index": response.get("action_index"), "latency_ms": {"client_round_trip": round(inference_ms, 3), **dict(response.get("latency_ms") or {})}})
                        log.info("OneRING step=%d action=%s round_trip_ms=%.1f", step, action, inference_ms)
                        if action in {"done", "end", "sub_done"}:
                            status = self._goal_status(target, observation)
                            if status.get("reached"): return self._result(True, "verified_goal_reached", step, sources, planners, updates, events, status, rejected)
                            rejected += 1
                            if rejected < self.config.max_unverified_stops:
                                continue
                            ring_failures += 1
                            ring_ready = False
                            ring_disabled = ring_failures >= 3
                            if not ring_disabled:
                                continue
                        if action in {"move_ahead", "move_back", "rotate_left", "rotate_right", "rotate_left_small", "rotate_right_small", "m", "b", "l", "r", "ls", "rs"}:
                            execute = getattr(self.embodiment, "execute_discrete_action", None)
                            if callable(execute):
                                execution_started = time.perf_counter()
                                motion = execute(action, observation, control_steps=self.config.onering_control_steps, max_yaw_step_rad=0.52)
                                execution_ms = (time.perf_counter() - execution_started) * 1000.0
                                events.append({"policy": "onering_execution", "action": action,
                                               "execution_ms": round(execution_ms, 3),
                                               "success": bool(motion.get("success", True)),
                                               "verified_motion": motion.get("verified_motion"),
                                               "execution_attempts": motion.get("execution_attempts"),
                                               "pose_before": motion.get("pose_before"),
                                               "pose_after": motion.get("pose_after"),
                                               "pose_target": motion.get("pose_target"),
                                               "translation_error_m": motion.get("translation_error_m"),
                                               "yaw_error_rad": motion.get("yaw_error_rad")})
                                if motion.get("success", True):
                                    ring_failures = 0
                                    observation_started = time.perf_counter()
                                    observation = self.embodiment.observe_rgbd()
                                    observation_ms = (time.perf_counter() - observation_started) * 1000.0
                                    events.append({"policy": "feedback_observation", "capture_transport_decode_ms": round(observation_ms, 3)})
                                    log.info("OneRING feedback step=%d execution_ms=%.1f observation_ms=%.1f", step, execution_ms, observation_ms)
                                    continue
                        ring_failures += 1; events.append({"policy": "onering", "fallback": "astar", "reason": "failed_or_unsupported_action", "failure_count": ring_failures}); ring_ready = False; ring_disabled = ring_failures >= 3
                    except Exception as exc: ring_failures += 1; events.append({"policy": "onering", "fallback": "astar", "failure_count": ring_failures, "error": f"{type(exc).__name__}: {exc}"}); ring_ready = False; ring_disabled = ring_failures >= 3
                if 0 < ring_failures < 3 and not ring_disabled and known is None:
                    # Re-open/synchronise the external target after a failed
                    # VLN action.  MolmoSpaces keeps task state independently
                    # from OneRING; merely resetting OneRING can otherwise
                    # leave the bridge waiting on a stale/completed task and
                    # subsequent waypoint messages are rejected.
                    prepare_retry = getattr(self.embodiment, "prepare_navigation", None)
                    if callable(prepare_retry):
                        try:
                            retry_state = prepare_retry(target)
                            events.append({"policy": "navigation_target_resync", "result": retry_state})
                        except Exception as exc:
                            events.append({"policy": "navigation_target_resync", "error": f"{type(exc).__name__}: {exc}"})
                    continue
                if ring_disabled and ring_failures >= 3 and not global_fallback_done:
                    global_sync = getattr(self.embodiment, "global_scene_sync", None)
                    if callable(global_sync):
                        try: events.append({"policy": "global_scene_sync", "result": global_sync(self.scene_graph)})
                        except Exception as exc: events.append({"policy": "global_scene_sync", "error": f"{type(exc).__name__}: {exc}"})
                    global_map = getattr(self.embodiment, "global_occupancy", None)
                    set_global_map = getattr(self.planner, "set_global_map", None)
                    if callable(global_map) and callable(set_global_map):
                        try: events.append({"policy": "global_occupancy", "result": set_global_map(global_map())})
                        except Exception as exc: events.append({"policy": "global_occupancy", "error": f"{type(exc).__name__}: {exc}"})
                    global_fallback_done = True
                known = self._known_goal(target, observation.base_pose)
                # A targetless local A* plan is an exploration command.  It is
                # useful for generic simulated bases, but is unsafe for the
                # RBY-1 MolmoSpaces adapter: after VLN failure there is no user
                # supplied destination, so do not send any movement at all.
                if known is None and bool(getattr(self.embodiment, "requires_explicit_navigation_goal", False)):
                    return self._result(
                        False,
                        "navigation_target_unavailable",
                        step,
                        sources,
                        planners,
                        updates,
                        events,
                        {"known": False, "reached": False, "target": target},
                        rejected,
                    )
                mode = "pointgoal" if known is not None else "nogoal"
                goal = world_goal_to_local(known, observation.base_pose) if known is not None else None
                if known is not None: sources["semantic_map"] += 1
                plan_global = getattr(self.planner, "plan_global", None)
                if ring_disabled and known is not None and bool(getattr(self.planner, "has_global_map", False)) and callable(plan_global):
                    plan = plan_global(observation, known)
                else:
                    plan = self.planner.plan(observation, mode=mode, goal=goal)
                plan.setdefault("requested_target", target)
                plan.setdefault("goal_world", known.tolist() if known is not None else None)
                name = str(plan.get("planner") or "astar"); accepted = self.tracker.set_plan(plan, observation, step_index=step)
                if not accepted.get("accepted"): return self._result(False, "planner_empty_trajectory", step, sources, planners, updates, events, None, rejected)
                planners[name] = planners.get(name, 0) + 1; point, detail = self.tracker.next_waypoint(observation); events.append({"planner": name, "tracking": detail})
                if point is None: return self._result(False, "planner_empty_trajectory", step, sources, planners, updates, events, None, rejected)
                result = self.embodiment.execute_local_waypoint(point, observation, control_steps=self.config.control_steps)
                if not result.get("success", True): return self._result(False, str(result.get("error", "waypoint_failed")), step, sources, planners, updates, events, None, rejected)
                observation = self.embodiment.observe_rgbd()
            return self._result(False, "max_steps", self.config.max_steps, sources, planners, updates, events, None, rejected)
        finally: self.mapper.close()

    def _known_goal(self, target: str, base_pose: np.ndarray) -> np.ndarray | None:
        finder = getattr(self.scene_graph, "find_objects_by_category", None)
        if not callable(finder): return None
        # SceneGraph historically used substring matching ("bottle" matched
        # "winebottle"). That is unsafe for navigation: a stale sampler object
        # could silently become the A* goal. Require an exact canonical match
        # on category/label/description while retaining common user aliases.
        aliases = {"fridge": "refrigerator", "trashcan": "trash can", "garbage can": "trash can"}
        requested = " ".join(str(target or "").lower().replace("_", " ").replace("-", " ").split())
        requested = aliases.get(requested, requested)
        candidates = finder(target) or []
        objects = []
        for obj in candidates:
            labels = [getattr(obj, "category", ""), getattr(obj, "description", "")]
            attrs = getattr(obj, "attributes", {})
            if isinstance(attrs, dict): labels.extend([attrs.get("name", ""), attrs.get("natural_name", "")])
            normalized = {
                " ".join(str(label or "").lower().replace("_", " ").replace("-", " ").split())
                for label in labels if str(label or "").strip()
            }
            normalized = {aliases.get(label, label) for label in normalized}
            if requested not in normalized:
                continue
            if np.isfinite(float(getattr(obj, "x", math.nan))) and np.isfinite(float(getattr(obj, "y", math.nan))):
                objects.append(obj)
        if not objects: return None
        obj = min(objects, key=lambda item: math.hypot(float(item.x) - base_pose[0], float(item.y) - base_pose[1])); return np.array([float(obj.x), float(obj.y)], dtype=float)

    def _goal_status(self, target: str, observation: RGBDObservation) -> dict[str, Any]:
        checker = getattr(self.embodiment, "check_navigation_goal", None)
        if callable(checker):
            try: return dict(checker(target, observation, tolerance_m=self.config.goal_tolerance_m))
            except Exception as exc: return {"known": False, "reached": False, "error": str(exc)}
        return {"known": False, "reached": False, "source": "unavailable"}

    @staticmethod
    def _result(success: bool, reason: str, steps: int, sources: dict[str, int], planners: dict[str, int], updates: Any, events: Any, status: dict[str, Any] | None, rejected: int) -> dict[str, Any]:
        result = {"success": bool(success), "terminal_reason": reason, "steps": int(steps), "planning_sources": dict(sources), "trajectory_planners": dict(planners), "map_updates": sum(bool(x.get("recorded")) for x in updates), "mapping": list(updates), "rejected_onering_stops": int(rejected), "trajectory_tracking_events": list(events)}
        if status is not None: result["goal_status"] = dict(status)
        return result


__all__ = ["OnlineNavigationConfig", "OnlineNavigator", "decode_rgbd_result", "local_waypoint_to_world", "world_goal_to_local"]
