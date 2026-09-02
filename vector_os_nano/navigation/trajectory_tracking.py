# SPDX-License-Identifier: Apache-2.0
"""Stable world-frame tracking for learned and geometric navigation paths."""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

from vector_os_nano.integrations.online_mapper import RGBDObservation


def base_local_to_world(points: np.ndarray, base_pose: np.ndarray) -> np.ndarray:
    values = np.asarray(points, dtype=float)
    pose = np.asarray(base_pose, dtype=float).reshape(-1)
    yaw = float(pose[2])
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-math.sin(yaw), math.cos(yaw)])
    return pose[:2] + values[:, 0:1] * forward + values[:, 1:2] * left


def world_to_base_local(point_xy: np.ndarray, base_pose: np.ndarray) -> list[float]:
    point = np.asarray(point_xy, dtype=float).reshape(-1)
    pose = np.asarray(base_pose, dtype=float).reshape(-1)
    delta = point[:2] - pose[:2]
    yaw = float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return [float(c * delta[0] + s * delta[1]), float(-s * delta[0] + c * delta[1])]


@dataclass
class TrajectoryTrackingConfig:
    waypoint_tolerance_m: float = 0.16
    lookahead_distance_m: float = 0.35
    max_command_distance_m: float = 0.45
    target_filter_alpha: float = 0.45
    smoothing_window: int = 3
    replan_interval_steps: int = 3
    progress_window_steps: int = 10
    min_progress_m: float = 0.04
    max_replan_heading_change_rad: float = math.radians(100.0)
    max_unstable_replans: int = 2


class TrajectoryTracker:
    """Keep a path across control ticks and emit smooth base-local waypoints."""

    def __init__(self, config: TrajectoryTrackingConfig | None = None) -> None:
        self.config = config or TrajectoryTrackingConfig()
        self.clear(reset_history=True)

    def clear(self, *, reset_history: bool = False) -> None:
        self._path = np.empty((0, 2), dtype=float)
        self._index = 0
        self._planned_at = -10**9
        self._planner = ""
        self._last_command: np.ndarray | None = None
        self._last_command_index: int | None = None
        if reset_history or not hasattr(self, "_positions"):
            self._positions: deque[np.ndarray] = deque(
                maxlen=max(2, int(self.config.progress_window_steps))
            )
            self._last_plan_heading: float | None = None
            self._unstable_replans = 0
            self._fallback_events: list[dict[str, Any]] = []
            self._plans = 0
            self._commands = 0

    @property
    def planner(self) -> str:
        return self._planner

    def needs_plan(self, step_index: int) -> bool:
        return (
            self._index >= len(self._path)
            or int(step_index) - self._planned_at >= max(1, int(self.config.replan_interval_steps))
        )

    def observe(self, observation: RGBDObservation) -> dict[str, Any]:
        position = np.asarray(observation.base_pose, dtype=float)[:2].copy()
        self._positions.append(position)
        if (
            self._index < len(self._path)
            and float(np.linalg.norm(self._path[-1] - position))
            <= self.config.waypoint_tolerance_m
        ):
            self._index = len(self._path)
        if len(self._positions) < self._positions.maxlen or self._commands == 0:
            return {"stalled": False}
        displacement = float(np.linalg.norm(self._positions[-1] - self._positions[0]))
        return {
            "stalled": displacement < self.config.min_progress_m,
            "window_displacement_m": displacement,
        }

    def set_plan(
        self,
        result: dict[str, Any],
        observation: RGBDObservation,
        *,
        step_index: int,
    ) -> dict[str, Any]:
        raw = np.asarray(result.get("trajectory") or [], dtype=float)
        if raw.ndim != 2 or raw.shape[0] == 0 or raw.shape[1] < 2:
            self.clear()
            return {"accepted": False, "reason": "invalid_trajectory"}
        raw = raw[np.all(np.isfinite(raw[:, :2]), axis=1)]
        if not len(raw):
            self.clear()
            return {"accepted": False, "reason": "nonfinite_trajectory"}

        frame = str(result.get("trajectory_frame", "base_local"))
        if frame == "world":
            world = raw[:, :2].copy()
        else:
            world = base_local_to_world(raw, observation.base_pose)
            frame = "base_local"
        world = self._smooth(world)

        base = np.asarray(observation.base_pose, dtype=float)[:2]
        useful = np.linalg.norm(world - base, axis=1) > self.config.waypoint_tolerance_m * 0.5
        if np.any(useful):
            world = world[np.argmax(useful):]
        if not len(world):
            self.clear()
            return {"accepted": False, "reason": "trajectory_at_robot"}

        heading = math.atan2(world[min(len(world) - 1, 2), 1] - base[1],
                             world[min(len(world) - 1, 2), 0] - base[0])
        heading_change = 0.0
        if self._last_plan_heading is not None:
            heading_change = abs(math.atan2(
                math.sin(heading - self._last_plan_heading),
                math.cos(heading - self._last_plan_heading),
            ))
            if heading_change > self.config.max_replan_heading_change_rad:
                self._unstable_replans += 1
            else:
                self._unstable_replans = max(0, self._unstable_replans - 1)
        self._last_plan_heading = heading

        self._path = world
        self._index = 0
        self._planned_at = int(step_index)
        self._planner = str(result.get("planner", "unknown"))
        self._plans += 1
        return {
            "accepted": True,
            "frame": frame,
            "points": len(world),
            "heading_change_rad": heading_change,
            "unstable": self._unstable_replans >= self.config.max_unstable_replans,
        }

    def next_waypoint(self, observation: RGBDObservation) -> tuple[list[float] | None, dict[str, Any]]:
        if self._index >= len(self._path):
            return None, {"path_complete": True}
        base = np.asarray(observation.base_pose, dtype=float)[:2]
        while self._index < len(self._path) - 1:
            distance = float(np.linalg.norm(self._path[self._index] - base))
            if distance > self.config.waypoint_tolerance_m:
                break
            self._index += 1

        target_index = self._index
        while target_index < len(self._path) - 1:
            if float(np.linalg.norm(self._path[target_index] - base)) >= self.config.lookahead_distance_m:
                break
            target_index += 1
        desired = self._path[target_index].copy()

        if self._last_command is not None and self._last_command_index != target_index:
            alpha = float(np.clip(self.config.target_filter_alpha, 0.0, 1.0))
            desired = (1.0 - alpha) * self._last_command + alpha * desired
        delta = desired - base
        distance = float(np.linalg.norm(delta))
        if distance > self.config.max_command_distance_m > 0.0:
            desired = base + delta * (self.config.max_command_distance_m / distance)
            distance = self.config.max_command_distance_m

        self._last_command = desired
        self._last_command_index = target_index
        self._commands += 1
        return world_to_base_local(desired, observation.base_pose), {
            "path_complete": False,
            "path_index": self._index,
            "target_index": target_index,
            "command_distance_m": distance,
            "world_target": desired.tolist(),
        }

    def record_fallback(self, reason: str, step_index: int) -> None:
        self._fallback_events.append({"reason": str(reason), "step": int(step_index)})
        self.clear()
        self._positions.clear()

    def summary(self) -> dict[str, Any]:
        return {
            "plans": self._plans,
            "commands": self._commands,
            "active_planner": self._planner,
            "path_index": self._index,
            "path_points": len(self._path),
            "unstable_replans": self._unstable_replans,
            "fallback_events": list(self._fallback_events),
        }

    def _smooth(self, points: np.ndarray) -> np.ndarray:
        window = max(1, int(self.config.smoothing_window))
        if window <= 1 or len(points) <= 2:
            return points.copy()
        result = points.copy()
        radius = window // 2
        for index in range(1, len(points) - 1):
            start = max(0, index - radius)
            stop = min(len(points), index + radius + 1)
            result[index] = np.mean(points[start:stop], axis=0)
        return result


__all__ = [
    "TrajectoryTracker",
    "TrajectoryTrackingConfig",
    "base_local_to_world",
    "world_to_base_local",
]
