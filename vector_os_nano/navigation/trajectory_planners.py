# SPDX-License-Identifier: Apache-2.0
"""Interchangeable local trajectory planners used by generic navigation."""
from __future__ import annotations

import heapq
import math
import base64
import io
from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

import numpy as np
from PIL import Image

from vector_os_nano.integrations.online_mapper import RGBDObservation


@runtime_checkable
class TrajectoryPlanner(Protocol):
    name: str
    def reset(self, intrinsics: np.ndarray) -> dict[str, Any]: ...
    def plan(
        self, observation: RGBDObservation, *, mode: str, goal: list[float] | None
    ) -> dict[str, Any]: ...


@dataclass
class AStarPlannerConfig:
    resolution_m: float = 0.15
    forward_range_m: float = 6.0
    lateral_range_m: float = 3.0
    robot_radius_m: float = 0.35
    depth_stride: int = 8
    ground_clearance_m: float = 0.12
    max_obstacle_height_m: float = 2.5
    global_resolution_m: float = 0.05
    global_max_expansions: int = 500000


class AStarTrajectoryPlanner:
    """Local RGB-D occupancy-grid A* planner."""

    name = "astar"

    def __init__(self, config: AStarPlannerConfig | None = None) -> None:
        self.config = config or AStarPlannerConfig()
        self._intrinsics: np.ndarray | None = None
        self._global_free: np.ndarray | None = None
        self._global_world_to_map: np.ndarray | None = None
        self._global_map_to_world: np.ndarray | None = None
        self._global_factor = 1
        self._global_metadata: dict[str, Any] = {}

    def reset(self, intrinsics: np.ndarray) -> dict[str, Any]:
        self._intrinsics = np.asarray(intrinsics, dtype=np.float32)
        # A planner instance may be cached by the agent across navigation
        # tasks. Never carry a previous robot/scene's global occupancy map into
        # a new RBY-1 episode (and never consult Go2's persistent
        # ~/.vector_os_nano/terrain_map.npz; this planner has no disk-map path).
        self._global_free = None
        self._global_world_to_map = None
        self._global_map_to_world = None
        self._global_factor = 1
        self._global_metadata = {}
        return {"ok": True, "planner": self.name}

    def plan(
        self, observation: RGBDObservation, *, mode: str, goal: list[float] | None
    ) -> dict[str, Any]:
        self._intrinsics = np.asarray(observation.intrinsics, dtype=np.float32)
        target = self._goal_xy(observation, mode, goal)
        occupancy = self._occupancy(observation)
        start = self._xy_to_cell(0.0, 0.0, occupancy.shape)
        finish = self._xy_to_cell(float(target[0]), float(target[1]), occupancy.shape)
        occupancy[start] = False
        occupancy[finish] = False
        cells = self._astar(occupancy, start, finish)
        if not cells:
            return {
                "trajectory": [], "planner": self.name, "reason": "no_path",
                "occupancy_grid": {
                    "shape": list(occupancy.shape),
                    "occupied_cells": int(np.count_nonzero(occupancy)),
                    "occupied_ratio": float(np.mean(occupancy)),
                    "robot_radius_m": float(self.config.robot_radius_m),
                },
            }
        points = [self._cell_to_xy(cell, occupancy.shape) for cell in cells]
        # Return a compact receding-horizon trajectory, preserving endpoints.
        step = max(1, len(points) // 12)
        compact = points[::step]
        if compact[-1] != points[-1]:
            compact.append(points[-1])
        trajectory: list[list[float]] = []
        for index, (x, y) in enumerate(compact):
            if index + 1 < len(compact):
                nx, ny = compact[index + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = trajectory[-1][2] if trajectory else 0.0
            trajectory.append([float(x), float(y), float(yaw)])
        return {
            "trajectory": trajectory,
            "planner": self.name,
            "trajectory_frame": "base_local",
            "occupancy_grid": {
                "shape": list(occupancy.shape),
                "occupied_cells": int(np.count_nonzero(occupancy)),
                "occupied_ratio": float(np.mean(occupancy)),
                "robot_radius_m": float(self.config.robot_radius_m),
            },
        }

    def set_global_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not payload.get("success", False):
            raise RuntimeError(str(payload.get("error", "global occupancy unavailable")))
        raw = base64.b64decode(str(payload["data"]), validate=True)
        free = np.asarray(Image.open(io.BytesIO(raw)).convert("L")) > 0
        px_per_m = float(payload["px_per_m"])
        factor = max(1, int(round(px_per_m * self.config.global_resolution_m)))
        h = (free.shape[0] // factor) * factor
        w = (free.shape[1] // factor) * factor
        free = free[:h, :w]
        if factor > 1:
            free = free.reshape(h // factor, factor, w // factor, factor).all(axis=(1, 3))
        self._global_free = free
        self._global_world_to_map = np.asarray(payload["world_to_map"], dtype=float)
        self._global_map_to_world = np.asarray(payload["map_to_world"], dtype=float)
        self._global_factor = factor
        self._global_metadata = {
            "shape": list(free.shape), "source_shape": list(payload.get("shape", [])),
            "resolution_m": factor / px_per_m, "px_per_m": px_per_m,
            "robot_radius_m": float(payload.get("robot_radius_m", 0.0)),
        }
        return dict(self._global_metadata)

    @property
    def has_global_map(self) -> bool:
        return self._global_free is not None

    def plan_global(
        self, observation: RGBDObservation, goal_world_xy: np.ndarray
    ) -> dict[str, Any]:
        if self._global_free is None:
            return {"trajectory": [], "planner": self.name, "reason": "global_map_unavailable"}
        start = self._world_to_global_cell(np.asarray(observation.base_pose)[:2])
        finish = self._world_to_global_cell(np.asarray(goal_world_xy)[:2])
        start = self._nearest_free(start)
        finish = self._nearest_free(finish)
        if start is None or finish is None:
            return {"trajectory": [], "planner": self.name, "reason": "global_endpoint_occupied", "global_map": dict(self._global_metadata)}
        cells = self._astar_bounded(~self._global_free, start, finish, self.config.global_max_expansions)
        if not cells:
            return {"trajectory": [], "planner": self.name, "reason": "global_no_path", "global_map": dict(self._global_metadata)}
        stride = max(1, len(cells) // 40)
        compact = cells[::stride]
        if compact[-1] != cells[-1]: compact.append(cells[-1])
        trajectory = []
        for index, cell in enumerate(compact):
            xy = self._global_cell_to_world(cell)
            next_xy = self._global_cell_to_world(compact[min(index + 1, len(compact) - 1)])
            yaw = math.atan2(next_xy[1] - xy[1], next_xy[0] - xy[0]) if index + 1 < len(compact) else (trajectory[-1][2] if trajectory else float(observation.base_pose[2]))
            trajectory.append([float(xy[0]), float(xy[1]), float(yaw)])
        return {"trajectory": trajectory, "planner": self.name, "trajectory_frame": "world", "global_map": dict(self._global_metadata), "global_path_cells": len(cells)}

    def _world_to_global_cell(self, xy: np.ndarray) -> tuple[int, int]:
        assert self._global_world_to_map is not None and self._global_free is not None
        world = np.array([float(xy[0]), float(xy[1]), 1.0]) if self._global_world_to_map.shape[1] == 3 else np.array([float(xy[0]), float(xy[1]), 0.0, 1.0])
        pixel = self._global_world_to_map @ world
        row, col = int(round(pixel[0] / self._global_factor)), int(round(pixel[1] / self._global_factor))
        return (int(np.clip(row, 0, self._global_free.shape[0] - 1)), int(np.clip(col, 0, self._global_free.shape[1] - 1)))

    def _global_cell_to_world(self, cell: tuple[int, int]) -> np.ndarray:
        assert self._global_map_to_world is not None
        row = cell[0] * self._global_factor + self._global_factor / 2.0
        col = cell[1] * self._global_factor + self._global_factor / 2.0
        world = self._global_map_to_world @ np.array([row, col, 1.0])
        return np.asarray(world[:2], dtype=float)

    def _nearest_free(self, cell: tuple[int, int], max_radius: int = 20) -> tuple[int, int] | None:
        assert self._global_free is not None
        if self._global_free[cell]: return cell
        for radius in range(1, max_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in (-radius, radius):
                    point = (cell[0] + dr, cell[1] + dc)
                    if 0 <= point[0] < self._global_free.shape[0] and 0 <= point[1] < self._global_free.shape[1] and self._global_free[point]: return point
            for dc in range(-radius + 1, radius):
                for dr in (-radius, radius):
                    point = (cell[0] + dr, cell[1] + dc)
                    if 0 <= point[0] < self._global_free.shape[0] and 0 <= point[1] < self._global_free.shape[1] and self._global_free[point]: return point
        return None

    @staticmethod
    def _astar_bounded(
        occupancy: np.ndarray, start: tuple[int, int], finish: tuple[int, int],
        max_expansions: int,
    ) -> list[tuple[int, int]]:
        queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        cost = {start: 0.0}; visited: set[tuple[int, int]] = set()
        moves = ((1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
                 (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
                 (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))
        while queue and len(visited) < int(max_expansions):
            _estimate, current_cost, current = heapq.heappop(queue)
            if current in visited: continue
            visited.add(current)
            if current == finish:
                path = [finish]
                while path[-1] != start: path.append(parent[path[-1]])
                path.reverse(); return path
            for dr, dc, step_cost in moves:
                neighbor = (current[0] + dr, current[1] + dc)
                if not (0 <= neighbor[0] < occupancy.shape[0] and 0 <= neighbor[1] < occupancy.shape[1]) or occupancy[neighbor]: continue
                new_cost = current_cost + step_cost
                if new_cost >= cost.get(neighbor, float("inf")): continue
                cost[neighbor] = new_cost; parent[neighbor] = current
                heuristic = math.hypot(finish[0] - neighbor[0], finish[1] - neighbor[1])
                heapq.heappush(queue, (new_cost + heuristic, new_cost, neighbor))
        return []

    def _goal_xy(
        self, observation: RGBDObservation, mode: str, goal: list[float] | None
    ) -> np.ndarray:
        if goal is None:
            return np.array([1.0, 0.0], dtype=float)
        values = np.asarray(goal, dtype=float).reshape(-1)
        if mode == "pointgoal" and values.size >= 2:
            return values[:2]
        if mode == "pixelgoal" and values.size >= 2:
            u = int(np.clip(round(values[0]), 0, observation.depth.shape[1] - 1))
            v = int(np.clip(round(values[1]), 0, observation.depth.shape[0] - 1))
            patch = observation.depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
            valid = patch[np.isfinite(patch) & (patch > 0.1)]
            z = float(np.median(valid)) if valid.size else 1.5
            fx = max(float(observation.intrinsics[0, 0]), 1e-6)
            lateral = -(float(u) - float(observation.intrinsics[0, 2])) * z / fx
            return np.array([z, lateral], dtype=float)
        return np.array([1.0, 0.0], dtype=float)

    def _occupancy(self, observation: RGBDObservation) -> np.ndarray:
        cfg = self.config
        rows = int(math.ceil(cfg.forward_range_m / cfg.resolution_m)) + 1
        cols = int(math.ceil(2.0 * cfg.lateral_range_m / cfg.resolution_m)) + 1
        grid = np.zeros((rows, cols), dtype=bool)
        depth = np.asarray(observation.depth, dtype=np.float32)
        k = np.asarray(self._intrinsics, dtype=float)
        stride = max(1, int(cfg.depth_stride))
        pose = np.asarray(observation.camera_pose, dtype=float)
        world_points: list[tuple[float, float, float, float, float]] = []
        for v in range(0, depth.shape[0], stride):
            for u in range(0, depth.shape[1], stride):
                z = float(depth[v, u])
                if not math.isfinite(z) or z <= 0.15 or z > cfg.forward_range_m:
                    continue
                lateral = -(u - k[0, 2]) * z / max(k[0, 0], 1e-6)
                if abs(lateral) > cfg.lateral_range_m:
                    continue
                if pose.shape == (4, 4) and np.all(np.isfinite(pose)):
                    camera_point = np.array([
                        (u - k[0, 2]) * z / max(k[0, 0], 1e-6),
                        (v - k[1, 2]) * z / max(k[1, 1], 1e-6), z, 1.0,
                    ], dtype=float)
                    world = pose @ camera_point
                    world_points.append((z, lateral, float(world[2]), float(u), float(v)))
                else:
                    world_points.append((z, lateral, 0.0, float(u), float(v)))

        if not world_points:
            return grid
        heights = np.asarray([item[2] for item in world_points], dtype=float)
        if pose.shape == (4, 4) and np.all(np.isfinite(pose)) and float(np.ptp(heights)) > cfg.ground_clearance_m:
            ground_z = float(np.percentile(heights, 10.0))
        else:
            # A flat/invalid pose-depth pair is common in tests and in sensors
            # that report a planar range image. Do not turn the entire plane
            # into a wall; require a real depth/height discontinuity instead.
            ground_z = float(np.min(heights))
            if float(np.ptp(heights)) <= cfg.ground_clearance_m:
                return grid

        for z, lateral, world_z, u, v in world_points:
            obstacle_height = world_z - ground_z
            if obstacle_height < cfg.ground_clearance_m or obstacle_height > cfg.max_obstacle_height_m:
                continue
            grid[self._xy_to_cell(z, lateral, grid.shape)] = True
        radius = max(1, int(math.ceil(cfg.robot_radius_m / cfg.resolution_m)))
        occupied = np.argwhere(grid)
        for row, col in occupied:
            r0, r1 = max(0, row - radius), min(rows, row + radius + 1)
            c0, c1 = max(0, col - radius), min(cols, col + radius + 1)
            grid[r0:r1, c0:c1] = True
        return grid

    def _xy_to_cell(self, x: float, y: float, shape: tuple[int, int]) -> tuple[int, int]:
        cfg = self.config
        row = int(round(np.clip(x, 0.0, cfg.forward_range_m) / cfg.resolution_m))
        col = int(round((np.clip(y, -cfg.lateral_range_m, cfg.lateral_range_m)
                         + cfg.lateral_range_m) / cfg.resolution_m))
        return min(shape[0] - 1, row), min(shape[1] - 1, col)

    def _cell_to_xy(self, cell: tuple[int, int], shape: tuple[int, int]) -> tuple[float, float]:
        del shape
        row, col = cell
        cfg = self.config
        return row * cfg.resolution_m, col * cfg.resolution_m - cfg.lateral_range_m

    @staticmethod
    def _astar(
        occupancy: np.ndarray, start: tuple[int, int], finish: tuple[int, int]
    ) -> list[tuple[int, int]]:
        queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        cost = {start: 0.0}
        visited: set[tuple[int, int]] = set()
        moves = ((1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
                 (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))
        while queue:
            _estimate, current_cost, current = heapq.heappop(queue)
            if current in visited:
                continue
            visited.add(current)
            if current == finish:
                path = [finish]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                path.reverse()
                return path
            for dr, dc, step_cost in moves:
                neighbor = (current[0] + dr, current[1] + dc)
                if not (0 <= neighbor[0] < occupancy.shape[0] and
                        0 <= neighbor[1] < occupancy.shape[1]):
                    continue
                if occupancy[neighbor]:
                    continue
                new_cost = current_cost + step_cost
                if new_cost >= cost.get(neighbor, float("inf")):
                    continue
                cost[neighbor] = new_cost
                parent[neighbor] = current
                heuristic = math.hypot(finish[0] - neighbor[0], finish[1] - neighbor[1])
                heapq.heappush(queue, (new_cost + heuristic, new_cost, neighbor))
        return []


def planners_from_config(config: dict[str, Any]) -> AStarTrajectoryPlanner:
    astar_raw = dict(config.get("astar", {}) or {})
    return AStarTrajectoryPlanner(AStarPlannerConfig(**{
        key: astar_raw[key] for key in AStarPlannerConfig.__dataclass_fields__ if key in astar_raw
    }))


__all__ = [
    "AStarPlannerConfig", "AStarTrajectoryPlanner", "TrajectoryPlanner",
    "planners_from_config",
]
