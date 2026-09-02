# SPDX-License-Identifier: Apache-2.0
"""Robot-independent RGB-D navigation embodiment adapters."""
from __future__ import annotations

import math
import time
from typing import Any, Protocol, runtime_checkable

import numpy as np

from vector_os_nano.integrations.online_mapper import RGBDObservation


@runtime_checkable
class NavigationEmbodiment(Protocol):
    """Minimum contract needed by the closed-loop VLN controller."""

    @property
    def name(self) -> str: ...
    def ensure_ready(self) -> dict[str, Any]: ...
    def observe_rgbd(self) -> RGBDObservation: ...
    def execute_local_waypoint(
        self, point: list[float], observation: RGBDObservation, *, control_steps: int = 1,
        **control_options: Any,
    ) -> dict[str, Any]: ...
    def execute_discrete_action(
        self, action: str, observation: RGBDObservation, *, control_steps: int = 1,
        **control_options: Any,
    ) -> dict[str, Any]: ...
    def stop(self) -> None: ...


def _intrinsics_matrix(value: Any, width: int, height: int) -> np.ndarray:
    """Normalize Vector CameraIntrinsics, mappings, or matrices to K."""
    if value is None:
        from vector_os_nano.perception.depth_projection import d435_intrinsics
        value = d435_intrinsics(width, height)
    array = np.asarray(value)
    if array.shape == (3, 3):
        return array.astype(np.float32)
    if isinstance(value, dict):
        getter = value.get
    else:
        getter = lambda key, default=0.0: getattr(value, key, default)
    return np.array(
        [[float(getter("fx")), 0.0, float(getter("cx", width / 2.0))],
         [0.0, float(getter("fy")), float(getter("cy", height / 2.0))],
         [0.0, 0.0, 1.0]], dtype=np.float32,
    )


def _camera_pose_matrix(value: Any) -> np.ndarray:
    """Normalize a 4x4 pose or Vector's ``(cam_xpos, cam_xmat)`` tuple."""
    array = np.asarray(value) if not isinstance(value, tuple) else None
    if array is not None and array.shape == (4, 4):
        return array.astype(np.float32)
    if isinstance(value, tuple) and len(value) == 2:
        pose = np.eye(4, dtype=np.float32)
        # MuJoCo camera axes are right/up/backward (OpenGL), whereas the RGB-D
        # projection uses right/down/forward (OpenCV).
        rotation_gl = np.asarray(value[1], dtype=float).reshape(3, 3)
        pose[:3, :3] = rotation_gl @ np.diag([1.0, -1.0, -1.0])
        pose[:3, 3] = np.asarray(value[0], dtype=float).reshape(3)
        return pose
    return np.eye(4, dtype=np.float32)


def discrete_action_to_world_waypoint(
    action: str,
    base_pose: np.ndarray,
    *,
    move_m: float = 0.20,
    turn_rad: float = math.radians(15.0),
    small_turn_rad: float = math.radians(6.0),
) -> list[float] | None:
    """Convert OneRING's navigation subset into a world-frame waypoint."""
    name = str(action).strip().lower()
    pose = np.asarray(base_pose, dtype=float).reshape(-1)
    if pose.size < 3 or not np.all(np.isfinite(pose[:3])):
        return None
    x, y, yaw = map(float, pose[:3])
    if name in {"move_ahead", "m"}:
        x += move_m * math.cos(yaw)
        y += move_m * math.sin(yaw)
    elif name in {"move_back", "b"}:
        x -= move_m * math.cos(yaw)
        y -= move_m * math.sin(yaw)
    elif name in {"rotate_left", "l"}:
        yaw += turn_rad
    elif name in {"rotate_right", "r"}:
        yaw -= turn_rad
    elif name in {"rotate_left_small", "ls"}:
        yaw += small_turn_rad
    elif name in {"rotate_right_small", "rs"}:
        yaw -= small_turn_rad
    else:
        return None
    return [x, y, math.atan2(math.sin(yaw), math.cos(yaw))]


class VectorBaseNavigationEmbodiment:
    """Adapter for Go2 and any Vector base exposing RGB-D + locomotion APIs."""

    def __init__(self, base: Any, perception: Any = None, *, width: int = 320, height: int = 240) -> None:
        self.base = base
        self.perception = perception
        self.width = int(width)
        self.height = int(height)

    @property
    def name(self) -> str:
        return str(getattr(self.base, "name", type(self.base).__name__))

    def ensure_ready(self) -> dict[str, Any]:
        if self.base is None:
            raise RuntimeError("onering_navigation requires a mobile base")
        if not callable(getattr(self.base, "get_rgbd_frame", None)):
            raise RuntimeError(f"base {self.name!r} does not provide get_rgbd_frame()")
        return {"loaded": True, "robot": self.name}

    def observe_rgbd(self) -> RGBDObservation:
        rgb, depth = self.base.get_rgbd_frame(self.width, self.height)
        rgb = np.asarray(rgb, dtype=np.uint8)
        depth = np.asarray(depth, dtype=np.float32)
        get_intrinsics = getattr(self.perception, "get_intrinsics", None)
        intrinsics = _intrinsics_matrix(
            get_intrinsics() if callable(get_intrinsics) else None,
            rgb.shape[1], rgb.shape[0],
        )
        get_camera_pose = getattr(self.base, "get_camera_pose", None)
        camera_pose = _camera_pose_matrix(get_camera_pose() if callable(get_camera_pose) else None)
        position = np.asarray(self.base.get_position(), dtype=float).reshape(-1)
        base_pose = np.array(
            [position[0], position[1], float(self.base.get_heading())], dtype=np.float32
        )
        return RGBDObservation(
            rgb=rgb, depth=depth, intrinsics=intrinsics, camera_pose=camera_pose,
            base_pose=base_pose, timestamp=time.time(), state={"robot": self.name},
        )

    def execute_local_waypoint(
        self, point: list[float], observation: RGBDObservation, *, control_steps: int = 1,
        **control_options: Any,
    ) -> dict[str, Any]:
        del observation, control_options
        local = np.asarray(point, dtype=float).reshape(-1)
        if local.size < 2 or not np.all(np.isfinite(local[:2])):
            return {"success": False, "error": "invalid local waypoint"}
        distance = float(np.linalg.norm(local[:2]))
        if distance < 1e-3:
            return {"success": True, "distance": 0.0}
        heading = math.atan2(float(local[1]), float(local[0]))
        walk = getattr(self.base, "walk", None)
        if not callable(walk):
            return {"success": False, "error": f"base {self.name!r} does not provide walk()"}
        if abs(heading) > 0.12:
            yaw_rate = math.copysign(0.50, heading)
            if not bool(walk(vx=0.0, vy=0.0, vyaw=yaw_rate,
                             duration=max(0.20, abs(heading) / 0.50))):
                return {"success": False, "error": "base turn failed"}
        travel = min(distance, 0.60 * max(1, int(control_steps)))
        duration = max(0.20, travel / 0.30)
        ok = bool(walk(vx=0.30, vy=0.0, vyaw=0.0, duration=duration))
        return {"success": ok, "distance": travel, "local_waypoint": local.tolist()}


    def stop(self) -> None:
        stop = getattr(self.base, "stop", None)
        if callable(stop):
            stop()

    def execute_discrete_action(
        self, action: str, observation: RGBDObservation, *, control_steps: int = 1,
        **control_options: Any,
    ) -> dict[str, Any]:
        name = str(action).strip().lower()
        walk = getattr(self.base, "walk", None)
        if not callable(walk):
            return {"success": False, "error": f"base {self.name!r} does not provide walk()"}
        if name in {"rotate_left", "l", "rotate_right", "r", "rotate_left_small", "ls", "rotate_right_small", "rs"}:
            angle = (
                float(control_options.get("small_turn_rad", math.radians(6.0)))
                if "small" in name or name in {"ls", "rs"}
                else float(control_options.get("turn_rad", math.radians(30.0)))
            )
            sign = 1.0 if name in {"rotate_left", "l", "rotate_left_small", "ls"} else -1.0
            yaw_rate = sign * 0.50
            ok = bool(walk(vx=0.0, vy=0.0, vyaw=yaw_rate, duration=max(0.20, angle / 0.50)))
            return {"success": ok, "action": name, "turn_rad": sign * angle}
        if name in {"move_back", "b"}:
            move_m = float(control_options.get("move_m", 0.20))
            ok = bool(walk(vx=-0.20, vy=0.0, vyaw=0.0, duration=max(0.20, move_m / 0.20)))
            return {"success": ok, "action": name, "distance": -move_m}
        waypoint = discrete_action_to_world_waypoint(
            action, observation.base_pose,
            move_m=float(control_options.get("move_m", 0.20)),
            turn_rad=float(control_options.get("turn_rad", math.radians(30.0))),
            small_turn_rad=float(control_options.get("small_turn_rad", math.radians(6.0))),
        )
        if waypoint is None:
            return {"success": False, "error": f"unsupported discrete navigation action: {action}"}
        local = [float(control_options.get("move_m", 0.20)), 0.0, 0.0]
        return self.execute_local_waypoint(local, observation, control_steps=control_steps, **control_options)

    def check_navigation_goal(
        self, target: str, observation: RGBDObservation, *, tolerance_m: float
    ) -> dict[str, Any]:
        del target, observation, tolerance_m
        return {"known": False, "reached": False, "source": "vector_base"}

    def prepare_navigation(self, target: str) -> dict[str, Any]:
        return {"prepared": False, "target": target, "source": "vector_base"}


class MolmoSpacesRBY1NavigationEmbodiment:
    """RBY1 adapter; the generic controller never depends on an RBY1 base class."""

    def __init__(self, bridge: Any, runtime_context: dict[str, Any], *, camera: str,
                 timeout_s: float, expected_resolution: tuple[int, int] | None = None,
                 robot_base: Any = None) -> None:
        self.bridge = bridge
        self.runtime_context = dict(runtime_context)
        self.camera = str(camera)
        self.timeout_s = float(timeout_s)
        self.expected_resolution = expected_resolution
        # The bridge remains the source of RGB-D/scene state.  Locomotion is
        # intentionally owned by Vector's robot layer when one is supplied.
        self.robot_base = robot_base

    @property
    def name(self) -> str:
        return "molmospaces_rby1"

    @property
    def requires_explicit_navigation_goal(self) -> bool:
        """Whether a planner must have a concrete world target.

        MolmoSpaces' RBY-1 adapter deliberately has no targetless navigation
        mode.  In particular, its local A* planner's ``nogoal`` mode is an
        exploration fallback and must never be used to move the real robot.
        """
        return True

    def ensure_ready(self) -> dict[str, Any]:
        state = self.bridge.observe()
        if not state.get("loaded", False):
            state = self.bridge.reset(
                scene_name=self.runtime_context.get("scene_name"),
                seed=self.runtime_context.get("seed"), metadata=self.runtime_context,
            )
        return state

    def observe_rgbd(self) -> RGBDObservation:
        from vector_os_nano.integrations.online_navigation import decode_rgbd_result
        result = self.bridge.execute(
            "observe RGB-D",
            context={**self.runtime_context, "structured_action": "observe_rgbd", "camera": self.camera},
            mode="structured", timeout_s=self.timeout_s,
        )
        observation = decode_rgbd_result(result)
        if str(result.get("camera", "")) != self.camera:
            raise RuntimeError(
                f"RBY-1 returned camera {result.get('camera')!r}, expected {self.camera!r}"
            )
        if self.expected_resolution is not None:
            expected_w, expected_h = self.expected_resolution
            actual_h, actual_w = observation.rgb.shape[:2]
            if (actual_w, actual_h) != (expected_w, expected_h):
                raise RuntimeError(
                    f"RBY-1 {self.camera} stream is {actual_w}x{actual_h}; "
                    f"expected GoPro stream {expected_w}x{expected_h}"
                )
        return observation

    def execute_local_waypoint(
        self, point: list[float], observation: RGBDObservation, *, control_steps: int = 1,
        **control_options: Any,
    ) -> dict[str, Any]:
        local = np.asarray(point, dtype=float).reshape(-1)
        if self.robot_base is not None:
            if local.size < 2 or not np.all(np.isfinite(local[:2])):
                return {"success": False, "error": "invalid local waypoint"}
            distance = float(np.linalg.norm(local[:2]))
            if distance < 1e-3:
                return {"success": True, "distance": 0.0, "controller": "vector_robot_layer"}
            walk = getattr(self.robot_base, "walk", None)
            if not callable(walk):
                return {"success": False, "error": "RBY1 robot layer has no walk()"}
            heading = math.atan2(float(local[1]), float(local[0]))
            if abs(heading) > 0.10:
                if not bool(walk(vx=0.0, vy=0.0, vyaw=math.copysign(0.50, heading),
                                 duration=max(0.20, abs(heading) / 0.50))):
                    return {"success": False, "error": "RBY1 turn command failed"}
            travel = min(distance, 0.45 * max(1, int(control_steps)))
            ok = bool(walk(vx=0.30, vy=0.0, vyaw=0.0, duration=max(0.20, travel / 0.30)))
            return {"success": ok, "distance": travel, "controller": "vector_robot_layer"}
        from vector_os_nano.integrations.online_navigation import local_waypoint_to_world
        waypoint = local_waypoint_to_world(point, observation.base_pose)
        return self._execute_world_waypoint(
            waypoint, observation, control_steps=control_steps,
            instruction="execute A* trajectory waypoint", **control_options,
        )

    def stop(self) -> None:
        if self.robot_base is not None:
            stop = getattr(self.robot_base, "stop", None)
            if callable(stop):
                stop()
        self.bridge.stop()

    def global_scene_sync(self, scene_graph: Any) -> dict[str, Any]:
        """Explicit legacy fallback: sync MolmoSpaces global objects once."""
        from types import SimpleNamespace
        from vector_os_nano.integrations.molmospaces.perception import sync_scene_to_context
        result = self.bridge.execute(
            "global fallback scene observation",
            context={**self.runtime_context, "structured_action": "list_scene_objects"},
            mode="structured", timeout_s=self.timeout_s,
        )
        objects = [dict(item) for item in result.get("objects", []) if isinstance(item, dict)]
        context = SimpleNamespace(world_model=None, services={"spatial_memory": scene_graph})
        return {"objects": len(objects), "sync": sync_scene_to_context(context, objects)}

    def global_occupancy(self) -> dict[str, Any]:
        """Fetch MolmoSpaces' cached global occupancy map for A* fallback."""
        return self.bridge.execute(
            "get global occupancy map",
            context={**self.runtime_context, "structured_action": "global_occupancy"},
            mode="structured", timeout_s=self.timeout_s,
        )

    def execute_discrete_action(
        self, action: str, observation: RGBDObservation, *, control_steps: int = 1,
        **control_options: Any,
    ) -> dict[str, Any]:
        waypoint = discrete_action_to_world_waypoint(
            action, observation.base_pose,
            move_m=float(control_options.get("move_m", 0.20)),
            turn_rad=float(control_options.get("turn_rad", math.radians(15.0))),
            small_turn_rad=float(control_options.get("small_turn_rad", math.radians(6.0))),
        )
        if waypoint is None:
            return {"success": False, "error": f"unsupported discrete navigation action: {action}"}
        if self.robot_base is not None:
            walk = getattr(self.robot_base, "walk", None)
            if not callable(walk):
                return {"success": False, "error": "RBY1 robot layer has no walk()"}
            name = str(action).strip().lower()
            if name in {"rotate_left", "l", "rotate_right", "r", "rotate_left_small", "ls", "rotate_right_small", "rs"}:
                angle = float(control_options.get(
                    "small_turn_rad" if "small" in name or name in {"ls", "rs"} else "turn_rad",
                    math.radians(6.0) if "small" in name or name in {"ls", "rs"} else math.radians(15.0),
                ))
                sign = 1.0 if name in {"rotate_left", "l", "rotate_left_small", "ls"} else -1.0
                ok = bool(walk(vx=0.0, vy=0.0, vyaw=sign * 0.50,
                               duration=max(0.20, angle / 0.50)))
                return {"success": ok, "action": name, "turn_rad": sign * angle,
                        "controller": "vector_robot_layer"}
            if name in {"move_ahead", "m"}:
                distance = float(control_options.get("move_m", 0.20))
                ok = bool(walk(vx=0.30, vy=0.0, vyaw=0.0,
                               duration=max(0.20, distance / 0.30)))
                return {"success": ok, "action": name, "distance": distance,
                        "controller": "vector_robot_layer"}
            if name in {"move_back", "b"}:
                distance = float(control_options.get("move_m", 0.20))
                ok = bool(walk(vx=-0.20, vy=0.0, vyaw=0.0,
                               duration=max(0.20, distance / 0.20)))
                return {"success": ok, "action": name, "distance": -distance,
                        "controller": "vector_robot_layer"}
        return self._execute_world_waypoint(
            waypoint, observation, control_steps=control_steps,
            instruction="execute OneRING discrete navigation action",
            max_translation_m=float(control_options.get("max_translation_m", 0.50)),
            max_yaw_step_rad=float(control_options.get("max_yaw_step_rad", 0.52)),
            onering_action=str(action),
        )

    @staticmethod
    def _result_base_pose(result: dict[str, Any]) -> np.ndarray | None:
        values = (((result.get("state") or {}).get("robot") or {}).get("base_pose"))
        if not isinstance(values, (list, tuple)) or len(values) < 7:
            return None
        x, y = float(values[0]), float(values[1])
        qw, qx, qy, qz = map(float, values[3:7])
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return np.array([x, y, yaw], dtype=float)

    def _execute_world_waypoint(
        self, waypoint: list[float], observation: RGBDObservation, *,
        control_steps: int, instruction: str, **options: Any,
    ) -> dict[str, Any]:
        """Execute an atomic RBY-1 action until its pose target is reached.
        
        Key invariant: for pure-rotation waypoints (x,y unchanged from start),
        retries update the target's x,y to the robot's current pose. This
        prevents drift-induced oscillation where the robot tries to correct
        a few cm of translation while rotating, causing back-and-forth motion.
        """
        max_attempts = max(1, int(options.pop("waypoint_max_attempts", 5)))
        translation_tolerance = float(options.pop("translation_tolerance_m", 0.03))
        yaw_tolerance = float(options.pop("yaw_tolerance_rad", math.radians(2.0)))
        before = np.asarray(observation.base_pose, dtype=float)[:3]
        target = np.asarray(waypoint, dtype=float)[:3]
        # Detect pure rotation: target x,y is within 2 cm of start
        is_rotation = float(np.linalg.norm(target[:2] - before[:2])) < 0.02
        last: dict[str, Any] = {}
        after: np.ndarray | None = None
        for attempt in range(max_attempts):
            try:
                last = self.bridge.execute(
                    instruction,
                    context={**self.runtime_context, "structured_action": "step_waypoint",
                             "waypoint": target.tolist(), "control_steps": int(control_steps),
                             **options},
                    mode="structured", timeout_s=self.timeout_s,
                )
            except Exception as exc:
                last = {"success": False, "error": f"{type(exc).__name__}: {exc}"}
            after = self._result_base_pose(last)
            if after is not None:
                translation_error = float(np.linalg.norm(target[:2] - after[:2]))
                yaw_error = abs(math.atan2(
                    math.sin(float(target[2] - after[2])),
                    math.cos(float(target[2] - after[2])),
                ))
                # Pure rotation: use looser translation tolerance (10 cm) because
                # MolmoSpaces' step_waypoint controller drifts 3-5 cm during
                # in-place rotation. Tight translation tolerance would cause
                # retries that try to correct harmless drift, creating oscillation.
                eff_trans_tol = 0.10 if is_rotation else translation_tolerance
                if (
                    last.get("success", True)
                    and translation_error <= eff_trans_tol
                    and yaw_error <= yaw_tolerance
                ):
                    return {
                        **last,
                        "verified_motion": True,
                        "execution_attempts": attempt + 1,
                        "pose_before": before.tolist(),
                        "pose_after": after.tolist(),
                        "pose_target": target.tolist(),
                        "translation_error_m": translation_error,
                        "yaw_error_rad": yaw_error,
                    }
            if attempt + 1 < max_attempts:
                # On retry: for pure rotation, update x,y to current pose so
                # the robot only finishes the remaining yaw, instead of trying
                # to correct drift-induced translation back to the original spot.
                # Also, if the robot overshot the target yaw, accept the current
                # yaw as the new target so the retry confirms the current pose
                # instead of commanding a reverse rotation.
                if is_rotation and after is not None:
                    target[:2] = after[:2]
                    if abs(math.atan2(
                        math.sin(float(target[2] - after[2])),
                        math.cos(float(target[2] - after[2])),
                    )) < yaw_tolerance:
                        target[2] = float(after[2])
                # Rendering/observing once initializes lazy task sensors and
                # controllers that can reject the first command after reset.
                try:
                    self.observe_rgbd()
                except Exception:
                    pass
        return {
            **last, "success": False, "verified_motion": False,
            "execution_attempts": max_attempts,
            "pose_before": before.tolist(),
            "pose_after": after.tolist() if after is not None else None,
            "pose_target": target.tolist(),
            "error": last.get("error") or "RBY-1 did not reach the discrete action target",
        }

    def check_navigation_goal(
        self, target: str, observation: RGBDObservation, *, tolerance_m: float
    ) -> dict[str, Any]:
        """Verify a OneRING stop against MolmoSpaces scene geometry."""
        result = self.bridge.execute(
            f"navigate to the {target}",
            context={**self.runtime_context, "structured_action": "navigation_status",
                     "target_types": [target], "tolerance_m": float(tolerance_m)},
            mode="structured", timeout_s=self.timeout_s,
        )
        if not isinstance(result, dict):
            return {"known": False, "reached": False, "source": "molmospaces"}
        result.setdefault("target", target)
        result.setdefault("tolerance_m", float(tolerance_m))
        result.setdefault("source", "molmospaces")
        return result

    def prepare_navigation(self, target: str) -> dict[str, Any]:
        result = self.bridge.execute(
            f"navigate to the {target}",
            context={**self.runtime_context, "structured_action": "prepare_navigation",
                     "target_types": [str(target)]},
            mode="structured", timeout_s=self.timeout_s,
        )
        if not isinstance(result, dict):
            return {"prepared": False, "error": "MolmoSpaces navigation target sync returned invalid data"}
        result.setdefault("prepared", bool(result.get("success", True)))
        result.setdefault("external_navigation_target", str(target))
        return result


def embodiment_from_context(context: Any, config: dict[str, Any]) -> NavigationEmbodiment:
    """Resolve an injected adapter, RBY1 bridge, or native Vector base."""
    injected = context.services.get("navigation_embodiment")
    if injected is not None:
        return injected(context) if callable(injected) else injected
    if isinstance(context.services.get("molmospaces_rby1"), dict):
        from vector_os_nano.skills.molmospaces_rby1 import _bridge_for_context, _runtime_context
        bridge, rby1 = _bridge_for_context(context)
        online = dict(config.get("online", {}) or {})
        runtime = _runtime_context(rby1)
        camera_system = str(runtime.get("camera_system", "gopro_d455")).lower()
        expected_resolution = (768, 576) if camera_system in {
            "gopro_d455", "rby1_gopro_d455", "gopro", "d455"
        } else None
        return MolmoSpacesRBY1NavigationEmbodiment(
            bridge, runtime, camera=str(online.get("camera", "head_camera")),
            timeout_s=float(rby1["endpoint"]["timeout_s"]),
            expected_resolution=expected_resolution,
            robot_base=(context.base
                        or context.services.get("rby1_base")
                        or context.services.get("rby1_robot")),
        )
    if context.base is not None:
        online = dict(config.get("online", {}) or {})
        return VectorBaseNavigationEmbodiment(
            context.base, context.perception,
            width=int(online.get("image_width", 320)), height=int(online.get("image_height", 240)),
        )
    raise RuntimeError("No navigation embodiment is available (need RBY1 bridge or RGB-D mobile base)")


__all__ = [
    "MolmoSpacesRBY1NavigationEmbodiment", "NavigationEmbodiment",
    "VectorBaseNavigationEmbodiment", "embodiment_from_context",
    "discrete_action_to_world_waypoint",
]
