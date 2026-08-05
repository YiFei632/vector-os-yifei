# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Pinocchio FK and position-only IK for the two G1 seven-DoF arms."""
from __future__ import annotations

import math
import threading
from typing import Any, Sequence

import numpy as np

from vector_os_nano.hardware.g1.profile import G1Profile, JOINT_POSITION_LIMITS


_pinocchio: Any = None


def _get_pinocchio() -> Any:
    global _pinocchio
    if _pinocchio is None:
        try:
            import pinocchio as pin  # noqa: PLC0415
        except ImportError as exc:  # pragma: no cover - optional dependency
            raise ImportError(
                "PinocchioG1Kinematics requires the optional 'ik' dependency "
                "(pip install 'vector-os-nano[ik]')"
            ) from exc
        _pinocchio = pin
    return _pinocchio


class PinocchioG1Kinematics:
    """Fixed-pelvis FK and damped least-squares position IK.

    The model is loaded lazily from ``profile.urdf_path``.  Waist and all
    joints outside the selected arm remain at the URDF neutral configuration;
    the TCP is the corresponding palm frame from ``profile.tcp_frames``.
    Orientation is reported by FK but intentionally not constrained by the
    repository's current position-only ``ArmProtocol.ik`` method.
    """

    def __init__(
        self,
        profile: G1Profile,
        *,
        max_iterations: int = 250,
        tolerance: float = 1e-4,
        damping: float = 1e-4,
        step_size: float = 0.6,
    ) -> None:
        if max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        numeric = (float(tolerance), float(damping), float(step_size))
        if not all(math.isfinite(value) and value > 0 for value in numeric):
            raise ValueError("IK tolerance, damping and step_size must be finite and positive")
        self.profile = profile
        self.max_iterations = int(max_iterations)
        self.tolerance = numeric[0]
        self.damping = numeric[1]
        self.step_size = numeric[2]
        self._model: Any = None
        self._q_indices: dict[str, tuple[int, ...]] = {}
        self._v_indices: dict[str, tuple[int, ...]] = {}
        self._frame_ids: dict[str, int] = {}
        self._load_lock = threading.Lock()

    def _ensure_model(self) -> tuple[Any, Any]:
        if self._model is not None:
            return _get_pinocchio(), self._model
        with self._load_lock:
            if self._model is not None:
                return _get_pinocchio(), self._model
            if not self.profile.urdf_path.is_file():
                raise FileNotFoundError(
                    f"G1 Pinocchio URDF does not exist: {self.profile.urdf_path}"
                )
            pin = _get_pinocchio()
            # No JointModelFreeFlyer: pelvis is deliberately fixed for the
            # current ArmProtocol, whose Cartesian targets are pelvis-relative.
            model = pin.buildModelFromUrdf(str(self.profile.urdf_path))
            q_indices: dict[str, tuple[int, ...]] = {}
            v_indices: dict[str, tuple[int, ...]] = {}
            frame_ids: dict[str, int] = {}
            for side in ("left", "right"):
                q_side: list[int] = []
                v_side: list[int] = []
                for name in self.profile.arm_joints[side]:
                    joint_id = int(model.getJointId(name))
                    if joint_id <= 0 or joint_id >= model.njoints:
                        raise ValueError(f"G1 URDF is missing arm joint {name!r}")
                    joint = model.joints[joint_id]
                    if int(joint.nq) != 1 or int(joint.nv) != 1:
                        raise ValueError(f"G1 arm joint {name!r} is not one-DoF")
                    q_side.append(int(joint.idx_q))
                    v_side.append(int(joint.idx_v))
                frame_name = self.profile.tcp_frames[side]
                frame_id = int(model.getFrameId(frame_name))
                if frame_id < 0 or frame_id >= model.nframes:
                    raise ValueError(f"G1 URDF is missing TCP frame {frame_name!r}")
                q_indices[side] = tuple(q_side)
                v_indices[side] = tuple(v_side)
                frame_ids[side] = frame_id
            self._q_indices = q_indices
            self._v_indices = v_indices
            self._frame_ids = frame_ids
            self._model = model
            return pin, model

    @staticmethod
    def _validate_side(side: str) -> str:
        if side not in {"left", "right"}:
            raise ValueError("G1 arm side must be 'left' or 'right'")
        return side

    def _arm_values(self, side: str, values: Sequence[float]) -> tuple[float, ...]:
        side = self._validate_side(side)
        result = tuple(float(value) for value in values)
        names = self.profile.arm_joints[side]
        if len(result) != len(names):
            raise ValueError(f"G1 {side} arm requires {len(names)} joint values")
        self.profile.validate_joint_positions(names, result)
        return result

    def _configuration(self, pin: Any, model: Any, side: str, values: Sequence[float]) -> Any:
        q = pin.neutral(model)
        q[np.asarray(self._q_indices[side], dtype=int)] = np.asarray(values, dtype=float)
        return q

    def fk(
        self,
        side: str,
        joint_positions: Sequence[float],
    ) -> tuple[list[float], list[list[float]]]:
        side = self._validate_side(side)
        values = self._arm_values(side, joint_positions)
        pin, model = self._ensure_model()
        q = self._configuration(pin, model, side, values)
        data = model.createData()
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        placement = data.oMf[self._frame_ids[side]]
        return (
            np.asarray(placement.translation, dtype=float).reshape(3).tolist(),
            np.asarray(placement.rotation, dtype=float).reshape(3, 3).tolist(),
        )

    def ik(
        self,
        side: str,
        target_xyz: tuple[float, float, float],
        current_joints: Sequence[float],
    ) -> list[float] | None:
        side = self._validate_side(side)
        target = np.asarray(tuple(float(value) for value in target_xyz), dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("G1 IK target must contain three finite values")
        arm = np.asarray(self._arm_values(side, current_joints), dtype=float)
        pin, model = self._ensure_model()
        data = model.createData()
        q_indices = np.asarray(self._q_indices[side], dtype=int)
        v_indices = np.asarray(self._v_indices[side], dtype=int)
        frame_id = self._frame_ids[side]
        names = self.profile.arm_joints[side]
        lower = np.asarray([JOINT_POSITION_LIMITS[name][0] for name in names], dtype=float)
        upper = np.asarray([JOINT_POSITION_LIMITS[name][1] for name in names], dtype=float)
        damping_identity = (self.damping * self.damping) * np.eye(3)
        previous_error = math.inf
        stalled_iterations = 0

        for _ in range(self.max_iterations):
            q = pin.neutral(model)
            q[q_indices] = arm
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            current = np.asarray(data.oMf[frame_id].translation, dtype=float).reshape(3)
            error = target - current
            error_norm = float(np.linalg.norm(error))
            if error_norm <= self.tolerance:
                solution = tuple(float(value) for value in arm)
                self.profile.validate_joint_positions(names, solution)
                return list(solution)

            jacobian_full = pin.computeFrameJacobian(
                model,
                data,
                q,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            # Pinocchio Motion/Jacobian convention is [linear; angular].
            jacobian = np.asarray(jacobian_full, dtype=float)[:3, v_indices]
            if jacobian.shape != (3, len(names)) or not np.isfinite(jacobian).all():
                raise ValueError("Pinocchio returned an invalid G1 frame Jacobian")
            try:
                delta = jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T + damping_identity,
                    error,
                )
            except np.linalg.LinAlgError:
                return None
            candidate = np.clip(arm + self.step_size * delta, lower, upper)
            if float(np.linalg.norm(candidate - arm)) <= 1e-10:
                stalled_iterations += 1
            elif error_norm >= previous_error - 1e-10:
                stalled_iterations += 1
            else:
                stalled_iterations = 0
            if stalled_iterations >= 20:
                return None
            arm = candidate
            previous_error = error_norm
        return None


__all__ = ["PinocchioG1Kinematics"]
