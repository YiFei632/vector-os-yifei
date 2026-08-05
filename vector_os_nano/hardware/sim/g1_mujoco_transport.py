# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""MuJoCo transport for the vendor G1 29-DoF + dual Dex3 MJCF.

Phase 1 uses a conservative kinematic locomotion controller so the robot can
move around the simulated room from terminal navigation commands.  The helper
is intentionally bounded and deterministic; it is a scaffold for the eventual
whole-body policy, not a claim that the vendor MJCF contains native walking
support.
"""
from __future__ import annotations

import logging
import math
from pathlib import Path
import threading
import time
from typing import Any, Sequence

from vector_os_nano.core.types import Pose3D
from vector_os_nano.hardware.g1.joint_map import G1JointMap
from vector_os_nano.hardware.g1.profile import (
    BODY_DDS_JOINTS,
    LEFT_ARM_JOINTS,
    LEFT_HAND_SEMANTIC_JOINTS,
    REV_1_0_XML_ACTUATOR_JOINTS,
    RIGHT_ARM_JOINTS,
    RIGHT_HAND_SEMANTIC_JOINTS,
    G1Profile,
)
from vector_os_nano.hardware.g1.state import (
    G1ControlMode,
    G1JointCommand,
    G1State,
    G1VelocityCommand,
)
from vector_os_nano.hardware.g1.transport import G1TransportCapabilities
from vector_os_nano.hardware.sim.g1_locomotion_controller import (
    G1KinematicLocomotionController,
    G1LocomotionSnapshot,
    build_standing_body_pose,
)

logger = logging.getLogger(__name__)

_STATE_JOINTS = (
    BODY_DDS_JOINTS + LEFT_HAND_SEMANTIC_JOINTS + RIGHT_HAND_SEMANTIC_JOINTS
)


def _mujoco() -> Any:
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - dependency error path
        raise RuntimeError(
            "MuJoCo G1 requires the optional simulation dependencies: "
            "pip install 'vector-os-nano[sim]'"
        ) from exc
    return mujoco


class MuJoCoG1Transport:
    """G1Transport backed by one local MuJoCo physics loop."""

    def __init__(
        self,
        profile: G1Profile,
        *,
        gui: bool = False,
        realtime: bool = True,
        fixed_base_manipulation: bool = False,
        locomotion_enabled: bool = True,
        scene_xml_path: str | Path | None = None,
    ) -> None:
        self._profile = profile
        self._gui = bool(gui)
        self._realtime = bool(realtime)
        self._fixed_base_manipulation = bool(fixed_base_manipulation)
        self._locomotion_enabled = bool(locomotion_enabled)
        self._scene_xml_path = Path(scene_xml_path).expanduser().resolve() if scene_xml_path else None
        self._connected = False
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._model: Any = None
        self._data: Any = None
        self._viewer: Any = None
        self._mode = G1ControlMode.PASSIVE
        self._sequence_id = 0
        self._joint_map = G1JointMap()
        self._joint_ids: dict[str, int] = {}
        self._qpos_adr: dict[str, int] = {}
        self._dof_adr: dict[str, int] = {}
        self._actuator_id: dict[str, int] = {}
        self._target: dict[str, float] = {}
        # Per-joint trajectories allow both arms/hands to be commanded from
        # independent controllers without one group cancelling another.
        self._trajectories: dict[str, tuple[float, float, float, float]] = {}
        self._locomotion_controller = G1KinematicLocomotionController(profile)
        self._last_locomotion_snapshot: G1LocomotionSnapshot | None = None

    @property
    def name(self) -> str:
        return "mujoco_g1_29dof_dex3"

    @property
    def profile(self) -> G1Profile:
        return self._profile

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def supports_locomotion(self) -> bool:
        return self._locomotion_enabled

    @property
    def supports_joint_groups(self) -> bool:
        return True

    @property
    def supports_root_state(self) -> bool:
        return True

    @property
    def supports_lidar(self) -> bool:
        return False

    @property
    def capabilities(self) -> G1TransportCapabilities:
        return G1TransportCapabilities(
            locomotion=self._locomotion_enabled,
            holonomic=False,
            lidar=False,
            emergency_damping=True,
            joint_groups=frozenset(
                {"body", "left_arm", "right_arm", "left_hand", "right_hand"}
            ),
        )

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._profile.validate_assets()
            mj = _mujoco()
            if self._locomotion_enabled:
                if self._scene_xml_path is None:
                    self._scene_xml_path = G1KinematicLocomotionController.build_room_scene_xml(
                        self._profile.mjcf_path
                    )
                model_path = self._scene_xml_path
            else:
                model_path = self._profile.mjcf_path
            model = mj.MjModel.from_xml_path(str(model_path))
            data = mj.MjData(model)
            self._validate_and_index(mj, model)
            self._model = model
            self._data = data
            self._configure_numerics()
            self._initialize_pose()
            mj.mj_forward(model, data)
            self._mode = G1ControlMode.STAND
            self._connected = True
            self._stop_event.clear()
            self._locomotion_controller.sync_root_pose(None)
            if self._gui:
                try:
                    import mujoco.viewer
                    self._viewer = mujoco.viewer.launch_passive(model, data)
                except Exception as exc:  # noqa: BLE001
                    self._connected = False
                    self._model = None
                    self._data = None
                    raise RuntimeError(f"failed to open MuJoCo G1 viewer: {exc}") from exc
            self._thread = threading.Thread(
                target=self._physics_loop,
                name="mujoco_g1_physics",
                daemon=True,
            )
            self._thread.start()
            logger.info("MuJoCo G1 connected: nq=%d nv=%d nu=%d", model.nq, model.nv, model.nu)

    def _validate_and_index(self, mj: Any, model: Any) -> None:
        if (int(model.nq), int(model.nv), int(model.nu)) != (50, 49, 43):
            raise ValueError(
                "unexpected G1 MJCF dimensions: "
                f"nq={model.nq}, nv={model.nv}, nu={model.nu}; expected 50/49/43"
            )
        actuator_names: list[str] = []
        actuator_id: dict[str, int] = {}
        for aid in range(model.nu):
            jid = int(model.actuator_trnid[aid, 0])
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, jid)
            if not name:
                raise ValueError(f"G1 actuator {aid} is not attached to a named joint")
            actuator_names.append(name)
            actuator_id[name] = aid
        if tuple(actuator_names) != REV_1_0_XML_ACTUATOR_JOINTS:
            raise ValueError(
                "G1 MJCF actuator order does not match rev_1_0 profile; "
                "select or create a matching G1Profile"
            )
        self._actuator_id = actuator_id
        for name in _STATE_JOINTS:
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"G1 MJCF is missing joint {name!r}")
            self._joint_ids[name] = int(jid)
            self._qpos_adr[name] = int(model.jnt_qposadr[jid])
            self._dof_adr[name] = int(model.jnt_dofadr[jid])

    @staticmethod
    def _standing_body_pose() -> dict[str, float]:
        return build_standing_body_pose()

    def _initialize_pose(self) -> None:
        assert self._data is not None
        # Free joint is [xyz, qw, qx, qy, qz].
        self._data.qpos[:7] = (0.0, 0.0, 0.80, 1.0, 0.0, 0.0, 0.0)
        initial = self._standing_body_pose()
        initial.update(
            {
                name: 0.0
                for name in LEFT_HAND_SEMANTIC_JOINTS
                + RIGHT_HAND_SEMANTIC_JOINTS
            }
        )
        for name, value in initial.items():
            self._data.qpos[self._qpos_adr[name]] = value
        self._data.qvel[:] = 0.0
        self._target = dict(initial)
        self._trajectories.clear()
        self._last_locomotion_snapshot = None
        self._locomotion_controller.sync_root_pose(
            Pose3D(x=0.0, y=0.0, z=0.80, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        )

    def _configure_numerics(self) -> None:
        """Add the damping/armature absent from the vendor visualization MJCF.

        The source file intentionally models raw torque motors and sets every
        DoF's damping and armature to zero.  A soft joint-space controller at a
        2 ms step becomes numerically violent in that configuration, so this
        adapter adds conservative simulation-only regularisation.  It never
        modifies the vendor file on disk.
        """
        assert self._model is not None
        for name, dof_adr in self._dof_adr.items():
            if "hand_" in name:
                self._model.dof_damping[dof_adr] = 0.10
                self._model.dof_armature[dof_adr] = 0.005
            elif name in LEFT_ARM_JOINTS or name in RIGHT_ARM_JOINTS:
                self._model.dof_damping[dof_adr] = 1.0
                self._model.dof_armature[dof_adr] = 0.02
            else:
                self._model.dof_damping[dof_adr] = 3.0
                self._model.dof_armature[dof_adr] = 0.05

    def disconnect(self) -> None:
        with self._lock:
            if not self._connected:
                return
            self._stop_event.set()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._lock:
            if self._viewer is not None:
                try:
                    self._viewer.close()
                except Exception:
                    pass
            self._viewer = None
            self._thread = None
            self._model = None
            self._data = None
            self._connected = False
            self._mode = G1ControlMode.PASSIVE

    def _require_connected(self) -> None:
        if not self._connected or self._model is None or self._data is None:
            raise RuntimeError("MuJoCo G1 is not connected")

    def set_mode(self, mode: G1ControlMode) -> None:
        self._require_connected()
        if mode is G1ControlMode.LOCOMOTION:
            if not self._locomotion_enabled:
                raise NotImplementedError(
                    "MuJoCo G1 locomotion is disabled in this transport"
                )
        with self._lock:
            self._mode = mode

    def command_velocity(self, command: G1VelocityCommand) -> None:
        self._require_connected()
        with self._lock:
            self._sequence_id = command.sequence_id
            if any(abs(value) > 1e-8 for value in (command.vx, command.vy, command.vyaw)):
                if not self._locomotion_enabled:
                    raise NotImplementedError(
                        "MuJoCo G1 transport has locomotion disabled; non-zero velocity rejected"
                    )
                self._locomotion_controller.set_velocity(
                    command.vx,
                    command.vy,
                    command.vyaw,
                    sequence_id=command.sequence_id,
                    ttl=command.ttl,
                )
                if self._mode is not G1ControlMode.EMERGENCY_DAMPING:
                    self._mode = G1ControlMode.LOCOMOTION
            else:
                self._locomotion_controller.clear()
                if self._mode is not G1ControlMode.EMERGENCY_DAMPING:
                    self._mode = G1ControlMode.STAND

    def _apply_locomotion_snapshot_locked(self, snapshot: G1LocomotionSnapshot, mj: Any) -> None:
        assert self._model is not None and self._data is not None
        root = snapshot.root_pose
        self._data.qpos[:7] = (
            float(root.x),
            float(root.y),
            float(root.z),
            float(root.qw),
            float(root.qx),
            float(root.qy),
            float(root.qz),
        )
        self._data.qvel[:3] = snapshot.root_linear_velocity
        self._data.qvel[3:6] = snapshot.root_angular_velocity
        for name, target in snapshot.joint_targets.items():
            if name in self._qpos_adr:
                self._data.qpos[self._qpos_adr[name]] = float(target)
                self._target[name] = float(target)
        self._last_locomotion_snapshot = snapshot
        if snapshot.active and self._mode is not G1ControlMode.EMERGENCY_DAMPING:
            self._mode = G1ControlMode.LOCOMOTION
        elif self._mode is not G1ControlMode.EMERGENCY_DAMPING:
            self._mode = G1ControlMode.STAND
        mj.mj_forward(self._model, self._data)

    @staticmethod
    def _expected_group_names(group: str) -> tuple[str, ...]:
        groups = {
            "left_arm": LEFT_ARM_JOINTS,
            "right_arm": RIGHT_ARM_JOINTS,
            "left_hand": LEFT_HAND_SEMANTIC_JOINTS,
            "right_hand": RIGHT_HAND_SEMANTIC_JOINTS,
            "body": BODY_DDS_JOINTS,
        }
        try:
            return groups[group]
        except KeyError as exc:
            raise ValueError(f"unknown G1 joint group {group!r}") from exc

    def command_joints(self, command: G1JointCommand) -> None:
        self._require_connected()
        expected = self._expected_group_names(command.group)
        if tuple(command.joint_names) != expected:
            raise ValueError(
                f"{command.group} requires exact semantic order {expected}; "
                f"received {command.joint_names}"
            )
        with self._lock:
            current = {
                name: float(self._data.qpos[self._qpos_adr[name]])
                for name in expected
            }
            started = time.monotonic()
            duration = float(command.duration)
            for name, target in zip(expected, command.positions):
                if duration <= 0:
                    self._target[name] = float(target)
                    self._trajectories.pop(name, None)
                else:
                    self._trajectories[name] = (
                        current[name], float(target), started, duration
                    )
            self._sequence_id = command.sequence_id
            self._mode = G1ControlMode.MANIPULATION

    def _update_trajectory_locked(self, now: float) -> None:
        if not self._trajectories:
            return
        completed: list[str] = []
        for name, (start, target, started, duration) in self._trajectories.items():
            alpha = min(1.0, max(0.0, (now - started) / duration))
            smooth = alpha * alpha * (3.0 - 2.0 * alpha)
            self._target[name] = start + smooth * (target - start)
            if alpha >= 1.0:
                completed.append(name)
        for name in completed:
            self._trajectories.pop(name, None)

    @staticmethod
    def _gains(name: str) -> tuple[float, float]:
        if "hand_" in name:
            return (8.0, 0.35)
        if name in LEFT_ARM_JOINTS or name in RIGHT_ARM_JOINTS:
            return (45.0, 4.5)
        if name.startswith("waist_"):
            return (60.0, 5.0)
        return (80.0, 8.0)

    def _apply_pd_locked(self) -> None:
        assert self._model is not None and self._data is not None
        for name in REV_1_0_XML_ACTUATOR_JOINTS:
            aid = self._actuator_id[name]
            q = float(self._data.qpos[self._qpos_adr[name]])
            dq = float(self._data.qvel[self._dof_adr[name]])
            kp, kd = self._gains(name)
            if self._mode is G1ControlMode.EMERGENCY_DAMPING:
                # Remove position authority while retaining viscous damping.
                # The coordinator latch prevents a later command from silently
                # re-entering manipulation mode until explicitly cleared.
                tau = -kd * dq
            else:
                tau = kp * (self._target[name] - q) - kd * dq
            jid = self._joint_ids[name]
            if bool(self._model.jnt_actfrclimited[jid]):
                low, high = self._model.jnt_actfrcrange[jid]
                tau = max(float(low), min(float(high), tau))
            self._data.ctrl[aid] = tau

    def _physics_loop(self) -> None:
        mj = _mujoco()
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            with self._lock:
                if not self._connected or self._model is None or self._data is None:
                    return
                self._update_trajectory_locked(time.monotonic())
                snapshot: G1LocomotionSnapshot | None = None
                if self._locomotion_enabled:
                    current_root = self._data.qpos[:7]
                    current_pose = Pose3D(
                        x=float(current_root[0]),
                        y=float(current_root[1]),
                        z=float(current_root[2]),
                        qw=float(current_root[3]),
                        qx=float(current_root[4]),
                        qy=float(current_root[5]),
                        qz=float(current_root[6]),
                    )
                    dt = float(self._model.opt.timestep)
                    snapshot = self._locomotion_controller.step(
                        dt,
                        current_pose=current_pose,
                    )
                    self._apply_locomotion_snapshot_locked(snapshot, mj)
                elif self._fixed_base_manipulation:
                    # Phase-one MuJoCo support is an upper-body test stand.  A
                    # floating G1 needs a learned whole-body balance policy;
                    # pinning the free joint is explicit and preferable to a
                    # humanoid that silently collapses during arm tests.
                    self._data.qpos[:7] = (0.0, 0.0, 0.80, 1.0, 0.0, 0.0, 0.0)
                    self._data.qvel[:6] = 0.0
                    mj.mj_forward(self._model, self._data)
                self._apply_pd_locked()
                mj.mj_step(self._model, self._data)
                if snapshot is not None:
                    # Re-apply the exact same kinematic snapshot after the
                    # physics step so the visible robot follows the bounded
                    # walk path even though this phase does not simulate
                    # balance.
                    self._apply_locomotion_snapshot_locked(snapshot, mj)
                if self._viewer is not None:
                    try:
                        if not self._viewer.is_running():
                            self._stop_event.set()
                        else:
                            self._viewer.sync()
                    except Exception:
                        self._stop_event.set()
            if self._realtime:
                dt = float(self._model.opt.timestep) if self._model is not None else 0.002
                next_tick += dt
                delay = next_tick - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -0.25:
                    next_tick = time.perf_counter()

    def read_state(self) -> G1State:
        self._require_connected()
        with self._lock:
            assert self._data is not None
            q = tuple(float(self._data.qpos[self._qpos_adr[name]]) for name in _STATE_JOINTS)
            dq = tuple(float(self._data.qvel[self._dof_adr[name]]) for name in _STATE_JOINTS)
            efforts = tuple(
                float(self._data.actuator_force[self._actuator_id[name]])
                for name in _STATE_JOINTS
            )
            root = self._data.qpos[:7]
            velocity = self._data.qvel[:6]
            pose = Pose3D(
                x=float(root[0]), y=float(root[1]), z=float(root[2]),
                qw=float(root[3]), qx=float(root[4]), qy=float(root[5]), qz=float(root[6]),
            )
            # Upright cosine from quaternion; z-axis dot world z.
            qw, qx, qy, qz = (float(root[3]), float(root[4]), float(root[5]), float(root[6]))
            upright = 1.0 - 2.0 * (qx * qx + qy * qy)
            fallen = float(root[2]) < 0.45 or upright < math.cos(math.radians(50.0))
            foot_contacts = (
                dict(self._last_locomotion_snapshot.foot_contacts)
                if self._last_locomotion_snapshot is not None
                else {"left": False, "right": False}
            )
            return G1State(
                timestamp=time.time(),
                joint_names=_STATE_JOINTS,
                joint_positions=q,
                joint_velocities=dq,
                joint_efforts=efforts,
                root_pose=pose,
                root_linear_velocity=tuple(float(v) for v in velocity[:3]),
                root_angular_velocity=tuple(float(v) for v in velocity[3:6]),
                foot_contacts=foot_contacts,
                control_mode=self._mode,
                fallen=fallen,
                sequence_id=self._sequence_id,
            )

    def stop(self) -> None:
        if not self._connected:
            return
        with self._lock:
            assert self._data is not None
            self._locomotion_controller.clear()
            self._trajectories.clear()
            for name in _STATE_JOINTS:
                self._target[name] = float(self._data.qpos[self._qpos_adr[name]])
            if self._mode is not G1ControlMode.EMERGENCY_DAMPING:
                self._mode = G1ControlMode.STAND

    def get_lidar_scan(self) -> None:
        return None


__all__ = ["MuJoCoG1Transport"]
