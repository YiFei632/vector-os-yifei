# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Kinematic locomotion helper for the simulated G1.

This is intentionally conservative: it does not claim a learned whole-body
policy.  Instead it gives the MuJoCo transport a bounded, repeatable walking
primitive so the robot can move around a room, satisfy ``walk`` / ``turn`` /
``navigate`` skill contracts, and later be swapped for a real controller.

The helper is split into two pieces:

1. a tiny locomotion state machine that integrates the root pose from body-frame
   velocity commands, and
2. a room-scene builder that wraps the vendor G1 MJCF in a lightweight indoor
   scene with a floor, walls, and a table.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import re
import time
from pathlib import Path
from typing import Any, Mapping

from vector_os_nano.core.types import Pose3D
from vector_os_nano.hardware.g1.profile import (
    BODY_DDS_JOINTS,
    G1Profile,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    WAIST_JOINTS,
)

_SCENE_DIR = Path(__file__).resolve().parent / "mjcf" / "g1"
_SCENE_XML = _SCENE_DIR / "scene_room.xml"
_ROOT_HEIGHT = 0.80
_FORWARD_SWING = 0.18
_TURN_SWAY = 0.03
_STEP_RATE_MIN = 0.70
_STEP_RATE_MAX = 1.55


def build_standing_body_pose() -> dict[str, float]:
    """Return a conservative standing pose for the G1 body joints."""
    values = {name: 0.0 for name in BODY_DDS_JOINTS}
    for side in ("left", "right"):
        values[f"{side}_hip_pitch_joint"] = -0.20
        values[f"{side}_knee_joint"] = 0.42
        values[f"{side}_ankle_pitch_joint"] = -0.23
        values[f"{side}_shoulder_pitch_joint"] = 0.35
        values[f"{side}_shoulder_roll_joint"] = 0.18 if side == "left" else -0.18
        values[f"{side}_elbow_joint"] = 0.87
    return values


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


_MESH_FILE_RE = re.compile(r'file="([^"]+\.(?:STL|stl|obj|OBJ))"')


def _rewrite_vendor_mesh_paths(vendor_xml: Path) -> Path:
    """Create a vendor MJCF copy with absolute mesh paths.

    MuJoCo resolves include-relative assets against the outer XML file, which
    breaks vendor MJCFs that store meshes in a sibling ``meshes/`` directory.
    This helper rewrites mesh file attributes to absolute paths once, then the
    room-scene wrapper includes the rewritten copy.
    """
    vendor_xml = Path(vendor_xml).expanduser().resolve()
    rewritten = _SCENE_DIR / vendor_xml.name
    text = vendor_xml.read_text(encoding="utf-8")
    mesh_root = vendor_xml.parent / "meshes"

    def _replace(match: re.Match[str]) -> str:
        raw = match.group(1)
        mesh_path = Path(raw)
        resolved = mesh_root / mesh_path.name
        return f'file="{resolved.as_posix()}"'

    patched = _MESH_FILE_RE.sub(_replace, text)
    rewritten.write_text(patched, encoding="utf-8")
    return rewritten


@dataclass
class G1LocomotionSnapshot:
    """What the transport applies to the MuJoCo model on each physics tick."""

    root_pose: Pose3D
    root_linear_velocity: tuple[float, float, float]
    root_angular_velocity: tuple[float, float, float]
    joint_targets: dict[str, float]
    foot_contacts: dict[str, bool]
    active: bool


class G1KinematicLocomotionController:
    """Small biped locomotion controller for MuJoCo G1.

    The controller keeps a persistent root pose, integrates velocity commands in
    the world frame, and synthesizes a light gait pattern over the standing pose.
    It is deliberately conservative and deterministic.
    """

    def __init__(self, profile: G1Profile) -> None:
        self._profile = profile
        self._standing_pose = build_standing_body_pose()
        self._root_pose = Pose3D(0.0, 0.0, _ROOT_HEIGHT, 0.0, 0.0, 0.0, 1.0)
        self._root_linear_velocity = (0.0, 0.0, 0.0)
        self._root_angular_velocity = (0.0, 0.0, 0.0)
        self._command = {
            "vx": 0.0,
            "vy": 0.0,
            "vyaw": 0.0,
            "sequence_id": 0,
            "timestamp": time.monotonic(),
            "ttl": 0.3,
        }
        self._phase = 0.0
        self._last_step = time.monotonic()

    def sync_root_pose(self, pose: Pose3D | None) -> None:
        if pose is not None:
            self._root_pose = Pose3D(
                x=float(pose.x),
                y=float(pose.y),
                z=float(pose.z) if math.isfinite(float(pose.z)) else _ROOT_HEIGHT,
                qx=float(pose.qx),
                qy=float(pose.qy),
                qz=float(pose.qz),
                qw=float(pose.qw),
            )
        self._last_step = time.monotonic()

    def set_velocity(
        self,
        vx: float,
        vy: float,
        vyaw: float,
        *,
        sequence_id: int,
        ttl: float = 0.3,
    ) -> None:
        limits = self._profile.velocity_limits
        self._command = {
            "vx": max(-limits["vx"], min(limits["vx"], float(vx))),
            "vy": max(-limits["vy"], min(limits["vy"], float(vy))),
            "vyaw": max(-limits["vyaw"], min(limits["vyaw"], float(vyaw))),
            "sequence_id": int(sequence_id),
            "timestamp": time.monotonic(),
            "ttl": float(ttl),
        }

    def clear(self) -> None:
        self._command = {
            "vx": 0.0,
            "vy": 0.0,
            "vyaw": 0.0,
            "sequence_id": int(self._command.get("sequence_id", 0)),
            "timestamp": time.monotonic(),
            "ttl": 0.3,
        }
        self._root_linear_velocity = (0.0, 0.0, 0.0)
        self._root_angular_velocity = (0.0, 0.0, 0.0)

    def active_command(self, now: float | None = None) -> tuple[float, float, float]:
        now = time.monotonic() if now is None else float(now)
        if now - float(self._command["timestamp"]) > float(self._command["ttl"]):
            return (0.0, 0.0, 0.0)
        return (
            float(self._command["vx"]),
            float(self._command["vy"]),
            float(self._command["vyaw"]),
        )

    def _yaw_from_pose(self) -> tuple[float, float, float]:
        """Return the current root pose as roll/pitch/yaw in radians."""
        qx = float(self._root_pose.qx)
        qy = float(self._root_pose.qy)
        qz = float(self._root_pose.qz)
        qw = float(self._root_pose.qw)

        # Standard Tait-Bryan ZYX extraction. We only use yaw, but returning
        # all three makes the helper easy to extend later.
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (roll, pitch, yaw)

    def _gait_targets(self, vx: float, vy: float, vyaw: float, dt: float) -> dict[str, float]:
        speed = math.hypot(vx, vy)
        if speed < 1e-4 and abs(vyaw) < 1e-4:
            return dict(self._standing_pose)

        self._phase += 2.0 * math.pi * (1.0 + 0.8 * min(speed / max(self._profile.velocity_limits["vx"], 1e-6), 1.0)) * dt
        swing_amp = min(_FORWARD_SWING, 0.04 + 0.22 * speed)
        lift_amp = 0.35 * swing_amp + 0.06
        roll_amp = _TURN_SWAY + 0.05 * min(abs(vyaw), self._profile.velocity_limits["vyaw"]) / max(self._profile.velocity_limits["vyaw"], 1e-6)
        pitch_sway = 0.08 * vyaw
        lateral_bias = 0.08 * vy

        target = dict(self._standing_pose)
        leg_phase = {
            "left": self._phase,
            "right": self._phase + math.pi,
        }
        for side, phase in leg_phase.items():
            swing = math.sin(phase)
            stance = math.cos(phase)
            side_sign = 1.0 if side == "left" else -1.0
            target[f"{side}_hip_pitch_joint"] = self._standing_pose[f"{side}_hip_pitch_joint"] + swing_amp * swing + pitch_sway
            target[f"{side}_knee_joint"] = self._standing_pose[f"{side}_knee_joint"] + lift_amp * max(0.0, -swing)
            target[f"{side}_ankle_pitch_joint"] = self._standing_pose[f"{side}_ankle_pitch_joint"] - 0.55 * swing_amp * swing - 0.5 * pitch_sway
            target[f"{side}_hip_roll_joint"] = self._standing_pose[f"{side}_hip_roll_joint"] + side_sign * (roll_amp + 0.03 * stance) + side_sign * lateral_bias
            target[f"{side}_ankle_roll_joint"] = -0.55 * target[f"{side}_hip_roll_joint"]
            target[f"{side}_hip_yaw_joint"] = 0.12 * vyaw

        for side in ("left", "right"):
            target[f"{side}_shoulder_pitch_joint"] = self._standing_pose[f"{side}_shoulder_pitch_joint"] - 0.18 * math.sin(self._phase + (math.pi if side == "left" else 0.0))
            target[f"{side}_shoulder_roll_joint"] = self._standing_pose[f"{side}_shoulder_roll_joint"] + (0.02 * vy + (0.03 if side == "left" else -0.03) * vyaw)
            target[f"{side}_shoulder_yaw_joint"] = self._standing_pose[f"{side}_shoulder_yaw_joint"] + (0.03 * vyaw)
            target[f"{side}_elbow_joint"] = self._standing_pose[f"{side}_elbow_joint"]

        for name in WAIST_JOINTS:
            target[name] = 0.0
        target["waist_yaw_joint"] = 0.10 * vyaw
        target["waist_roll_joint"] = 0.04 * vy
        target["waist_pitch_joint"] = -0.03 * speed
        return target

    def step(
        self,
        dt: float,
        *,
        now: float | None = None,
        current_pose: Pose3D | None = None,
    ) -> G1LocomotionSnapshot:
        now = time.monotonic() if now is None else float(now)
        dt_value = max(0.0, float(dt))
        if current_pose is not None and self._last_step == 0.0:
            self.sync_root_pose(current_pose)
        vx, vy, vyaw = self.active_command(now)
        if dt_value > 0.0 and any(abs(v) > 1e-8 for v in (vx, vy, vyaw)):
            _, _, yaw = self._yaw_from_pose()
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            world_vx = cos_y * vx - sin_y * vy
            world_vy = sin_y * vx + cos_y * vy
            x = self._root_pose.x + world_vx * dt_value
            y = self._root_pose.y + world_vy * dt_value
            yaw = yaw + vyaw * dt_value
            qx, qy, qz, qw = _yaw_to_quaternion(yaw)
            self._root_pose = Pose3D(x=x, y=y, z=_ROOT_HEIGHT, qx=qx, qy=qy, qz=qz, qw=qw)
            self._root_linear_velocity = (world_vx, world_vy, 0.0)
            self._root_angular_velocity = (0.0, 0.0, vyaw)
            active = True
        else:
            self._root_pose = Pose3D(
                x=self._root_pose.x,
                y=self._root_pose.y,
                z=_ROOT_HEIGHT,
                qx=self._root_pose.qx,
                qy=self._root_pose.qy,
                qz=self._root_pose.qz,
                qw=self._root_pose.qw,
            )
            self._root_linear_velocity = (0.0, 0.0, 0.0)
            self._root_angular_velocity = (0.0, 0.0, 0.0)
            active = False

        joint_targets = self._gait_targets(vx, vy, vyaw, dt_value)
        foot_contacts = {
            "left": bool(math.cos(self._phase) >= 0.0),
            "right": bool(math.cos(self._phase + math.pi) >= 0.0),
        }
        self._last_step = now
        return G1LocomotionSnapshot(
            root_pose=self._root_pose,
            root_linear_velocity=self._root_linear_velocity,
            root_angular_velocity=self._root_angular_velocity,
            joint_targets=joint_targets,
            foot_contacts=foot_contacts,
            active=active,
        )

    @staticmethod
    def build_room_scene_xml(vendor_mjcf: str | Path) -> Path:
        """Create a small indoor scene that wraps the vendor G1 MJCF."""
        vendor = _rewrite_vendor_mesh_paths(Path(vendor_mjcf))
        _SCENE_DIR.mkdir(parents=True, exist_ok=True)
        xml = f"""\
<mujoco model="g1_room">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" cone="elliptic" impratio="100"/>

  <include file="{vendor.as_posix()}"/>

  <worldbody>
    <geom name="g1_room_floor" type="plane" size="12 12 0.1" rgba="0.85 0.85 0.85 1"/>
    <geom name="wall_north" type="box" pos="0 6 1.0" size="12 0.05 1.0" rgba="0.6 0.6 0.6 1"/>
    <geom name="wall_south" type="box" pos="0 -6 1.0" size="12 0.05 1.0" rgba="0.6 0.6 0.6 1"/>
    <geom name="wall_east" type="box" pos="6 0 1.0" size="0.05 12 1.0" rgba="0.6 0.6 0.6 1"/>
    <geom name="wall_west" type="box" pos="-6 0 1.0" size="0.05 12 1.0" rgba="0.6 0.6 0.6 1"/>

    <body name="table" pos="1.4 0.0 0.75">
      <geom type="box" size="0.55 0.35 0.03" rgba="0.55 0.38 0.2 1"/>
      <geom type="box" pos="-0.48 -0.28 -0.35" size="0.03 0.03 0.35" rgba="0.45 0.30 0.15 1"/>
      <geom type="box" pos="-0.48 0.28 -0.35" size="0.03 0.03 0.35" rgba="0.45 0.30 0.15 1"/>
      <geom type="box" pos="0.48 -0.28 -0.35" size="0.03 0.03 0.35" rgba="0.45 0.30 0.15 1"/>
      <geom type="box" pos="0.48 0.28 -0.35" size="0.03 0.03 0.35" rgba="0.45 0.30 0.15 1"/>
    </body>
  </worldbody>
</mujoco>
"""
        _SCENE_XML.write_text(xml, encoding="utf-8")
        return _SCENE_XML


__all__ = [
    "G1KinematicLocomotionController",
    "G1LocomotionSnapshot",
    "build_standing_body_pose",
]
