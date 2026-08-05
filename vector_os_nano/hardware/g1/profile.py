# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Static description of the supported G1 EDU rev-1.0 embodiment.

The public URDF, MJCF, Unitree body DDS messages, and pinned IsaacLab task do
not share one flat joint order.  All G1 code must therefore use the semantic
names defined here and an explicit
:class:`~vector_os_nano.hardware.g1.joint_map.G1JointMap`.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import math
import os
from pathlib import Path
from typing import Any, Mapping

import yaml


LEFT_LEG_JOINTS: tuple[str, ...] = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
)

RIGHT_LEG_JOINTS: tuple[str, ...] = (
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)

WAIST_JOINTS: tuple[str, ...] = (
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
)

LEFT_ARM_JOINTS: tuple[str, ...] = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)

RIGHT_ARM_JOINTS: tuple[str, ...] = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

# Unitree LowCmd/LowState 29-DoF order (hands are transported separately).
BODY_DDS_JOINTS: tuple[str, ...] = (
    LEFT_LEG_JOINTS
    + RIGHT_LEG_JOINTS
    + WAIST_JOINTS
    + LEFT_ARM_JOINTS
    + RIGHT_ARM_JOINTS
)

# HAL/Skill semantic order.  Both sides intentionally use the same finger
# order; no backend numeric layout is allowed to leak through this contract.
LEFT_HAND_SEMANTIC_JOINTS: tuple[str, ...] = (
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
)

RIGHT_HAND_SEMANTIC_JOINTS: tuple[str, ...] = (
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
)

# Hand actuator order observed in the supplied rev-1.0 vendor MJCF.  These
# names describe the model file only: the official Dex3 SDK sample addresses
# real motors numerically and does not publish a finger-name mapping.  Do not
# reuse these tuples as a physical HandCmd/HandState contract without a
# firmware-specific motor-ID validation.
LEFT_HAND_REV_1_0_XML_JOINTS: tuple[str, ...] = (
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
)

RIGHT_HAND_REV_1_0_XML_JOINTS: tuple[str, ...] = RIGHT_HAND_SEMANTIC_JOINTS

# g1_29dof_with_hand_rev_1_0.xml actuator order.  Notice that the left hand
# occurs before the right arm.
REV_1_0_XML_ACTUATOR_JOINTS: tuple[str, ...] = (
    LEFT_LEG_JOINTS
    + RIGHT_LEG_JOINTS
    + WAIST_JOINTS
    + LEFT_ARM_JOINTS
    + LEFT_HAND_REV_1_0_XML_JOINTS
    + RIGHT_ARM_JOINTS
    + RIGHT_HAND_REV_1_0_XML_JOINTS
)

# The imported articulation/tree order used by the current IsaacLab G1 asset.
# Keep it separate from DDS even when a backend happens to perform the reorder.
LEFT_HAND_ISAACLAB_JOINTS: tuple[str, ...] = (
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
)
RIGHT_HAND_ISAACLAB_JOINTS: tuple[str, ...] = (
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
)


# Conservative limits from g1_29dof_with_hand_rev_1_0.urdf.  In particular,
# these deliberately do not use the slightly wider thumb_1 limits in the MJCF.
JOINT_POSITION_LIMITS: Mapping[str, tuple[float, float]] = {
    "left_hip_pitch_joint": (-2.5307, 2.8798),
    "left_hip_roll_joint": (-0.5236, 2.9671),
    "left_hip_yaw_joint": (-2.7576, 2.7576),
    "left_knee_joint": (-0.087267, 2.8798),
    "left_ankle_pitch_joint": (-0.87267, 0.5236),
    "left_ankle_roll_joint": (-0.2618, 0.2618),
    "right_hip_pitch_joint": (-2.5307, 2.8798),
    "right_hip_roll_joint": (-2.9671, 0.5236),
    "right_hip_yaw_joint": (-2.7576, 2.7576),
    "right_knee_joint": (-0.087267, 2.8798),
    "right_ankle_pitch_joint": (-0.87267, 0.5236),
    "right_ankle_roll_joint": (-0.2618, 0.2618),
    "waist_yaw_joint": (-2.618, 2.618),
    "waist_roll_joint": (-0.52, 0.52),
    "waist_pitch_joint": (-0.52, 0.52),
    "left_shoulder_pitch_joint": (-3.0892, 2.6704),
    "left_shoulder_roll_joint": (-1.5882, 2.2515),
    "left_shoulder_yaw_joint": (-2.618, 2.618),
    "left_elbow_joint": (-1.0472, 2.0944),
    "left_wrist_roll_joint": (-1.972222054, 1.972222054),
    "left_wrist_pitch_joint": (-1.614429558, 1.614429558),
    "left_wrist_yaw_joint": (-1.614429558, 1.614429558),
    "right_shoulder_pitch_joint": (-3.0892, 2.6704),
    "right_shoulder_roll_joint": (-2.2515, 1.5882),
    "right_shoulder_yaw_joint": (-2.618, 2.618),
    "right_elbow_joint": (-1.0472, 2.0944),
    "right_wrist_roll_joint": (-1.972222054, 1.972222054),
    "right_wrist_pitch_joint": (-1.614429558, 1.614429558),
    "right_wrist_yaw_joint": (-1.614429558, 1.614429558),
    "left_hand_thumb_0_joint": (-1.04719755, 1.04719755),
    "left_hand_thumb_1_joint": (-0.61086523, 1.04719755),
    "left_hand_thumb_2_joint": (0.0, 1.74532925),
    "left_hand_index_0_joint": (-1.57079632, 0.0),
    "left_hand_index_1_joint": (-1.74532925, 0.0),
    "left_hand_middle_0_joint": (-1.57079632, 0.0),
    "left_hand_middle_1_joint": (-1.74532925, 0.0),
    "right_hand_thumb_0_joint": (-1.04719755, 1.04719755),
    "right_hand_thumb_1_joint": (-1.04719755, 0.61086523),
    "right_hand_thumb_2_joint": (-1.74532925, 0.0),
    "right_hand_index_0_joint": (0.0, 1.57079632),
    "right_hand_index_1_joint": (0.0, 1.74532925),
    "right_hand_middle_0_joint": (0.0, 1.57079632),
    "right_hand_middle_1_joint": (0.0, 1.74532925),
}


def _expand_path(value: str | os.PathLike[str]) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(str(value)))).resolve()


def _normalise_sha256(value: str | None, *, label: str) -> str | None:
    if value is None:
        return None
    digest = str(value).strip().lower()
    if len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
        raise ValueError(f"{label} must be a 64-character SHA256 hex digest")
    return digest


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class G1Profile:
    """Validated description of one concrete G1 embodiment."""

    profile_id: str
    model_name: str
    mode_machine: int
    urdf_path: Path
    mjcf_path: Path
    mesh_dir: Path
    body_dof: int = 29
    hand_dof_per_side: int = 7
    hands: tuple[str, ...] = ("left", "right")
    hand_model: str = "dex3_1"
    waist_locked: bool = False
    wrist_motor: str = "4010"
    root_frame: str = "pelvis"
    navigation_frame: str = "base_link"
    left_tcp_frame: str = "left_hand_palm_link"
    right_tcp_frame: str = "right_hand_palm_link"
    default_arm: str = "right"
    default_hand: str = "right"
    velocity_limits: Mapping[str, float] = field(
        default_factory=lambda: {"vx": 0.6, "vy": 0.4, "vyaw": 1.0}
    )
    poses: Mapping[str, tuple[float, ...]] = field(default_factory=dict)
    asset_root_env: str | None = "VECTOR_G1_ASSET_ROOT"
    urdf_sha256: str | None = None
    mjcf_sha256: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "urdf_path", _expand_path(self.urdf_path))
        object.__setattr__(self, "mjcf_path", _expand_path(self.mjcf_path))
        object.__setattr__(self, "mesh_dir", _expand_path(self.mesh_dir))
        object.__setattr__(self, "hands", tuple(str(side) for side in self.hands))
        object.__setattr__(
            self,
            "hand_model",
            str(self.hand_model).strip().lower().replace("-", "_"),
        )
        object.__setattr__(
            self,
            "velocity_limits",
            {key: float(value) for key, value in self.velocity_limits.items()},
        )
        object.__setattr__(
            self,
            "poses",
            {key: tuple(float(value) for value in values) for key, values in self.poses.items()},
        )
        object.__setattr__(
            self,
            "urdf_sha256",
            _normalise_sha256(self.urdf_sha256, label="urdf_sha256"),
        )
        object.__setattr__(
            self,
            "mjcf_sha256",
            _normalise_sha256(self.mjcf_sha256, label="mjcf_sha256"),
        )

        if not self.profile_id or not self.model_name:
            raise ValueError("G1 profile_id and model_name cannot be empty")
        if self.mode_machine < 0:
            raise ValueError("mode_machine must be non-negative")
        if self.body_dof != len(BODY_DDS_JOINTS):
            raise ValueError(
                f"G1 profile body_dof={self.body_dof}, expected {len(BODY_DDS_JOINTS)}"
            )
        if self.hand_dof_per_side != 7 or self.hand_model != "dex3_1":
            raise ValueError("this G1 profile currently supports Dex3-1 with 7 motors per hand")
        if (
            not self.hands
            or len(set(self.hands)) != len(self.hands)
            or any(side not in {"left", "right"} for side in self.hands)
        ):
            raise ValueError(f"invalid G1 hand sides: {self.hands!r}")
        if self.default_arm not in {"left", "right"}:
            raise ValueError("default_arm must be 'left' or 'right'")
        if self.default_hand not in self.hands:
            raise ValueError("default_hand must name an installed hand")
        required_limits = {"vx", "vy", "vyaw"}
        if set(self.velocity_limits) != required_limits:
            raise ValueError(
                f"velocity_limits must contain exactly {sorted(required_limits)}"
            )
        for key, value in self.velocity_limits.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"velocity limit {key} must be finite and positive")
        for name, values in self.poses.items():
            if not values or not all(math.isfinite(value) for value in values):
                raise ValueError(f"pose {name!r} must contain finite joint values")

    @property
    def arm_joints(self) -> Mapping[str, tuple[str, ...]]:
        return {"left": LEFT_ARM_JOINTS, "right": RIGHT_ARM_JOINTS}

    @property
    def hand_joints(self) -> Mapping[str, tuple[str, ...]]:
        return {
            "left": LEFT_HAND_SEMANTIC_JOINTS,
            "right": RIGHT_HAND_SEMANTIC_JOINTS,
        }

    @property
    def all_joint_names(self) -> tuple[str, ...]:
        names = BODY_DDS_JOINTS
        if "left" in self.hands:
            names += LEFT_HAND_SEMANTIC_JOINTS
        if "right" in self.hands:
            names += RIGHT_HAND_SEMANTIC_JOINTS
        return names

    @property
    def actuator_dof(self) -> int:
        return len(self.all_joint_names)

    @property
    def tcp_frames(self) -> Mapping[str, str]:
        return {"left": self.left_tcp_frame, "right": self.right_tcp_frame}

    def joint_group(self, group: str) -> tuple[str, ...]:
        groups: dict[str, tuple[str, ...]] = {
            "body": BODY_DDS_JOINTS,
            "left_arm": LEFT_ARM_JOINTS,
            "right_arm": RIGHT_ARM_JOINTS,
        }
        if "left" in self.hands:
            groups["left_hand"] = LEFT_HAND_SEMANTIC_JOINTS
        if "right" in self.hands:
            groups["right_hand"] = RIGHT_HAND_SEMANTIC_JOINTS
        try:
            return groups[group]
        except KeyError as exc:
            raise ValueError(f"unknown or unavailable G1 joint group {group!r}") from exc

    def validate_joint_positions(
        self,
        joint_names: tuple[str, ...],
        positions: tuple[float, ...],
    ) -> None:
        if len(joint_names) != len(positions):
            raise ValueError("joint names and positions must have equal length")
        for name, position in zip(joint_names, positions):
            if name not in JOINT_POSITION_LIMITS:
                raise ValueError(f"no configured G1 limit for joint {name!r}")
            value = float(position)
            if not math.isfinite(value):
                raise ValueError(f"non-finite target for G1 joint {name!r}")
            lower, upper = JOINT_POSITION_LIMITS[name]
            if value < lower or value > upper:
                raise ValueError(
                    f"target {value} for {name!r} is outside [{lower}, {upper}]"
                )

    def validate_assets(self) -> None:
        missing = [
            str(path)
            for path in (self.urdf_path, self.mjcf_path, self.mesh_dir)
            if not path.exists()
        ]
        if missing:
            raise FileNotFoundError("missing G1 model assets: " + ", ".join(missing))
        if not self.urdf_path.is_file():
            raise FileNotFoundError(str(self.urdf_path))
        if not self.mjcf_path.is_file():
            raise FileNotFoundError(str(self.mjcf_path))
        if not self.mesh_dir.is_dir():
            raise NotADirectoryError(str(self.mesh_dir))
        for label, path, expected in (
            ("URDF", self.urdf_path, self.urdf_sha256),
            ("MJCF", self.mjcf_path, self.mjcf_sha256),
        ):
            if expected is not None:
                actual = _sha256(path)
                if actual != expected:
                    raise ValueError(
                        f"G1 {label} SHA256 mismatch: expected {expected}, got {actual}"
                    )

    @classmethod
    def from_mapping(
        cls,
        data: Mapping[str, Any],
        *,
        base_dir: Path | None = None,
    ) -> "G1Profile":
        robot = dict(data.get("robot", data))
        assets = dict(data.get("assets", {}))
        frames = dict(data.get("frames", {}))
        control = dict(data.get("control", {}))
        defaults = dict(data.get("defaults", {}))
        poses_raw = dict(data.get("poses", {}))

        root_env_raw = assets.get("root_env", "VECTOR_G1_ASSET_ROOT")
        root_env = str(root_env_raw).strip() if root_env_raw else None
        environment_root = os.environ.get(root_env, "").strip() if root_env else ""
        root_raw: str | os.PathLike[str] = environment_root or assets.get("root", ".")
        expanded_root = Path(os.path.expandvars(os.path.expanduser(str(root_raw))))
        if not expanded_root.is_absolute():
            expanded_root = (base_dir or Path.cwd()) / expanded_root
        root_path = expanded_root.resolve()

        def asset_path(key: str) -> Path:
            raw = assets.get(key)
            if raw is None:
                raise ValueError(f"G1 profile assets.{key} is required")
            expanded = Path(os.path.expandvars(os.path.expanduser(str(raw))))
            if not expanded.is_absolute():
                expanded = root_path / expanded
            return expanded.resolve()

        hashes_raw = assets.get("sha256", {}) or {}
        if not isinstance(hashes_raw, Mapping):
            raise ValueError("G1 profile assets.sha256 must be a mapping")
        hands_value = robot.get("hands", ("left", "right"))
        poses = {key: tuple(float(v) for v in value) for key, value in poses_raw.items()}
        profile = cls(
            profile_id=str(robot.get("profile_id", "g1_edu_flagship_a_rev_1_0")),
            model_name=str(robot.get("model_name", "g1_29dof_with_hand_rev_1_0")),
            mode_machine=int(robot.get("mode_machine", 5)),
            urdf_path=asset_path("urdf"),
            mjcf_path=asset_path("mjcf"),
            mesh_dir=asset_path("meshes"),
            body_dof=int(robot.get("body_dof", 29)),
            hand_dof_per_side=int(robot.get("hand_dof_per_side", 7)),
            hands=tuple(str(side) for side in hands_value),
            hand_model=str(robot.get("hand_model", robot.get("hand_type", "dex3_1"))),
            waist_locked=bool(robot.get("waist_locked", False)),
            wrist_motor=str(robot.get("wrist_motor", "4010")),
            root_frame=str(frames.get("root", "pelvis")),
            navigation_frame=str(frames.get("navigation", "base_link")),
            left_tcp_frame=str(frames.get("left_tcp", "left_hand_palm_link")),
            right_tcp_frame=str(frames.get("right_tcp", "right_hand_palm_link")),
            default_arm=str(defaults.get("arm", "right")),
            default_hand=str(defaults.get("hand", "right")),
            velocity_limits={
                "vx": float(control.get("max_vx", 0.6)),
                "vy": float(control.get("max_vy", 0.4)),
                "vyaw": float(control.get("max_vyaw", 1.0)),
            },
            poses=poses,
            asset_root_env=root_env,
            urdf_sha256=assets.get("urdf_sha256", hashes_raw.get("urdf")),
            mjcf_sha256=assets.get("mjcf_sha256", hashes_raw.get("mjcf")),
        )
        profile.validate_assets()
        return profile

    @classmethod
    def from_yaml(cls, path: str | os.PathLike[str]) -> "G1Profile":
        profile_path = _expand_path(path)
        with profile_path.open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream) or {}
        if not isinstance(raw, Mapping):
            raise ValueError(f"G1 profile must be a mapping: {profile_path}")
        return cls.from_mapping(raw, base_dir=profile_path.parent)


__all__ = [
    "G1Profile",
    "LEFT_LEG_JOINTS",
    "RIGHT_LEG_JOINTS",
    "WAIST_JOINTS",
    "LEFT_ARM_JOINTS",
    "RIGHT_ARM_JOINTS",
    "BODY_DDS_JOINTS",
    "LEFT_HAND_SEMANTIC_JOINTS",
    "RIGHT_HAND_SEMANTIC_JOINTS",
    "LEFT_HAND_REV_1_0_XML_JOINTS",
    "RIGHT_HAND_REV_1_0_XML_JOINTS",
    "LEFT_HAND_ISAACLAB_JOINTS",
    "RIGHT_HAND_ISAACLAB_JOINTS",
    "REV_1_0_XML_ACTUATOR_JOINTS",
    "JOINT_POSITION_LIMITS",
]
