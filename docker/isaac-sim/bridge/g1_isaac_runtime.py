#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics
"""Fail-fast launcher for Unitree's official G1 Isaac Lab runtime.

This process intentionally *executes* ``unitree_sim_isaaclab/sim_main.py``
instead of importing it: upstream parses argv and creates ``AppLauncher`` at
module import time, so an in-process plugin would have unsafe global side
effects.  The process boundary is the plugin boundary.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

from g1_file_protocol import write_manifest
from g1_manifest import G1_ROBOT_TYPE


DEFAULT_TASK = "Isaac-Move-Cylinder-G129-Dex3-Wholebody"
SUPPORTED_TASKS = (DEFAULT_TASK,)

# These paths are hard-coded by the pinned upstream task/configuration.  A
# differently named user USD is not silently substituted because joint order,
# actuator gains, and the whole-body policy contract must match exactly.
REQUIRED_ASSET_PATHS: tuple[str, ...] = (
    "assets/robots/g1-29dof_wholebody_dex3/g1_29dof_with_dex3_rev_1_0.usd",
    "assets/objects/small_warehouse/small_warehouse_digital_twin.usd",
    "assets/objects/PackingTable_2/PackingTable.usd",
    "assets/objects/PackingTable/PackingTable.usd",
)


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be true or false, got {value!r}")


@dataclass(frozen=True)
class G1RuntimeConfig:
    unitree_root: Path
    state_dir: Path
    python_executable: Path
    task: str
    policy_relative_path: Path
    device: str
    headless: bool
    enable_cameras: bool
    no_render: bool

    @classmethod
    def from_environ(cls) -> "G1RuntimeConfig":
        root = Path(
            os.environ.get("UNITREE_SIM_ROOT", "/opt/unitree_sim_isaaclab")
        ).expanduser()
        policy = Path(os.environ.get("G1_POLICY_PATH", "assets/model/policy.onnx"))
        return cls(
            unitree_root=root,
            state_dir=Path(os.environ.get("ISAAC_STATE_DIR", "/tmp/isaac_state")),
            python_executable=Path(
                os.environ.get("ISAACSIM_PYTHON_EXE", "/isaac-sim/python.sh")
            ),
            task=os.environ.get("G1_TASK", DEFAULT_TASK),
            policy_relative_path=policy,
            device=os.environ.get("ISAAC_DEVICE", "cuda:0"),
            headless=_env_bool("ISAAC_HEADLESS", True),
            enable_cameras=_env_bool("G1_ENABLE_CAMERAS", True),
            no_render=_env_bool("G1_NO_RENDER", False),
        )

    @property
    def sim_main(self) -> Path:
        return self.unitree_root / "sim_main.py"

    @property
    def policy_path(self) -> Path:
        return self.unitree_root / self.policy_relative_path

    def required_files(self) -> list[Path]:
        return [
            self.sim_main,
            # The pinned teleimager submodule uses a PEP 517 ``src`` layout.
            self.unitree_root
            / "teleimager/src/teleimager/image_server.py",
            self.unitree_root
            / "tasks/g1_tasks/move_cylinder_g1_29dof_dex3_wholebody/__init__.py",
            *(self.unitree_root / relative for relative in REQUIRED_ASSET_PATHS),
            self.policy_path,
        ]

    def validate(self, *, check_python_modules: bool = True) -> None:
        errors: list[str] = []
        if self.task not in SUPPORTED_TASKS:
            errors.append(
                f"G1_TASK={self.task!r} is unsupported; phase 1 supports "
                f"{DEFAULT_TASK!r} only"
            )
        if self.policy_relative_path.is_absolute() or ".." in self.policy_relative_path.parts:
            errors.append(
                "G1_POLICY_PATH must be relative to UNITREE_SIM_ROOT because the "
                "official launcher resolves it relative to that repository"
            )
        if self.no_render and self.enable_cameras:
            errors.append("G1_NO_RENDER=true is incompatible with G1_ENABLE_CAMERAS=true")
        if not self.python_executable.is_file():
            errors.append(f"Isaac Sim Python launcher is missing: {self.python_executable}")
        for path in self.required_files():
            if not path.is_file() or path.stat().st_size == 0:
                errors.append(f"required G1 runtime file is missing or empty: {path}")
        if check_python_modules:
            for module in ("isaaclab", "unitree_sdk2py", "pinocchio", "pink", "teleimager"):
                if importlib.util.find_spec(module) is None:
                    errors.append(
                        f"required Python package {module!r} is unavailable in Isaac Sim Python"
                    )
        if errors:
            detail = "\n  - ".join(errors)
            raise RuntimeError(
                "G1 Isaac startup preflight failed:\n  - "
                f"{detail}\n"
                "Mount the complete official asset bundle at "
                f"{self.unitree_root / 'assets'} and provide the official whole-body "
                "policy (default: assets/model/policy.onnx). No fallback controller "
                "or placeholder robot will be used."
            )

    def command(self) -> list[str]:
        args = [
            str(self.python_executable),
            str(self.sim_main),
            "--task",
            self.task,
            "--robot_type",
            "g129",
            "--enable_dex3_dds",
            "--enable_wholebody_dds",
            "--model_path",
            self.policy_relative_path.as_posix(),
            "--device",
            self.device,
        ]
        if self.headless:
            args.append("--headless")
        if self.enable_cameras:
            args.append("--enable_cameras")
        if self.no_render:
            args.append("--no_render")
        return args


def _write_preflight_manifest(config: G1RuntimeConfig) -> None:
    write_manifest(
        config.state_dir,
        {
            "runtime": "unitreerobotics/unitree_sim_isaaclab",
            "task": config.task,
            "policy_path": str(config.policy_path),
            "status": "preflight_ok",
        },
    )
    (config.state_dir / "robot_type").write_text(G1_ROBOT_TYPE, encoding="utf-8")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="run preflight and print the resolved command without launching Isaac Lab",
    )
    args = parser.parse_args(argv)
    config = G1RuntimeConfig.from_environ()
    config.validate()
    config.state_dir.mkdir(parents=True, exist_ok=True)
    _write_preflight_manifest(config)
    command = config.command()
    if args.check:
        print(json.dumps({"robot_type": G1_ROBOT_TYPE, "command": command}))
        return
    os.chdir(config.unitree_root)
    os.execv(str(config.python_executable), command)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[g1-runtime] FATAL: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
