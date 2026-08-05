# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Vector OS Nano Agent — the main entry point.

This is the ONE class users interact with. It wires together:
- Hardware (arm + gripper)
- Perception pipeline
- Skill registry
- World model
- Task executor
"""
from __future__ import annotations

import logging
from typing import Any

from vector_os_nano.core.config import load_config
from vector_os_nano.core.executor import TaskExecutor
from vector_os_nano.core.skill import Skill, SkillContext, SkillRegistry
from vector_os_nano.core.types import ExecutionResult
from vector_os_nano.core.world_model import WorldModel

logger = logging.getLogger(__name__)


class Agent:
    """Robot arm / mobile base control via structured skill calls.

    Usage::

        from vector_os_nano import Agent, SO101
        arm = SO101(port="/dev/ttyACM0")
        agent = Agent(arm=arm)
        agent.execute_skill("pick", {"object_label": "red cup"})

    All hardware arguments are optional — Agent degrades gracefully when
    components are absent (useful for unit testing and partial deployments).

    Args:
        arm: Object implementing ArmProtocol.  None disables motion.
        gripper: Object implementing GripperProtocol.  If None and arm is an
            SO101Arm (has ``_bus``), a SO101Gripper is created automatically.
        perception: Object implementing PerceptionProtocol.  None disables
            vision-based skills.
        skills: Additional Skill instances to register alongside the built-in
            defaults.
        config: Configuration override — a dict, a path to a YAML file, or
            None to use the built-in defaults.
        auto_perception: When True and perception=None and config specifies
            ``camera.type == "realsense"``, automatically construct a
            RealSenseCamera + VLMDetector + EdgeTAMTracker + PerceptionPipeline.
            Defaults to False because loading the VLM and tracker is slow and
            consumes GPU memory — callers that want auto-setup must opt in
            explicitly.
        arms/grippers/hands/bases: Optional named hardware registries.  These
            are additive to the legacy singular arguments and allow composite
            robots to expose, for example, independent left and right limbs.
        services: Named services injected into every SkillContext.
        default_*_name: Explicit registry entries used by the legacy singular
            views (``context.arm``, ``context.gripper``, and ``context.base``).
    """

    def __init__(
        self,
        arm: Any = None,
        gripper: Any = None,
        perception: Any = None,
        skills: list[Skill] | None = None,
        config: dict | str | None = None,
        auto_perception: bool = False,
        base: Any = None,
        # Deprecated LLM kwargs — accepted but ignored for backward compat
        llm: Any = None,
        llm_api_key: str | None = None,
        *,
        arms: dict[str, Any] | None = None,
        grippers: dict[str, Any] | None = None,
        hands: dict[str, Any] | None = None,
        bases: dict[str, Any] | None = None,
        services: dict[str, Any] | None = None,
        default_arm_name: str | None = None,
        default_gripper_name: str | None = None,
        default_hand_name: str | None = None,
        default_base_name: str | None = None,
    ) -> None:
        # ---- Configuration --------------------------------------------------
        if isinstance(config, dict):
            self._config: dict = config
        else:
            self._config = load_config(config)

        # ---- Hardware -------------------------------------------------------
        # Track which constructor surface owns each registry.  Legacy code in
        # this repository still replaces ``agent._arm`` / ``agent._base`` after
        # construction; when the corresponding plural argument was omitted,
        # those singular aliases must remain the source of truth.
        self._legacy_arms_source = arms is None
        self._legacy_grippers_source = grippers is None
        self._legacy_bases_source = bases is None
        self._arms, self._default_arm_name = self._merge_hardware_registry(
            "arm", arms, arm, default_arm_name
        )

        # When a composite robot uses matching side names, selecting an arm
        # also selects its matching gripper/hand unless the caller explicitly
        # chose another default.  Legacy singular arguments remain authoritative.
        paired_gripper_default = default_gripper_name
        if (
            gripper is None
            and paired_gripper_default is None
            and self._default_arm_name in (grippers or {})
        ):
            paired_gripper_default = self._default_arm_name
        self._grippers, self._default_gripper_name = self._merge_hardware_registry(
            "gripper", grippers, gripper, paired_gripper_default
        )

        paired_hand_default = default_hand_name
        if (
            paired_hand_default is None
            and self._default_arm_name in (hands or {})
        ):
            paired_hand_default = self._default_arm_name
        self._hands, self._default_hand_name = self._merge_hardware_registry(
            "hand", hands, None, paired_hand_default
        )
        self._bases, self._default_base_name = self._merge_hardware_registry(
            "base", bases, base, default_base_name
        )
        self._services: dict[str, Any] = dict(services or {})

        # Singular aliases are intentionally retained: existing skills, worlds,
        # prompts, and downstream integrations still consume these attributes.
        self._arm = self._selected_hardware(self._arms, self._default_arm_name)
        self._gripper = self._selected_hardware(
            self._grippers, self._default_gripper_name
        )
        self._base = self._selected_hardware(self._bases, self._default_base_name)

        # Auto-create SO101Gripper when arm shares a serial bus and no gripper
        # was explicitly supplied.
        if (
            gripper is None
            and not self._grippers
            and arm is not None
            and hasattr(arm, "_bus")
            and arm._bus is not None
        ):
            try:
                from vector_os_nano.hardware.so101.gripper import SO101Gripper  # lazy

                self._gripper = SO101Gripper(arm._bus)
                self._grippers = {"default": self._gripper}
                self._default_gripper_name = "default"
            except Exception as exc:
                logger.warning("Could not auto-create SO101Gripper: %s", exc)

        # ---- Perception -----------------------------------------------------
        self._perception = perception

        # Auto-create perception pipeline when caller opts in and no pipeline
        # was supplied explicitly.  Heavy imports (torch, transformers, pyrealsense2)
        # are deferred to here so normal Agent construction stays fast.
        if auto_perception and self._perception is None:
            cam_type = self._config.get("camera", {}).get("type", "")
            if cam_type == "realsense":
                try:
                    from vector_os_nano.perception.realsense import RealSenseCamera
                    from vector_os_nano.perception.vlm import VLMDetector
                    from vector_os_nano.perception.tracker import EdgeTAMTracker
                    from vector_os_nano.perception.pipeline import PerceptionPipeline

                    cam = RealSenseCamera()
                    cam.connect()
                    vlm_det = VLMDetector()
                    tracker = EdgeTAMTracker()
                    self._perception = PerceptionPipeline(
                        camera=cam, vlm=vlm_det, tracker=tracker
                    )
                    logger.info("Auto-created perception pipeline (RealSense + VLM + tracker)")
                except Exception as exc:
                    logger.warning("Could not auto-create perception pipeline: %s", exc)

        # ---- World model ----------------------------------------------------
        self._world_model = WorldModel()

        # ---- Skill registry -------------------------------------------------
        self._skill_registry = SkillRegistry()

        # Register built-in skills
        from vector_os_nano.skills import get_default_skills  # lazy-ish (already imported by wave 2)

        for skill in get_default_skills():
            self._skill_registry.register(skill)

        # Register caller-supplied custom skills
        if skills:
            for skill in skills:
                self._skill_registry.register(skill)

        # ---- Executor -------------------------------------------------------
        self._executor = TaskExecutor()

        # ---- Lazy-init state ------------------------------------------------
        self._ik_solver: Any = None
        self._calibration: Any = None

        # ---- Sync world model with hardware ----------------------------------
        self._sync_robot_state()

    @staticmethod
    def _merge_hardware_registry(
        kind: str,
        registry: dict[str, Any] | None,
        legacy_value: Any,
        default_name: str | None,
    ) -> tuple[dict[str, Any], str | None]:
        """Merge a legacy singular device with a named registry.

        Existing singular construction becomes a one-entry ``default`` registry.
        Supplying both forms is accepted only when the singular device is already
        present in the named registry; otherwise the request is ambiguous and is
        rejected instead of silently selecting the wrong hardware.
        """
        merged: dict[str, Any] = dict(registry or {})

        if legacy_value is not None:
            matching_names = [
                name for name, device in merged.items() if device is legacy_value
            ]
            if not merged:
                inserted_name = default_name or "default"
                merged[inserted_name] = legacy_value
                default_name = inserted_name
            elif not matching_names:
                raise ValueError(
                    f"Conflicting {kind} and {kind}s registry: the singular "
                    f"device is not present in the named registry"
                )
            elif default_name is None:
                default_name = matching_names[0]
            elif merged.get(default_name) is not legacy_value:
                raise ValueError(
                    f"Conflicting default {kind} {default_name!r}: it does not "
                    f"refer to the singular {kind}"
                )

        if default_name is not None and default_name not in merged:
            available = ", ".join(merged) or "none"
            raise ValueError(
                f"Unknown default {kind} {default_name!r}; available {kind}s: "
                f"{available}"
            )
        if default_name is None and merged:
            default_name = next(iter(merged))

        return merged, default_name

    @staticmethod
    def _selected_hardware(
        registry: dict[str, Any], default_name: str | None
    ) -> Any:
        if not registry:
            return None
        if default_name is not None:
            return registry[default_name]
        return next(iter(registry.values()))

    @staticmethod
    def _uses_so101_external_ik(arm: Any) -> bool:
        """True only for implementations built around the SO-101 IK model.

        Capability duck-typing is intentionally insufficient here: injecting an
        SO-101 kinematic model into a different robot can produce unsafe targets,
        and importing Pinocchio is both heavy and platform-sensitive.
        """
        cls = type(arm)
        return (cls.__module__, cls.__name__) in {
            ("vector_os_nano.hardware.so101.arm", "SO101Arm"),
            ("vector_os_nano.hardware.sim.pybullet_arm", "SimulatedArm"),
        }

    @staticmethod
    def _registry_snapshot(
        registry: dict[str, Any],
        default_name: str | None,
        singular_value: Any,
        legacy_source: bool,
    ) -> tuple[dict[str, Any], str | None]:
        """Return the live registry view without mutating Agent-owned maps."""
        if not legacy_source:
            return dict(registry), default_name
        if singular_value is None:
            return {}, None
        name = default_name or "default"
        return {name: singular_value}, name

    def _hardware_registry_snapshots(self) -> tuple[
        tuple[dict[str, Any], str | None],
        tuple[dict[str, Any], str | None],
        tuple[dict[str, Any], str | None],
    ]:
        return (
            self._registry_snapshot(
                self._arms,
                self._default_arm_name,
                self._arm,
                self._legacy_arms_source,
            ),
            self._registry_snapshot(
                self._grippers,
                self._default_gripper_name,
                self._gripper,
                self._legacy_grippers_source,
            ),
            self._registry_snapshot(
                self._bases,
                self._default_base_name,
                self._base,
                self._legacy_bases_source,
            ),
        )

    def _iter_unique_hardware(
        self, *, reverse: bool = False, include_perception: bool = True
    ):
        """Yield every registered peripheral once, deduplicated by identity."""
        (arms, _), (grippers, _), (bases, _) = self._hardware_registry_snapshots()
        groups = [
            tuple(bases.values()),
            tuple(arms.values()),
            tuple(self._hands.values()),
            tuple(grippers.values()),
            (
                (self._perception,)
                if include_perception and self._perception is not None
                else ()
            ),
        ]
        devices = [device for group in groups for device in group]
        if reverse:
            devices.reverse()

        seen: set[int] = set()
        for device in devices:
            identity = id(device)
            if identity in seen:
                continue
            seen.add(identity)
            yield device

    # -------------------------------------------------------------------------
    # Public API — primary entry point
    # -------------------------------------------------------------------------

    def _sync_robot_state(self) -> None:
        """Sync world model robot state from hardware."""
        if self._arm is not None:
            try:
                joints = self._arm.get_joint_positions()
                ee_pos = (0.0, 0.0, 0.0)
                if self._ik_solver is not None:
                    try:
                        pos, _ = self._ik_solver.fk(joints)
                        ee_pos = tuple(pos)
                    except Exception:
                        pass
                self._world_model.update_robot_state(
                    joint_positions=tuple(joints),
                    ee_position=ee_pos,
                )
            except Exception as exc:
                logger.debug("Could not sync robot state: %s", exc)

        if self._base is not None:
            try:
                pos = self._base.get_position()
                heading = self._base.get_heading()
                self._world_model.update_robot_state(
                    position_xy=(pos[0], pos[1]),
                    heading=heading,
                )
            except Exception as exc:
                logger.debug("Could not sync base state: %s", exc)

    # -------------------------------------------------------------------------
    # Convenience methods
    # -------------------------------------------------------------------------

    def home(self) -> bool:
        """Move arm to home position.

        Returns:
            True when the home skill succeeds, False otherwise.
        """
        return self.execute_skill("home").success

    def stop(self) -> None:
        """Request a normal best-effort stop on every motion peripheral.

        This generic lifecycle helper is not a certified emergency stop.  G1
        callers must use the explicit ``emergency_stop`` skill/robot method to
        enter the software emergency latch.
        """
        for device in self._iter_unique_hardware(include_perception=False):
            stop = getattr(device, "stop", None)
            if not callable(stop):
                continue
            try:
                stop()
            except Exception as exc:
                # Emergency stop must continue across independent limbs even if
                # one driver is already faulted.
                logger.warning("Could not stop %s: %s", type(device).__name__, exc)

    def connect(self) -> None:
        """Connect to all hardware peripherals."""
        connected: list[Any] = []
        try:
            for device in self._iter_unique_hardware():
                connect = getattr(device, "connect", None)
                if not callable(connect):
                    continue
                connect()
                connected.append(device)
        except Exception:
            # Avoid leaving a composite robot half-connected.  Cleanup remains
            # best-effort and the original connection error is re-raised.
            for device in reversed(connected):
                disconnect = getattr(device, "disconnect", None)
                if not callable(disconnect):
                    continue
                try:
                    disconnect()
                except Exception as exc:
                    logger.warning(
                        "Connection rollback failed for %s: %s",
                        type(device).__name__, exc,
                    )
            raise

    def disconnect(self) -> None:
        """Disconnect from all hardware peripherals."""
        for device in self._iter_unique_hardware(reverse=True):
            disconnect = getattr(device, "disconnect", None)
            if not callable(disconnect):
                continue
            try:
                disconnect()
            except Exception as exc:
                # Teardown must not leak the other limb because one driver has
                # already disappeared.
                logger.warning(
                    "Could not disconnect %s: %s", type(device).__name__, exc
                )

    def execute_skill(
        self,
        skill_name: str,
        params: dict | None = None,
        on_message: Any = None,
        on_step: Any = None,
        on_step_done: Any = None,
    ) -> ExecutionResult:
        """Execute a skill directly with structured parameters.

        Bypasses string parsing and alias matching — ideal for MCP tool calls
        where params are already structured (e.g., object_label, mode).

        If the skill has auto_steps, builds a plan with the correct params
        for each step. Otherwise executes the single skill directly.
        """
        from vector_os_nano.core.types import TaskPlan, TaskStep

        params = params or {}
        skill_obj = self._skill_registry.get(skill_name)
        if skill_obj is None:
            return ExecutionResult(
                success=False, status="failed",
                failure_reason=f"Unknown skill: {skill_name}",
            )

        # Determine object query from params (for detect step in auto_steps)
        object_query = (
            params.get("object_label")
            or params.get("object_id")
            or params.get("query")
            or ""
        )
        limb_params = {
            key: params[key]
            for key in ("arm", "side")
            if key in params
        }
        configured_pick_mode = (
            self._config.get("skills", {})
            .get("pick", {})
            .get("default_mode", "drop")
        )
        effective_pick_mode = params.get("mode", configured_pick_mode)

        # Build steps: use auto_steps if available, otherwise single skill
        raw_auto = getattr(skill_obj, "__skill_auto_steps__", [])
        auto_steps = list(raw_auto) if raw_auto else [skill_name]
        if skill_name == "pick" and self._perception is None:
            # A caller may seed a simulator/world-model object with an already
            # grounded pelvis/base-frame position.  Requiring DetectSkill in
            # that case would fail solely because no camera is attached, before
            # the existing Cartesian pick controller gets a chance to run.
            grounded = None
            object_id = params.get("object_id")
            if object_id:
                grounded = self._world_model.get_object(str(object_id))
            elif params.get("object_label"):
                matches = self._world_model.get_objects_by_label(
                    str(params["object_label"])
                )
                grounded = matches[0] if matches else None
            if grounded is not None:
                auto_steps = [skill_name]
        steps = []
        for i, step_skill in enumerate(auto_steps):
            step_params: dict = {}
            if step_skill == skill_name:
                # Pass ALL original params to the target skill
                step_params = dict(params)
                if skill_name == "pick" and "mode" not in step_params:
                    # Resolve deployment defaults into the executable plan so
                    # Skill behavior, auto-home policy and WorldModel effects
                    # all observe the same explicit mode.
                    step_params["mode"] = effective_pick_mode
            elif step_skill == "detect" and object_query:
                # Pass specific query to detect step so VLM returns matching labels.
                # "all objects" causes label mismatches in world model fallback.
                step_params = {"query": object_query}
            elif step_skill == "scan":
                # Composite robots must perceive with the same named limb that
                # will execute the target action.  This is a no-op for legacy
                # calls, where no arm/side selector is present.
                step_params = dict(limb_params)
            steps.append(TaskStep(
                step_id=f"s{i+1}",
                skill_name=step_skill,
                parameters=step_params,
                depends_on=[f"s{i}"] if i > 0 else [],
                preconditions=[],
                postconditions=[],
            ))

        # Add home at end if not already there — but NOT when:
        # - pick mode='hold' (HomeSkill opens gripper, would drop held object)
        # - gripper_close / gripper_open (HomeSkill opens gripper, overrides state)
        # - home itself
        no_auto_home_skills = {
            # Gripper/hand state is itself the requested result.
            "gripper_close",
            "gripper_open",
            "dex3_pose",
            # Base/navigation/safety actions must never trigger an unrelated
            # arm move after they complete (especially not after stop).
            "walk",
            "turn",
            "stand",
            "navigate",
            "where_am_i",
            "stop",
            "emergency_stop",
        }
        skip_home = (
            (skill_name == "pick" and effective_pick_mode == "hold")
            or skill_name == "home"
            or skill_name in no_auto_home_skills
        )
        if not skip_home and (not steps or steps[-1].skill_name != "home"):
            steps.append(TaskStep(
                step_id=f"s{len(steps)+1}",
                skill_name="home",
                parameters=dict(limb_params),
                depends_on=[f"s{len(steps)}"],
                preconditions=[],
                postconditions=[],
            ))

        goal = f"{skill_name} {object_query}".strip()
        plan = TaskPlan(goal=goal, steps=steps)

        if on_message:
            step_names = [s.skill_name for s in steps]
            on_message(f"Executing: {' → '.join(step_names)}")

        context = self._build_context()
        result = self._executor.execute(
            plan, self._skill_registry, context,
            on_step=on_step, on_step_done=on_step_done,
        )
        self._sync_robot_state()

        return result

    def register_skill(self, skill: Skill) -> None:
        """Register a custom skill, immediately available to the planner.

        Overwrites any existing skill with the same name.

        Args:
            skill: Object satisfying the Skill protocol.
        """
        self._skill_registry.register(skill)

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def world(self) -> WorldModel:
        """The current world model."""
        return self._world_model

    @property
    def skills(self) -> list[str]:
        """Names of all currently registered skills."""
        return self._skill_registry.list_skills()

    # -------------------------------------------------------------------------
    # Context manager
    # -------------------------------------------------------------------------

    def __enter__(self) -> "Agent":
        self.connect()
        return self

    def __exit__(self, *args: Any) -> None:
        self.disconnect()

    # -------------------------------------------------------------------------
    # Context construction
    # -------------------------------------------------------------------------

    def build_context(self) -> SkillContext:
        """Build a SkillContext from current agent state.

        IK solver and calibration are initialised lazily on first call.

        Returns:
            SkillContext bundling all resources skills need during execution.
        """
        # Lazy-init IK solver
        if (
            self._ik_solver is None
            and self._arm is not None
            and self._uses_so101_external_ik(self._arm)
        ):
            try:
                from vector_os_nano.hardware.so101.ik_solver import IKSolver  # lazy

                self._ik_solver = IKSolver()
                self._arm.set_ik_solver(self._ik_solver)
            except Exception as exc:
                logger.debug("IK solver not available: %s", exc)

        # Lazy-init calibration
        if self._calibration is None:
            try:
                from pathlib import Path

                from vector_os_nano.perception.calibration import Calibration  # lazy

                cal_file: str = self._config.get("calibration", {}).get("file", "")
                if cal_file:
                    # Resolve relative paths: try ~/Desktop/vector_os/<cal_file> then cwd
                    cal_path = Path(cal_file)
                    if not cal_path.is_absolute():
                        candidate = Path.home() / "Desktop" / "vector_os_nano" / cal_path
                        if not candidate.exists():
                            candidate = Path.cwd() / cal_path
                        if candidate.exists():
                            cal_path = candidate
                    self._calibration = Calibration.load(str(cal_path))
                else:
                    self._calibration = Calibration()
            except Exception as exc:
                logger.debug("Calibration not available: %s", exc)

        # Build services dict for skills that need injected dependencies
        services: dict = dict(self._services)
        if hasattr(self, "_vlm") and self._vlm is not None:
            services["vlm"] = self._vlm
        if hasattr(self, "_spatial_memory") and self._spatial_memory is not None:
            services["spatial_memory"] = self._spatial_memory
        if hasattr(self, "_skill_registry"):
            services["skill_registry"] = self._skill_registry

        (arms, default_arm_name), (
            grippers, default_gripper_name
        ), (bases, default_base_name) = self._hardware_registry_snapshots()

        return SkillContext(
            arms=arms,
            grippers=grippers,
            hands=dict(self._hands),
            bases=bases,
            perception_sources=(
                {"default": self._perception}
                if self._perception is not None
                else {}
            ),
            services=services,
            default_arm_name=default_arm_name,
            default_gripper_name=default_gripper_name,
            default_hand_name=self._default_hand_name,
            default_base_name=default_base_name,
            world_model=self._world_model,
            calibration=self._calibration,
            config=self._config,
        )

    def _build_context(self) -> SkillContext:
        """Backward-compatible alias for :meth:`build_context`."""
        return self.build_context()
