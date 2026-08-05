# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Whole-body lifecycle manager and command arbiter for G1."""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Sequence

from vector_os_nano.hardware.g1.profile import G1Profile
from vector_os_nano.hardware.g1.state import (
    G1ControlMode,
    G1JointCommand,
    G1State,
    G1VelocityCommand,
)
from vector_os_nano.hardware.g1.transport import (
    G1CapabilityError,
    G1Transport,
    G1TransportCapabilities,
)


class G1ControlCoordinator:
    """Serialize all access to one G1 transport.

    ``G1Base``, both arm adapters, and both hand adapters are lightweight
    views over this object.  Lifecycle owners are reference-counted by name,
    so connecting several views opens the transport once and disconnecting one
    view cannot tear it down while another view still uses it.
    """

    def __init__(self, profile: G1Profile, transport: G1Transport) -> None:
        if transport.profile != profile:
            raise ValueError("G1 transport and coordinator must use the same profile")
        self.profile = profile
        self.transport = transport
        self._lock = threading.RLock()
        self._sequence_id = 0
        self._mode = G1ControlMode.PASSIVE
        self._emergency = False
        self._last_velocity = (0.0, 0.0, 0.0)
        self._owners: set[str] = set()

    @property
    def mode(self) -> G1ControlMode:
        with self._lock:
            return self._mode

    @property
    def connected(self) -> bool:
        return bool(self.transport.connected)

    @property
    def capabilities(self) -> G1TransportCapabilities:
        capabilities = getattr(self.transport, "capabilities", None)
        if not isinstance(capabilities, G1TransportCapabilities):
            raise G1CapabilityError(
                f"G1 transport {self.transport.name!r} does not declare capabilities"
            )
        return capabilities

    @property
    def lifecycle_owners(self) -> frozenset[str]:
        with self._lock:
            return frozenset(self._owners)

    @property
    def last_velocity(self) -> tuple[float, float, float]:
        with self._lock:
            return self._last_velocity

    @property
    def emergency_latched(self) -> bool:
        """Whether the software emergency command latch is active."""
        with self._lock:
            return self._emergency

    def _next_sequence(self) -> int:
        self._sequence_id += 1
        return self._sequence_id

    @staticmethod
    def _owner_name(owner: str | None) -> str:
        name = "coordinator" if owner is None else str(owner).strip()
        if not name:
            raise ValueError("G1 lifecycle owner cannot be empty")
        return name

    def _require_connected(self) -> None:
        if not self.transport.connected:
            raise RuntimeError("G1 transport is not connected")

    def connect(self, *, owner: str | None = None) -> None:
        owner_name = self._owner_name(owner)
        with self._lock:
            if owner_name in self._owners and self.transport.connected:
                return
            if not self.transport.connected:
                self.transport.connect()
            if not self.transport.connected:
                # A transport must never silently report a successful connect.
                try:
                    self.transport.disconnect()
                finally:
                    raise ConnectionError(
                        f"G1 transport {self.transport.name!r} did not connect"
                    )
            self._owners.add(owner_name)

    def disconnect(self, *, owner: str | None = None) -> None:
        owner_name = self._owner_name(owner)
        with self._lock:
            if owner_name not in self._owners:
                return
            self._owners.remove(owner_name)
            if self._owners:
                return
            try:
                self.stop(emergency=False)
            finally:
                if self.transport.connected:
                    self.transport.disconnect()
                self._mode = G1ControlMode.PASSIVE
                self._last_velocity = (0.0, 0.0, 0.0)

    def read_state(self) -> G1State:
        with self._lock:
            self._require_connected()
            state = self.transport.read_state()
            if not isinstance(state, G1State):
                raise TypeError("G1 transport read_state() must return G1State")
            if state.joint_names != self.profile.all_joint_names:
                raise ValueError(
                    "G1 state joint order mismatch: expected HAL semantic order "
                    f"{self.profile.all_joint_names!r}, got {state.joint_names!r}"
                )
            return state

    def set_mode(self, mode: G1ControlMode) -> None:
        mode = G1ControlMode(mode)
        with self._lock:
            self._require_connected()
            if self._emergency and mode is not G1ControlMode.EMERGENCY_DAMPING:
                raise RuntimeError("G1 software emergency stop is latched")
            if mode in {G1ControlMode.STAND, G1ControlMode.LOCOMOTION} and not self.capabilities.locomotion:
                raise G1CapabilityError(
                    f"G1 transport {self.transport.name!r} has no standing/locomotion controller"
                )
            if (
                mode is G1ControlMode.MANIPULATION
                and self._mode is G1ControlMode.LOCOMOTION
            ):
                self._send_zero_velocity_locked()
            self.transport.set_mode(mode)
            self._mode = mode

    def command_velocity(
        self,
        vx: float,
        vy: float,
        vyaw: float,
        *,
        ttl: float = 0.3,
    ) -> None:
        with self._lock:
            self._require_connected()
            if self._emergency:
                raise RuntimeError("G1 software emergency stop is latched")
            if not self.capabilities.locomotion:
                raise G1CapabilityError(
                    f"G1 transport {self.transport.name!r} has no locomotion controller"
                )
            limits = self.profile.velocity_limits
            values = (float(vx), float(vy), float(vyaw))
            if not all(math.isfinite(value) for value in values):
                raise ValueError("velocity command contains a non-finite value")
            clipped = (
                max(-limits["vx"], min(limits["vx"], values[0])),
                max(-limits["vy"], min(limits["vy"], values[1])),
                max(-limits["vyaw"], min(limits["vyaw"], values[2])),
            )
            moving = any(abs(value) > 1e-6 for value in clipped)
            target_mode = G1ControlMode.LOCOMOTION if moving else G1ControlMode.STAND
            if self._mode is not target_mode:
                self.transport.set_mode(target_mode)
                self._mode = target_mode
            command = G1VelocityCommand(
                vx=clipped[0],
                vy=clipped[1],
                vyaw=clipped[2],
                sequence_id=self._next_sequence(),
                ttl=ttl,
            )
            self.transport.command_velocity(command)
            self._last_velocity = clipped

    def _send_zero_velocity_locked(self) -> None:
        if not self.capabilities.locomotion:
            raise G1CapabilityError(
                f"G1 transport {self.transport.name!r} has no locomotion controller"
            )
        command = G1VelocityCommand(
            vx=0.0,
            vy=0.0,
            vyaw=0.0,
            sequence_id=self._next_sequence(),
            ttl=0.3,
        )
        self.transport.command_velocity(command)
        self._last_velocity = (0.0, 0.0, 0.0)

    def command_group(
        self,
        group: str,
        joint_names: Sequence[str],
        positions: Sequence[float],
        *,
        duration: float = 0.0,
        ttl: float | None = None,
    ) -> None:
        with self._lock:
            self._require_connected()
            if self._emergency:
                raise RuntimeError("G1 software emergency stop is latched")
            expected_names = self.profile.joint_group(group)
            names = tuple(str(name) for name in joint_names)
            values = tuple(float(value) for value in positions)
            if names != expected_names:
                raise ValueError(
                    f"G1 group {group!r} requires exact joint order {expected_names!r}, "
                    f"got {names!r}"
                )
            self.profile.validate_joint_positions(names, values)
            duration_value = float(duration)
            ttl_value = float(ttl if ttl is not None else max(1.0, duration_value + 0.5))
            if not math.isfinite(duration_value) or duration_value < 0:
                raise ValueError("joint command duration must be finite and non-negative")
            if not math.isfinite(ttl_value) or ttl_value <= 0:
                raise ValueError("joint command TTL must be finite and positive")
            if not self.capabilities.supports_group(group):
                raise G1CapabilityError(
                    f"G1 transport {self.transport.name!r} does not control group {group!r}"
                )
            if group.endswith("_arm") or group.endswith("_hand"):
                if self._mode is G1ControlMode.LOCOMOTION:
                    self._send_zero_velocity_locked()
                if self._mode is not G1ControlMode.MANIPULATION:
                    self.transport.set_mode(G1ControlMode.MANIPULATION)
                    self._mode = G1ControlMode.MANIPULATION
            command = G1JointCommand(
                group=group,
                joint_names=names,
                positions=values,
                sequence_id=self._next_sequence(),
                duration=duration_value,
                ttl=ttl_value,
            )
            self.transport.command_joints(command)

    def hold_group(self, group: str) -> None:
        names = self.profile.joint_group(group)
        state = self.read_state()
        self.command_group(group, names, state.positions_for(names), duration=0.0)

    def wait_group_target(
        self,
        group: str,
        positions: Sequence[float],
        *,
        tolerance: float = 0.08,
        timeout: float = 5.0,
    ) -> bool:
        """Poll semantic joint state until a commanded group converges.

        ArmProtocol.move_joints is blocking.  Keeping convergence here makes
        that contract identical for MuJoCo, Isaac and a future real transport,
        instead of treating a published target as completed motion.
        """
        names = self.profile.joint_group(group)
        targets = tuple(float(value) for value in positions)
        tolerance = float(tolerance)
        timeout = float(timeout)
        if len(targets) != len(names):
            raise ValueError(f"G1 group {group!r} target has the wrong dimension")
        if not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError("joint convergence tolerance must be finite and positive")
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("joint convergence timeout must be finite and non-negative")
        deadline = time.monotonic() + timeout
        while True:
            state = self.read_state()
            current = state.positions_for(names)
            if max(abs(actual - target) for actual, target in zip(current, targets)) <= tolerance:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)

    def get_lidar_scan(self) -> Any:
        with self._lock:
            self._require_connected()
            if not self.capabilities.lidar:
                return None
            return self.transport.get_lidar_scan()

    def wait_stable(
        self,
        *,
        max_speed: float = 0.05,
        duration: float = 0.5,
        timeout: float = 5.0,
    ) -> bool:
        max_speed = float(max_speed)
        duration = float(duration)
        timeout = float(timeout)
        if (
            not all(math.isfinite(value) for value in (max_speed, duration, timeout))
            or max_speed < 0
            or duration < 0
            or timeout < 0
        ):
            raise ValueError("stability thresholds must be finite and non-negative")
        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        while time.monotonic() <= deadline:
            state = self.read_state()
            speed = math.hypot(*state.root_linear_velocity[:2])
            if not state.fallen and state.fault is None and speed <= max_speed:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= duration:
                    return True
            else:
                stable_since = None
            time.sleep(0.02)
        return False

    def stop(self, *, emergency: bool = False) -> bool:
        """Request a whole-body stop and report whether a backend accepted it.

        Normal lifecycle stops remain best-effort and non-throwing.  An
        explicitly requested emergency stop always latches locally, but raises
        when no connected backend stop/damping path accepted the command.  This
        prevents a software latch alone from being reported as physical stop
        delivery.
        """
        with self._lock:
            connected = bool(self.transport.connected)
            delivered = False
            emergency_delivered = False
            damping_delivered = False
            failures: list[str] = []
            try:
                capabilities = self.capabilities
                has_locomotion = capabilities.locomotion
                has_emergency_damping = capabilities.emergency_damping
            except Exception as exc:
                has_locomotion = False
                has_emergency_damping = False
                failures.append(f"capability query: {exc}")
            if connected and has_locomotion:
                try:
                    self._send_zero_velocity_locked()
                    delivered = True
                except Exception as exc:
                    failures.append(f"zero velocity: {exc}")
            if connected:
                if emergency and has_emergency_damping:
                    try:
                        self.transport.set_mode(G1ControlMode.EMERGENCY_DAMPING)
                        delivered = True
                        emergency_delivered = True
                        damping_delivered = True
                    except Exception as exc:
                        failures.append(f"emergency damping: {exc}")
                try:
                    self.transport.stop()
                    delivered = True
                    if emergency:
                        emergency_delivered = True
                except Exception as exc:
                    failures.append(f"transport stop: {exc}")
            self._last_velocity = (0.0, 0.0, 0.0)
            if emergency:
                self._emergency = True
                self._mode = G1ControlMode.EMERGENCY_DAMPING
            elif self._emergency:
                # A normal stop is idempotent inside the emergency latch; it
                # must never make the coordinator appear recovered.
                self._mode = G1ControlMode.EMERGENCY_DAMPING
            else:
                self._mode = (
                    G1ControlMode.STAND
                    if connected and has_locomotion
                    else G1ControlMode.PASSIVE
                )

            emergency_failed = emergency and (
                (has_emergency_damping and not damping_delivered)
                or (not has_emergency_damping and not emergency_delivered)
            )
            if emergency_failed:
                detail = "; ".join(failures) or "transport is not connected"
                raise RuntimeError(
                    "G1 emergency stop was latched locally but the backend "
                    f"stop contract was not satisfied: {detail}"
                )
            return delivered

    def emergency_stop(self) -> bool:
        """Latch commands and request zero/hold or real damping when supported."""
        return self.stop(emergency=True)

    def clear_emergency(self) -> None:
        """Explicitly release the latch into the safest supported idle mode.

        A locomotion backend recovers to ``STAND``.  A fixed-base manipulation
        backend (for example the phase-one MuJoCo controller) has no standing
        policy, so it recovers to ``PASSIVE`` and can accept a later arm/hand
        command.  Recovery is deliberately never performed by ``connect()`` or
        ``disconnect()``.
        """
        with self._lock:
            self._require_connected()
            target_mode = (
                G1ControlMode.STAND
                if self.capabilities.locomotion
                else G1ControlMode.PASSIVE
            )
            self.transport.set_mode(target_mode)
            self._emergency = False
            self._mode = target_mode


__all__ = ["G1ControlCoordinator"]
