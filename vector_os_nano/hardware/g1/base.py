# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""BaseProtocol adapter for the lower-body view of one G1 robot."""
from __future__ import annotations

import math
import time
from typing import Any

from vector_os_nano.core.types import Odometry
from vector_os_nano.hardware.g1.coordinator import G1ControlCoordinator
from vector_os_nano.hardware.g1.state import G1ControlMode


class G1Base:
    """Expose G1 locomotion through the repository's generic base contract.

    Constructing this adapter does not create a locomotion controller.  Calls
    to :meth:`walk` and :meth:`set_velocity` fail with ``G1CapabilityError``
    until the selected transport declares a working locomotion backend.
    """

    def __init__(
        self,
        coordinator: G1ControlCoordinator,
        *,
        owner: str = "base",
    ) -> None:
        self.coordinator = coordinator
        self._owner = owner

    @property
    def name(self) -> str:
        return "g1"

    @property
    def connected(self) -> bool:
        return self.coordinator.connected

    @property
    def supports_holonomic(self) -> bool:
        return self.coordinator.capabilities.holonomic

    @property
    def supports_lidar(self) -> bool:
        return self.coordinator.capabilities.lidar

    @property
    def supports_emergency_damping(self) -> bool:
        """Whether the backend has a real damping mode, not only a stop latch."""
        return self.coordinator.capabilities.emergency_damping

    def connect(self) -> None:
        self.coordinator.connect(owner=self._owner)

    def disconnect(self) -> None:
        self.coordinator.disconnect(owner=self._owner)

    def stop(self) -> bool:
        return self.coordinator.stop(emergency=False)

    def emergency_stop(self) -> bool:
        """Latch commands and request the backend's strongest supported stop."""
        return self.coordinator.emergency_stop()

    @property
    def emergency_latched(self) -> bool:
        return self.coordinator.emergency_latched

    def clear_emergency(self) -> None:
        """Explicitly release a previously latched emergency stop."""
        self.coordinator.clear_emergency()

    def stand(
        self,
        *,
        wait: bool = False,
        timeout: float = 5.0,
        stable_duration: float = 0.5,
        max_speed: float = 0.05,
    ) -> bool:
        """Request STAND without inventing a backend-independent stand controller.

        When ``wait`` is false, ``True`` means the backend accepted the mode
        transition.  When true, the result additionally requires a stable,
        non-fallen state for ``stable_duration`` within ``timeout``.
        """
        self.coordinator.set_mode(G1ControlMode.STAND)
        if not wait:
            return True
        return self.coordinator.wait_stable(
            max_speed=max_speed,
            duration=stable_duration,
            timeout=timeout,
        )

    def walk(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vyaw: float = 0.0,
        duration: float = 1.0,
    ) -> bool:
        duration_value = float(duration)
        if not math.isfinite(duration_value) or duration_value < 0:
            raise ValueError("walk duration must be finite and non-negative")
        started = False
        try:
            self.coordinator.command_velocity(vx, vy, vyaw)
            started = True
            deadline = time.monotonic() + duration_value
            # The Isaac ROS/file gateway intentionally expires velocity
            # channels after 500 ms.  Refresh well inside that watchdog window
            # so a multi-second walk/turn does not silently become a 0.5 s
            # pulse while this method continues reporting healthy state.
            refresh_period = 0.2
            next_refresh = time.monotonic() + refresh_period
            while time.monotonic() < deadline:
                state = self.coordinator.read_state()
                if state.fallen or state.fault is not None:
                    return False
                now = time.monotonic()
                if now >= next_refresh and now < deadline:
                    self.coordinator.command_velocity(vx, vy, vyaw)
                    next_refresh = now + refresh_period
                time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
            final_state = self.coordinator.read_state()
            return not final_state.fallen and final_state.fault is None
        finally:
            if started:
                # This may surface a genuine backend failure.  It should not be
                # hidden as a successful completed walk.
                self.coordinator.command_velocity(0.0, 0.0, 0.0)

    def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        self.coordinator.command_velocity(vx, vy, vyaw)

    def get_position(self) -> list[float]:
        pose = self.coordinator.read_state().root_pose
        return [float(pose.x), float(pose.y), float(pose.z)]

    def get_heading(self) -> float:
        pose = self.coordinator.read_state().root_pose
        siny_cosp = 2.0 * (pose.qw * pose.qz + pose.qx * pose.qy)
        cosy_cosp = 1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_velocity(self) -> list[float]:
        velocity = self.coordinator.read_state().root_linear_velocity
        return [float(value) for value in velocity]

    def get_odometry(self) -> Odometry:
        state = self.coordinator.read_state()
        pose = state.root_pose
        velocity = state.root_linear_velocity
        return Odometry(
            timestamp=state.timestamp,
            x=pose.x,
            y=pose.y,
            z=pose.z,
            qx=pose.qx,
            qy=pose.qy,
            qz=pose.qz,
            qw=pose.qw,
            vx=velocity[0],
            vy=velocity[1],
            vz=velocity[2],
            vyaw=state.root_angular_velocity[2],
        )

    def get_lidar_scan(self) -> Any:
        return self.coordinator.get_lidar_scan()


__all__ = ["G1Base"]
