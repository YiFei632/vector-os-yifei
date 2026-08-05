# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Composition root for one G1 body, two arms, and installed Dex3-1 hands."""
from __future__ import annotations

from vector_os_nano.hardware.g1.arm import G1Arm
from vector_os_nano.hardware.g1.base import G1Base
from vector_os_nano.hardware.g1.coordinator import G1ControlCoordinator
from vector_os_nano.hardware.g1.hand import G1GripperAdapter, G1Hand
from vector_os_nano.hardware.g1.ik import G1ArmKinematics
from vector_os_nano.hardware.g1.profile import G1Profile
from vector_os_nano.hardware.g1.state import G1State
from vector_os_nano.hardware.g1.transport import G1Transport


class G1Robot:
    """Create all named views over one shared coordinator/transport."""

    def __init__(
        self,
        profile: G1Profile,
        transport: G1Transport,
        *,
        kinematics: G1ArmKinematics | None = None,
    ) -> None:
        self.profile = profile
        self.transport = transport
        self.coordinator = G1ControlCoordinator(profile, transport)
        self.base = G1Base(self.coordinator)
        self.arms: dict[str, G1Arm] = {
            side: G1Arm(self.coordinator, side, kinematics=kinematics)
            for side in ("left", "right")
        }
        self.hands: dict[str, G1Hand] = {
            side: G1Hand(self.coordinator, side) for side in profile.hands
        }
        self.grippers: dict[str, G1GripperAdapter] = {
            side: G1GripperAdapter(hand) for side, hand in self.hands.items()
        }
        self._owner = "robot"

    @property
    def name(self) -> str:
        return "g1"

    @property
    def connected(self) -> bool:
        return self.coordinator.connected

    @property
    def left_arm(self) -> G1Arm:
        return self.arms["left"]

    @property
    def right_arm(self) -> G1Arm:
        return self.arms["right"]

    @property
    def left_hand(self) -> G1Hand:
        return self.hands["left"]

    @property
    def right_hand(self) -> G1Hand:
        return self.hands["right"]

    @property
    def default_arm(self) -> G1Arm:
        return self.arms[self.profile.default_arm]

    @property
    def default_hand(self) -> G1Hand:
        return self.hands[self.profile.default_hand]

    @property
    def default_gripper(self) -> G1GripperAdapter:
        return self.grippers[self.profile.default_hand]

    def connect(self) -> None:
        self.coordinator.connect(owner=self._owner)

    def disconnect(self) -> None:
        self.coordinator.disconnect(owner=self._owner)

    def stop(self) -> bool:
        return self.coordinator.stop(emergency=False)

    def emergency_stop(self) -> bool:
        return self.coordinator.emergency_stop()

    @property
    def emergency_latched(self) -> bool:
        return self.coordinator.emergency_latched

    def clear_emergency(self) -> None:
        """Explicitly clear the software emergency latch; never implicit."""
        self.coordinator.clear_emergency()

    def read_state(self) -> G1State:
        return self.coordinator.read_state()

    def __enter__(self) -> "G1Robot":
        self.connect()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.disconnect()


__all__ = ["G1Robot"]
