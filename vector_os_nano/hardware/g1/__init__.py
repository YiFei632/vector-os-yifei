# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Unitree G1 EDU hardware abstraction.

Backend implementations live outside this package.  They provide one
``G1Transport`` which is shared by all adapters exposed by ``G1Robot``.
"""

from vector_os_nano.hardware.g1.arm import G1Arm
from vector_os_nano.hardware.g1.base import G1Base
from vector_os_nano.hardware.g1.coordinator import G1ControlCoordinator
from vector_os_nano.hardware.g1.hand import G1GripperAdapter, G1Hand
from vector_os_nano.hardware.g1.ik import G1ArmKinematics
from vector_os_nano.hardware.g1.joint_map import G1JointMap
from vector_os_nano.hardware.g1.pinocchio_kinematics import PinocchioG1Kinematics
from vector_os_nano.hardware.g1.profile import G1Profile
from vector_os_nano.hardware.g1.robot import G1Robot
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

__all__ = [
    "G1Arm",
    "G1ArmKinematics",
    "G1Base",
    "G1CapabilityError",
    "G1ControlCoordinator",
    "G1ControlMode",
    "G1GripperAdapter",
    "G1Hand",
    "G1JointCommand",
    "G1JointMap",
    "G1Profile",
    "G1Robot",
    "G1State",
    "G1Transport",
    "G1TransportCapabilities",
    "G1VelocityCommand",
    "PinocchioG1Kinematics",
]
