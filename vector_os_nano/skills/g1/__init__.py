# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Skills for Unitree G1 locomotion, navigation, and Dex3-1 hands."""
from vector_os_nano.skills.g1.dex3 import Dex3PoseSkill
from vector_os_nano.skills.g1.stance import G1StandSkill, StandSkill
from vector_os_nano.skills.g1.stop import (
    EmergencyStopSkill,
    G1EmergencyStopSkill,
    G1StopSkill,
    StopSkill,
)
from vector_os_nano.skills.g1.turn import G1TurnSkill, TurnSkill
from vector_os_nano.skills.g1.walk import G1WalkSkill, WalkSkill
from vector_os_nano.skills.go2.where_am_i import WhereAmISkill
from vector_os_nano.skills.navigate import NavigateSkill


def get_g1_skills() -> list:
    """Return one instance of each skill supported by the G1 package."""
    return [
        WalkSkill(),
        TurnSkill(),
        StandSkill(),
        NavigateSkill(),
        WhereAmISkill(),
        StopSkill(),
        EmergencyStopSkill(),
        Dex3PoseSkill(),
    ]


__all__ = [
    "Dex3PoseSkill",
    "EmergencyStopSkill",
    "G1EmergencyStopSkill",
    "G1StandSkill",
    "G1StopSkill",
    "G1TurnSkill",
    "G1WalkSkill",
    "NavigateSkill",
    "StandSkill",
    "StopSkill",
    "TurnSkill",
    "WalkSkill",
    "WhereAmISkill",
    "get_g1_skills",
]
