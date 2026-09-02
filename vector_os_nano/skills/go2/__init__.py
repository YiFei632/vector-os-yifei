# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Go2 quadruped skill package.

Exports all Go2 skills and the get_go2_skills() factory.
"""
from vector_os_nano.skills.go2.walk import WalkSkill
from vector_os_nano.skills.go2.turn import TurnSkill
from vector_os_nano.skills.go2.stance import StandSkill, SitSkill, LieDownSkill
from vector_os_nano.skills.go2.explore import ExploreSkill
from vector_os_nano.skills.go2.where_am_i import WhereAmISkill
from vector_os_nano.skills.go2.stop import StopSkill
from vector_os_nano.skills.go2.look import LookSkill, DescribeSceneSkill
from vector_os_nano.skills.go2.patrol import PatrolSkill
from vector_os_nano.skills.navigate import NavigateSkill
from vector_os_nano.skills.navigation_vln import OneRINGNavigationSkill


def get_go2_skills() -> list:
    """Return one instance of each Go2 skill."""
    return [
        WalkSkill(), TurnSkill(),
        StandSkill(), SitSkill(), LieDownSkill(),
        NavigateSkill(),
        ExploreSkill(),
        WhereAmISkill(),
        StopSkill(),
        LookSkill(),
        DescribeSceneSkill(),
        PatrolSkill(),
        # Registered last so natural-language navigation aliases select the
        # generic RGB-D VLN skill; callers can still invoke ``navigate`` by name
        # for a known SceneGraph room.
        OneRINGNavigationSkill(),
    ]


__all__ = [
    "WalkSkill",
    "TurnSkill",
    "StandSkill",
    "SitSkill",
    "LieDownSkill",
    "NavigateSkill",
    "ExploreSkill",
    "WhereAmISkill",
    "StopSkill",
    "LookSkill",
    "DescribeSceneSkill",
    "PatrolSkill",
    "OneRINGNavigationSkill",
    "get_go2_skills",
]
