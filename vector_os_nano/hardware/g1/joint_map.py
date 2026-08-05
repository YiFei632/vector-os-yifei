# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Name-based joint-vector conversion utilities for G1."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence

from vector_os_nano.hardware.g1.profile import (
    BODY_DDS_JOINTS,
    LEFT_HAND_ISAACLAB_JOINTS,
    LEFT_HAND_REV_1_0_XML_JOINTS,
    LEFT_HAND_SEMANTIC_JOINTS,
    REV_1_0_XML_ACTUATOR_JOINTS,
    RIGHT_HAND_ISAACLAB_JOINTS,
    RIGHT_HAND_REV_1_0_XML_JOINTS,
    RIGHT_HAND_SEMANTIC_JOINTS,
)


def _unique(names: Sequence[str], *, label: str) -> tuple[str, ...]:
    result = tuple(str(name) for name in names)
    duplicates = sorted({name for name in result if result.count(name) > 1})
    if duplicates:
        raise ValueError(f"duplicate {label} joint names: {duplicates}")
    return result


def _finite_values(values: Sequence[float], *, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains a non-finite joint value")
    return result


@dataclass(frozen=True)
class G1JointMap:
    """Validated conversion between named joint-vector layouts."""

    xml_actuator_names: tuple[str, ...] = REV_1_0_XML_ACTUATOR_JOINTS
    body_dds_names: tuple[str, ...] = BODY_DDS_JOINTS
    left_hand_semantic_names: tuple[str, ...] = LEFT_HAND_SEMANTIC_JOINTS
    right_hand_semantic_names: tuple[str, ...] = RIGHT_HAND_SEMANTIC_JOINTS
    left_hand_xml_names: tuple[str, ...] = LEFT_HAND_REV_1_0_XML_JOINTS
    right_hand_xml_names: tuple[str, ...] = RIGHT_HAND_REV_1_0_XML_JOINTS
    left_hand_isaaclab_names: tuple[str, ...] = LEFT_HAND_ISAACLAB_JOINTS
    right_hand_isaaclab_names: tuple[str, ...] = RIGHT_HAND_ISAACLAB_JOINTS

    def __post_init__(self) -> None:
        for field_name in (
            "xml_actuator_names",
            "body_dds_names",
            "left_hand_semantic_names",
            "right_hand_semantic_names",
            "left_hand_xml_names",
            "right_hand_xml_names",
            "left_hand_isaaclab_names",
            "right_hand_isaaclab_names",
        ):
            object.__setattr__(
                self,
                field_name,
                _unique(getattr(self, field_name), label=field_name),
            )
        expected = set(
            self.body_dds_names
            + self.left_hand_semantic_names
            + self.right_hand_semantic_names
        )
        _unique(
            self.body_dds_names
            + self.left_hand_semantic_names
            + self.right_hand_semantic_names,
            label="semantic",
        )
        if set(self.left_hand_xml_names) != set(self.left_hand_semantic_names):
            raise ValueError("left Dex3 XML and semantic contracts differ by name")
        if set(self.right_hand_xml_names) != set(self.right_hand_semantic_names):
            raise ValueError("right Dex3 XML and semantic contracts differ by name")
        actual = set(self.xml_actuator_names)
        if actual != expected:
            missing = sorted(expected - actual)
            extra = sorted(actual - expected)
            raise ValueError(f"G1 XML actuator contract mismatch: missing={missing}, extra={extra}")
        left_in_xml = tuple(
            name for name in self.xml_actuator_names if name in self.left_hand_xml_names
        )
        right_in_xml = tuple(
            name for name in self.xml_actuator_names if name in self.right_hand_xml_names
        )
        if left_in_xml != self.left_hand_xml_names:
            raise ValueError("left Dex3 XML actuator order does not match the rev-1.0 contract")
        if right_in_xml != self.right_hand_xml_names:
            raise ValueError("right Dex3 XML actuator order does not match the rev-1.0 contract")

    @staticmethod
    def indices(source_names: Sequence[str], target_names: Sequence[str]) -> tuple[int, ...]:
        source = _unique(source_names, label="source")
        target = _unique(target_names, label="target")
        lookup = {name: index for index, name in enumerate(source)}
        missing = [name for name in target if name not in lookup]
        if missing:
            raise ValueError(f"source joint vector is missing: {missing}")
        return tuple(lookup[name] for name in target)

    @classmethod
    def reorder(
        cls,
        values: Sequence[float],
        source_names: Sequence[str],
        target_names: Sequence[str],
    ) -> tuple[float, ...]:
        if len(values) != len(source_names):
            raise ValueError(
                f"joint vector length {len(values)} does not match names length {len(source_names)}"
            )
        source_values = _finite_values(values, label="source vector")
        return tuple(source_values[index] for index in cls.indices(source_names, target_names))

    @classmethod
    def merge(
        cls,
        source_names: Sequence[str],
        current_values: Sequence[float],
        updates: Mapping[str, float] | Iterable[tuple[str, float]],
    ) -> tuple[float, ...]:
        names = _unique(source_names, label="source")
        if len(current_values) != len(names):
            raise ValueError("current joint vector length does not match source names")
        lookup = {name: index for index, name in enumerate(names)}
        merged = list(_finite_values(current_values, label="current vector"))
        items = updates.items() if isinstance(updates, Mapping) else updates
        for name, value in items:
            if name not in lookup:
                raise ValueError(f"cannot update unknown joint {name!r}")
            numeric = float(value)
            if not math.isfinite(numeric):
                raise ValueError(f"cannot update joint {name!r} with a non-finite value")
            merged[lookup[name]] = numeric
        return tuple(merged)

    @property
    def body_xml_indices(self) -> tuple[int, ...]:
        return self.indices(self.xml_actuator_names, self.body_dds_names)

    @property
    def left_hand_xml_indices(self) -> tuple[int, ...]:
        return self.indices(self.xml_actuator_names, self.left_hand_semantic_names)

    @property
    def right_hand_xml_indices(self) -> tuple[int, ...]:
        return self.indices(self.xml_actuator_names, self.right_hand_semantic_names)

    def xml_to_body_dds(self, values: Sequence[float]) -> tuple[float, ...]:
        return self.reorder(values, self.xml_actuator_names, self.body_dds_names)

    def xml_to_hand_semantic(
        self, side: str, values: Sequence[float]
    ) -> tuple[float, ...]:
        if side == "left":
            names = self.left_hand_semantic_names
        elif side == "right":
            names = self.right_hand_semantic_names
        else:
            raise ValueError("side must be 'left' or 'right'")
        return self.reorder(values, self.xml_actuator_names, names)

    def split_xml(
        self,
        values: Sequence[float],
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        """Return body DDS plus both hands in the HAL semantic order."""
        return (
            self.xml_to_body_dds(values),
            self.xml_to_hand_semantic("left", values),
            self.xml_to_hand_semantic("right", values),
        )

    def semantic_to_xml(
        self,
        body_values: Sequence[float],
        *,
        left_hand_values: Sequence[float],
        right_hand_values: Sequence[float],
    ) -> tuple[float, ...]:
        """Merge the public HAL body/hand vectors into vendor XML order."""
        if len(body_values) != len(self.body_dds_names):
            raise ValueError("body semantic vector must contain 29 values")
        if len(left_hand_values) != 7 or len(right_hand_values) != 7:
            raise ValueError("each Dex3 semantic vector must contain 7 values")
        named = dict(
            zip(
                self.body_dds_names,
                _finite_values(body_values, label="body semantic vector"),
            )
        )
        named.update(
            zip(
                self.left_hand_semantic_names,
                _finite_values(left_hand_values, label="left-hand semantic vector"),
            )
        )
        named.update(
            zip(
                self.right_hand_semantic_names,
                _finite_values(right_hand_values, label="right-hand semantic vector"),
            )
        )
        return tuple(named[name] for name in self.xml_actuator_names)

    def hand_semantic_to_isaaclab(
        self, side: str, values: Sequence[float]
    ) -> tuple[float, ...]:
        if side == "left":
            return self.reorder(
                values,
                self.left_hand_semantic_names,
                self.left_hand_isaaclab_names,
            )
        if side == "right":
            return self.reorder(
                values,
                self.right_hand_semantic_names,
                self.right_hand_isaaclab_names,
            )
        raise ValueError("side must be 'left' or 'right'")

    def hand_isaaclab_to_semantic(
        self, side: str, values: Sequence[float]
    ) -> tuple[float, ...]:
        if side == "left":
            return self.reorder(
                values,
                self.left_hand_isaaclab_names,
                self.left_hand_semantic_names,
            )
        if side == "right":
            return self.reorder(
                values,
                self.right_hand_isaaclab_names,
                self.right_hand_semantic_names,
            )
        raise ValueError("side must be 'left' or 'right'")


__all__ = ["G1JointMap"]
