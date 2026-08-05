# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class G1GraspObservation:
    rgb: Any | None = None
    depth: Any | None = None
    detections: list[dict[str, Any]] | None = None


class G1GraspPerception:
    def __init__(self, base: Any, *, width: int = 320, height: int = 240) -> None:
        self.base = base
        self.width = width
        self.height = height

    @property
    def is_available(self) -> bool:
        return True

    def capture(self) -> G1GraspObservation:
        """
        第一版可以先接现有相机/深度接口。
        如果还没有真实感知源，就先返回空检测，但不要伪造 ground truth。
        """
        return G1GraspObservation(rgb=None, depth=None, detections=[])

    def detect(self, query: str) -> list[dict[str, Any]]:
        """
        返回检测列表；第一版可以先把现有 detector 接进来。
        这里不要直接读 world model 的 ground truth。
        """
        obs = self.capture()
        return obs.detections or []

    def grasp_point_from_rgbd(self, *args: Any, **kwargs: Any) -> tuple[float, float, float] | None:
        """
        从 RGB-D 得到 3D 抓取点。第一版可以先复用现有通用逻辑，
        但要把相机内参、外参、坐标系都改成 G1 配置。
        """
        return None