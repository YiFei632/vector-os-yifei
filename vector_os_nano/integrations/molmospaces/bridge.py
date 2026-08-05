# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""High-level convenience wrapper around the MolmoSpaces RBY1 client."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .client import MolmoSpacesRBY1Client
from .protocol import MolmoSpacesRBY1Endpoint


@dataclass
class MolmoSpacesRBY1Bridge:
    """Persistent bridge state kept in ``app_state`` or a tool instance."""

    endpoint: MolmoSpacesRBY1Endpoint = field(default_factory=MolmoSpacesRBY1Endpoint)
    client_name: str = "vector-os-nano"
    scene_name: str | None = None
    client: MolmoSpacesRBY1Client | None = None

    def connect(self) -> dict[str, Any]:
        if self.client is None:
            self.client = MolmoSpacesRBY1Client(
                self.endpoint,
                client_name=self.client_name,
            )
        return self.client.connect()

    @property
    def connected(self) -> bool:
        return bool(self.client and self.client.connected)

    def close(self) -> None:
        if self.client is not None:
            self.client.close()
        self.client = None

    def observe(self) -> dict[str, Any]:
        self._ensure_connected()
        assert self.client is not None
        return self.client.observe()

    def reset(
        self,
        *,
        scene_name: str | None = None,
        robot_base_pose: list[float] | None = None,
        seed: int | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        self._ensure_connected()
        assert self.client is not None
        if scene_name is not None:
            self.scene_name = scene_name
        return self.client.reset(
            scene_name=scene_name or self.scene_name,
            robot_base_pose=robot_base_pose,
            seed=seed,
            metadata=metadata,
        )

    def execute(
        self,
        instruction: str,
        *,
        context: dict[str, Any] | None = None,
        mode: str = "auto",
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        self._ensure_connected()
        assert self.client is not None
        return self.client.execute(
            instruction,
            context=context,
            mode=mode,
            timeout_s=timeout_s,
        )

    def stop(self) -> dict[str, Any]:
        self._ensure_connected()
        assert self.client is not None
        return self.client.stop()

    def _ensure_connected(self) -> None:
        if not self.connected:
            self.connect()


__all__ = ["MolmoSpacesRBY1Bridge"]
