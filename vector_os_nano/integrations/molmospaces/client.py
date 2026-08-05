# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""TCP JSON-lines client for a MolmoSpaces RBY1 runtime."""
from __future__ import annotations

import json
import logging
import socket
import threading
from typing import Any

from .protocol import MolmoSpacesRBY1Endpoint, MolmoSpacesRBY1Error, MolmoSpacesRBY1Request

logger = logging.getLogger(__name__)


class MolmoSpacesRBY1Client:
    """Blocking client for a MolmoSpaces RBY1 bridge server.

    The client is deliberately small:
    - a single TCP socket
    - newline-delimited JSON request/response
    - one in-flight request at a time
    """

    def __init__(
        self,
        endpoint: MolmoSpacesRBY1Endpoint | None = None,
        *,
        client_name: str = "vector-os-nano",
    ) -> None:
        self._endpoint = endpoint or MolmoSpacesRBY1Endpoint()
        self._client_name = str(client_name)
        self._sock: socket.socket | None = None
        self._reader: Any = None
        self._writer: Any = None
        self._lock = threading.RLock()
        self._metadata: dict[str, Any] = {}

    @property
    def endpoint(self) -> MolmoSpacesRBY1Endpoint:
        return self._endpoint

    @property
    def connected(self) -> bool:
        return self._sock is not None

    @property
    def metadata(self) -> dict[str, Any]:
        return dict(self._metadata)

    def connect(self) -> dict[str, Any]:
        if self.connected:
            return self.metadata
        sock = socket.create_connection(
            (self._endpoint.host, int(self._endpoint.port)),
            timeout=float(self._endpoint.timeout_s),
        )
        sock.settimeout(float(self._endpoint.timeout_s))
        self._sock = sock
        self._reader = sock.makefile("r", encoding="utf-8", newline="\n")
        self._writer = sock.makefile("w", encoding="utf-8", newline="\n")
        try:
            response = self.request(
                "handshake",
                client_name=self._client_name,
                protocol=1,
            )
        except Exception:
            self.close()
            raise
        self._metadata = dict(response.get("metadata", {}))
        self._metadata.setdefault("server_name", response.get("server_name", "molmospaces-rby1"))
        return self.metadata

    def close(self) -> None:
        with self._lock:
            reader = self._reader
            writer = self._writer
            sock = self._sock
            self._reader = None
            self._writer = None
            self._sock = None
            self._metadata = {}
        for stream in (reader, writer):
            try:
                if stream is not None:
                    stream.close()
            except Exception:
                pass
        try:
            if sock is not None:
                sock.close()
        except Exception:
            pass

    def request(self, message_type: str, **payload: Any) -> dict[str, Any]:
        if not self.connected:
            raise MolmoSpacesRBY1Error("MolmoSpaces bridge is not connected")
        request = MolmoSpacesRBY1Request(message_type, payload).to_payload()
        raw = json.dumps(request, ensure_ascii=False, separators=(",", ":"))
        with self._lock:
            assert self._writer is not None and self._reader is not None
            self._writer.write(raw)
            self._writer.write("\n")
            self._writer.flush()
            line = self._reader.readline()
        if not line:
            raise MolmoSpacesRBY1Error("MolmoSpaces bridge closed the connection")
        try:
            response = json.loads(line)
        except json.JSONDecodeError as exc:
            raise MolmoSpacesRBY1Error(f"invalid JSON response: {line!r}") from exc
        if not isinstance(response, dict):
            raise MolmoSpacesRBY1Error("bridge response must be a JSON object")
        if not response.get("ok", True):
            raise MolmoSpacesRBY1Error(str(response.get("error", "bridge request failed")))
        return response

    def observe(self) -> dict[str, Any]:
        response = self.request("observe")
        return dict(response.get("state", {}))

    def reset(
        self,
        *,
        scene_name: str | None = None,
        robot_base_pose: list[float] | None = None,
        seed: int | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {}
        if scene_name is not None:
            payload["scene_name"] = str(scene_name)
        if robot_base_pose is not None:
            payload["robot_base_pose"] = list(robot_base_pose)
        if seed is not None:
            payload["seed"] = int(seed)
        if metadata is not None:
            payload["metadata"] = dict(metadata)
        response = self.request("reset", **payload)
        return dict(response.get("state", {}))

    def execute(
        self,
        instruction: str,
        *,
        context: dict[str, Any] | None = None,
        mode: str = "auto",
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "instruction": str(instruction),
            "mode": str(mode),
        }
        if context:
            payload["context"] = dict(context)
        if timeout_s is not None:
            payload["timeout_s"] = float(timeout_s)
        response = self.request("execute", **payload)
        return dict(response.get("result", response))

    def stop(self) -> dict[str, Any]:
        response = self.request("stop")
        return dict(response.get("result", response))


__all__ = ["MolmoSpacesRBY1Client", "MolmoSpacesRBY1Error"]
