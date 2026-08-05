# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Wire protocol helpers for the MolmoSpaces RBY1 bridge.

The protocol is intentionally plain JSON over a single TCP connection so the
bridge can be implemented in a separate process or even a different repo
without introducing a hard dependency on websockets, ROS2, or gRPC.

Request/response shape
----------------------
Each message is one UTF-8 JSON object per line.

Requests include:
    - {"type": "handshake", ...}
    - {"type": "observe"}
    - {"type": "reset", ...}
    - {"type": "execute", "instruction": "...", ...}
    - {"type": "stop"}

Responses include:
    - {"type": "handshake_ack", "ok": true, "metadata": {...}}
    - {"type": "observe_result", "ok": true, "state": {...}}
    - {"type": "command_result", "ok": true, "result": {...}}
    - {"type": "error", "ok": false, "error": "..."}
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


class MolmoSpacesRBY1Error(RuntimeError):
    """Raised when the MolmoSpaces bridge reports an error."""


@dataclass(frozen=True)
class MolmoSpacesRBY1Endpoint:
    """Connection parameters for a MolmoSpaces RBY1 bridge server."""

    host: str = "127.0.0.1"
    port: int = 8765
    timeout_s: float = 300.0


@dataclass(frozen=True)
class MolmoSpacesRBY1Request:
    """Convenience wrapper for bridge requests."""

    type: str
    payload: dict[str, Any] = field(default_factory=dict)

    def to_payload(self) -> dict[str, Any]:
        data = {"type": self.type}
        data.update(self.payload)
        return data
