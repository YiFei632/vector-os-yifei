# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Contract tests for the MolmoSpaces RBY1 bridge layer."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from vector_os_nano.integrations.molmospaces import (
    MolmoSpacesRBY1Bridge,
    MolmoSpacesRBY1Client,
)
from vector_os_nano.integrations.molmospaces.protocol import MolmoSpacesRBY1Endpoint
from vector_os_nano.vcli.tools.base import ToolContext
from vector_os_nano.vcli.tools.molmospaces_rby1 import MolmoSpacesRBY1Tool


class _DummySocket:
    """In-memory socket replacement for request/response validation."""

    def __init__(self) -> None:
        self.requests: list[dict[str, Any]] = []
        self.responses: list[str] = []
        self.closed = False
        self.timeout_s: float | None = None
        self._write_buffer = ""

    def settimeout(self, timeout: float) -> None:
        self.timeout_s = float(timeout)

    def makefile(self, mode: str, encoding: str = "utf-8", newline: str = "\n"):
        if "r" in mode:
            return _DummyReader(self)
        if "w" in mode:
            return _DummyWriter(self)
        raise ValueError(f"unsupported mode: {mode}")

    def close(self) -> None:
        self.closed = True


class _DummyReader:
    def __init__(self, sock: _DummySocket) -> None:
        self._sock = sock

    def readline(self) -> str:
        if not self._sock.responses:
            return ""
        return self._sock.responses.pop(0)

    def close(self) -> None:
        return None


class _DummyWriter:
    def __init__(self, sock: _DummySocket) -> None:
        self._sock = sock

    def write(self, data: str) -> int:
        self._sock._write_buffer += data
        return len(data)

    def flush(self) -> None:
        while "\n" in self._sock._write_buffer:
            line, remainder = self._sock._write_buffer.split("\n", 1)
            self._sock._write_buffer = remainder
            if not line:
                continue
            request = json.loads(line)
            self._sock.requests.append(request)
            self._sock.responses.append(json.dumps(_response_for(request)) + "\n")

    def close(self) -> None:
        return None


def _response_for(request: dict[str, Any]) -> dict[str, Any]:
    msg_type = request.get("type")
    if msg_type == "handshake":
        return {
            "type": "handshake_ack",
            "ok": True,
            "server_name": "molmospaces-rby1",
            "metadata": {"scene_name": "test_room", "robot": "rby1"},
        }
    if msg_type == "observe":
        return {
            "type": "observe_result",
            "ok": True,
            "state": {"pose": [1.0, 2.0, 3.0], "objects": ["mug"]},
        }
    if msg_type == "reset":
        return {
            "type": "command_result",
            "ok": True,
            "state": {
                "scene_name": request.get("scene_name"),
                "robot_base_pose": request.get("robot_base_pose"),
                "seed": request.get("seed"),
                "metadata": request.get("metadata", {}),
            },
        }
    if msg_type == "execute":
        return {
            "type": "command_result",
            "ok": True,
            "result": {
                "accepted": True,
                "instruction": request.get("instruction"),
                "mode": request.get("mode", "auto"),
                "context": request.get("context", {}),
            },
        }
    if msg_type == "stop":
        return {
            "type": "command_result",
            "ok": True,
            "result": {"stopped": True},
        }
    return {
        "type": "error",
        "ok": False,
        "error": f"unsupported request type: {msg_type}",
    }


def _patch_connection(monkeypatch, sock: _DummySocket) -> None:
    import vector_os_nano.integrations.molmospaces.client as client_mod

    monkeypatch.setattr(client_mod.socket, "create_connection", lambda *args, **kwargs: sock)


def test_client_roundtrip_observe_reset_execute_stop(monkeypatch) -> None:
    sock = _DummySocket()
    _patch_connection(monkeypatch, sock)

    endpoint = MolmoSpacesRBY1Endpoint(host="127.0.0.1", port=8765, timeout_s=2.0)
    client = MolmoSpacesRBY1Client(endpoint)

    metadata = client.connect()
    assert metadata["scene_name"] == "test_room"
    assert metadata["robot"] == "rby1"

    state = client.observe()
    assert state["pose"] == [1.0, 2.0, 3.0]
    assert state["objects"] == ["mug"]

    reset_state = client.reset(
        scene_name="scene_a",
        robot_base_pose=[0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
        seed=7,
        metadata={"foo": "bar"},
    )
    assert reset_state["scene_name"] == "scene_a"
    assert reset_state["seed"] == 7
    assert reset_state["metadata"] == {"foo": "bar"}

    result = client.execute(
        "open the fridge",
        context={"target": "fridge"},
        mode="direct",
    )
    assert result["accepted"] is True
    assert result["instruction"] == "open the fridge"
    assert result["mode"] == "direct"
    assert result["context"] == {"target": "fridge"}

    stop_result = client.stop()
    assert stop_result["stopped"] is True

    assert [req["type"] for req in sock.requests] == [
        "handshake",
        "observe",
        "reset",
        "execute",
        "stop",
    ]


def test_bridge_and_tool_share_persistent_client(monkeypatch) -> None:
    sock = _DummySocket()
    _patch_connection(monkeypatch, sock)

    endpoint = MolmoSpacesRBY1Endpoint(host="127.0.0.1", port=8765, timeout_s=2.0)
    bridge_wrapper = MolmoSpacesRBY1Bridge(endpoint=endpoint)
    tool = MolmoSpacesRBY1Tool()
    context = ToolContext(
        agent=None,
        cwd=Path("."),
        session=None,
        permissions=None,
        abort=__import__("threading").Event(),
        app_state={"molmospaces_rby1_bridge": bridge_wrapper},
    )

    connect_result = tool.execute({"action": "connect"}, context)
    assert connect_result.is_error is False
    assert "Connected to MolmoSpaces RBY1" in connect_result.content

    execute_result = tool.execute(
        {
            "action": "execute",
            "instruction": "move to the table",
            "context": {"task": "navigation"},
            "mode": "auto",
        },
        context,
    )
    assert execute_result.is_error is False
    assert "move to the table" in execute_result.content

    observe_result = tool.execute({"action": "observe"}, context)
    assert observe_result.is_error is False
    assert '"pose": [' in observe_result.content

    stop_result = tool.execute({"action": "stop"}, context)
    assert stop_result.is_error is False
    assert '"stopped": true' in stop_result.content.lower()


def test_tool_returns_error_on_bridge_timeout(monkeypatch) -> None:
    class _TimeoutBridge:
        def connect(self):
            raise TimeoutError("timed out")

        def close(self):
            self.closed = True

    bridge = _TimeoutBridge()
    tool = MolmoSpacesRBY1Tool()
    context = ToolContext(
        agent=None,
        cwd=Path("."),
        session=None,
        permissions=None,
        abort=__import__("threading").Event(),
        app_state={"molmospaces_rby1_bridge": bridge},
    )

    result = tool.execute({"action": "connect"}, context)

    assert result.is_error is True
    assert "timed out" in result.content
    assert bridge.closed is True
