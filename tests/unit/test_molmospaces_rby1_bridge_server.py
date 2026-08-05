# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Unit tests for the MolmoSpaces RBY1 bridge server reference implementation."""
from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_script_module():
    path = Path(__file__).resolve().parents[2] / "scripts" / "molmospaces_rby1_bridge_server.py"
    spec = importlib.util.spec_from_file_location("molmospaces_rby1_bridge_server", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeAdapter:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict]] = []

    def connect(self):
        self.calls.append(("connect", {}))
        return {"scene_name": "scene_x"}

    def observe(self):
        self.calls.append(("observe", {}))
        return {"pose": [1, 2, 3]}

    def reset(self, *, scene_name=None, robot_base_pose=None, seed=None, metadata=None):
        payload = {
            "scene_name": scene_name,
            "robot_base_pose": robot_base_pose,
            "seed": seed,
            "metadata": metadata,
        }
        self.calls.append(("reset", payload))
        return payload

    def execute(self, instruction, *, context=None, mode="auto", timeout_s=None):
        payload = {
            "instruction": instruction,
            "context": context,
            "mode": mode,
            "timeout_s": timeout_s,
        }
        self.calls.append(("execute", payload))
        return payload

    def stop(self):
        self.calls.append(("stop", {}))
        return {"stopped": True}


def test_dispatcher_routes_handshake_observe_reset_execute_stop() -> None:
    mod = _load_script_module()
    adapter = _FakeAdapter()
    dispatcher = mod.BridgeDispatcher(adapter)

    assert dispatcher.handle({"type": "handshake"})["type"] == "handshake_ack"
    assert dispatcher.handle({"type": "observe"})["state"] == {"pose": [1, 2, 3]}
    assert dispatcher.handle({"type": "reset", "scene_name": "kitchen"})["state"]["scene_name"] == "kitchen"
    assert dispatcher.handle({"type": "execute", "instruction": "open fridge"})["result"]["instruction"] == "open fridge"
    assert dispatcher.handle({"type": "stop"})["result"] == {"stopped": True}

    assert [name for name, _payload in adapter.calls] == [
        "connect",
        "observe",
        "reset",
        "execute",
        "stop",
    ]


def test_load_adapter_imports_factory_from_module(tmp_path: Path, monkeypatch) -> None:
    mod = _load_script_module()
    module_dir = tmp_path / "bridge_impl"
    module_dir.mkdir()
    module_file = module_dir / "my_adapter.py"
    module_file.write_text(
        "class Adapter:\n"
        "    def __init__(self, args=None):\n"
        "        self.args = args\n"
        "def create_adapter(args=None):\n"
        "    return Adapter(args)\n"
    )
    monkeypatch.syspath_prepend(str(module_dir))

    adapter = mod.load_adapter("my_adapter:create_adapter")
    assert adapter.__class__.__name__ == "Adapter"
