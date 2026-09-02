#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Reference MolmoSpaces RBY1 bridge server.

This file is intentionally dependency-light. It implements the wire protocol
used by vector_os_nano.integrations.molmospaces and delegates the actual robot /
scene behavior to a pluggable adapter.

The adapter lives in the MolmoSpaces environment, where you have access to
``molmo_spaces`` and the resources cache.

Expected adapter contract
-------------------------
Your adapter object must provide:

    connect() -> dict
    observe() -> dict
    reset(scene_name=None, robot_base_pose=None, seed=None, metadata=None) -> dict
    execute(instruction, context=None, mode="auto", timeout_s=None) -> dict
    stop() -> dict

Suggested location in the MolmoSpaces repo:
    molmo_spaces/bridge/rby1_interactive_adapter.py

You can load it with:

    python scripts/molmospaces_rby1_bridge_server.py \
      --adapter molmo_spaces.bridge.rby1_interactive_adapter:create_adapter
"""
from __future__ import annotations

import argparse
import importlib
import json
import logging
import socketserver
from typing import Any, Protocol

logger = logging.getLogger(__name__)


class MolmoSpacesRBY1Adapter(Protocol):
    def connect(self) -> dict[str, Any]: ...

    def observe(self) -> dict[str, Any]: ...

    def reset(
        self,
        *,
        scene_name: str | None = None,
        robot_base_pose: list[float] | None = None,
        seed: int | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]: ...

    def execute(
        self,
        instruction: str,
        *,
        context: dict[str, Any] | None = None,
        mode: str = "auto",
        timeout_s: float | None = None,
    ) -> dict[str, Any]: ...

    def stop(self) -> dict[str, Any]: ...

class BridgeDispatcher:
    """Protocol router, independent of the transport layer."""

    def __init__(self, adapter: MolmoSpacesRBY1Adapter) -> None:
        self._adapter = adapter

    def handle(self, request: dict[str, Any]) -> dict[str, Any]:
        msg_type = str(request.get("type", "")).strip()
        if msg_type == "handshake":
            metadata = self._adapter.connect()
            return {
                "type": "handshake_ack",
                "ok": True,
                "server_name": "molmospaces-rby1",
                "metadata": metadata,
            }
        if msg_type == "observe":
            state = self._adapter.observe()
            return {"type": "observe_result", "ok": True, "state": state}
        if msg_type == "reset":
            state = self._adapter.reset(
                scene_name=request.get("scene_name"),
                robot_base_pose=request.get("robot_base_pose"),
                seed=request.get("seed"),
                metadata=dict(request.get("metadata", {}) or {}),
            )
            return {"type": "command_result", "ok": True, "state": state}
        if msg_type == "execute":
            result = self._adapter.execute(
                str(request.get("instruction", "")),
                context=dict(request.get("context", {}) or {}),
                mode=str(request.get("mode", "auto")),
                timeout_s=request.get("timeout_s"),
            )
            return {"type": "command_result", "ok": True, "result": result}
        if msg_type == "stop":
            result = self._adapter.stop()
            return {"type": "command_result", "ok": True, "result": result}
        return {
            "type": "error",
            "ok": False,
            "error": f"unsupported request type: {msg_type}",
        }


def load_adapter(spec: str, *, args: argparse.Namespace | None = None) -> MolmoSpacesRBY1Adapter:
    """Load an adapter from ``module:factory``.

    The factory may accept zero arguments or one ``argparse.Namespace``.
    """
    if ":" not in spec:
        raise ValueError("Adapter spec must be module:factory")
    module_name, factory_name = spec.split(":", 1)
    module = importlib.import_module(module_name)
    factory = getattr(module, factory_name)
    try:
        return factory(args)  # type: ignore[misc]
    except TypeError:
        return factory()  # type: ignore[misc]


class _BridgeTCPHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        server: "_BridgeTCPServer" = self.server  # type: ignore[assignment]
        while True:
            line = self.rfile.readline()
            if not line:
                return
            try:
                request = json.loads(line.decode("utf-8"))
            except json.JSONDecodeError as exc:
                payload = {"type": "error", "ok": False, "error": f"invalid JSON: {exc}"}
            else:
                if not isinstance(request, dict):
                    payload = {"type": "error", "ok": False, "error": "request must be a JSON object"}
                else:
                    try:
                        payload = server.dispatcher.handle(request)
                    except Exception as exc:  # noqa: BLE001
                        logger.exception("Bridge dispatch failed")
                        payload = {"type": "error", "ok": False, "error": str(exc)}
            self.wfile.write((json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8"))
            self.wfile.flush()


class _BridgeTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], dispatcher: BridgeDispatcher) -> None:
        super().__init__(server_address, _BridgeTCPHandler)
        self.dispatcher = dispatcher


def serve_forever(adapter: MolmoSpacesRBY1Adapter, host: str, port: int) -> None:
    dispatcher = BridgeDispatcher(adapter)
    with _BridgeTCPServer((host, port), dispatcher) as server:
        logger.info("MolmoSpaces RBY1 bridge listening on %s:%s", host, port)
        server.serve_forever()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MolmoSpaces RBY1 bridge server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--adapter",
        required=True,
        help="Adapter factory spec, module:factory. The factory may optionally accept argparse.Namespace.",
    )
    parser.add_argument("--log-level", default="INFO")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))
    adapter = load_adapter(args.adapter, args=args)
    serve_forever(adapter, args.host, args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
