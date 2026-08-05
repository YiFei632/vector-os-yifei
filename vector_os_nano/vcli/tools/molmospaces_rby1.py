# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Bridge tool for driving a MolmoSpaces RBY1 runtime from vector-cli."""
from __future__ import annotations

import json
from typing import Any

from vector_os_nano.integrations.molmospaces import MolmoSpacesRBY1Bridge
from vector_os_nano.integrations.molmospaces.client import MolmoSpacesRBY1Error
from vector_os_nano.integrations.molmospaces.protocol import MolmoSpacesRBY1Endpoint
from vector_os_nano.vcli.tools.base import PermissionResult, ToolContext, ToolResult, tool


def _bridge_from_context(context: ToolContext) -> MolmoSpacesRBY1Bridge:
    assert context.app_state is not None
    bridge = context.app_state.get("molmospaces_rby1_bridge")
    if bridge is None:
        bridge = MolmoSpacesRBY1Bridge()
        context.app_state["molmospaces_rby1_bridge"] = bridge
    return bridge


@tool(
    name="molmospaces_rby1",
    description=(
        "Connect to an external MolmoSpaces RBY1 runtime and send interactive "
        "navigation / manipulation instructions to it. Use this for the "
        "vector-os-nano × MolmoSpaces integration path, not for benchmark eval."
    ),
    read_only=False,
    permission="ask",
)
class MolmoSpacesRBY1Tool:
    """Interactive bridge tool for the MolmoSpaces RBY1 runtime."""

    input_schema: dict[str, Any] = {
        "type": "object",
        "properties": {
            "action": {
                "type": "string",
                "enum": ["connect", "observe", "reset", "execute", "stop"],
                "default": "execute",
                "description": "Bridge action to perform.",
            },
            "host": {
                "type": "string",
                "default": "127.0.0.1",
                "description": "MolmoSpaces runtime host.",
            },
            "port": {
                "type": "integer",
                "default": 8765,
                "description": "MolmoSpaces runtime TCP port.",
            },
            "timeout_s": {
                "type": "number",
                "default": 300.0,
                "description": "Socket timeout in seconds.",
            },
            "scene_name": {
                "type": "string",
                "description": "Optional scene identifier for reset.",
            },
            "robot_base_pose": {
                "type": "array",
                "items": {"type": "number"},
                "minItems": 7,
                "maxItems": 7,
                "description": "Optional robot base pose [x, y, z, qw, qx, qy, qz].",
            },
            "seed": {
                "type": "integer",
                "description": "Optional deterministic seed for reset.",
            },
            "instruction": {
                "type": "string",
                "description": "Natural-language instruction to forward to the runtime.",
            },
            "context": {
                "type": "object",
                "description": "Additional context for the runtime command.",
            },
            "mode": {
                "type": "string",
                "enum": ["auto", "direct"],
                "default": "auto",
                "description": "Execution mode hint forwarded to the runtime.",
            },
        },
        "required": ["action"],
    }

    def check_permissions(self, params: dict[str, Any], context: ToolContext) -> PermissionResult:
        return PermissionResult(behavior="allow", reason="Runtime bridge action")

    def execute(self, params: dict[str, Any], context: ToolContext) -> ToolResult:
        action = str(params.get("action", "execute"))
        host = str(params.get("host", "127.0.0.1"))
        port = int(params.get("port", 8765))
        timeout_s = float(params.get("timeout_s", 300.0))

        bridge = _bridge_from_context(context)
        bridge.endpoint = MolmoSpacesRBY1Endpoint(host=host, port=port, timeout_s=timeout_s)

        try:
            if action == "connect":
                metadata = bridge.connect()
                return ToolResult(
                    content=f"Connected to MolmoSpaces RBY1 at {host}:{port}",
                    metadata={"bridge": metadata},
                )
            if action == "observe":
                state = bridge.observe()
                return ToolResult(content=json.dumps(state, ensure_ascii=False, indent=2), metadata=state)
            if action == "reset":
                state = bridge.reset(
                    scene_name=params.get("scene_name"),
                    robot_base_pose=params.get("robot_base_pose"),
                    seed=params.get("seed"),
                    metadata=dict(params.get("context", {}) or {}),
                )
                return ToolResult(content=json.dumps(state, ensure_ascii=False, indent=2), metadata=state)
            if action == "stop":
                result = bridge.stop()
                bridge.close()
                return ToolResult(content=json.dumps(result, ensure_ascii=False, indent=2), metadata=result)
            if action == "execute":
                instruction = str(params.get("instruction", "")).strip()
                if not instruction:
                    return ToolResult(content="Missing instruction for execute action", is_error=True)
                result = bridge.execute(
                    instruction,
                    context=dict(params.get("context", {}) or {}),
                    mode=str(params.get("mode", "auto")),
                    timeout_s=timeout_s,
                )
                return ToolResult(content=json.dumps(result, ensure_ascii=False, indent=2), metadata=result)
        except (MolmoSpacesRBY1Error, TimeoutError, OSError) as exc:
            bridge.close()
            return ToolResult(content=f"MolmoSpaces RBY1 bridge error: {exc}", is_error=True)
        return ToolResult(content=f"Unknown action: {action}", is_error=True)


__all__ = ["MolmoSpacesRBY1Tool"]
