# MolmoSpaces RBY1 bridge server

This document describes the remaining MolmoSpaces-side piece for the
vector-os-nano ↔ MolmoSpaces RBY1 integration.

The vector-os-nano side now speaks a small newline-delimited JSON protocol over
TCP. The MolmoSpaces side only needs to expose an adapter with five methods:
`connect`, `observe`, `reset`, `execute`, and `stop`.

## Where the files go

In the `mlspaces` environment, add:

- `molmo_spaces/bridge/rby1_interactive_adapter.py`
- `scripts/molmospaces_rby1_bridge_server.py`

You can copy `scripts/molmospaces_rby1_bridge_server.py` from this repository.
It is intentionally dependency-light and only requires a concrete adapter.

## Resource setup

Use the assets cache path you already have:

```bash
conda activate mlspaces
export MLSPACES_CACHE_DIR=~/.cache/molmo-spaces-resources
export MLSPACES_ASSETS_DIR=/media/fishyu/fish-14tb-11/YiFei/molmospaces/molmo-spaces-resources
```

If the cache needs to be materialized or refreshed:

```bash
python -m molmo_spaces.molmo_spaces_constants
```

That should ensure the `rby1` robot assets and scene resources are reachable
through the resource manager.

## Adapter contract

Implement an adapter that provides:

```python
def connect() -> dict: ...
def observe() -> dict: ...
def reset(
    *,
    scene_name: str | None = None,
    robot_base_pose: list[float] | None = None,
    seed: int | None = None,
    metadata: dict | None = None,
) -> dict: ...
def execute(
    instruction: str,
    *,
    context: dict | None = None,
    mode: str = "auto",
    timeout_s: float | None = None,
) -> dict: ...
def stop() -> dict: ...
```

The bridge server returns these payloads as-is inside the protocol envelope.

## Recommended implementation shape

Keep the adapter small:

1. load the scene / task / env for the current `scene_name`,
2. expose one stable observation snapshot for `observe`,
3. route `execute` to the relevant planner or controller,
4. translate `reset` into environment reset + scene selection,
5. stop motion in `stop`.

For your first phase, you do not need benchmark evaluation. You only need a
runtime that accepts terminal commands and routes them to the active simulation.

If you later want benchmark support, you can layer the benchmark task sampler on
top of the same adapter without changing the vector-os-nano protocol.

## Bridge server launch

After creating the adapter, start the bridge in `mlspaces`:

```bash
conda activate mlspaces
export MLSPACES_CACHE_DIR=~/.cache/molmo-spaces-resources
export MLSPACES_ASSETS_DIR=/media/fishyu/fish-14tb-11/YiFei/molmospaces/molmo-spaces-resources

python scripts/molmospaces_rby1_bridge_server.py \
  --adapter molmo_spaces.bridge.rby1_interactive_adapter:create_adapter \
  --host 127.0.0.1 \
  --port 8765
```

## vector-os-nano client launch

In the `vector-os-nano` environment:

```bash
conda activate vector-os-nano
vector-cli \
  --molmospaces-rby1-host 127.0.0.1 \
  --molmospaces-rby1-port 8765 \
  --molmospaces-rby1-agent-text
```

Then use:

```text
/rby1 connect
/rby1 observe
/rby1 reset kitchen_scene
/rby1 go to the table and pick up the mug
```

With `--molmospaces-rby1-agent-text`, recognized non-slash input is parsed into
structured RBY1 skills before execution, so this also works:

```text
go to the table and pick up the mug
走到桌子并拿起杯子
```

For bridge debugging, `--molmospaces-rby1-direct-text` is still available. It
forwards non-slash input as one opaque instruction and bypasses skill planning.

## Minimal adapter skeleton

Put this in `molmo_spaces/bridge/rby1_interactive_adapter.py` and then fill in
the environment-specific parts:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class RBY1InteractiveAdapter:
    def connect(self) -> dict[str, Any]:
        return {"connected": True}

    def observe(self) -> dict[str, Any]:
        return {"state": "fill me in"}

    def reset(
        self,
        *,
        scene_name: str | None = None,
        robot_base_pose: list[float] | None = None,
        seed: int | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return {
            "scene_name": scene_name,
            "robot_base_pose": robot_base_pose,
            "seed": seed,
            "metadata": metadata or {},
        }

    def execute(
        self,
        instruction: str,
        *,
        context: dict[str, Any] | None = None,
        mode: str = "auto",
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        return {
            "accepted": True,
            "instruction": instruction,
            "mode": mode,
            "context": context or {},
            "timeout_s": timeout_s,
        }

    def stop(self) -> dict[str, Any]:
        return {"stopped": True}


def create_adapter(_args=None) -> RBY1InteractiveAdapter:
    return RBY1InteractiveAdapter()
```

This skeleton is enough to wire the protocol end-to-end. Replace the stub
returns with calls into the real MolmoSpaces env, task sampler, planner, or
controller stack.
