# MolmoSpaces RBY1 bridge server

For the OneRING + local A* navigation process and launch commands, see
[RBY1_ONERING_NAVIGATION.md](RBY1_ONERING_NAVIGATION.md).

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
export MLSPACES_ASSETS_DIR=/media/fishyu/fish-14tb-12/YiFei/molmospaces/molmo-spaces-resources
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
export MLSPACES_ASSETS_DIR=/media/fishyu/fish-14tb-12/YiFei/molmospaces/molmo-spaces-resources

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
  --molmospaces-rby1-vgg
```

Then use:

```text
/rby1 connect
/rby1 observe
/rby1 reset kitchen_scene
/rby1 go to the table and pick up the mug
```

With `--molmospaces-rby1-vgg`, the CLI creates a lightweight RBY1 Agent whose
skill registry contains only MolmoSpaces RBY1 skills. Non-slash input then goes
through the normal VGG path (`GoalDecomposer -> StrategySelector -> GoalExecutor`)
against that RBY1 skill vocabulary:

```text
go to the table and pick up the mug
走到桌子并拿起杯子
```

For deterministic parser testing, `--molmospaces-rby1-agent-text` is also
available. It handles the same common navigation/pick phrases through a small
rule parser and still executes the structured RBY1 skills.

For bridge debugging, `--molmospaces-rby1-direct-text` is still available. It
forwards non-slash input as one opaque instruction and bypasses skill planning.

The RBY1-VGG embodiment exposes these structured skills:

```text
rby1_observe
rby1_sync_scene
rby1_detect_object
onering_navigation
astar_plan
grounding_dino_detect
rby1_pick_object
rby1_place_object
rby1_stop
```

`rby1_observe` and `rby1_sync_scene` synchronize MolmoSpaces scene objects into
Vector's world model / scene graph. `rby1_detect_object` uses the bridge-backed
perception source. `onering_navigation`, `astar_plan`, and
`grounding_dino_detect` are generic Vector tools rather than RBY1-only actions;
Go2 and future RGB-D mobile robots discover the same tools at startup.

The GUI viewer is process-isolated (GLFW) from the MolmoSpaces EGL offscreen
renderer, so `observe_rgbd` remains available while the viewer is open.

The current reference adapter advertises navigation, RGB observation, and scene
object state. Pick/place skills are present as structured VGG actions, but they
return an honest `manipulation_unsupported` result until a CuRobo-backed
MolmoSpaces manipulation adapter advertises `pick_object` / `place_object`.

For an already loaded scene, generic navigation keeps the same MuJoCo scene,
observes RGB-D, updates the semantic topology map, and executes OneRING discrete
actions or local A* waypoints.

Bridge reset 只加载场景、机器人和相机，并进入无目标待命状态。sampler 为创建
`NavToObjTask` 临时选择的对象会在 reset 后立即清空；内置 MolmoSpaces A* policy
不会被创建、预热或执行。此时 state 中的 `task_description` 为
`Waiting for Vector navigation target`，`external_navigation_target` 为 `null`。

The online controller also uses two structured bridge actions internally:
`prepare_navigation` retargets the active `nav_to_obj` task to the user's
language target without reloading the scene, and `navigation_status` returns a
distance-grounded arrival verdict. A OneRING `done` action is never accepted as
success without this verdict.

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
