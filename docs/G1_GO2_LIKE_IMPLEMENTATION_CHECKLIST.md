# G1 仿真路线实施清单：先做“房间 + 抓取 + 场景交互”，再补“行走 + 导航”

本文档用于你自己按文件逐项实现。目标不是一次性把 G1 做成 Go2 的完全等价物，而是先把 G1 接到和 Go2 类似的产品路线：

1. 先让 G1 能进入一个房间场景；
2. 先让它能做场景交互、抓取、放置；
3. 再补移动、导航；
4. 最后再补门、冰箱这类 articulated object 交互。

当前仓库里，G1 的 MuJoCo transport 已经存在，但它明确是固定基座路线，不提供 locomotion controller。第一阶段不要强行改它，应该先把“场景层”和“技能层”补齐。

---

## 一、文件级清单

### 1. 建议新增的文件

| 文件 | 目的 |
|---|---|
| `vector_os_nano/vcli/worlds/g1_sim_oracle.py` | G1 场景的 verify/oracle 函数：`at_position`、`facing`、`visited`、`rooms_producer`。 |
| `vector_os_nano/vcli/worlds/g1.py` | 如果你希望 G1 有独立 world adapter，就把 persona、verify namespace、capability 注册放这里。 |
| `vector_os_nano/perception/g1_grasp_perception.py` | G1 专用抓取感知前端；第一版可以复用现有检测/深度到 3D 点的逻辑，但要独立参数、独立标定。 |
| `vector_os_nano/hardware/sim/mujoco_g1.py` | G1 的场景装配/房间模板生成器；结构上对齐 `mujoco_go2.py` 的“场景 + 后端入口”职责。 |
| `vector_os_nano/hardware/sim/g1_room.xml` | G1 房间场景模板；建议先最小可用，再逐步增加桌子、门、冰箱等。 |
| `scripts/g1_vnav_bridge.py` | 如果你后面要接 ROS2/nav stack，这个脚本对应 Go2 的 bridge。 |
| `vector_os_nano/skills/g1/open_door.py` | 门交互技能，第二/三阶段再加。 |
| `vector_os_nano/skills/g1/open_fridge.py` | 冰箱交互技能，第二/三阶段再加。 |

### 2. 建议修改的文件

| 文件 | 修改要点 |
|---|---|
| `config/robots/g1_edu_flagship_a.yaml` | 增加场景、控制器、感知、地图、默认速度等配置。 |
| `vector_os_nano/vcli/tools/sim_tool.py` | 把 G1 的场景加载、感知、技能注册串起来。 |
| `vector_os_nano/vcli/cli.py` | 增加 `--sim-g1` 或等价入口。 |
| `vector_os_nano/vcli/prompt.py` | 让系统 prompt 说明 G1 能做什么、不能做什么。 |
| `vector_os_nano/vcli/robot_context.py` | 把 G1 当前 room / scene graph / 手臂与手状态注入 prompt。 |
| `vector_os_nano/core/spatial_memory.py` | 扩展 room / object / articulated object 的状态模型。 |
| `vector_os_nano/skills/perception_grasp.py` | 让 G1 走自己的 perception-grasp 路径，不要混用 Go2/Piper 假设。 |
| `vector_os_nano/skills/g1/__init__.py` | 注册 G1 版本的技能。 |
| `vector_os_nano/hardware/sim/g1_mujoco_transport.py` | 第二阶段再改：真正要走移动时，这里才需要从“拒绝 locomotion”变成“支持 locomotion”。 |
| `vector_os_nano/skills/navigate.py` | 让导航技能对 G1 的 base / scene 也成立。 |
| `vector_os_nano/vcli/worlds/registry.py` | 注册 G1 world/scenario 名称。 |

### 3. 不建议删除的文件

| 文件 | 说明 |
|---|---|
| 无 | 这一阶段不建议删源码。生成物只加 `.gitignore`，不要删仓库源码。 |

---

## 二、实施顺序

### Phase 1：先把 G1 接成“房间 + 抓取 + 场景语义”

这一步的目标是：

- G1 能加载一个房间；
- G1 能进入当前 room / object 的语义环境；
- G1 能用现有的通用抓取链完成 tabletop 级别的 pick/place；
- 不要求真正走路。

优先级建议：

1. `config/robots/g1_edu_flagship_a.yaml`
2. `vector_os_nano/vcli/tools/sim_tool.py`
3. `vector_os_nano/vcli/worlds/g1_sim_oracle.py`
4. `vector_os_nano/perception/g1_grasp_perception.py`
5. `vector_os_nano/skills/perception_grasp.py`
6. `vector_os_nano/vcli/prompt.py`
7. `vector_os_nano/vcli/robot_context.py`
8. `vector_os_nano/core/spatial_memory.py`
9. `vector_os_nano/vcli/worlds/registry.py`
10. `vector_os_nano/vcli/cli.py`

### Phase 2：再补 G1 的移动控制 / 导航

目标是：

- G1 不再是固定基座；
- `command_velocity()` 不再直接失败；
- `navigate()` 能驱动 G1 base 做移动；
- 再考虑 nav stack、lidar、地图、避障。

### Phase 3：门、冰箱等 articulated object

目标是：

- 门和冰箱有关节状态；
- 能感知把手；
- 能执行 open-door / open-fridge 任务；
- 能把这些状态写入 world model / scene graph。

---

## 三、需要新增的代码骨架

下面给的是“可直接照着建文件”的骨架。你可以先按这些骨架落地，再逐步把细节补齐。

### 3.1 `vector_os_nano/vcli/worlds/g1_sim_oracle.py`

这个文件的职责是：给 G1 的 verifier 提供稳定的、不会抛异常的场景谓词。

```python
# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math
from typing import Any, Callable


_AT_POSITION_TOL_M = 0.5
_FACING_TOL_RAD = math.radians(20.0)


def _get_base(agent: Any) -> Any | None:
    if agent is None:
        return None
    base = getattr(agent, "_base", None)
    if base is None:
        return None
    if getattr(base, "_connected", True) is False:
        return None
    return base


def _base_position(base: Any) -> list[float] | None:
    try:
        pos = base.get_position()
        return [float(pos[0]), float(pos[1]), float(pos[2])]
    except Exception:
        return None


def _base_heading(base: Any) -> float | None:
    try:
        return float(base.get_heading())
    except Exception:
        return None


def _angle_delta(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def make_at_position(agent: Any) -> Callable[..., bool]:
    def at_position(x: Any, y: Any, tol: Any = _AT_POSITION_TOL_M) -> bool:
        base = _get_base(agent)
        if base is None:
            return False
        try:
            tx, ty, t = float(x), float(y), float(tol)
        except (TypeError, ValueError):
            return False
        pos = _base_position(base)
        if pos is None:
            return False
        return math.dist((pos[0], pos[1]), (tx, ty)) <= t

    return at_position


def make_facing(agent: Any) -> Callable[..., bool]:
    def facing(heading: Any, tol: Any = _FACING_TOL_RAD) -> bool:
        base = _get_base(agent)
        if base is None:
            return False
        try:
            target, t = float(heading), float(tol)
        except (TypeError, ValueError):
            return False
        yaw = _base_heading(base)
        if yaw is None:
            return False
        return _angle_delta(yaw, target) <= t

    return facing


def make_visited(agent: Any, rooms: dict[str, tuple[float, float, float, float]]) -> Callable[..., bool]:
    room_boxes = {
        str(name): tuple(float(v) for v in box)
        for name, box in (rooms or {}).items()
    }

    def visited(room: Any) -> bool:
        base = _get_base(agent)
        if base is None:
            return False
        box = room_boxes.get(str(room))
        if box is None:
            return False
        pos = _base_position(base)
        if pos is None:
            return False
        x_min, y_min, x_max, y_max = box
        return x_min <= pos[0] <= x_max and y_min <= pos[1] <= y_max

    return visited


def make_rooms_producer(
    rooms: dict[str, tuple[float, float, float, float]],
) -> Callable[..., dict[str, Any]]:
    room_boxes = {
        str(name): tuple(float(v) for v in box)
        for name, box in (rooms or {}).items()
    }

    def rooms_producer(**_: Any) -> dict[str, Any]:
        out: list[dict[str, Any]] = []
        for name in sorted(room_boxes):
            x_min, y_min, x_max, y_max = room_boxes[name]
            out.append(
                {
                    "name": name,
                    "x": (x_min + x_max) / 2.0,
                    "y": (y_min + y_max) / 2.0,
                }
            )
        return {"rooms": out, "count": len(out)}

    return rooms_producer
```

### 3.2 `vector_os_nano/vcli/worlds/g1.py`

如果你想让 G1 有独立 world，而不是完全复用 `robot` world，就加这个。

```python
# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from typing import Any

from vector_os_nano.vcli.worlds.g1_sim_oracle import (
    make_at_position,
    make_facing,
    make_rooms_producer,
    make_visited,
)


class G1World:
    name = "g1"

    def is_robot(self) -> bool:
        return True

    def persona_blocks(self) -> tuple[str, str]:
        role_prompt = (
            "You are controlling a Unitree G1 robot with dual arms and dual hands. "
            "Phase 1: room semantics + grasping + placement. "
            "Do not assume locomotion is available unless the backend says so."
        )
        tool_instructions = (
            "Use navigate/pick/place/scan/home/where_am_i. "
            "Use room and object names from the scene graph."
        )
        return role_prompt, tool_instructions

    def register_tools(self, registry: Any, agent: Any) -> None:
        return None

    def build_verify_namespace(self, agent: Any) -> dict[str, Any]:
        rooms = {}
        sg = getattr(agent, "_spatial_memory", None)
        if sg is not None and hasattr(sg, "get_all_rooms"):
            for room in sg.get_all_rooms():
                rid = getattr(room, "room_id", None)
                box = getattr(room, "bbox", None)
                if rid and box:
                    rooms[str(rid)] = tuple(box)
        ns = {
            "at_position": make_at_position(agent),
            "facing": make_facing(agent),
            "visited": make_visited(agent, rooms),
            "rooms": make_rooms_producer(rooms),
        }
        return ns

    def register_capabilities(self, registry: Any, agent: Any, backend: Any) -> None:
        return None

    def decompose_vocab(self):
        return None

    def derive_vocab_from_registry(self) -> bool:
        return True
```

### 3.3 `vector_os_nano/perception/g1_grasp_perception.py`

这个文件第一版的目标不是“更聪明”，而是“独立、可替换、能给 G1 用”。

```python
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
```

### 3.4 `vector_os_nano/hardware/sim/mujoco_g1.py`

这个文件的职责是：给 G1 一个和 Go2 类似的“场景装配层”。  
如果你不想增加新层，也可以把等价逻辑放回 `g1_mujoco_transport.py`，但我建议单独拆出来，后续更干净。

```python
# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from pathlib import Path


_THIS_DIR = Path(__file__).resolve().parent


def build_room_scene_xml(*, with_articulated_objects: bool = False) -> Path:
    """
    返回最终用于运行的 G1 场景 XML 路径。
    第一版可以先直接写到一个 runtime 文件里，再由 transport 读取。
    """
    out = _THIS_DIR / "mjcf" / "g1" / "scene_room.xml"
    out.parent.mkdir(parents=True, exist_ok=True)
    xml = """\
<mujoco model="g1_room">
  <compiler angle="radian" meshdir="assets" autolimits="true"/>
  <include file="../../g1_29dof_with_hand_rev_1_0.xml"/>
</mujoco>
"""
    out.write_text(xml, encoding="utf-8")
    return out
```

### 3.5 `vector_os_nano/hardware/sim/g1_room.xml`

这是一个最小房间模板示意。  
第一版建议只放静态房间、桌子、地面，再慢慢加门和冰箱。

```xml
<mujoco model="g1_room">
  <compiler angle="radian" autolimits="true" meshdir="assets"/>
  <option timestep="0.002" cone="elliptic" impratio="100"/>

  <include file="g1_29dof_with_hand_rev_1_0.xml"/>

  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.85 0.85 0.85 1"/>

    <body name="table" pos="1.0 0.0 0.75">
      <geom type="box" size="0.6 0.4 0.02" rgba="0.6 0.4 0.2 1"/>
      <geom type="box" pos="0 0 -0.35" size="0.03 0.03 0.35" rgba="0.5 0.3 0.15 1"/>
      <geom type="box" pos="0.55 0 -0.35" size="0.03 0.03 0.35" rgba="0.5 0.3 0.15 1"/>
    </body>

    <!-- 后续再加 door/fridge 等可交互物体 -->
  </worldbody>
</mujoco>
```

### 3.6 `scripts/g1_vnav_bridge.py`

如果你后面要走 ROS2 / nav stack，这个脚本会是 G1 的桥。第一版先保留最小通信结构。

```python
#!/usr/bin/env python3
from __future__ import annotations

def main() -> int:
    """
    这里先放最小骨架：
    - 连接 G1 sim/base
    - 发布里程计 / tf / scan（如果有）
    - 订阅 cmd_vel
    - 调用 G1 base controller
    """
    raise NotImplementedError("Implement G1 nav bridge")


if __name__ == "__main__":
    raise SystemExit(main())
```

### 3.7 `vector_os_nano/skills/g1/open_door.py`

这类 skill 建议留到后面，但文件结构可以先建。

```python
# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from vector_os_nano.core.skill import skill
from vector_os_nano.core.types import SkillResult


@skill(aliases=["open door", "开门", "把门打开"], direct=False)
class OpenDoorSkill:
    name = "open_door"
    description = "Open a hinged door using the current arm/hand."

    def execute(self, params, context) -> SkillResult:
        return SkillResult(
            success=False,
            error_message="OpenDoorSkill not implemented yet",
        )
```

### 3.8 `vector_os_nano/skills/g1/open_fridge.py`

```python
# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

from vector_os_nano.core.skill import skill
from vector_os_nano.core.types import SkillResult


@skill(aliases=["open fridge", "开冰箱", "把冰箱打开"], direct=False)
class OpenFridgeSkill:
    name = "open_fridge"
    description = "Open a fridge door using the current arm/hand."

    def execute(self, params, context) -> SkillResult:
        return SkillResult(
            success=False,
            error_message="OpenFridgeSkill not implemented yet",
        )
```

---

## 四、现有文件里要插入的关键代码

下面这些不是新文件，但你实现时一定会碰到。

### 4.1 `vector_os_nano/vcli/tools/sim_tool.py`

你要保证 G1 在启动时接入：

- profile；
- transport；
- kinematics；
- skill registry；
- scene graph；
- G1 perception。

建议的插入点逻辑如下：

```python
if sim_type == "g1":
    from vector_os_nano.hardware.g1 import G1Profile, G1Robot
    from vector_os_nano.hardware.sim.g1_mujoco_transport import MuJoCoG1Transport
    from vector_os_nano.hardware.g1.pinocchio_kinematics import PinocchioG1Kinematics
    from vector_os_nano.skills.g1 import get_g1_skills
    from vector_os_nano.perception.g1_grasp_perception import G1GraspPerception

    profile = G1Profile.from_yaml(selected_profile)
    transport = MuJoCoG1Transport(profile, gui=gui)
    kinematics = PinocchioG1Kinematics(profile)
    robot = G1Robot(profile, transport, kinematics=kinematics)

    agent = Agent(
        arms=robot.arms,
        grippers=robot.grippers,
        hands=robot.hands,
        bases={"g1": robot.base},
        default_arm_name=profile.default_arm,
        default_gripper_name=profile.default_hand,
        default_hand_name=profile.default_hand,
        default_base_name="g1",
        skills=get_g1_skills(),
        config=config,
    )

    agent._sim_type = "g1"
    agent._g1_robot = robot
    agent._perception = G1GraspPerception(robot.base)
```

### 4.2 `vector_os_nano/skills/perception_grasp.py`

这里的关键是：不要把 G1 的抓取强行绑定到 Go2/Piper 的场景假设。  
你需要把后端 perception 抽象成“只返回 3D grasp point”，后面的 `PickSkill` 才是真正执行器。

你应该补一段类似这样的路由：

```python
def _resolve_perception_backend(context):
    perception = getattr(context, "perception", None)
    if perception is None:
        return None
    if perception.__class__.__name__ in {"Go2GraspPerception", "G1GraspPerception"}:
        return perception
    return perception
```

然后确保：

```python
pick_params = {
    "target_xyz": grasp_point,
    "mode": "hold",
    "arm": arm_name,
}
```

最后仍然调用通用 `PickSkill`。

### 4.3 `vector_os_nano/vcli/prompt.py`

你需要把 G1 的能力边界写清楚。  
第一阶段的 prompt 应该明确：

- 支持 room/object 语义；
- 支持抓取 / 放置；
- 暂不保证 locomotion；
- 门/冰箱交互未完成时不要让模型幻想已支持。

建议插入类似这段：

```python
G1_CAPABILITY_BLOCK = """
G1 robot:
- Has dual arms and dual Dex3 hands.
- Phase 1 supports room/object interaction and tabletop grasping.
- Do not assume locomotion unless the backend explicitly supports it.
- Door/fridge interaction may be unavailable unless the corresponding skills are registered.
"""
```

### 4.4 `vector_os_nano/vcli/robot_context.py`

把这些字段注入 prompt：

```python
{
    "robot_name": "g1",
    "current_room": current_room_name,
    "scene_rooms": room_count,
    "scene_objects": object_count,
    "has_base": True,
    "has_left_arm": True,
    "has_right_arm": True,
    "has_left_hand": True,
    "has_right_hand": True,
}
```

### 4.5 `vector_os_nano/core/spatial_memory.py`

建议至少扩展出这些状态：

```python
class SceneObject:
    object_id: str
    label: str
    room_id: str | None = None
    pose: dict[str, float] | None = None
    object_type: str = "object"  # object / door / fridge / receptacle
    opened: bool = False
    held_by: str | None = None
```

如果后面要做门和冰箱，`opened` / `joint_state` 这类字段会很重要。

### 4.6 `vector_os_nano/skills/g1/__init__.py`

第一阶段建议只注册真正可用的技能，不要把未完成的门/冰箱技能提前暴露。

```python
from vector_os_nano.skills.g1.dex3 import Dex3PoseSkill
from vector_os_nano.skills.g1.stance import G1StandSkill, StandSkill
from vector_os_nano.skills.g1.stop import EmergencyStopSkill, G1StopSkill, StopSkill
from vector_os_nano.skills.g1.turn import G1TurnSkill, TurnSkill
from vector_os_nano.skills.g1.walk import G1WalkSkill, WalkSkill
from vector_os_nano.skills.navigate import NavigateSkill
from vector_os_nano.skills.go2.where_am_i import WhereAmISkill


def get_g1_skills() -> list:
    return [
        WalkSkill(),
        TurnSkill(),
        StandSkill(),
        NavigateSkill(),
        WhereAmISkill(),
        StopSkill(),
        EmergencyStopSkill(),
        Dex3PoseSkill(),
    ]
```

---

## 五、测试清单

建议你至少补这些测试：

| 文件 | 测什么 |
|---|---|
| `tests/unit/test_g1_world_oracle.py` | `at_position` / `facing` / `visited`。 |
| `tests/unit/test_g1_grasp_perception.py` | G1 perception backend 的接口形状。 |
| `tests/integration/test_g1_room_scene.py` | 场景 XML 能加载，场景对象注册正确。 |
| `tests/vcli/test_sim_g1_flow.py` | `--sim-g1`、技能注册、prompt 注入。 |
| `tests/unit/test_nav_client.py` | 如果你开始接移动控制，这里补 G1 分支。 |

如果你先不做 locomotion，就不要强行改 nav 的行为测试；先把抓取链测通更重要。

---

## 六、你现在最短的可执行路径

如果只追求“先跑通，不追求完整”，我建议你只先做这 8 个点：

1. `config/robots/g1_edu_flagship_a.yaml`
2. `vector_os_nano/vcli/tools/sim_tool.py`
3. `vector_os_nano/vcli/worlds/g1_sim_oracle.py`
4. `vector_os_nano/perception/g1_grasp_perception.py`
5. `vector_os_nano/skills/perception_grasp.py`
6. `vector_os_nano/vcli/prompt.py`
7. `vector_os_nano/vcli/robot_context.py`
8. `vector_os_nano/core/spatial_memory.py`

这条线的意义是：先让 G1 具备“房间语义 + 抓取语义 + 任务路由”，然后再补移动和 articulated object。

