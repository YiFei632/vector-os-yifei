# Unitree G1 EDU 旗舰版 A 集成说明

本文记录当前仓库对 **Unitree G1 EDU 旗舰版 A、29 个机身自由度、双 Dex3-1 灵巧手** 的实际集成状态、接口约束、启动方式和后续输入要求。内容以本仓库现有代码为准；规划中的能力会明确标为“未完成”或“待实机验证”。

> 当前结论：G1 的模型配置、43 执行器名称映射、统一硬件抽象、双臂/双手选择、Pinocchio 固定骨盆运动学和 MuJoCo 固定基座关节控制已经落地。Isaac Sim 的 Docker、官方 Unitree 运行时适配、DDS/文件/ROS 2 桥和主机 transport 已实现并通过契约测试，但仓库中没有官方 USD 资产与全身策略，因此尚未完成真实 Isaac 容器端到端验收。真机 DDS transport、G1 传感器、导航栈以及门/冰箱交互仍未实现。

## 1. 已冻结的机器人型号

| 项目 | 当前配置 |
| --- | --- |
| 产品 | Unitree G1 EDU 旗舰版 A |
| 机身自由度 | 29：双腿 12 + 腰部 3 + 双臂 14 |
| 灵巧手 | 左右各一只 Dex3-1，每只 7 个执行器 |
| 总执行器数 | 43 = 29 + 7 + 7 |
| `mode_machine` | `5` |
| 腰部 | 3 DoF，未锁定 |
| 腕部电机配置 | `4010` |
| 根坐标系 | `pelvis` |
| 导航坐标系 | `base_link` |
| 左/右 TCP | `left_hand_palm_link` / `right_hand_palm_link` |
| 默认臂/手 | 右臂 / 右手 |

这里的“29 DoF”只指 LowCmd/LowState 管理的机身部分；Dex3 的 14 个关节通过独立手部通道控制。不要把该型号描述为“机身 43 DoF”。

当前开发机使用的模型资产是：

```text
/media/fishyu/fish-14tb-11/YiFei/unitree_ros/robots/g1_description/
├── g1_29dof_with_hand_rev_1_0.urdf
├── g1_29dof_with_hand_rev_1_0.xml
└── meshes/
```

配置文件为 `config/robots/g1_edu_flagship_a.yaml`。其中固定了下列 SHA256：

```text
URDF  97da67732d067c3147fc5fb7b7bafc8982718f4e7f8c92ff82266a4d9c07200d
MJCF  77c98e7d34e428c6cdbd1e0d665e989cbc43bf17abb8a7c048494b4bc51ead88
```

其他机器不必复用配置中的绝对路径。推荐令环境变量覆盖 `assets.root`：

```bash
export VECTOR_G1_ASSET_ROOT=/absolute/path/to/g1_description
```

`G1Profile` 会检查 URDF、MJCF、mesh 目录和可选 SHA256。若上游模型发生变化，应建立新的 profile 并重新验收，不应直接关闭哈希检查来沿用 rev-1.0 的控制参数。

### 1.1 当前模型资产的边界

- URDF 用于固定骨盆的 Pinocchio 运动学；其中没有启用浮动根关节。
- MJCF 有 free joint，加载后模型尺寸为 `nq=50`、`nv=49`、`nu=43`。
- 当前 MJCF 主要是模型/力矩执行器描述，没有可直接使用的站立或行走策略，也没有 keyframe。
- 当前资产没有提供可供 HAL 使用的 LiDAR、RGB-D、触觉、足底接触或可靠抓持检测通道。`G1State` 预留了部分字段，但 transport 没有伪造这些测量值。
- URDF 与 MJCF 的少数限位/力矩参数并不完全一致。当前 HAL 以更保守的 URDF 关节位置限位为准。
- 用户已提供的 URDF/MJCF/mesh **不能替代 Isaac 官方 USD 和与其配套的策略文件**。

## 2. 43 执行器及三种已验证的关节顺序

本集成最重要的约束是：**Vector 语义顺序、vendor MJCF actuator 顺序和官方 IsaacLab/桥接顺序不是同一个顺序。** 所有边界转换都必须按关节名进行，不能把某个后端的整数下标泄漏到 Skill 或 Agent 层。

### 2.1 公共关节分组

29 个机身关节的公共分组如下：

```text
左腿 6:
  left_hip_pitch_joint, left_hip_roll_joint, left_hip_yaw_joint,
  left_knee_joint, left_ankle_pitch_joint, left_ankle_roll_joint

右腿 6:
  right_hip_pitch_joint, right_hip_roll_joint, right_hip_yaw_joint,
  right_knee_joint, right_ankle_pitch_joint, right_ankle_roll_joint

腰部 3:
  waist_yaw_joint, waist_roll_joint, waist_pitch_joint

左臂 7:
  left_shoulder_pitch_joint, left_shoulder_roll_joint,
  left_shoulder_yaw_joint, left_elbow_joint,
  left_wrist_roll_joint, left_wrist_pitch_joint, left_wrist_yaw_joint

右臂 7:
  right_shoulder_pitch_joint, right_shoulder_roll_joint,
  right_shoulder_yaw_joint, right_elbow_joint,
  right_wrist_roll_joint, right_wrist_pitch_joint, right_wrist_yaw_joint
```

### 2.2 顺序 A：Vector HAL 语义顺序

这是 `G1Profile.all_joint_names`、`G1State`、`G1Arm`、`G1Hand` 和 Skill 层看到的标准顺序：

| 全局下标 | 分组 | 分组内部顺序 |
| --- | --- | --- |
| 0–5 | 左腿 | pitch, roll, yaw, knee, ankle pitch, ankle roll |
| 6–11 | 右腿 | pitch, roll, yaw, knee, ankle pitch, ankle roll |
| 12–14 | 腰部 | yaw, roll, pitch |
| 15–21 | 左臂 | shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw |
| 22–28 | 右臂 | shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw |
| 29–35 | 左手 | thumb 0/1/2, index 0/1, middle 0/1 |
| 36–42 | 右手 | thumb 0/1/2, index 0/1, middle 0/1 |

单手 7 维向量的精确语义顺序为：

```text
thumb_0, thumb_1, thumb_2, index_0, index_1, middle_0, middle_1
```

该顺序是 Skill、`G1Hand`、`G1State` 和 `G1JointCommand` 的公共契约，**不是**物理 Dex3 DDS 数字线序。

### 2.3 顺序 B：rev-1.0 MJCF actuator 顺序

`g1_29dof_with_hand_rev_1_0.xml` 的 43 个 actuator 排列为：

| XML actuator 下标 | 分组 | 分组内部顺序 |
| --- | --- | --- |
| 0–5 | 左腿 | 与语义顺序相同 |
| 6–11 | 右腿 | 与语义顺序相同 |
| 12–14 | 腰部 | 与语义顺序相同 |
| 15–21 | 左臂 | 与语义顺序相同 |
| 22–28 | 左手 | thumb 0/1/2, **middle 0/1, index 0/1** |
| 29–35 | 右臂 | 与语义顺序相同 |
| 36–42 | 右手 | thumb 0/1/2, **index 0/1, middle 0/1** |

因此机身 29 维向量在 XML 中不是连续的：

```text
body semantic <- XML indices [0..21, 29..35]
left hand semantic <- XML indices [22, 23, 24, 27, 28, 25, 26]
right hand semantic <- XML indices [36, 37, 38, 39, 40, 41, 42]
```

### 2.4 顺序 C：锁定的官方 IsaacLab task 顺序

当前 pin 住的 `unitree_sim_isaaclab` G129 + Dex3 任务使用：

| 全局下标 | 分组 | 分组内部顺序 |
| --- | --- | --- |
| 0–21 | 左腿、右腿、腰部、左臂 | 与语义顺序相同 |
| 22–28 | 右臂 | 与语义顺序相同 |
| 29–35 | 左手 | thumb 0/1/2, **middle 0/1, index 0/1** |
| 36–42 | 右手 | thumb 0/1/2, **middle 0/1, index 0/1** |

两只手从 Vector 语义顺序转换到 Isaac 顺序的排列均为：

```text
[0, 1, 2, 5, 6, 3, 4]
```

`G1JointMap`、`G1IsaacTransport` 和容器 manifest 都把这层明确命名为 `ISAAC_TASK`/`ISAACLAB`，使用名称映射完成转换。测试中使用不对称的 7 维向量，分别检查 HAL↔XML 和 HAL↔Isaac，防止 index/middle 被静默交换。

### 2.5 真机 Dex3 DDS motor 顺序：刻意留待验收

当前代码**没有**把 rev-1.0 XML actuator 顺序声明成真机 `HandCmd`/`HandState` motor 顺序。核对依据包括 [Unitree 官方 Dex3 SDK 示例](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/dex3/g1_dex3_example.cpp) 和 [Unitree 官方 IsaacLab 仓库](https://github.com/unitreerobotics/unitree_sim_isaaclab)。原因是：

- Unitree 官方 Dex3 SDK 示例只按 `motor_cmd()[0..6]` 操作，并给出对应限位，但没有发布 motor ID 到 thumb/index/middle 名称的左右手映射；
- 用户提供的 XML 中，左手为 thumb/middle/index，右手为 thumb/index/middle；
- 当前锁定的官方 IsaacLab task 中，左右手均为 thumb/middle/index。

因此 XML 和 Isaac 顺序都已冻结并测试，但它们不足以证明旗舰版 A 当前固件上的真实线序。未来实现 `G1RealTransport` 前，必须取得该机序列号/固件对应的 Dex3 motor-ID 表，或在吊装、低 `kp`、限速和单电机点动条件下完成 0–6 到关节名的验收，再把映射作为独立真机配置加入。这样可以避免右手 index/middle 被静默互换。

## 3. 代码目录与职责

```text
config/robots/g1_edu_flagship_a.yaml       型号、资产、坐标系、限速和 Skill 姿态

vector_os_nano/hardware/g1/
├── profile.py                             型号冻结、关节名、限位、资产与哈希校验
├── joint_map.py                           HAL/机身 DDS/XML/IsaacLab 名称重排
├── state.py                               控制模式、速度/关节命令、43 关节状态
├── transport.py                           后端协议和显式 capability
├── coordinator.py                         共享生命周期、命令串行化、模式和急停锁存
├── base.py                                BaseProtocol 视图
├── arm.py                                 左/右 7-DoF ArmProtocol 视图
├── hand.py                                左/右 Dex3 完整接口和 GripperProtocol 适配
├── ik.py                                  运动学协议
├── pinocchio_kinematics.py                固定骨盆双臂 FK/位置 IK
└── robot.py                               一个 transport 上的整机组合根

vector_os_nano/hardware/sim/
├── g1_mujoco_transport.py                 MJCF + 关节 PD 的固定基座操作后端
└── g1_isaac_transport.py                  主机 ROS 2 到 Isaac 容器的 G1 transport

vector_os_nano/skills/g1/
├── walk.py, turn.py, stance.py, stop.py   G1 底盘 Skill 与 capability 检查
├── dex3.py                                双手具名姿态/精确 7 关节 Skill
└── __init__.py                            G1 Skill 集合

docker/isaac-sim/
├── Dockerfile                             Isaac Sim/IsaacLab/Unitree 版本固定
├── docker-compose.yaml                    GPU、网络、资产挂载和健康检查
├── docker-entrypoint.sh                   Go2/G1 插件分派及进程监管
├── unitree-cyclonedds.xml                 Unitree 仿真 DDS 的 loopback 隔离
└── bridge/
    ├── g1_manifest.py                     29+7+7 wire manifest
    ├── g1_file_protocol.py                原子 JSON 命令/状态协议
    ├── g1_isaac_runtime.py                官方任务和资产 fail-fast 预检
    ├── g1_dds_bridge.py                   官方 Unitree DDS 与 JSON 之间的网关
    └── ros2_publisher.py                  JSON 与主机 ROS 2 topic 之间的网关
```

### 3.1 核心接口

- `G1Profile`：一个具体 embodiment 的唯一配置源。`mode_machine`、关节数量、手型、TCP、资产和哈希不匹配时失败。
- `G1Transport`：Isaac、MuJoCo 和未来真机后端必须实现的最小协议。
- `G1TransportCapabilities`：只声明后端已实现的能力，不根据“机器人理论上能做什么”进行猜测。
- `G1ControlCoordinator`：让 base、两条 arm、两只 hand 共用一个 transport、锁和单调命令序号；具名 owner 保证连接/断开幂等且不会由某一只手提前关闭整机。
- `G1Base`：速度、里程计、站立请求、停止和软件急停视图。
- `G1Arm`：精确 7 维关节控制；只有注入真实运动学后端时才开放 FK/IK/Cartesian。
- `G1Hand`：精确 7 维 Dex3 控制，提供 `open()`、`power_grasp()`、`pinch()` 和原始关节目标。
- `G1GripperAdapter`：把一只 Dex3 映射到现有标量 gripper Skill；它不是对全部手指能力的替代。
- `G1Robot`：输出 `base`、`arms["left"|"right"]`、`hands[...]` 和 `grippers[...]`。

### 3.2 后端 capability 矩阵

| 能力 | Isaac transport 当前声明 | MuJoCo transport 当前声明 |
| --- | --- | --- |
| 速度/行走 | `True`，依赖官方 whole-body policy | `False`，非零速度明确拒绝 |
| 横移 | `True`，依赖同一策略 | `False` |
| 直接 body 29 关节组 | 不向 HAL 广告；腿/腰由策略拥有 | `True`，仅测试用途 |
| 左/右臂 | `True` | `True` |
| 左/右手 | `True` | `True` |
| LiDAR | `False` | `False` |
| 真正 emergency damping | `False`；仅软件锁存 + 零速/关节保持 | `True`；仿真粘性阻尼 |
| 根状态 | ROS `/state_estimation` | MJCF free joint；固定基座模式下被钉住 |

Isaac 列描述的是代码契约；因为官方 USD/策略尚未提供给此仓库，行走与双臂/双手的真实容器执行仍需完成第 10.5 节的 live acceptance。

## 4. 按实施顺序的当前完成度

| 顺序 | 工作项 | 状态 | 已验证内容 |
| --- | --- | --- | --- |
| 1 | 冻结 G1 EDU 旗舰版 A rev-1.0 | 已完成 | profile、mode 5、29+7+7、资产 SHA256 |
| 2 | 建立三种已验证关节顺序和严格名称映射 | 已完成 | HAL/机身 DDS/XML/Isaac 排列、维度、NaN、重复名测试；真机 Dex3 DDS 显式延期至硬件验收 |
| 3 | 建立与 Go2 类似的硬件分层 | 已完成 | base/arm/hand/transport/coordinator/robot 接口和共享生命周期 |
| 4 | Agent 的双臂/双手具名注册 | 已完成 | `left`/`right` 选择、右侧默认、通用 Skill pose 配置 |
| 5 | Pinocchio 双 7-DoF 运动学 | 已完成，范围受限 | 真实 URDF FK、近距离位置 IK、限位和不可达返回 `None` |
| 6 | MuJoCo rev-1.0 后端 | 已完成固定基座操作基线 | 43 状态、双臂独立轨迹、阻塞收敛、关节 PD、无 locomotion 的显式失败 |
| 7 | Isaac Docker/官方 runtime/桥接 | 代码与契约测试已完成 | 版本 pin、fail-fast 预检、DDS 隔离、JSON/ROS topic、host transport 单测 |
| 8 | Isaac 真正加载官方 USD/策略并运动 | 待完成 | 缺少官方资产和 policy，尚无 live 容器结果 |
| 9 | G1 传感器、场景导航和交互 | 待完成 | 当前无 G1 LiDAR/RGB-D bridge、外参、地图和可交互门/冰箱 |
| 10 | 真机 G1 transport | 待完成 | 当前没有 `G1RealTransport`，仿真 DDS 被刻意限制在 loopback |

## 5. MuJoCo：当前可用范围

MuJoCo 后端直接加载用户提供的 rev-1.0 XML，并校验 `50/49/43` 模型尺寸和完整 actuator 名称顺序。它增加了仿真专用 damping/armature 和一个保守的关节 PD；不会修改 vendor XML 文件。

默认 profile 设置：

```yaml
simulation:
  mujoco:
    fixed_base_manipulation: true
    locomotion: unsupported_without_policy
```

当 `fixed_base_manipulation=true` 时，每个物理步都会把 free root 固定在 `(0, 0, 0.8)` 并清零根速度。这使它适合：

- 双臂 7-DoF 关节轨迹；
- 双 Dex3 关节姿态；
- Pinocchio 位置 IK 后的关节目标验证；
- Skill 的左右肢体路由和接口回归。

它**不代表**以下能力已经实现：

- 动态平衡、站起、蹲下、行走或横移；
- 浮动基座下的全身协调；
- 足底接触控制、跌倒恢复；
- 碰撞感知抓取、触觉或真实抓持判断。

因此 `MuJoCoG1Transport.capabilities.locomotion` 为 `False`。`G1Base.walk()`、`stand()`、`turn` 和依赖 locomotion 的 `navigate` 会明确失败，而不是返回假的成功。`stop()` 仍可安全停止轨迹并保持当前关节目标。

通过 Vector CLI 的 `start_simulation` 工具启动时，结构化参数为：

```json
{
  "sim_type": "g1",
  "backend": "mujoco",
  "gui": false,
  "controller_mode": "whole_body",
  "profile_path": "config/robots/g1_edu_flagship_a.yaml"
}
```

`controller_mode` 当前只接受 `whole_body`；这表示统一的 G1 控制契约，不表示 MuJoCo 已具备 whole-body locomotion policy。

## 6. Isaac Sim：官方 USD、策略与 Docker

### 6.1 固定的软件组合

`docker/isaac-sim/Dockerfile` 当前固定：

| 组件 | 版本/提交 |
| --- | --- |
| NVIDIA Isaac Sim | `5.1.0` |
| Isaac Lab | `v2.3.2` |
| CycloneDDS source | `0.10.5` |
| `unitree_sdk2_python` | `65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5` |
| `unitree_sim_isaaclab` | `e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc` |
| ROS 2 | Jazzy，容器和主机使用 domain 0 |

构建镜像需要能够访问 NVIDIA registry、Ubuntu/ROS 软件源和上述 Git 仓库，并要求 NVIDIA driver、Docker Engine、Compose v2 和 `nvidia-container-toolkit`。

### 6.2 必须额外提供的官方资产

Compose 将 `${G1_ASSET_DIR}` 挂载到容器的 `/opt/unitree_sim_isaaclab/assets`。该目录至少必须包含：

```text
assets/
├── model/
│   └── policy.onnx
├── robots/
│   └── g1-29dof_wholebody_dex3/
│       └── g1_29dof_with_dex3_rev_1_0.usd
└── objects/
    ├── small_warehouse/
    │   └── small_warehouse_digital_twin.usd
    ├── PackingTable/
    │   └── PackingTable.usd
    └── PackingTable_2/
        └── PackingTable.usd
```

官方任务名只允许：

```text
Isaac-Move-Cylinder-G129-Dex3-Wholebody
```

默认策略路径是相对 `UNITREE_SIM_ROOT` 的 `assets/model/policy.onnx`。预检会同时检查 `sim_main.py`、teleimager、任务模块、上述 USD/场景资产、policy、`isaaclab` 和 `unitree_sdk2py`。任何缺失都会在启动 SimulationApp 前失败；当前代码不会退回方块机器人、正弦步态、零动作或其他占位控制器。

### 6.3 启动命令

推荐使用仓库脚本：

```bash
export ISAAC_ROBOT_TYPE=g1_29dof_dex3
export G1_ASSET_DIR=/absolute/path/to/unitree_sim_isaaclab/assets
export G1_POLICY_PATH=assets/model/policy.onnx
./scripts/launch_isaac.sh
```

GUI 模式：

```bash
export ISAAC_ROBOT_TYPE=g1_29dof_dex3
export G1_ASSET_DIR=/absolute/path/to/unitree_sim_isaaclab/assets
./scripts/launch_isaac.sh --gui
```

也可以显式构建并前台运行：

```bash
docker build --network host -t vector-isaac-sim:latest docker/isaac-sim/

ISAAC_ROBOT_TYPE=g1_29dof_dex3 \
G1_ASSET_DIR=/absolute/path/to/unitree_sim_isaaclab/assets \
docker compose -f docker/isaac-sim/docker-compose.yaml up
```

查看状态与停止：

```bash
docker inspect --format='{{.State.Health.Status}}' vector-isaac-sim
docker logs -f vector-isaac-sim
./scripts/stop_isaac.sh
```

说明：

- `launch_isaac.sh --scene ...` 是现有通用/Go2 启动参数；G1 使用官方任务内固定的场景资产，不能据此认为任意 Vector scene 已注入 G1 任务。
- 脚本对 Go2 的外层健康等待默认是 180 秒，对 G1 默认是 360 秒；可用 `ISAAC_LAUNCH_TIMEOUT_SEC` 覆盖。容器内 G1 初始化上限默认是 300 秒。首次 shader 编译较慢时，应结合 `docker logs` 判断真实状态。
- `G1_NO_RENDER=true` 时必须同时令 `G1_ENABLE_CAMERAS=false`，否则预检拒绝冲突配置。
- Docker healthy 证明 29+7+7 状态和根里程计四路 DDS 流在最近的 `G1_DDS_STATE_TIMEOUT_SEC`（默认 1 秒）内均有有效更新，且机器人类型文件存在；任一路变旧都会撤销 ready 并使 health 失败。它仍不单独证明策略按命令真实运动或 Agent 目标已经收敛。

容器健康后，再在已经 source ROS 2 Jazzy 的主机进程中连接 Vector Agent。Vector CLI 的结构化工具参数为：

```json
{
  "sim_type": "g1",
  "backend": "isaac",
  "gui": false,
  "controller_mode": "whole_body",
  "profile_path": "config/robots/g1_edu_flagship_a.yaml"
}
```

`start_simulation` 不负责替用户下载官方 USD/policy，也不负责启动缺失的 Docker 容器；`G1IsaacTransport.connect()` 会等待完整 `/joint_states` 与 `/state_estimation`。

Isaac 分支仍会先在宿主机加载并校验 profile 中的 URDF、MJCF、mesh 和 SHA256，因此除容器内 USD/policy 外，宿主机也必须保留 rev-1.0 资产或设置 `VECTOR_G1_ASSET_ROOT`。相对 `profile_path` 按 CLI 进程当前目录解析，建议从仓库根运行或传绝对路径。结构化参数中的 `gui` 不会重启或改变已经运行的 Isaac 容器；GUI/headless 必须在 `launch_isaac.sh --gui` 或 Docker 环境变量中决定。

## 7. Isaac 进程、DDS、共享文件与 ROS 2 topic

### 7.1 进程拓扑

```text
主机 Vector Agent
  └─ G1IsaacTransport (ROS 2 Jazzy, domain 0)
       ⇅ ROS 2 topics over host network
容器 ros2_publisher.py (system Python 3.12, domain 0)
       ⇅ /tmp/isaac_state/*.json
容器 g1_dds_bridge.py (Isaac Python 3.11, Unitree domain 1, loopback only)
       ⇅ official Unitree DDS topics
容器 unitree_sim_isaaclab/sim_main.py + whole-body policy
```

分进程是必要的：Isaac Sim Python 3.11 与 ROS 2 Jazzy 的 Python 3.12 `rclpy` 扩展不能在同一个解释器中混用。

### 7.2 Unitree 仿真 DDS 隔离

`unitree-cyclonedds.xml` 将官方 Unitree DDS 限定为：

- domain 1；
- network interface `lo`；
- 禁用 multicast；
- peer 仅 `localhost`；
- 禁用 shared memory。

桥接进程使用的 DDS topic 是：

| 方向（相对 DDS bridge） | Topic | 内容 |
| --- | --- | --- |
| 发布 | `rt/run_command/cmd` | `[vx, -vy, -yaw_rate, height]` 策略命令 |
| 发布 | `rt/lowcmd` | 29 机身关节目标 |
| 发布 | `rt/dex3/left/cmd` | 左 Dex3 7 关节目标 |
| 发布 | `rt/dex3/right/cmd` | 右 Dex3 7 关节目标 |
| 订阅 | `rt/lowstate` | 29 机身状态和 IMU |
| 订阅 | `rt/dex3/left/state` | 左手状态 |
| 订阅 | `rt/dex3/right/state` | 右手状态 |
| 订阅 | `rt/sim_state` | 仿真根位姿/速度 |

这里的双手数组下标遵循锁定的 `unitree_sim_isaaclab` task（左右均为 thumb/middle/index），不能据此推断真机 Dex3 motor ID。`g1_manifest.py` 因此使用 `G1_*_HAND_ISAAC_TASK_JOINTS`；保留的旧短名称只用于兼容当前桥代码。未来 `G1RealTransport` 必须加载经过固件/硬件验收的独立 DDS 映射，不能复用 XML 或 Isaac 数组下标。

这条隔离是安全边界：仿真使用了与真机相同风格的 topic，不能把 Unitree 仿真 domain 改到物理机器人网络。ROS 2 domain 0 是另一条主机可见的桥接链路。

### 7.3 容器内原子文件协议

共享目录默认为 `/tmp/isaac_state`：

```text
robot_manifest.json    schema、robot_type、43 关节名、通道尺寸、运行状态
g1_command.json        主机命令的分通道快照
g1_state.json          29+7+7 状态、IMU、odom
robot_type             当前插件类型
ready                  29+7+7 与根里程计持续新鲜时存在的可撤销就绪标志
```

协议版本当前为 `schema_version=1`。命令通道为：

| 通道 | 长度 | 内容 |
| --- | ---: | --- |
| `base` | 4 | `vx, vy, yaw_rate, standing_height` |
| `body` | 29 | 桥内部完整机身快照/诊断目标；host HAL 不广告该能力 |
| `left_arm` | 7 | 左臂目标 |
| `right_arm` | 7 | 右臂目标 |
| `left_hand` | 7 | 左手 Isaac wire 顺序目标 |
| `right_hand` | 7 | 右手 Isaac wire 顺序目标 |

每个通道独立保存 `sequence`、`monotonic_ns`、`ttl_ms` 和有限数值向量，文件通过临时文件 + `os.replace` 原子更新。ROS bridge 默认给 base 命令 500 ms TTL，给关节命令 2000 ms TTL。TTL 控制命令是否能被首次接受；已接受的关节位置目标仍由后端保持，不会因为文件项过期而突然放松电机。

主机 `G1IsaacTransport.stop()` 会发布零底盘速度，并把左右臂/手目标替换为最新观测关节位置；任一发布失败会向急停调用者传播。它没有伪装成 damping：锁定的官方 task 没有阻尼命令通道，`capabilities.emergency_damping=False`，直接请求 `EMERGENCY_DAMPING` 会明确失败。G1 coordinator 的 `emergency_stop()` 在 Isaac 上表示“阻止后续命令 + zero/hold”，在 MuJoCo 上才另外进入已实现的粘性阻尼模式。

`body` 是容器桥为官方 LowCmd 数据结构保留的低层 wire 通道，不是当前 Agent/Skill 公共 API。Isaac whole-body policy 拥有腿和腰，`G1IsaacTransport.capabilities` 只开放左右臂和左右手；直接向 ROS `body` topic 发布会绕过 HAL 的 capability、仲裁和限位检查，不属于受支持的控制路径。

### 7.4 主机可见的 ROS 2 topic

| Topic | 消息类型 | 方向（相对容器 ROS bridge） |
| --- | --- | --- |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 订阅 |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 订阅 |
| `/state_estimation` | `nav_msgs/msg/Odometry` | 发布 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 发布 43 关节 |
| `/g1/body/joint_states` | `sensor_msgs/msg/JointState` | 发布 29 关节 |
| `/g1/left_arm/joint_states` | `sensor_msgs/msg/JointState` | 发布 7 关节 |
| `/g1/right_arm/joint_states` | `sensor_msgs/msg/JointState` | 发布 7 关节 |
| `/g1/left_hand/joint_states` | `sensor_msgs/msg/JointState` | 发布 7 关节 |
| `/g1/right_hand/joint_states` | `sensor_msgs/msg/JointState` | 发布 7 关节 |
| `/g1/body/joint_commands` | `std_msgs/msg/Float64MultiArray` | 订阅 29 维低层桥通道；`G1IsaacTransport` 不暴露直接 body 控制 |
| `/g1/left_arm/joint_commands` | `std_msgs/msg/Float64MultiArray` | 订阅 7 维 |
| `/g1/right_arm/joint_commands` | `std_msgs/msg/Float64MultiArray` | 订阅 7 维 |
| `/g1/left_hand/joint_commands` | `std_msgs/msg/Float64MultiArray` | 订阅 7 维 |
| `/g1/right_hand/joint_commands` | `std_msgs/msg/Float64MultiArray` | 订阅 7 维 |

还保留 `/arm/joint_states` 和 `/arm/joint_commands` 的旧 6-DoF 兼容 topic；G1 新代码不应使用它们，因为它们会根据 `G1_LEGACY_ARM_SIDE` 丢弃/补齐第七个关节。

虽然 `ros2_publisher.py` 创建了 `/registered_scan`、`/camera/image` 和 `/camera/depth` publisher，当前 G1 路径没有为它们建立实际发布 timer/数据通道；因此 `G1IsaacTransport.capabilities.lidar=False`，也不能把 `G1_ENABLE_CAMERAS=true` 理解为 Vector Agent 已收到相机帧。当前 `/tf` 中的 sensor offset 也是通用占位值，不是 G1 标定外参。

主机检查命令：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 topic list
ros2 topic hz /state_estimation
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

## 8. Pinocchio 双臂运动学

`PinocchioG1Kinematics(profile)` 的实际范围是：

- solver 类首次调用 FK/IK 时才 import `pinocchio` 并加载 profile URDF；`SimStartTool._start_g1()` 会在连接 transport 前主动对左右臂各做一次零位 FK，从而在 CLI 启动阶段暴露依赖、URDF、关节名或 TCP 错误；
- 使用无 `JointModelFreeFlyer` 的固定 pelvis 模型；
- 左右臂各使用 profile 中精确的 7 个关节；
- TCP 是 `left_hand_palm_link` 或 `right_hand_palm_link`；
- FK 返回 pelvis 坐标系中的三维位置和 `3×3` 旋转矩阵；
- IK 是 position-only 的阻尼最小二乘；
- 其他关节和腰部保持 URDF neutral；
- 每次迭代裁剪到保守 URDF 关节限位；
- 数值失败、停滞或迭代后仍不收敛时返回 `None`。

它当前**不提供**：

- 末端姿态约束；
- 腰部、腿部或移动底座参与的全身 IK；
- 自碰撞、环境碰撞、抓取接触或奇异位形路径规划；
- 动态、力矩、负载、稳定裕度或时间最优轨迹；
- 世界/相机坐标到 pelvis 坐标的标定转换。

因此它可用于当前 `ArmProtocol` 的短程位置目标和固定基座验证，但不足以单独保证“开门”“拉冰箱门”等接触任务安全完成。需要 `pin>=3.9.0`；单独构造 solver 本身不会强制加载依赖，第一次 FK/IK 才会报告缺失或 ABI 不兼容。通过 `start_simulation` 启动 G1 时，上述 FK 预检会把错误提前到启动阶段。

真实 rev-1.0 URDF 的当前烟雾验证结果：

```text
left  zero-arm FK = [0.241275,  0.151654, 0.095231]
right zero-arm FK = [0.241275, -0.151644, 0.095231]
两侧向 +Z 移动 1 cm 的 IK 末端误差 = 4.097e-05 m
远端目标 (10, 10, 10) -> None
```

## 9. Agent 与 Skill 的双臂、双手选择

### 9.1 组合一个 G1 Agent

下面的代码使用与 `SimStartTool._start_g1()` 相同的 `G1Profile/G1Robot/Agent` 组合骨架，以 MuJoCo 为例；为简洁起见没有合并 `config/user.yaml`：

```python
from pathlib import Path
import yaml

from vector_os_nano.core.agent import Agent
from vector_os_nano.hardware.g1 import (
    G1Profile,
    G1Robot,
    PinocchioG1Kinematics,
)
from vector_os_nano.hardware.sim.g1_mujoco_transport import MuJoCoG1Transport
from vector_os_nano.skills.g1 import get_g1_skills

profile_path = Path("config/robots/g1_edu_flagship_a.yaml")
deployment = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
profile = G1Profile.from_yaml(profile_path)

transport = MuJoCoG1Transport(
    profile,
    gui=False,
    fixed_base_manipulation=True,
)
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
    config={
        "skills": deployment.get("skills", {}),
        "simulation": deployment.get("simulation", {}),
    },
)

try:
    agent.connect()
    # 使用 Agent 的通用具名 Skill。
    assert agent.execute_skill("home", {"arm": "left"}).success
    assert agent.execute_skill("wave", {"arm": "right"}).success
    assert agent.execute_skill("gripper_open", {"hand": "left"}).success
    assert agent.execute_skill("gripper_close", {"hand": "right"}).success
finally:
    agent.disconnect()
```

Isaac 只需在容器健康、主机 ROS 2 已 source 的前提下，把 transport 替换为：

```python
from vector_os_nano.hardware.sim.g1_isaac_transport import G1IsaacTransport

transport = G1IsaacTransport(profile)
```

### 9.2 硬件层明确选择左右臂/手

```python
robot.arms["left"].move_joints(
    [0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0],
    duration=2.0,
)
robot.arms["right"].move_joints(
    [0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0],
    duration=2.0,
)

robot.hands["left"].pinch(duration=0.8)
robot.hands["right"].power_grasp(duration=1.0)
robot.hands["right"].set_joint_positions(
    [-0.2, -0.5, -1.0, 0.8, 1.0, 0.6, 0.8],
    duration=0.5,
)
```

所有 7 维目标都经过维度、有限值和 URDF 限位检查。

### 9.3 直接执行 Dex3 Skill

Vector CLI 的 Skill wrapper 会直接执行无 `auto_steps` 的 `Dex3PoseSkill`。Python 中可通过公开 `build_context()` 做同样的事：

```python
from vector_os_nano.skills.g1.dex3 import Dex3PoseSkill

context = agent.build_context()

result = Dex3PoseSkill().execute(
    {"side": "left", "preset": "pinch", "duration": 0.8},
    context,
)
assert result.success

result = Dex3PoseSkill().execute(
    {
        "side": "right",
        "positions": [-0.2, -0.5, -1.0, 0.8, 1.0, 0.6, 0.8],
        "duration": 0.5,
    },
    context,
)
assert result.success
```

`side` 对 `dex3_pose` 是必填项；`preset` 和 `positions` 必须二选一。通用 arm Skill 使用 `arm="left"|"right"`，通用 gripper Skill 使用 `hand="left"|"right"`。参数省略时 profile 默认选择右侧。

### 9.4 当前 Skill 能力边界

`get_g1_skills()` 注册：

```text
walk, turn, stand, navigate, where_am_i, stop, emergency_stop, dex3_pose
```

Agent 的默认 Skill 还包括 `home`、`scan`、`wave`、`handover`、`pick`、`place` 等；G1 YAML 为左右 7-DoF 臂提供了相应 pose、handover 方向姿态、pick workspace、默认 `hold` 模式和按臂 drop pose。`pick/place/home/scan/wave/handover` 接受 `arm="left"|"right"`，gripper Skill 接受 `hand="left"|"right"`。具名 `pick` 的前置 `scan` 和自动 `home` 会继承同一侧；当 profile 默认或调用参数选择 `mode="hold"` 时，Agent 不会追加会张开该手的 `home`。需要操作手的 Skill 会在动作前检查同侧 gripper/hand 和后端 capability，registry 侧别错配也会失败。当前测试证明的是左右路由、向量维度、阶段返回值和 capability 失败语义，不等价于所有 Skill 已在 G1 + Isaac 场景中端到端成功。

当前 WorldModel 是明确的**单持物模型**：`held_object` 同时记录 `held_by="left"|"right"`。非持物侧执行 `home`、`handover`、`place` 或 `gripper_open` 会在运动前以 `wrong_limb` 失败，不能清掉另一只手的物体；已有持物时直接发起第二次 `pick` 会以 `already_holding` 失败。这保证阶段一不会跨手错误释放，但还不支持左右手同时各持一个物体。硬件层仍可独立或并行发送两只手的关节目标；若后续 Agent 需要双物体/双手协作，应把 WorldModel 扩展为左右手各自的状态与持物谓词。

特别是：

- MuJoCo 的 `walk`、`turn`、`stand`、`navigate` 会因为没有 locomotion policy 而失败。
- `stop` 是普通最佳努力停止请求；`emergency_stop` 会先在 coordinator 锁住全部后续命令，再请求后端最强的停止路径，且不会自动追加 arm `home`。MuJoCo 实现真实仿真粘性阻尼；Isaac 因官方 task 没有 damping 通道，只执行零底盘速度和四个臂/手组的当前位置保持，并在发布失败时返回失败。必须由操作者显式调用 `robot.clear_emergency()` 或 `robot.base.clear_emergency()` 恢复；连接、断开和自然语言指令都不会自动解锁。以上软件路径都不替代真机物理急停。
- Isaac 的 `walk`/`turn` 依赖尚待 live 验收的官方 policy。
- `navigate` 已复用通用 Skill；其 dead-reckoning fallback 会检查转向/前进返回值，并要求最终 XY 里程计进入到达半径，不能把被拒绝或无位移的命令报告为到达。但当前 G1 Agent 没有自动注入 G1 专用地图、LiDAR、`spatial_memory` 或 navigation service；没有它们时仍不具备可靠避障导航。
- 只有通用 `pick/place` 可调用当前 G1 的 position-only IK；依赖 `ik_top_down` 的 `PickTopDownSkill`、`PlaceTopDownSkill` 和 `PerceptionGraspSkill` 目前不支持 G1。
- G1 `pick/place` 的 Cartesian 目标必须已经位于固定 `pelvis` 坐标系。通用 Skill 文档中的 “base frame” 尚未为 G1 实现 `base_link → pelvis` 显式变换，也没有 G1 相机标定、掌心姿态约束、碰撞检查和场景接触闭环，因此不能宣称真实抓取/放置已经完成。
- `G1Hand.close()` 成功只表示关节命令被接受；当前 Isaac/MuJoCo transport 没有可靠填充 `hand_holding`/触觉/抓力，`pick` 也不能据此证明物体已抓牢。物理抓持与释放仍须由场景接触、attach 状态或真实传感器作为独立验收证据。
- 尚无 `open_door`、`open_fridge` 等 articulated-object Skill，也没有门把手/冰箱把手的 affordance、关节状态和接触反馈。

## 10. 测试与验收

### 10.1 推荐开发依赖

```bash
python -m pip install -e ".[sim,ik,dev]"
```

ROS 2 Jazzy/rclpy 由系统 apt 环境提供，不在 pip extra 中。

### 10.2 资产/profile 校验

```bash
export VECTOR_G1_ASSET_ROOT=/absolute/path/to/g1_description

python - <<'PY'
from vector_os_nano.hardware.g1 import G1Profile

profile = G1Profile.from_yaml("config/robots/g1_edu_flagship_a.yaml")
profile.validate_assets()
print(profile.profile_id)
print(len(profile.all_joint_names))
print(profile.urdf_path)
print(profile.mjcf_path)
PY
```

期望关节数量为 `43`，且两份 SHA256 校验通过。

### 10.3 当前 G1 回归命令

```bash
python -m pytest -q \
  tests/unit/hardware/g1/test_g1_hardware.py \
  tests/unit/test_agent_hardware_registries.py \
  tests/unit/test_g1_skills.py \
  tests/unit/test_g1_skill_profiles.py \
  tests/unit/test_g1_sim_start.py \
  tests/unit/test_isaac_g1_bridge.py \
  tests/hardware/sim/test_g1_mujoco_transport.py
```

本次使用用户提供的 rev-1.0 URDF/MJCF 资产完成分环境验证。G1 契约集合在系统测试环境中为 `130 passed, 2 skipped`；两个 skip 分别由该解释器缺少 Pinocchio 和 MuJoCo 引起。随后在包含相应原生依赖的环境中补跑两部分，最终等价结果为：

```text
137 passed
```

其中单独执行 `tests/hardware/sim/test_g1_mujoco_transport.py` 的结果为
`6 passed`，真实 URDF 的 Pinocchio 测试为 `1 passed`。实际资产烟雾检查包括：

- MJCF 返回 43 个语义关节，根高度 `0.8 m`，未判定跌倒；
- G1 `ArmProtocol.move_joints()` 会阻塞等待状态收敛；当前右臂烟雾目标在 `0.08 rad` 阈值内返回，实测最大误差约 `0.0760 rad`；
- 左右臂的逐关节轨迹可同时进行，不会因后发 group 命令互相取消；
- 非零 locomotion 命令明确抛出 `NotImplementedError`；
- 左右 Pinocchio `+Z 1 cm` 位置 IK 误差均为 `4.097e-05 m`；
- 不可达目标返回 `None`；
- MuJoCo emergency damping 保持锁存，显式恢复后才重新接受关节命令；
- Isaac 明确拒绝伪造 damping，zero/hold 发布失败不会被吞掉。

此外，将无重复的 G1、Agent 执行、技能/WorldModel 和导航回归集合合计，结果为 `499 passed`。仓库旧的
`tests/integration/test_agent.py` 仍针对当前 `HEAD` 已不存在的 LLM 模块、
SessionMemory 与 `Agent.execute()` API，因此不属于本次 G1 验收集合。

### 10.4 MuJoCo 验收标准

至少应满足：

1. profile 和两份 SHA256 通过；
2. `nq/nv/nu == 50/49/43`；
3. 43 个 actuator 名称顺序完全匹配 rev-1.0；
4. `/left_arm`、`/right_arm`、两只手互不串组；
5. 不对称手指向量经过往返映射后 index/middle 不交换；
6. 所有命令拒绝 NaN、错误维度、错误顺序和越限值；
7. 固定基座操作保持 root `(0,0,0.8)`；
8. 非零速度请求明确失败，不能报告行走成功。

### 10.5 Isaac live 验收标准

目前尚未执行，拿到官方资产后按顺序检查：

1. `g1_isaac_runtime.py` preflight 找到唯一支持任务、一个机器人 USD、三个场景/物体 USD 和 policy；
2. 容器 health 变为 `healthy`，manifest `status=ready`；该 ready 要求 body、双手和根里程计四路状态持续新鲜，任一路超过默认 1 秒会被撤销；
3. `g1_state.json` 连续收到 29 + 7 + 7，且 ROS `/joint_states` 有 43 个唯一名字；
4. `/state_estimation` 持续更新且 host transport 不报 stale state；
5. 对左右臂分别发送小幅、限位内目标，确认另一侧和腿/腰未被错误覆盖；
6. 对双手发送不对称 7 维目标，确认 index/middle 映射正确；
7. 在空旷场景以低速测试前后、横移、转向和零速停止，记录根轨迹、跌倒状态和策略频率；
8. 验证命令过期、序号回退、容器进程退出和 host disconnect 均 fail closed；
9. 确认 Unitree domain 1 只在容器 loopback，不出现在真机网卡；
10. 完成以上项目后，才能把 Isaac locomotion 从“代码声明”提升为“实测能力”。

## 11. 用户后续还需要提供的内容

### 11.1 Isaac 官方资产与版本证据

- 完整 `unitree_sim_isaaclab/assets` 目录，而不只是 URDF/MJCF/mesh；
- `g1_29dof_with_dex3_rev_1_0.usd`；
- 与当前 pin 住任务严格匹配的 `policy.onnx`；
- small warehouse 和两套 PackingTable USD；
- 上述文件的来源、许可、版本/commit 和 SHA256；
- 若官方资源布局不同，提供上游任务配置，不能只改文件名绕过预检。

### 11.2 策略与控制契约

- policy 的 observation/action 定义、关节顺序和历史帧长度；
- action scale、default joint pose、normalization 参数；
- policy/control/physics 频率与 decimation；
- 支持的 `vx/vy/yaw/height` 范围；
- 站立初始化、恢复、跌倒检测和 emergency damping 语义；
- policy 是否拥有腕部/手臂，以及外部 arm target 如何与 whole-body policy 合并；
- 策略训练时使用的 USD、质量、惯量、摩擦和 actuator 参数。

没有这些信息，不能判断“能加载 ONNX”是否等于“与该 G1 embodiment 控制契约匹配”。

### 11.3 传感器和外参

针对实际购买配置，需要逐项提供：

- RGB/RGB-D/鱼眼相机型号、分辨率、帧率、内参和畸变；
- LiDAR 型号、点云 topic、时间戳和扫描频率；
- IMU、足底力/接触、腕部力矩和 Dex3 触觉/电流能力；
- 每个传感器相对 `pelvis`/`base_link`/手掌的 6D 外参；
- ROS frame 名、时间同步方式和标定文件；
- Isaac 中对应的 sensor USD prim、噪声模型和 update rate。

当前 `/tf` 的通用 sensor offset 不是标定结果，必须在进入导航/抓取验收前替换。

### 11.4 交互场景资产

为了实现“遇到门开门、遇到冰箱开冰箱”，还需提供：

- 可交互门和冰箱的 USD/MJCF/URDF，而不是单一静态 mesh；
- 门铰链、冰箱铰链/抽屉 prismatic joint、轴向、限位、阻尼和初始状态；
- 把手的 collision geometry、抓取 frame 和可达方向；
- 质量、惯量、摩擦、闭门器/磁吸力等接触参数；
- 场景语义 ID、房间/门连接图、导航网格或地图；
- 任务成功定义，例如门角度、冰箱开度、是否保持抓持；
- 允许左手、右手或双手协作的任务规则。

此后还需要单独实现并验收对象检测/定位、预抓取、接触建立、约束轨迹、开合状态观测、失败恢复和释放等 Skill。

### 11.5 真机网络与安全信息

当前仓库没有真机 `G1Transport`。实现前至少需要：

- G1 firmware 和 `unitree_sdk2_python`/SDK2 兼容版本；
- 控制 PC 网卡名、机器人 IP/网段、是否直连、允许的防火墙规则；
- 真机 DDS domain、network interface 和官方 topic/IDL 版本；
- `mode_machine=5` 在该 firmware 上的进入/退出流程；
- 29 机身 motor ID 和两只 Dex3 motor ID 的真机顺序确认；
- 控制权租约、低层/高层模式切换、watchdog 和命令频率要求；
- 硬件急停、软件 damping、断网、进程崩溃和电源恢复 SOP；
- 出厂零位、关节偏置、手指方向、负载和限位校准；
- 允许测试的场地、安全员、吊装/防跌倒装置和分阶段速度/力矩限制。

仿真 `emergency_stop()` 只是软件模式/停止请求，不可替代经过验证的物理急停。真机 transport 也不能复用容器中 domain 1 的仿真 DDS 配置；两者必须保持物理网络隔离。

## 12. 建议的后续实施顺序

1. 提供并哈希固定官方 Isaac USD、场景资产和 `policy.onnx`。
2. 在隔离网络上完成第 10.5 节的 Isaac live 验收，先空场低速，再做双臂/双手。
3. 接入并标定 G1 实际相机、LiDAR、IMU/接触数据，删除占位外参。
4. 为 G1 注入地图、SceneGraph 和 navigation service，再验收交互式导航。
5. 导入可关节化门/冰箱资产，先固定基座单臂开合，再做移动操作和双臂协作。
6. 若需要 MuJoCo 行走，提供与该 MJCF 匹配的全身策略/控制器，验收后才把 `locomotion=True`。
7. 最后实现真机 transport；先只读状态，再分组关节控制，再站立/低速行走，始终保留物理急停和网络隔离。
