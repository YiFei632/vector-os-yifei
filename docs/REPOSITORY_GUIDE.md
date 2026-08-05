# Vector OS Nano 仓库详解

> 文档性质：用户明确要求创建的非规范性代码导读。它用于解释当前仓库“每一部分做什么”，不替代 [`CLAUDE.md`](../CLAUDE.md)、[`ARCHITECTURE.md`](ARCHITECTURE.md) 或 [`agent-kernel-STATUS.md`](agent-kernel-STATUS.md)。
>
> 审阅基线：`master`，提交 `cd7029a`；审阅日期：2026-07-14。
>
> 判定原则：代码优先于注释，实际入口优先于设计文档，真实运行链优先于目录或文件名所暗示的能力。

## 目录

1. [一页读懂 Vector OS Nano](#1-一页读懂-vector-os-nano)
2. [仓库全景与核心对象](#2-仓库全景与核心对象)
3. [启动过程与运行时生命周期](#3-启动过程与运行时生命周期)
4. [Native、VGG 与 ReAct 三种执行机制](#4-nativevgg-与-react-三种执行机制)
5. [确定性验证与证据链](#5-确定性验证与证据链)
6. [`vector_os_nano/` 主包逐部分详解](#6-vector_os_nano-主包逐部分详解)
7. [典型端到端数据流](#7-典型端到端数据流)
8. [仿真、ROS2 与外部导航系统](#8-仿真ros2-与外部导航系统)
9. [配置、脚本、部署与辅助资产](#9-配置脚本部署与辅助资产)
10. [状态、持久化与生成物](#10-状态持久化与生成物)
11. [依赖、安装与打包边界](#11-依赖安装与打包边界)
12. [测试体系与验收门](#12-测试体系与验收门)
13. [安全与信任边界](#13-安全与信任边界)
14. [当前成熟度与技术债务](#14-当前成熟度与技术债务)
15. [如何扩展仓库](#15-如何扩展仓库)
16. [现有文档与推荐阅读顺序](#16-现有文档与推荐阅读顺序)
17. [附录：目录、入口和术语速查](#17-附录目录入口和术语速查)

---

## 1. 一页读懂 Vector OS Nano

### 1.1 它是什么

Vector OS Nano 是一个面向物理智能的 Agent 编排运行时。它试图把下列能力接到同一条可验证执行链上：

- 通用语言模型；
- 专用小模型，例如开放词汇检测器；
- 经典机器人技能，例如 IK 抓放和关节控制；
- 导航、探索、感知和外部 ROS2 系统；
- 真机或多个仿真后端；
- 确定性的世界状态验证与失败恢复。

项目的 North Star 可以压缩为：

~~~text
自然语言
  → 规划
  → 把每一步路由给正确的模型、技能或工具
  → 执行长链任务
  → 用真实世界谓词验证每一步
  → 失败时重试或重新规划
  → 只有证据充分时才宣布完成
~~~

这也是为什么仓库最有辨识度的部分不是某一个导航器或抓取算法，而是 `ExecutionTrace → Evidence Gate → Verdict` 这条“诚实验证”链。

### 1.2 它不是什么

它不是：

- 单独的机械臂 SDK；
- 一个全新的导航算法实现；
- 一个训练或微调框架；
- 已经稳定发布的跨机器人操作系统；
- 自包含的 FAR、TARE、Nav2 或 SysNav 分发包；
- 仅凭 Skill 返回 `success=True` 就认定任务完成的脚本集合。

仓库尽量编排外部导航和感知能力，而不是重新实现所有底层算法。当前状态更接近研究型 Alpha monorepo：核心机制和 MuJoCo 路径较深，部署、可移植性和多条演进路径尚未完全收口。

### 1.3 当前真正支持的前端、模型、机器人和仿真器

| 类别 | 当前代码中的实际能力 |
|---|---|
| 用户前端 | `vector-cli`、`vector-os-mcp`、`vector-eval`、结构化 Python SDK |
| 聊天模型 | Anthropic API；OpenAI-compatible API，包括 OpenRouter、Ollama、vLLM 等 |
| 专用模型 | Grounding DINO 检测器；Moondream/Go2 VLM；EdgeTAM 分割跟踪 |
| 真机 | SO-101 机械臂与夹爪；RealSense D405 |
| 主仿真 | MuJoCo SO-101、MuJoCo Go2、Go2+Piper；G1 rev-1.0 固定基座操作后端 |
| 遗留仿真 | PyBullet 桌面机械臂 |
| 实验后端 | Gazebo Harmonic；Isaac Sim（Go2 与 G1 29-DoF + 双 Dex3 插件），经 ROS2 proxy 接入 |
| 外部系统 | FAR、TARE、Nav2、SLAM、SysNav，需要仓库外 ROS2 工作区 |

“跨模型”当前最具体的落地是聊天 LLM 加上可被 producer 路由的 Grounding DINO 检测器。G1 已从纯路线图推进到 profile/HAL、双臂双手路由、固定 pelvis 运动学、MuJoCo 固定基座控制和 Isaac bridge 契约；VLA 和更完整的模型 zoo 仍主要是架构接口或路线图。

### 1.4 成熟度摘要

| 状态 | 代表能力 |
|---|---|
| 已有较完整实现 | Agent Kernel、World/Skill/Tool 注册、VGG 验证、SO-101、MuJoCo 桌面臂、MuJoCo Go2、Go2+Piper、MCP 桌面臂 |
| 部分落地 | Go2 导航、TARE/FAR 集成、感知抓取、SysNav 物体同步、ROS2 proxy、G1 固定基座操作基线 |
| 实验后端 | Gazebo、Isaac Sim、完整 ROS2 launch；G1 Isaac 尚待官方 USD/policy live 验收 |
| 路线图 | G1 真机/传感器/动态行走与交互场景、通用 VLA、完整跨 embodiment 切换、稳定的 nav+grasp 产品链 |

最新状态记录明确指出：nav+grasp 的机械链、感知精度和 docking 已推进到较深阶段，但抓取仍会因可达性/站位问题间歇失败，尚不能称为可靠落地。

---

## 2. 仓库全景与核心对象

### 2.1 顶层结构

~~~text
vector-os-nano/
├── vector_os_nano/       Python 主包
│   ├── core/             结构化 SDK、状态和确定性 TaskExecutor
│   ├── vcli/             Agent Kernel、CLI、工具、VGG、World
│   ├── skills/           机械臂、Go2、导航和移动操作技能
│   ├── perception/       VLM、检测、分割、RGB-D、点云和标定
│   ├── hardware/         HAL、真机、MuJoCo、ROS2 proxy、模型资产
│   ├── playground/       预设场景 World 插件
│   ├── mcp/              MCP tools/resources/server
│   ├── ros2/             ROS2 节点和目标 launch 拓扑
│   └── integrations/     外部系统适配，例如 SysNav
├── config/               默认配置、导航参数、房间拓扑和 RViz
├── scripts/              启动、停止、桥接、smoke、诊断和研究探针
├── gazebo/               Gazebo 世界、模型、传感器和 launch
├── docker/isaac-sim/     Isaac Sim GPU 容器和 ROS2 文件桥
├── foxglove/             Foxglove bridge 与 dashboard
├── tests/                unit/vcli/harness/integration/skills/hardware/e2e
├── docs/                 架构、状态、决策和子系统文档
├── examples/             SDK 使用示例，部分已过时
├── images/               README 和演示素材
├── pyproject.toml        Python 打包、依赖组、入口和 pytest 配置
├── sim.sh                `vector-cli --sim` 薄包装
├── .mcp.json             当前开发机 MCP 示例配置
└── LICENSE / NOTICE      Apache-2.0 与第三方声明
~~~

### 2.2 三个最容易混淆的对象

#### `VectorEngine`：语言和工具编排器

位置：`vector_os_nano/vcli/engine.py`。

它负责：

- 调用 LLM backend；
- 维护模型—工具循环；
- 注册并筛选工具；
- 初始化 VGG；
- 绑定当前 World 的验证命名空间；
- 调度 Native、VGG 和 ReAct；
- 输出统一的 turn、trace、snapshot 和 verdict。

它才是自然语言层面的主要“大脑”。

#### `World`：领域适配器

位置：`vector_os_nano/vcli/worlds/`。

World 让同一个 Kernel 在不同领域中运行。它提供：

- persona 与提示块；
- 领域工具；
- 可供验证器调用的真实世界谓词；
- GoalDecomposer 能使用的词表；
- 可路由的模型 capability；
- 是否从 SkillRegistry 自动派生词表的策略。

当前主要 World 是：

- `DevWorld`：无机器人、面向代码和文件操作；
- `RobotWorld`：SO-101、Piper、Go2；
- `PlaygroundWorld`：命名场景和场景级 oracle。

#### `core.Agent`：硬件与服务容器

位置：`vector_os_nano/core/agent.py`。

它组装：

- arm；
- gripper；
- mobile base；
- perception；
- SkillRegistry；
- WorldModel；
- TaskExecutor；
- IK、标定和上下文服务。

它的公共执行接口是 `execute_skill(skill_name, params)`。构造参数中的旧 LLM 字段仅为兼容而保留，当前不会让 `core.Agent` 自己完成 NL 规划。

### 2.3 Skill、Tool、Primitive 与 Capability

| 概念 | 典型用途 | 示例 |
|---|---|---|
| Skill | 领域级、结构化机器人动作 | `pick`、`walk`、`explore` |
| Tool | Agent 可调用的通用或运维动作 | `file_read`、`bash`、`sim_start` |
| Primitive | 给受限代码或规划器使用的原子函数 | `set_velocity`、`get_position` |
| Capability | 可路由的带类型模型/算法能力 | chat LLM、Grounding DINO detector |
| Verify predicate | 只读、确定性的完成条件 | `holding_object(...)`、`at_position(...)` |

这些概念有重叠，但信任边界不同：Skill 和 Tool 可以产生副作用；verify predicate 必须只读；Capability 只能报告运行结果，不能自行宣布最终验证通过。

### 2.4 关键数据结构

- `TaskPlan / TaskStep`：`core.TaskExecutor` 使用的较早结构化执行计划；
- `GoalTree / SubGoal`：VGG 使用的冻结 DAG；
- `Blackboard`：保存每一步的结构化输出；
- `StepRecord`：记录策略、执行、验证、因果和结果数据；
- `ExecutionTrace`：一轮任务的完整、可重放记录；
- `VerdictReport`：面向 CLI/评测的机器可读结果。

---

## 3. 启动过程与运行时生命周期

### 3.1 `vector-cli` 启动顺序

`vector_os_nano/vcli/cli.py` 的主要启动流程是：

1. 解析参数；
2. 解析 LLM 凭据和模型；
3. 根据 `--sim`、`--sim-go2` 或无 flag 构造 Agent；
4. 选择当前 World；
5. 发现通用工具并按类别注册；
6. 将 Agent 中每个 Skill 包装为 Tool；
7. 构造 PermissionContext；
8. 创建或恢复 Session；
9. 构造动态系统提示和 RobotContext；
10. 创建 LLM backend 与 VectorEngine；
11. 调用 `engine.init_vgg(...)`；
12. 进入 REPL 或执行一次性 `-p` turn。

### 3.2 无机器人、桌面臂和 Go2 模式

#### 无 flag

- `_init_agent()` 返回 `None`；
- WorldRegistry 选择 `DevWorld`；
- robot/diag/system 类工具被禁用；
- sim 类工具仍保留，因此用户可以在对话中启动仿真。

#### `--sim`

- 构造 `MuJoCoArm`；
- 构造 `MuJoCoGripper`；
- 构造直接读取仿真真值的 `MuJoCoPerception`；
- 将三者注入 `core.Agent`；
- 选择 `RobotWorld`。

这是桌面 SO-101 仿真路径。

#### `--sim-go2`

- 构造带房间场景的 `MuJoCoGo2`；
- 启动后台物理线程并站立；
- 单独注册 Go2 Skill；
- 可选创建 Go2 VLM；
- 加载 SceneGraph 持久化数据；
- 尝试启动 ROS2 bridge 和外部导航栈。

这条路径同时承担较多系统集成工作，也是对环境依赖最重的 CLI 模式。

### 3.3 World 选择

优先级是：

1. 用户显式传入 `--scenario ID`：加载 `playground` 并按名称解析；
2. 存在 Agent：选择 `RobotWorld`；
3. 无 Agent：选择 `DevWorld`。

未知场景会列出有效集合并明确报错，不会静默退到其他 World。

### 3.4 LLM backend、配置和认证

`vcli/backends/` 提供统一 `LLMBackend` 协议：

- `anthropic.py`：Anthropic SDK、streaming 和 prompt cache；
- `openai_compat.py`：OpenAI Chat Completions 兼容格式；
- `types.py`：统一响应、tool call 和 token 类型；
- `text_llm_adapter.py`：将完整 backend 缩成 SceneGraph 需要的窄文本协议。

`vcli/config.py` 负责：

- `~/.vector/config.yaml`；
- provider/model/base URL；
- API key；
- Claude Code/OAuth 凭据发现；
- CLI、环境变量和持久化配置之间的优先级。

`vcli/oauth.py` 实现 Anthropic OAuth Authorization Code + PKCE 流。它属于 CLI 认证能力，而不是机器人内核本身。

README 所说的“默认 DeepSeek”是有条件的凭据选择，不应理解为所有环境中写死的唯一默认模型。

### 3.5 Prompt 和实时机器人上下文

`prompt.py` 生成：

- Dev 或 Robot persona；
- 工具使用说明；
- 当前工作目录和项目上下文；
- Skills、WorldModel 和硬件状态。

`dynamic_prompt.py` 在每次 LLM 调用前刷新动态块；`robot_context.py` 从 base、SceneGraph、导航和硬件提取实时上下文。因此长 Session 不需要一直使用启动时的过期机器人状态。

### 3.6 Session 和运行时持久化

`session.py` 使用 JSONL 保存：

- user 消息；
- assistant 文本和 tool-use；
- tool results；
- token usage；
- Session metadata。

默认数据位于 `~/.vector/` 下。CLI 支持恢复最近 Session、清空、压缩和导出。

### 3.7 并发、后台任务和停止

- ReAct 中只读工具可以并行执行；
- 有副作用的工具串行执行；
- Go2 MuJoCo 使用后台物理线程；
- ExploreSkill 使用可取消的后台探索线程；
- ROS2 proxy 使用共享的 MultiThreadedExecutor；
- macOS MuJoCo viewer 可能要求主线程 pump；
- `vcli/cognitive/abort.py` 提供跨 VGG、导航和探索的全局 abort signal；
- 精确的 stop 词会走硬编码急停路径，避免等待 LLM。

这些机制解释了为什么“停止仿真”“停止机器人”“中断当前 VGG turn”和“结束后台探索”并不是完全相同的生命周期动作。

---

## 4. Native、VGG 与 ReAct 三种执行机制

三者不是三个对等前端，而是分层路由关系。

### 4.1 交互式裸 REPL

对用户输入先进行廉价 intent 分类：

- action-shaped 输入：默认先尝试 Native；
- Native 只要调度了一个 action，就拥有本轮；
- Native 零动作或失败时，继续进入 Unified；
- Unified 对 action 走 VGG；
- Unified 对 chat/tool-use 走 ReAct，并包装为 answer-only trace。

`VECTOR_REPL_NATIVE=0` 可以关闭交互式 Native 首选。`VECTOR_LEGACY_TURN=1` 可以恢复更早的开放 ReAct 分支。

### 4.2 一次性 `vector-cli -p`

这里与裸 REPL 不同：

- 无 flag：默认直接走 `vgg_decompose → vgg_execute`；
- `--native-first`：先 Native，零动作时回退 VGG；
- `--native-loop`：只走 Native，不做 VGG 回退；
- `--json`：输出单条机器可读 `VECTOR_VERDICT`。

因此不能用 `-p` 默认行为推断交互式 REPL 的默认行为。

### 4.3 MCP

MCP 当前不调用 Native：

- `natural_language`：进入 `run_turn_unified`；
- “直接 Skill MCP tool”：先转换成一条自然语言指令，再进入 `run_turn_unified`；
- `run_goal`：直接调用 VGG decompose/execute；
- `VECTOR_LEGACY_TURN=1`：才回旧 `run_turn`。

也就是说，MCP 中看到一个名为 `pick` 的 tool，不代表它直接调用 `Agent.execute_skill("pick")`。

### 4.4 Native loop

Native 给模型一个较窄的 action surface：

- 当前 World 的机器人 Skill；
- code 类 Tool；
- 有 mobile base 时的坐标 `navigate`；
- 合成的 `verify(expr)`；
- 合成的 `finish`。

模型自行决定：

1. 调哪个 action；
2. action 之间如何组合；
3. 何时验证；
4. 验证失败后是否重试；
5. 何时结束。

`NativeStepRunner` 把每个“action chain → verify”对转换成一个 `StepRecord`，最终产生与 VGG 相同类型的 `ExecutionTrace`。Native 自己不计算 verified，仍由共同的 evidence/verdict 层判定。

Native 的优点是把分解和恢复更多交给前沿模型；代价是它绕开了 VGG 的显式 GoalTree、StrategySelector 和 Harness。

### 4.5 VGG 计划闭环

VGG 的链路是：

~~~text
自然语言
  → GoalDecomposer
  → GoalTree / SubGoal DAG
  → StrategySelector
  → GoalExecutor
  → Blackboard 捕获结构化结果
  → GoalVerifier
  → VGGHarness 重试或重规划
  → ExecutionTrace
  → Evidence Gate
~~~

关键特征：

- 简单 Skill/alias 可以走无 LLM 快路径；
- 复杂任务由 LLM 输出受校验的 JSON GoalTree；
- SubGoal 可以显式依赖前序步骤；
- `foreach` 可在运行时展开；
- 后续参数可以使用 `${step.output.path}`；
- verify 表达式必须通过 AST 校验；
- 策略可以是 skill、primitive、code、tool 或 capability；
- 失败会记录类型和上下文供重规划使用。

VGG 当前仍然是 `-p` 默认、MCP `run_goal` 和 Unified action 路径，不能简单称为“已弃用代码”。

### 4.6 ReAct 工具循环

`VectorEngine.run_turn()` 是传统的：

~~~text
用户消息
  → LLM
  → tool calls
  → 权限检查
  → 执行工具
  → tool results 回写 Session
  → 再调用 LLM
  → 直到没有 tool call
~~~

它保留：

- streaming；
- tool hooks；
- 权限询问；
- intent-based tool category filtering；
- 只读工具并发；
- 副作用工具串行；
- token usage。

它本身不天然产生物理动作的可验证 trace。Unified Controller 会把纯聊天结果包装成受限的 answer-only GoalTree，以统一前端输出形状。

---

## 5. 确定性验证与证据链

### 5.1 为什么 Skill 成功还不够

机器人 Skill 的成功通常只表示：

- 指令成功发出；
- IK 找到解；
- 轨迹执行函数返回；
- 夹爪闭合命令完成；
- 导航客户端报告结束。

它不一定证明：

- 物体真的被抓住；
- 机器人真的到达用户指定坐标；
- 状态变化由当前 actor 造成；
- verify 没有选择一个本来就为真的条件；
- 模型没有通过选择容易满足的常量来自证。

因此最终 verdict 独立于 Skill 自报结果。

### 5.2 GoalVerifier

`goal_verifier.py` 在受限环境中求值 verify 表达式：

- 只暴露 World 提供的命名空间；
- 只保留有限安全 builtins；
- 拒绝 import、赋值、函数/类定义和 dunder；
- 带硬超时；
- 异常时返回失败；
- 结果同时保留 bool gate 和原始 observation。

这里仍会使用 Python `eval` 对已验证 AST 求值，但不是对模型输出做无限制 eval；安全边界由 AST allowlist、命名空间和 timeout 共同构成。

### 5.3 Oracle

Oracle 是 actor 不能随意伪造的实时只读数据源，例如：

- 仿真 base 的真实位置和朝向；
- 仿真 arm 的关节、FK 和夹爪状态；
- 场景物体位置；
- DevWorld 中的文件存在性、文件内容和 grep 计数；
- WorldModel 或 SceneGraph 中的结构化状态。

verify 必须实际消费适当 oracle，而不是只写 `True`、空表达式或与任务无关的恒真式。

### 5.4 Actor causation

`actor_causation.py` 试图回答：

> 即使目标状态为真，它是否由这次受控机器人动作造成？

例如机器人启动前已经位于目标点，如果本轮没有运动但 `at_position(...)` 为真，不能把这轮行动评为 GROUNDED。

当前 actor-causation 对部分控制通道仍不完整。特别是经外部导航栈产生的 `cmd_vel` 还没有完全纳入因果计数，因此某些物理上成功的导航会诚实地只得到 RAN。

### 5.5 坐标目标真实性

`coord_goal.py` 进一步检查：

- verify 中使用的坐标是否与用户要求一致；
- 坐标类型和维数是否匹配；
- verify 是否只是检查了任意着陆点；
- 动作是否真的有必要。

这是为了阻止“走到了错误位置，再用错误位置自证成功”。

### 5.6 VisualVerifier

VisualVerifier 是确定性谓词无法表达时的视觉兜底。它可以为调试和补充判断提供信息，但不能把本应失败的确定性证据门放宽。LLM/VLM 判断不是默认最终裁判。

### 5.7 Verdict 等级

| 等级 | 含义 |
|---|---|
| `GROUNDED` | 动作成功、确定性 verify 消费实时 oracle、结果为真，并通过因果和目标真实性检查 |
| `RAN` | 动作执行过，但证据不足、verify 不可信、actor 未造成变化或所需 oracle 不完整 |
| `FAILED` | 步骤执行失败或产生失败 trace |
| `NO_TRACE` | 没有形成可评估的执行轨迹 |

answer-only 是受限例外：纯回答可以没有物理 oracle，但带副作用的 action 不可借此绕过证据门。

### 5.8 证据的下游用途

同一证据分类不仅控制最终 verdict，也控制：

- StrategyStats 的成功奖励；
- ExperienceCompiler 是否编译模板；
- Trace replay/eval 是否通过；
- CLI 的 JSON 退出码。

这避免“用户看到失败，但学习系统把它当成功经验”的分叉。

---

## 6. `vector_os_nano/` 主包逐部分详解

### 6.1 包级入口

#### `vector_os_nano/__init__.py`

定义 v0.1 的主要公共 API：

- `Agent`；
- `SO101`；
- `Skill`；
- `SkillResult` / `ExecutionResult`；
- `MuJoCoArm` / `MuJoCoGripper`；
- `__version__`。

重型或可选后端采用 try/import 或懒加载，使基础 import 尽量不要求 MuJoCo、ROS2 或真机依赖。

#### `version.py`

单一版本号来源，当前为 `0.1.0`。

### 6.2 `core/`：结构化机器人 SDK

#### `types.py`

定义跨模块 frozen dataclass：

- `Pose3D`、`BBox3D`、`CameraIntrinsics`；
- `Detection`、`TrackedObject`、点云和相机帧类型；
- `SkillResult`；
- `TaskStep`、`TaskPlan`；
- `StepTrace`、`ExecutionResult`；
- odometry、laser scan 等移动机器人类型。

它是 Skill、Perception、Hardware 和 Executor 之间的共享数据契约。

#### `skill.py`

包含：

- `Skill` Protocol；
- `@skill` decorator；
- alias、direct、auto_steps、failure modes 等元数据；
- `SkillContext`；
- `SkillRegistry`；
- Skill → JSON Schema 导出。

SkillContext 向 Skill 提供 arm、gripper、base、perception、world model、config、services 和消息回调。SkillRegistry 既供直接结构化执行，也供 VGG 动态词表和 MCP tool schema 使用。

别名存在重叠时最终路由会受注册顺序影响；同名 Skill/alias 的覆盖行为是当前需要注意的扩展风险。

#### `agent.py`

`core.Agent` 的主要职责：

- 加载配置；
- 保存硬件组件；
- 自动复用 SO-101 串口总线构造 Gripper；
- 可选自动构造 RealSense + VLM + EdgeTAM；
- 创建 WorldModel；
- 创建并填充 SkillRegistry；
- 创建 TaskExecutor；
- 懒加载 IK 和标定；
- 同步机器人状态；
- 将 `execute_skill()` 转为 TaskPlan。

`execute_skill()` 会展开 Skill 的 `auto_steps`，并对大量技能自动追加 `home`。这对桌面机械臂的收尾有用，但对 perception-only 或 base-only Skill 可能产生语义冲突，是旧 SDK 路径的重要边界。

#### `executor.py`

确定性 TaskExecutor：

1. 拓扑排序；
2. 检查 TaskStep preconditions；
3. 从 SkillRegistry 获取 Skill；
4. 构造 SkillContext；
5. 执行 Skill；
6. 应用 WorldModel effects；
7. 检查 postconditions；
8. 记录 StepTrace；
9. 任一步失败则 fail-fast。

它不调用 LLM，也不等同于 VGG GoalExecutor。二者有不同的计划类型、验证语义和恢复能力。

#### `world_model.py`

保存会话级：

- ObjectState；
- RobotState；
- 夹爪/held object；
- 物体可见性、可达性和简单空间关系；
- YAML 持久化；
- pick/place/home 等已知效果。

它主要服务机械臂和会话状态，不是完整三维语义地图。

#### `spatial_memory.py`

旧的扁平空间记忆：

- 命名地点；
- visit count；
- 每个地点看到的物体；
- 描述；
- 有上限的事件日志；
- YAML 持久化；
- lock 保护。

#### `scene_graph.py`

新的三层语义空间图：

~~~text
RoomNode
  └── ViewpointNode
        └── ObjectNode
~~~

支持：

- 房间中心和面积；
- 视点、FOV 和覆盖率；
- 物体、属性和置信度；
- 门和房间连通关系；
- BFS door chain；
- VLM 房间排序；
- 模拟房间布局 seed；
- YAML 持久化；
- 对旧 SpatialMemory 的部分兼容 API。

“兼容”主要是接口层面，WorldModel、SpatialMemory 和 SceneGraph 并不会自动完全双向同步。

#### `nav_client.py`

统一两个 ROS2 导航接口：

- CMU topic mode：`/way_point`、`/state_estimation`、`/goal_reached`；
- Nav2 action mode：`NavigateToPose`；
- auto mode：先探测 Nav2，再回退 CMU。

ROS2 import 全部延迟；没有 ROS2 时模块仍可 import，导航调用返回不可用。

#### `config.py`

加载根目录 `config/default.yaml`，对用户 dict/YAML 做 deep merge 和基本验证。

### 6.3 `vcli/`：Agent Kernel 和产品入口

#### `cli.py`

负责：

- 参数；
- REPL；
- Slash commands；
- 凭据；
- 仿真/Agent 初始化；
- World 解析；
- Tool 注册；
- Skill wrapper；
- Session；
- prompt；
- Native/VGG/Unified 分流；
- console 进度和 machine verdict。

它体积很大，同时承担 frontend、composition root 和部分部署编排，是当前耦合最明显的文件之一。

#### `engine.py`

负责：

- backend-agnostic ReAct；
- tool dispatch；
- 只读并发和副作用串行；
- VGG 初始化；
- World verify namespace；
- GoalDecomposer、Executor、Harness、CapabilityRegistry 接线；
- Native delegate；
- Unified Controller；
- evidence gate；
- experience compilation；
- snapshot。

#### `native_loop.py`

实现 Native producer、合成 verify/finish tool、action-chain 记录和 ExecutionTrace 构造。

#### `backends/`

屏蔽不同 LLM API 的消息和 tool-call 格式差异。内部以统一 response type 运行。

#### `tools/base.py`

定义：

- Tool Protocol；
- `@tool` decorator；
- ToolResult；
- PermissionResult；
- ToolContext；
- ToolRegistry；
- CategorizedToolRegistry。

#### `tools/file_tools.py`

- FileRead；
- FileWrite；
- FileEdit；
- 路径约束；
- 覆盖前读取要求；
- 唯一匹配编辑。

#### `tools/search_tools.py`

- Glob；
- Grep；
- 优先使用 ripgrep；
- 无 rg 时 Python fallback。

#### `tools/bash_tool.py`

- 有超时的 subprocess；
- 输出截断；
- destructive pattern hard deny；
- 其他命令按权限策略 ask。

#### `tools/web_tool.py`

- 只读 HTTP fetch；
- 阻止 localhost、loopback 和 RFC1918 私网；
- 简单 HTML 文本化；
- 防止常见 SSRF。

#### 机器人和运维 Tool

- `robot.py`：RobotStatus、WorldQuery；
- `skill_wrapper.py`：把每个 Skill 变成 Tool；
- `scene_graph_tool.py`：房间、门、物体、door-chain 查询；
- `ros2_tools.py`：topic、node、log 诊断；
- `nav_tools.py`：导航和 terrain 状态；
- `sim_tool.py`：运行时启动、停止或切换仿真；
- `sysnav_sim_tool.py`：仿真加 SysNav bridge；
- `viz_tool.py`：Foxglove bridge；
- `reload_tool.py`：热重载 Skill。

#### `permissions.py` 和 `tool_execution.py`

标准 ReAct/VGG Tool 路径的权限顺序包括：

1. Tool 自身 hard deny；
2. no-permission 模式；
3. 用户 deny；
4. Tool 显式 allow；
5. Session always allow；
6. 只读自动 allow；
7. Tool ask；
8. 默认 ask。

`tool_execution.py` 抽出共同的 resolve-and-execute seam，供 ReAct 和 VGG ToolDispatcher 复用。

#### `hooks.py`

Tool 执行前后回调，用于遥测、显示、状态检查和后处理。Hook 异常不应让主任务崩溃。

#### `intent_router.py`

通过关键词和消息形状选择 Tool category，减少发送给模型的工具数量和 token。

#### `session.py`

JSONL Session 和 token 使用统计。

#### `prompt.py`、`dynamic_prompt.py`、`robot_context.py`

组合静态 persona 与实时机器人状态。

#### `turn_status.py`

控制 Rich live region，避免推理模型长时间无文本输出时的终端重复绘制和提示符错乱。

#### `verdict.py`

将 evidence gate 结果变成稳定 JSON、状态枚举和退出码。

#### `eval_runner.py`

面向 DevWorld 的 headless verify-as-eval runner，可在 CI 中按任务列表执行和自评分。

### 6.4 `vcli/cognitive/`：VGG 认知层

| 模块 | 核心作用 |
|---|---|
| `types.py` | GoalTree、SubGoal、ForEachSpec、StepRecord、ExecutionTrace |
| `goal_decomposer.py` | NL → GoalTree；JSON 提取、修复、依赖和 AST 校验 |
| `strategy_selector.py` | Skill/Primitive/Code/Tool/Capability 路由 |
| `goal_executor.py` | 拓扑执行、timeout、Blackboard、foreach、verify、failure class |
| `goal_verifier.py` | 受限谓词求值 |
| `vgg_harness.py` | step retry、tree replan、pipeline recovery |
| `blackboard.py` | 结构化输出存储和 `${step.path}` 解析 |
| `observation.py` | JSON-safe step/run snapshot |
| `trace_store.py` | trace 保存、加载、重放和 evidence gate |
| `evidence_classifier.py` | oracle 消费和非恒真性判断 |
| `actor_causation.py` | actor 是否造成机器人状态变化 |
| `coord_goal.py` | 坐标目标真实性 |
| `visual_verifier.py` | VLM 视觉验证兜底 |
| `capabilities/` | Capability Protocol、Registry 和 chat adapter |
| `code_executor.py` | 受限 Python 代码执行、AST allowlist、速度 clamp、timeout |
| `tool_dispatcher.py` | VGG Tool strategy 到真实 Tool/Permission 的桥 |
| `strategy_stats.py` | 分策略成功率统计 |
| `template_library.py` | 可持久化 GoalTemplate |
| `experience_compiler.py` | 从有证据的 trace 编译模板 |
| `object_memory.py` | 随时间衰减的物体置信度 |
| `predict.py` | 基于 SceneGraph 拓扑做纯函数预测 |
| `abort.py` | 全局停止信号 |
| `vocab_from_registry.py` | 从 SkillRegistry 单一生成 prompt/allowlist/参数帮助 |

### 6.5 `vcli/worlds/`：领域插件

#### `base.py`

World Protocol 和 DecomposeVocab。

#### `dev.py`

提供开发/代码领域：

- Dev persona；
- `file_exists`、`path_contains`、`grep_count` 等 verify；
- tool_call/chat 策略；
- chat LLM capability。

#### `robot.py`

提供机器人领域：

- Robot persona；
- 依据当前 arm/base 合并 sim oracle；
- 从 SkillRegistry 派生词表；
- 有 arm 且依赖存在时注册 Grounding DINO detector capability。

#### `registry.py`

World 名称到惰性 factory 的注册表；支持按名称和 Agent 是否存在解析。

#### `arm_sim_oracle.py` / `go2_sim_oracle.py`

单一来源的仿真验证谓词，避免 RobotWorld 和 PlaygroundWorld 各写一套。

### 6.6 `vcli/primitives/`：可组合原语

Primitive 使用一个模块级 `PrimitiveContext`，把硬件和世界状态暴露成受限函数：

- `locomotion.py`：walk、turn、set_velocity 等；
- `navigation.py`：导航、door chain、room；
- `perception.py`：相机、VLM、lidar；
- `world.py`：SceneGraph 查询。

它们主要服务 CaP-X/CodeExecutor 或 VGG primitive strategy。与 Skill 相比，Primitive 更原子、状态更薄；与 Tool 相比，它们不直接面向通用 Agent 权限接口。

### 6.7 `skills/`：机器人技能库

#### 默认桌面机械臂技能

- `home.py`：回 home 并打开夹爪；
- `scan.py`：移动到扫描姿态；
- `detect.py`：感知并写入 WorldModel；
- `describe.py`：VLM 场景描述；
- `pick.py`：标定、IK、预抓取、下降、闭合、抬起；
- `place.py`：上方姿态、下降、释放、回撤；
- `gripper.py`：打开/关闭；
- `handover.py`：转向用户并释放；
- `wave.py`：关节摆动。

#### Piper top-down

- `pick_top_down.py`；
- `place_top_down.py`。

这两者明确属于 demo-quality 路径：垂直抓放、单目标、无通用碰撞规划，主要服务 Go2+Piper MuJoCo。

#### 移动抓放

- `mobile_pick.py`：目标解析 → 接近位姿 → 导航 → 稳定 → top-down pick；
- `mobile_place.py`：接近放置区 → 导航 → 稳定 → top-down place。

#### 感知抓取

`perception_grasp.py` 是当前最复杂的操作 Skill：

1. 对目标进行 detector/color routing；
2. 可选 docking 和重定位；
3. 获取 RGB-D；
4. Grounding DINO/前指解析；
5. EdgeTAM mask 或 box fallback；
6. depth centroid；
7. camera → world；
8. Piper top-down IK；
9. 微调、闭合和抬起；
10. 由外部 oracle 验证是否真的持有目标。

它拒绝回退到直接读取仿真物体 GT pose，目的是保持 sim-to-real 传感器语义。

#### 导航

`navigate.py` 支持：

- 目标坐标；
- 房间别名；
- SceneGraph 房间中心；
- ROS2 proxy；
- NavStackClient；
- door-chain waypoint；
- 若干降级路径。

#### `skills/go2/`

- `walk.py`：定时 body velocity；
- `turn.py`：yaw 旋转；
- `stance.py`：stand/sit/lie；
- `stop.py`：急停；
- `look.py`：VLM 场景/房间识别和空间记忆；
- `where_am_i.py`：位置、朝向和房间；
- `explore.py`：后台 TARE/探索编排；
- `patrol.py`：多房间导航和观察。

Go2 Skills 不在默认桌面臂集合内，由 Go2 Agent wiring 单独注册。

#### `skills/utils/`

- `approach_pose.py`：移动操作接近几何；
- `place_region.py`：放置区域和 AABB；
- `terminal_dock.py`：经验化闭环 terminal docking。

### 6.8 `perception/`：视觉、RGB-D 和三维定位

#### `base.py`

PerceptionProtocol，约束 RGB、depth、detect、track 等能力。

#### `pipeline.py`

通用感知编排：

~~~text
aligned RGB-D
  → VLM bbox
  → EdgeTAM init/track
  → mask
  → point cloud
  → depth outlier removal
  → robust centroid / BBox3D
  → Detection / TrackedObject
~~~

支持 foreground update 和后台 tracking thread。

#### `realsense.py`

D405 连接、RGB/depth stream 和 alignment。`pyrealsense2` 在连接时懒加载。

#### `vlm.py`

Moondream：

- 本地 Transformers；
- Moondream Station；
- Cloud API。

#### `vlm_go2.py`

Go2 专用的远程/本地 VLM 封装，提供场景描述、房间判断、对象查找和成本统计。

#### `grounding_dino.py`

开放词汇 2D detector：

- 模型和 processor 懒加载；
- offline-first；
- 输入仅 RGB + text query；
- 不读取仿真 GT pose；
- CUDA 可用时使用 GPU。

#### `detector_capability.py`

将 Grounding DINO 包装成 VGG Capability。Capability 的 success 只说明调用结果，不替代最终世界验证。

#### `tracker.py`

EdgeTAM box-prompt segmentation 和连续跟踪。

#### 数值几何

- `pointcloud.py`：纯 NumPy RGB-D 投影；
- `_centroid.py`：最近深度簇、IQR 和 trimmed mean；
- `grasp_point.py`：mask/depth → camera point → world grasp point；
- `depth_projection.py`：D435/MuJoCo 内参和坐标系转换；
- `calibration.py`：affine/RBF 标定和误差统计。

#### `front_object.py`

通过 HSV、深度和连通域处理“前面的”“红色/绿色/蓝色”目标。当前实现明显针对验收场景调参，不应等同于通用显著物体检测器。

#### `go2_grasp_perception.py`

将 MuJoCo Go2 的 RGB、metric depth、camera pose 适配成真实机器人风格的感知接口。它与 `MuJoCoPerception` 的区别是：前者不暴露物体 GT pose，后者主要是桌面仿真/测试的真值后端。

### 6.9 `hardware/`：HAL、真机和仿真

#### 顶层协议

- `arm.py`：ArmProtocol；
- `gripper.py`：GripperProtocol；
- `base.py`：BaseProtocol，包含 blocking `walk` 和 streaming `set_velocity`；
- `hardware/ros2/runtime.py`：共享 ROS2 executor。

#### `so101/`

- `serial_bus.py`：Feetech SCS 总线；
- `joint_config.py`：电机 ID、方向、零位、范围和编码器转换；
- `arm.py`：连接、关节插值、笛卡尔运动和 stop；
- `gripper.py`：共享串口总线的夹爪；
- `ik_solver.py`：Pinocchio FK/IK。

#### `urdf/`

SO-101 URDF、仿真 URDF、机械结构 mesh 和演示物体 mesh。

#### `sim/mujoco_arm.py`、`mujoco_gripper.py`

桌面 SO-101 仿真。Arm 实现 HAL；Gripper 通过 MuJoCo 接触/weld 近似抓取。

#### `sim/mujoco_perception.py`

从 MuJoCo scene 直接生成检测/位置，适合测试和桌面仿真，不代表 sim-to-real 感知。

#### `sim/mujoco_go2.py`

Go2 仿真主体：

- 模型组合；
- 1 kHz 物理线程；
- 正弦步态；
- 可选 convex MPC；
- stand/sit/lie；
- body velocity；
- camera、depth、lidar、odom；
- viewer/headless；
- Go2+Piper 和抓取 weld；
- 运行时场景 XML 生成。

#### `sim/mujoco_piper.py` / `mujoco_piper_gripper.py`

挂载在 Go2 模型上的 6-DoF Piper，与 Go2 共享 MjModel/MjData；Piper 自身不重复 stepping。

#### ROS2 proxy

- `go2_ros2_proxy.py`：统一 FAR/local planner/door-chain 导航接口；
- `piper_ros2_proxy.py`：Piper joint/gripper 命令与本地 IK；
- `gazebo_go2_proxy.py`：在 Go2 proxy 上增加 Gazebo 存活检查；
- `isaac_sim_proxy.py`：Isaac 后端 proxy；
- `isaac_sim_arm_proxy.py`：Isaac arm 实验接口。

#### 遗留和辅助

- `pybullet_arm.py`、`pybullet_gripper.py`：旧桌面仿真；
- `viewer_mode.py`：平台相关 viewer drive mode；
- `sim_clock.py`：wall-clock controller 与 sim-time 的一致性；
- `sensors/gt_odom.py`：真值里程计；
- `sensors/lidar360.py`：虚拟 Mid-360；
- `sensors/pano360.py`：六面渲染拼接全景；
- `mjcf/`、`objects/`：Go2、Piper、房间、纹理和抓取对象资产。

### 6.10 `playground/`：预设场景插件

- `scenario.py`：静态场景 DTO；
- `catalog.py`：`tabletop`、`tabletop_tray`、`go2_room`；
- `world.py`：PlaygroundWorld；
- `verify/`：对 kernel sim-oracle 的兼容重导出。

导入 `playground` 时会把场景 factory 注册进进程级 WorldRegistry。Kernel 不在模块加载时硬引用 Playground，依赖方向保持为 Playground → Kernel。

部分 Go2 房间 XML 不是 clean tree 中的静态文件，而是 MuJoCoGo2 在运行时生成。

### 6.11 `mcp/`：MCP 前端

#### `tools.py`

- 将 Skill schema 变成 MCP Tool；
- 添加 natural_language；
- 添加 diagnostics；
- 添加 debug_perception；
- 添加 run_goal；
- 将调用路由到 Unified/VGG。

#### `resources.py`

提供：

- `world://state`；
- `world://objects`；
- `world://robot`；
- `camera://overhead`；
- `camera://front`；
- `camera://side`；
- `camera://live`。

#### `server.py`

- 构造 MuJoCo 桌面臂或 SO-101 真机栈；
- 创建 VectorEngine 和 Session；
- 注册 MCP tool/resource handlers；
- 支持 stdio；
- 支持 SSE/HTTP；
- 退出时断开 Agent。

MCP 当前不是 Go2 ROS2 导航桥。

### 6.12 `ros2/`：节点与目标拓扑

#### Nodes

- `hardware_bridge.py`：SO-101 joint state、joint command、trajectory/gripper action；
- `perception_node.py`：RGB/depth subscription、检测 JSON 和 overlay；
- `world_model_node.py`：joint/detection → WorldModel 和查询服务；
- `skill_server.py`：Skill → `/skill/<name>` Trigger service；
- `agent_node.py`：Agent execute/plan/status；
- `go2_bridge.py`：早期 MuJoCo Go2 → ROS2 bridge；
- `scene_graph_viz.py`：SceneGraph → RViz MarkerArray。

#### `launch/nano.launch.py`

描述硬件、感知、WorldModel、Skill 和 Agent 的分阶段启动拓扑。

但是当前仓库没有完整 ament package、`package.xml` 和对应 executable 安装入口，部分 Node `main()` 也明确要求由程序注入 Agent/Pipeline。因此这一目录应理解为“节点实现和目标集成结构”，不是开箱即用的独立 ROS 包。

### 6.13 `integrations/sysnav_bridge/`

#### `topic_interfaces.py`

定义 SysNav 消息的 shadow dataclass 和 ObjectNode → ObjectState 转换，使无 ROS2 的测试也能覆盖字段映射。

#### `live_bridge.py`

懒加载 `rclpy` 和 `tare_planner.msg`，订阅 `/object_nodes_list`，把物体写入 WorldModel。

当前 RoomNode 只有 shadow 类型，尚没有完整 room topic subscription 和 SceneGraph 拓扑同步。

---

## 7. 典型端到端数据流

### 7.1 DevWorld 中修改代码

~~~text
“修改某个配置并运行测试”
  → 裸 REPL 判断为 action-shaped
  → Native 得到 file_read/file_edit/bash/verify
  → 模型读取文件
  → 调用 file_edit
  → 调用 verify(path_contains(...) / tests_pass(...))
  → NativeStepRunner 生成 StepRecord
  → evidence classifier 检查是否读取 DevWorld oracle
  → GROUNDED / RAN / FAILED
~~~

如果 Native 没有调度 action，则 Unified 可进入 VGG tool_call 或 ReAct 路径。

### 7.2 桌面机械臂简单 Skill

~~~text
“挥手”
  → RobotWorld
  → Native 或 VGG 选择 wave Skill
  → SkillWrapperTool
  → core SkillContext
  → arm.move_joints(...)
  → verify arm/joint oracle
  → verdict
~~~

简单 alias 在 VGG 路径中可以绕过 LLM decomposition。

### 7.3 检测、三维定位和抓取

~~~text
目标文本
  → Grounding DINO bbox
  → EdgeTAM mask
  → metric depth
  → masked point cloud
  → depth cluster + robust centroid
  → camera-to-world
  → top-down IK
  → Piper motion + gripper
  → holding_object(...) oracle
  → actor causation
  → verdict
~~~

Detector 和 Skill 只能产生候选动作；是否真的抓住目标由独立 oracle 判定。

### 7.4 Go2 导航

~~~text
目标坐标/房间
  → navigate Skill 或 Native coordinate navigate
  → Go2ROS2Proxy / NavStackClient
  → FAR / local planner / door chain
  → /cmd_vel_nav
  → base motion
  → at_position(...) oracle
  → actor-causation
~~~

当 `cmd_vel` 因果链未被完整记录时，即使机器人到点也可能只得到 RAN。

### 7.5 Go2 探索

~~~text
“探索房间”
  → ExploreSkill
  → 后台线程
  → TARE/FAR/ROS bridge
  → lidar/odom/pano/RGB-D
  → SceneGraph / WorldModel
  → 可取消、可插入 stop 或新导航任务
~~~

Skill 可能在后台工作真正完成前先返回“线程已启动”，因此最终完成状态必须通过后续观察和 verify 判断。

### 7.6 Nav + Grasp

~~~text
导航到桌边
  → terminal dock
  → 重新观察目标
  → detector/segmentation/depth
  → approach
  → Piper grasp
  → lift
  → holding_object(...)
~~~

这条链已经机械贯通，但最新状态仍是间歇成功，问题集中在 docking 后的可达性和目标定位/选择稳定性。

### 7.7 MCP 请求

~~~text
MCP client 调 natural_language 或 Skill-named tool
  → 构造自然语言 instruction
  → run_turn_unified
  → action: VGG
     chat: ReAct + answer-only trace
  → 返回 TextContent
~~~

MCP resources 可同时读取 WorldModel 和相机图像。

---

## 8. 仿真、ROS2 与外部导航系统

### 8.1 Embodiment × Backend

| Embodiment | 直接 Python | MuJoCo | ROS2 Proxy | Gazebo | Isaac | 真机 |
|---|---:|---:|---:|---:|---:|---:|
| SO-101 | 是 | 是 | 有节点包装 | 否 | 否 | 是 |
| Go2 | 协议层 | 是 | 是 | 实验 | 实验/archived | 本仓无真机 SDK |
| Go2 + Piper | 是 | 是 | 是 | 未完整 | 未完整 | 本仓无完整真机链 |
| G1 29-DoF + 双 Dex3 | HAL/具名 Agent | 固定基座关节控制 | Isaac host ROS2 transport | 否 | 代码/契约测试，待官方资产 live 验收 | 本仓无真机 transport |

### 8.2 当前移动仿真的主链

最具操作性的移动链不是 `ros2/launch/nano.launch.py`，而是：

~~~text
scripts/go2_vnav_bridge.py
  MuJoCo Go2 + Piper + Odom + LiDAR + RGB-D
          ↓ ROS2 topics
外部 vector_navigation_stack
  terrain analysis + local planner + FAR + TARE
          ↓ ROS2 topics
Go2ROS2Proxy / PiperROS2Proxy
          ↓
Agent Skills
~~~

`scripts/go2_vnav_bridge.py` 同时承担：

- MuJoCo stepping 的外部接线；
- odom、TF 和 point cloud；
- RGB、depth、camera pose；
- path following；
- SceneGraph marker；
- Piper joint/gripper；
- nav status IPC。

### 8.3 Gazebo

`gazebo/` 包含：

- apartment 和 empty-room world；
- Go2 SDF/xacro；
- RGB、depth、lidar、IMU；
- ros_gz bridge；
- ros2_control；
- launch；
- GazeboGo2Proxy。

它已经形成“可接入统一话题契约”的实验后端，但仍依赖外部 `go2_description`、ROS2 workspace 和本机路径；部分 controller 参数和 mesh 路径没有收口。现有测试主要检查文件和结构，不代表真实 Gazebo controller 已跑通。

### 8.4 Isaac Sim

`docker/isaac-sim/` 采用双进程思路：

- Isaac Python 运行物理和策略；
- ROS Jazzy Python 读取共享文件并发布话题。

当前共享链主要覆盖 odom/joints/velocity command。虽然 publisher 声明了 lidar/RGB/depth topic，但传感器数据的文件读取和 timer 发布链尚未闭合。Isaac arm proxy 也没有对应完整容器端机械臂控制节点。因此应视为 Go2 物理/里程计原型，而不是完整高保真部署。

### 8.5 ROS2 话题契约

主要 topic 包括：

- `/state_estimation`；
- `/registered_scan`；
- `/cmd_vel_nav`；
- `/goal_point`；
- `/way_point`；
- `/camera/color/image_raw`；
- depth/camera pose；
- `/joint_states`；
- Piper joint/gripper command；
- `/scene_graph_markers`。

Gazebo、Isaac 和 MuJoCo proxy 尽量复用这一契约，使 Agent 上层不需要知道具体 simulator。

### 8.6 外部工作区

下列能力不由本仓库独立提供：

- FAR planner；
- TARE exploration；
- CMU local planner / terrain analysis；
- Nav2、SLAM Toolbox、AMCL；
- SysNav ROS2 messages/nodes；
- 部分 Go2 description/controller；
- Go2 convex-MPC 相关工作区。

大量 `scripts/launch_*.sh` 会 source 开发机上的这些工作区。

### 8.7 Foxglove

`foxglove/` 只消费 ROS2 数据：

- 启动 foxglove_bridge；
- 提供包含 3D、点云、地图、frontier、路径、相机、SceneGraph、odom 和 velocity 的 dashboard。

它不参与控制和验证。

---

## 9. 配置、脚本、部署与辅助资产

### 9.1 `config/`

#### `default.yaml`

默认配置分区：

- agent；
- llm；
- arm；
- camera；
- perception；
- calibration；
- skills；
- ros2；
- mcp。

#### 导航配置

- `nav.yaml`：导航、stall、waypoint、探索和墙体逃逸；
- `far_go2_indoor.yaml`：FAR 室内参数；
- `local_planner_go2.yaml`：传感器偏移、外形、速度和容差；
- `tare_go2_indoor.yaml`：TARE 探索；
- `room_layout.yaml`：房间和门；
- `*.rviz`：可视化。

后几类主要由外部 ROS2 stack 消费，不是 Python Kernel 自身的算法参数。

### 9.2 `scripts/` 分类

#### 主桥

- `go2_vnav_bridge.py`：完整 MuJoCo/ROS2/导航/Piper bridge；
- `go2_nav_bridge.py`：较轻 Nav2/SLAM bridge；
- `go2_vnav_bridge.py` 相关 camera、lidar、path follower 工具。

#### 启动/停止

- `launch_bridge.sh`；
- `launch_vnav.sh`；
- `launch_explore.sh`；
- `launch_nav_only.sh`；
- `launch_nav_explore.sh`；
- `launch_nav2.sh`；
- `launch_slam.sh`；
- `launch_gazebo.sh` / `stop_gazebo.sh`；
- `launch_isaac.sh` / `stop_isaac.sh`；
- `launch_vnav.sh`、`launch_vnav_e2e` 相关脚本。

#### Smoke 和机器级测试

- `test_fullstack.sh`；
- `test_integration.sh`；
- `test_nav2_integration.sh`；
- `test_tare_dataflow.sh`；
- `test_vnav.sh` / `test_vnav_e2e.sh`；
- `test_slam.sh`；
- `test_visualization.sh`。

#### 诊断

- `diag_tare.sh`；
- `monitor_vgraph.py`；
- `smoke_sysnav_sim.py`。

#### 研究探针

- `probe_r36_*` 到 `probe_r40_*`；
- `verify_pick_top_down.py`；
- `verify_loco_pick_place.py`。

这些文件记录了 nav+grasp campaign 的具体实验问题和调参过程，不应视为稳定公共 API。

### 9.3 根级运行入口

- `README.md`：面向使用者的项目定位、安装、CLI、MuJoCo 和 MCP 快速开始；适合了解产品表面，但不应替代当前代码路径和 STATUS；
- `CLAUDE.md`：项目 North Star、不可变规则、验收接口和文档治理；
- `sim.sh`：薄包装；
- `scripts/vector-sim`：更完整的 sim launcher；
- `.env.example`：凭据示例；
- `.gitignore`：过滤模型缓存、场景生成物、Session、日志、虚拟环境和本机配置；
- `.mcp.json`：当前开发机 MCP 示例；
- `conftest.py`：根级 pytest 兼容/约束；
- `pyproject.toml`：唯一正式 Python build metadata。

### 9.4 `examples/`

用于展示：

- SO-101 quickstart；
- 无 LLM 的结构化 Skill 调用；
- 自定义 Skill；
- PyBullet simulation；
- ROS2 Node 启动。

这些示例目前不能视为可直接复制的权威 API：

- 多个文件仍调用已不存在的 `Agent.execute()`；
- 旧 `llm_api_key` 参数不会让 `core.Agent` 自动规划；
- PyBullet 示例的“尚未实现”和安装 extra 说明已经过期；
- ROS2 示例把普通 Node 写成 LifecycleNode。

实际调用应以 `Agent.execute_skill()`、`vector-cli` 和 MCP 当前入口为准。

### 9.5 `images/`

只用于 README、截图和演示 GIF，不参与运行时。

### 9.6 License

仓库主体采用 Apache License 2.0。`NOTICE` 记录第三方资产和归属；外部 SysNav 等系统有自己的许可证边界，集成层避免复制其源码。

---

## 10. 状态、持久化与生成物

### 10.1 四套状态视图

#### WorldModel

会话级物体和机器人操作状态，偏机械臂和 Skill pre/postcondition。

#### SpatialMemory

旧扁平房间、观察和事件记忆。

#### SceneGraph

新 Room/Viewpoint/Object 图、门和空间推理。

#### Sim Oracle

直接读取 MuJoCo 物理真值的只读验证层。

它们回答的问题不同，但当前没有统一的自动双向同步。调试时必须先确认某个模块读取的是哪套“真相”。

### 10.2 Blackboard

Blackboard 只在一次 VGG run 内存在，用于步骤间结构化数据流，不等于长期 WorldModel 或 SceneGraph。

### 10.3 常见持久化位置

- `~/.vector/config.yaml`：CLI 配置；
- `~/.vector/oauth_credentials.json`：OAuth；
- `~/.vector/sessions/*.jsonl`：会话；
- `~/.vector/traces/`：执行 trace；
- 持久化目录中的 `strategy_stats.json`；
- GoalTemplate；
- `~/.vector_os_nano/scene_graph.yaml`；
- `/tmp/vector_*.log`；
- `/tmp/vector_nav_*` IPC/status；
- 仿真诊断和抓取图像。

### 10.4 运行时生成场景

Go2 room 场景可能由 `mujoco_go2.py` 动态生成 `scene_room.xml`。Playground catalog 可以引用这一目标路径，但 clean checkout 中不一定已有文件。

### 10.5 Secrets

不应提交：

- `.env`；
- API key；
- OAuth token；
- 本机私有配置；
- 真机凭据；
- 带敏感数据的 Session/export。

---

## 11. 依赖、安装与打包边界

### 11.1 基础依赖

基础安装面向 Agent Kernel：

- rich；
- prompt_toolkit；
- httpx；
- PyYAML；
- python-dotenv；
- anthropic；
- openai；
- numpy；
- pyserial。

### 11.2 Optional extras

| Extra | 用途 |
|---|---|
| `sim` | MuJoCo |
| `perception` | torch、transformers、timm、Pillow、HF、SciPy、OpenCV |
| `real` | pyrealsense2、Open3D、PyBullet |
| `ik` | Pinocchio |
| `mcp` | MCP SDK |
| `go2` | CasADi、Pinocchio |
| `all` | sim + perception + ik + mcp + go2，不含 real |
| `dev` | pytest、coverage、lark |

ROS2 是 apt/system dependency，不由 pip extra 安装。

### 11.3 依赖边界问题

- `scservo_sdk` 在 SO-101 串口层运行时 import，但没有明确出现在 extras 中；
- `[sim]` 只安装 MuJoCo，旧 PyBullet 示例却提示安装 `[sim]`；
- `[all]` 有意排除真机 native 依赖；
- ROS2 Humble/Jazzy 在注释、示例和脚本中没有完全统一。

### 11.4 Wheel 边界

Hatch wheel 当前只声明包含 `vector_os_nano`。根目录的：

- `config/`；
- `scripts/`；
- `gazebo/`；
- `docker/`；
- `foxglove/`；
- `examples/`

并不会自动成为标准 wheel 资产。

同时 `core/config.py` 从“仓库根目录”推导 `config/default.yaml`。在纯 wheel 安装布局中，这种路径假设需要额外验证和修正。

---

## 12. 测试体系与验收门

### 12.1 测试文件分布

当前共有 288 个 `test_*.py` 文件：

| 目录 | 文件数 | 主要覆盖 |
|---|---:|---|
| `tests/unit` | 115 | 类型、配置、纯函数、感知、仿真、world、integration adapter |
| `tests/harness` | 76 | MuJoCo、Go2、传感器、VLM、导航、VGG 的里程碑层级 |
| `tests/vcli` | 74 | CLI、Native、权限、World、VGG、verdict、PTY |
| `tests/integration` | 12 | Agent、sim、SysNav、完整 CLI seam |
| `tests/skills` | 6 | mobile/top-down pick/place |
| `tests/hardware` | 4 | ROS runtime 和 proxy |
| `tests/e2e` | 1 | headless MuJoCo explore/navigate 链 |

这是测试文件数，不是 test case 数。

### 12.2 Marker

`pyproject.toml` 注册：

- level0–level4；
- ros2；
- integration；
- live_llm；
- cli_main；
- capability。

`tests/conftest.py` 强制：

> 任何标记为 `capability` 的产品能力测试，必须同时标记 `cli_main`。

这样可以防止只测试旁路 helper，却宣称产品入口已经支持该能力。

### 12.3 三种“测试通过”

必须区分：

1. Python unit/static contract 通过；
2. 真实 MuJoCo integration/harness 通过；
3. ROS2 + 外部导航工作区的机器级 smoke 通过。

Gazebo/Isaac/ROS2 的许多测试只检查配置、AST、mock 或文件结构，不代表真实容器、DDS、controller 和外部导航栈已实际启动。

### 12.4 本次环境运行结果

2026-07-14 在当前环境运行：

~~~text
conda run -n base python -m pytest -q tests/vcli
~~~

结果：

~~~text
575 passed, 30 skipped, 1 xfailed, 7 failed
~~~

7 个失败集中在：

- 缺可选 `mcp` 包；
- clean tree 中缺运行时生成的 Go2 `scene_room.xml`；
- REPL 测试尝试写受当前 sandbox 限制的 `~/.vector/sessions`。

这不是仓库的永久基准，只是该日期和环境的一次运行记录；结论是核心断言覆盖很深，但当前 checkout 不能称为开箱全绿。

### 12.5 最终产品验收

项目治理把“裸 `vector-cli` + 自然语言”定义为最终验收接口。pytest、probe 和脚本是内部验证工具，不能替代真实 CLI 入口和机器可读 verdict。

---

## 13. 安全与信任边界

### 13.1 标准 Tool 路径

ReAct 与 VGG ToolDispatcher 共用权限执行 seam。安全措施包括：

- Tool hard deny；
- 文件路径约束；
- Bash destructive deny pattern；
- read-only 自动允许；
- 写入和外部副作用询问；
- Web SSRF 防护；
- output truncation；
- timeout；
- Session deny/always-allow。

### 13.2 CodeExecutor

受限生成代码：

- AST 验证；
- 默认禁止 import；
- 无 `open`、`exec`、`eval`、`__import__`；
- dunder 拒绝；
- velocity clamp；
- timeout；
- stdout capture。

### 13.3 Verify sandbox

模型提供的 verify 表达式同样是不可信输入，必须经过 AST 和命名空间限制。

### 13.4 Native 权限一致性风险

当前 `NativeStepRunner.dispatch_skill()` 直接调用 `tool.execute(params, context)`，没有经过标准 `resolve_permission` 和 hooks。Native 又会暴露：

- code 类 file/bash 工具；
- 机器人电机 Skill。

因此默认 REPL Native action path 与标准 ReAct/VGG 权限链存在高优先级不一致。生产使用前应让 Native 复用共同的 tool-execution seam，或对其 action surface 做等价的硬限制。

### 13.5 MCP 和 ROS2

- MCP SSE 可监听外部地址；
- MCP tool 可能触发真机动作；
- 当前 `.mcp.json` 默认指向硬件模式和本机绝对路径；
- ROS2 topic 默认依赖 DDS 网络边界；
- 外部导航和 SysNav 消息均应视为不可信输入。

部署前需要额外考虑认证、网络隔离、真机急停和最小权限。

---

## 14. 当前成熟度与技术债务

### 14.1 已较完整落地

- Kernel/World seam；
- Skill/Tool/Capability registry；
- Native producer；
- VGG GoalTree、Blackboard、Verifier、Harness；
- evidence classifier 和 machine verdict；
- SO-101 串口、夹爪、FK/IK；
- MuJoCo 桌面臂；
- MuJoCo Go2、Piper、RGB-D、LiDAR、odom；
- Grounding DINO、EdgeTAM、点云抓取；
- MCP 桌面臂和 SO-101 stack；
- 深度较高的 Python/MuJoCo tests。

### 14.2 部分落地或依赖外部系统

- FAR/TARE/Nav2/SLAM；
- SysNav；
- Go2+Piper nav+grasp；
- ROS2 Go2/Piper proxy；
- SceneGraph 自动构建和房间语义；
- Playground 场景产品化。
- G1 固定 pelvis 位置 IK、MuJoCo 固定基座操作和 Isaac bridge；真实抓取、平衡行走与 live 容器验收仍依赖后续输入。

### 14.3 路线图或实验脚手架

- G1 真机 transport、已标定传感器、动态行走/导航和门/冰箱等接触交互；
- 通用 VLA；
- 完整异构模型 zoo；
- 统一 embodiment 热切换；
- 开箱即用 ROS package；
- 完整 Gazebo 后端；
- 完整 Isaac RGB-D/LiDAR/arm；
- 稳定的真实移动操作。

### 14.4 三套执行语义

仓库同时存在：

1. `core.TaskExecutor`；
2. VGG `GoalExecutor/VGGHarness`；
3. Native `NativeStepRunner`。

它们对：

- 何为成功；
- pre/postcondition；
- retry；
- 输出捕获；
- 权限；
- trace；
- failure propagation

的处理并不完全一致。

### 14.5 多套状态真相

- WorldModel；
- SpatialMemory；
- SceneGraph；
- sim oracle。

它们不自动完全同步，容易出现一个模块认为“已抓住”，另一个模块仍认为“物体在桌上”的情况。

### 14.6 Skill 多代并存

- legacy SO-101 pick/place；
- Piper top-down；
- mobile pick/place；
- perception grasp。

自然语言最终选中哪个实现，可能受 World、Skill 注册顺序、alias 和具体入口影响。

### 14.7 Native trace 边界

当前 Native verify 生成的 StepRecord 对 action ToolResult error 的传播不如 VGG 完整；verify-only、已有状态和失败 action 组合仍值得继续 red-team。

### 14.8 Core SDK 边界

- `execute_skill()` 会对大量 Skill 自动追加 `home`；
- Core executor 和 Skill 声明的 pre/postcondition 使用方式不完全对称；
- WorldModel effects 对 Skill 名称有硬编码；
- Base/Gripper 生命周期没有像 Arm 一样完全统一；
- Core 成功语义可能比 VGG evidence gate 更宽松。

### 14.9 感知边界

- RealSense color/depth getter 的帧配对需要进一步收口；
- depth scale、CameraIntrinsics 类型和米/毫米契约存在多套定义；
- Pipeline 与 grasp centroid 仍有重复数值实现；
- front-object 和 perception-grasp 中有较多场景常量；
- EdgeTAM fallback 到 bbox mask 时精度会显著下降。

### 14.10 导航和探索边界

- 外部 stack 路径依赖本机 workspace；
- actor causation 尚未覆盖所有 `cmd_vel`；
- Explore 后台失败不能自然回写到已返回的 SkillResult；
- Patrol、auto-observe、mobile approach heading 等路径仍有接口和语义债务；
- room alias 和静态 layout 仍较多。

### 14.11 部署和打包边界

- ROS2 不是完整 ament package；
- Gazebo 有外部模型和绝对路径；
- Isaac 传感器链不完整；
- 非 Python 资产不自动进入 wheel；
- Humble/Jazzy 假设不统一；
- `.mcp.json` 和大量 shell 脚本不可移植。

### 14.12 文档与示例漂移

已确认的例子：

- 多个 example 调用不存在的 `Agent.execute()`；
- `Agent` 的旧 LLM 参数实际被忽略；
- PyBullet 示例称实现不存在，但类已经存在；
- PyBullet 示例的 extra 安装提示仍不正确；
- ROS2 示例把普通 Node 描述为 LifecycleNode；
- `ARCHITECTURE.md` 的部分流程图仍把两个前端都描述为 Unified-first；
- `engine.py` 注释仍称 Unified dark-launched；
- STATUS 保留已合并 feature branch 的历史文字；
- SysNav integration 文档链接指向不存在的文件；
- MCP 配置写死旧开发机路径。

### 14.13 当前 nav+grasp 的诚实结论

可以说：

- 机械链完整；
- 感知精度已经达到厘米级样例；
- docking 能收敛；
- 某些 trial 可真实抓住正确物体。

不能说：

- 已稳定端到端落地；
- 任意颜色/目标都可靠；
- 已达到产品级成功率；
- 所有导航都能被 evidence gate 评为 GROUNDED。

---

## 15. 如何扩展仓库

### 15.1 添加 Skill

1. 实现 Skill Protocol；
2. 使用 `@skill` 声明 name、aliases、parameters、effects、failure modes；
3. 只通过 SkillContext 访问硬件和服务；
4. 明确结构化 SkillResult；
5. 避免与现有 alias 冲突；
6. 决定加入默认 arm、Go2 或特定 World 注册；
7. 添加 Skill 单测；
8. 添加真实 CLI capability 测试；
9. 添加独立 verify predicate，不让 Skill 自证。

### 15.2 添加 Tool

1. 实现 Tool Protocol；
2. 定义 JSON schema；
3. 正确声明只读/副作用；
4. 实现 hard deny 和路径约束；
5. 注册到正确 category；
6. 确认 ReAct、VGG ToolDispatcher 和 Native 的权限一致性；
7. 添加 timeout、输出限制和测试。

### 15.3 添加 World

1. 实现 persona；
2. 注册领域 Tool；
3. 构造只读 verify namespace；
4. 提供或派生 DecomposeVocab；
5. 注册模型 Capability；
6. 使用 WorldRegistry 懒 factory；
7. 保持 Kernel 不在模块加载时硬 import 该领域；
8. 添加未知 World fail-loud 测试。

### 15.4 添加验证谓词

验证谓词必须：

- 只读；
- 确定性；
- 从 actor 无法任意改写的 oracle 读取；
- 有明确参数类型；
- 对异常 fail-closed；
- 与用户目标真实对应；
- 同时加入 prompt vocabulary 和 verifier allowlist；
- 配 actor-causation 测试。

### 15.5 添加 Capability

1. 实现 typed input/output；
2. 声明 kind 和 side-effecting；
3. 由 World 注册；
4. 只返回执行观察，不自行 verified；
5. 让 GoalExecutor 和 evidence spine 继续做最终裁决；
6. 添加冷启动、依赖缺失和失败降级测试。

### 15.6 添加硬件后端

1. 实现 Arm/Base/Gripper Protocol；
2. 生命周期幂等；
3. stop 必须安全；
4. 明确坐标系、单位和线程模型；
5. 提供 capabilities；
6. 避免让 Skill import 具体后端；
7. 添加 mock contract 和至少一个真实 backend integration test。

### 15.7 添加仿真器

优先复用：

- HAL Protocol；
- ROS2 topic contract；
- Go2/Piper proxy；
- camera/depth/odom/lidar 类型；
- sim oracle；
- actor-causation instrumentation。

仅增加“文件存在”测试不足以声明 simulator capability 完成。

---

## 16. 现有文档与推荐阅读顺序

### 16.1 规范文档

1. [`CLAUDE.md`](../CLAUDE.md)：North Star、规则和文档治理；
2. [`agent-kernel-STATUS.md`](agent-kernel-STATUS.md)：当前状态和下一步；
3. [`ARCHITECTURE.md`](ARCHITECTURE.md)：持久设计；
4. [`DECISIONS.md`](DECISIONS.md)：架构决策历史；
5. [`tricky-bugs.md`](tricky-bugs.md)：隐藏 bug casebook。

### 16.2 子系统文档

- [`cli-tool-system.md`](cli-tool-system.md)；
- [`skill-protocol.md`](skill-protocol.md)；
- [`sim-dev-guide.md`](sim-dev-guide.md)。

### 16.3 代码和文档冲突时

优先级建议：

1. 当前实际入口；
2. 当前测试；
3. 当前实现；
4. STATUS 的最新事实；
5. ARCHITECTURE 的持久设计；
6. 旧注释和 example。

ARCHITECTURE 自己已经说明部分章节描述的是 redesign 之前的实现，因此不能只读流程图而不检查 CLI 当前分流。

### 16.4 推荐接手顺序

#### 想理解 Agent Kernel

~~~text
CLAUDE.md
→ STATUS
→ ARCHITECTURE
→ vcli/cli.py
→ vcli/native_loop.py
→ vcli/engine.py
→ vcli/cognitive/
→ vcli/worlds/
~~~

#### 想理解机器人 SDK

~~~text
core/agent.py
→ core/skill.py
→ core/executor.py
→ core/world_model.py
→ skills/
→ hardware/
~~~

#### 想理解抓取

~~~text
skills/perception_grasp.py
→ perception/go2_grasp_perception.py
→ perception/grounding_dino.py
→ perception/tracker.py
→ perception/grasp_point.py
→ hardware/sim/mujoco_piper.py
→ worlds/arm_sim_oracle.py
~~~

#### 想理解移动导航

~~~text
skills/navigate.py
→ skills/go2/explore.py
→ hardware/sim/go2_ros2_proxy.py
→ scripts/go2_vnav_bridge.py
→ config/nav.yaml
→ FAR/TARE/local planner 外部工作区
~~~

#### 想理解部署

~~~text
pyproject.toml
→ vcli/cli.py::_init_agent
→ scripts/
→ ros2/
→ gazebo/
→ docker/isaac-sim/
~~~

---

## 17. 附录：目录、入口和术语速查

### 17.1 顶层目录速查

| 路径 | 一句话作用 |
|---|---|
| `vector_os_nano/core` | 结构化硬件/Skill SDK |
| `vector_os_nano/vcli` | Agent Kernel 和产品入口 |
| `vector_os_nano/vcli/cognitive` | VGG 计划、验证、恢复和经验 |
| `vector_os_nano/vcli/worlds` | 领域插件和 oracle |
| `vector_os_nano/vcli/primitives` | 模型可组合原语 |
| `vector_os_nano/skills` | 机器人动作库 |
| `vector_os_nano/perception` | 视觉、RGB-D、点云和标定 |
| `vector_os_nano/hardware` | HAL、真机、仿真和 proxy |
| `vector_os_nano/playground` | 预设场景 World |
| `vector_os_nano/mcp` | MCP server、tools、resources |
| `vector_os_nano/ros2` | ROS2 节点和目标拓扑 |
| `vector_os_nano/integrations` | 外部系统适配 |
| `config` | 运行和导航参数 |
| `scripts` | 桥、启动、smoke、诊断和探针 |
| `gazebo` | Gazebo 世界/模型/launch |
| `docker/isaac-sim` | Isaac 容器原型 |
| `foxglove` | ROS2 可视化 |
| `tests` | 多层测试体系 |
| `docs` | 设计、状态和决策记录 |

### 17.2 命令入口

| 命令 | 入口 | 用途 |
|---|---|---|
| `vector-cli` | `vector_os_nano.vcli.cli:main` | 交互/一次性 Agent |
| `vector-os-mcp` | `vector_os_nano.mcp.server:main_sync` | MCP stdio/SSE |
| `vector-eval` | `vector_os_nano.vcli.eval_runner:main` | verify-as-eval |

### 17.3 术语

| 术语 | 含义 |
|---|---|
| Kernel | 与具体机器人解耦的 Agent 编排机制 |
| World | 把领域 persona、工具、词表、capability 和 oracle 接入 Kernel |
| Embodiment | 机器人身体配置，例如 SO-101、Go2、Go2+Piper |
| Oracle | actor 不能任意伪造的实时只读世界数据 |
| VGG | Verified Goal Graph，显式 GoalTree 的执行/验证闭环 |
| Native | 模型直接驱动 action/verify/finish 的 producer |
| ReAct | 模型和通用 Tool 交替调用的对话循环 |
| Blackboard | 一次 VGG run 的结构化步骤数据存储 |
| Grounded | 有真实 oracle、因果和目标真实性支持的完成 |
| RAN | 执行过但没有充分证据 |
| Capability | 可路由的模型或算法能力 |
| Sim oracle | 直接读取仿真物理状态的验证谓词 |
| Strangler fallback | 新 Native 路径逐步取代旧 planner 时保留的回退 |

### 17.4 最终心智模型

如果只记住一句话，可以记住：

> `core.Agent` 提供身体，`World` 提供领域，`Skill/Tool/Capability` 提供动作，`VectorEngine` 负责选择与编排，`ExecutionTrace + Evidence Gate` 决定它是否真的完成。
