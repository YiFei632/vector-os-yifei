# MolmoSpaces RBY-1 + OneRING + A* 启动说明

当前导航链路只包含 OneRING、Vector 本地目标识别和 RGB-D A*：

1. 未知目标优先由 OneRING 输出离散前进/后退/旋转动作。
2. 已知或已识别的目标坐标直接交给本地 A*。
3. OneRING 不可用、输出非导航动作、动作执行失败或连续错误停止时，Vector 先从 MolmoSpaces bridge 获取其缓存的全局 `ProcTHORMap` 和目标对象坐标，再切换到全局 A*；地图获取失败才退回当前帧局部 A*。

OneRING 控制回路中不再同步调用通用远程 VLM 进行场景描述/房间识别，也不再逐个
对象运行 Grounding-DINO。这些操作曾使每个 30°关键帧阻塞约 15–25 秒。未知环境
的最新 RGB 帧现在直接进入 OneRING；全局对象与地图读取只在连续三次失败后的
A* 回退阶段执行。

普通终端文本统一进入 DeepSeek agent-layer/ReAct tool loop；VGG fast path、
硬编码 RBY-1 文本 parser 和 direct-text 旁路不再抢占输入。`/rby1`、`/model`
和 `!` shell 等显式控制命令仍按各自的管理路径执行；紧急停止词保留本地安全旁路。

MolmoSpaces reset 后处于无目标待命状态：不会保留 sampler 临时抽取的对象，不会
构造或预热其内置 A* policy，也不会自行移动。OneRING skill 启动时才通过 bridge
的 `prepare_navigation` 将用户目标类别同步到 MolmoSpaces `NavToObjTask`。因此
MolmoSpaces 终端中的 task description、距离和可见性评估与 OneRING instruction
使用同一目标。

## 终端 1：OneRING

```bash
cd /media/fishyu/fish-14tb-12/YiFei/vector-os-nano

CUDA_VISIBLE_DEVICES=1 \
MPLCONFIGDIR=/tmp/matplotlib-onering \
conda run --no-capture-output -n ring \
python scripts/onering_model_server.py \
  --repository /media/fishyu/fish-14tb-12/YiFei/OneRING \
  --checkpoint /media/fishyu/fish-14tb-12/YiFei/OneRING/checkpoints/OneRING/ring_model_step_40356421.ckpt \
  --siglip-dir /media/fishyu/fish-14tb-12/YiFei/OneRING/checkpoints/SigLIP \
  --debug-dir /media/fishyu/fish-14tb-12/YiFei/OneRING_Debug \
  --device cuda:0 \
  --max-seq-len 128 \
  --max-steps 80 \
  --host 127.0.0.1 \
  --port 7801
```

健康检查：

```bash
curl -s -X POST http://127.0.0.1:7801/health
```

实时输入窗口：

```text
http://127.0.0.1:7801/debug
```

页面每 250 ms 刷新导航/操作相机输入、当前指令和最终动作。每一步输入也会保存到：

```text
/media/fishyu/fish-14tb-12/YiFei/OneRING_Debug/navigation_*.png
/media/fishyu/fish-14tb-12/YiFei/OneRING_Debug/manipulation_*.png
/media/fishyu/fish-14tb-12/YiFei/OneRING_Debug/latest.json
```

服务终端会逐步打印：

```text
[OneRING] step=1 action=rotate_left input=(H,W,3)/(H,W,3) debug=...
```

Vector 的 `INFO` 日志还会逐步打印 `round_trip_ms`、`execution_ms` 和
`observation_ms`。`latest.json` 中的 `latency_ms` 会进一步拆分 OneRING 服务端的
图像解码、模型推理、debug 保存和总耗时。RBY-1 只有一条 GoPro 输入时，同一帧
只压缩和传输一次，服务端会将其复用于 manipulation 输入。

## 终端 2：MolmoSpaces bridge

```bash
cd /media/fishyu/fish-14tb-12/YiFei/molmospaces

export MLSPACES_CACHE_DIR=~/.cache/molmo-spaces-resources
export MLSPACES_ASSETS_DIR=/media/fishyu/fish-14tb-12/YiFei/molmospaces/molmo-spaces-resources
export PYTHONPATH=/media/fishyu/fish-14tb-12/YiFei/molmospaces

conda run --no-capture-output -n mlspaces \
python /media/fishyu/fish-14tb-12/YiFei/vector-os-nano/scripts/molmospaces_rby1_bridge_server.py \
  --adapter molmo_spaces.bridge.rby1_interactive_adapter:create_adapter \
  --host 127.0.0.1 --port 8765 --log-level INFO
```

## 终端 3：Vector CLI

```bash
cd /media/fishyu/fish-14tb-12/YiFei/vector-os-nano

export ONERING_URL=http://127.0.0.1:7801
export ONERING_TIMEOUT=60
export ONERING_ENABLED=1
export ONERING_CONTROL_STEPS=1
export RBY1_ASTAR_ROBOT_RADIUS=0.05
export RBY1_CAMERA_SYSTEM=gopro_d455
export RBY1_VLN_CAMERA=head_camera
export RBY1_GOPRO_RECTIFIED_VFOV=94.0

export GROUNDING_DINO_REPO=/media/fishyu/fish-14tb-12/YiFei/GroundingDINO
export GROUNDING_DINO_CHECKPOINT=/media/fishyu/fish-14tb-12/YiFei/GroundingDINO/checkpoints/groundingdino_swint_ogc.pth
export GROUNDING_DINO_DEVICE=cuda:0

CUDA_VISIBLE_DEVICES=2 conda run --no-capture-output -n vector_os_nano \
vector-cli \
  --molmospaces-rby1-host 127.0.0.1 \
  --molmospaces-rby1-port 8765 \
  --molmospaces-rby1-scene 0 \
  --molmospaces-rby1-timeout 300 \
  --molmospaces-rby1-vgg \
  --molmospaces-rby1-viewer \
  --verbose
```

进入 CLI 后：

```text
/rby1 connect
/rby1 reset 0
/rby1 observe
导航到桌子
```

在发送导航指令之前，`/rby1 observe` 应显示：

```text
task_description: Waiting for Vector navigation target
external_navigation_target: null
```

此时 `step_waypoint` 会被拒绝，opaque 的 `/rby1 go ...` 也不会启动 MolmoSpaces
内置 policy。请将普通导航文本直接输入 Vector CLI，使其经过 DeepSeek agent-layer
并调用 `onering_navigation`；`prepare_navigation` 成功后才会激活目标与运动控制。

`/rby1 reset` 会显式请求 `RBY1GoProD455CameraSystem`。OneRING 只接受该系统的
`head_camera`。Bridge 将 MuJoCo 的垂直 FOV 设为 GoPro 文档对应的 `94°`，
再将 `1024x576` 渲染中心裁剪为 `768x576` 的 4:3 rectified 流；OneRING
会校验输入必须为 `768x576`，不会静默使用旧的 `640x480` 或未裁剪流。

## 验证

OneRING 正常执行时：

```text
planning_sources.onering > 0
```

切换到本地 A* 后：

```text
trajectory_planners.astar > 0
```

定向测试：

```bash
pytest -q \
  tests/unit/test_onering_navigation.py \
  tests/unit/test_onering_model_server.py \
  tests/unit/vcli/test_rby1_slash_command.py
```
