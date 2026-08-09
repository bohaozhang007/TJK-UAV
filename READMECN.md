# TJK-UAV

[English README](README.md)

TJK-UAV 是一个面向无人机的单目标感知与飞行控制框架。它能够搜索指定目标、选择合适视角、跟踪并接近目标，将目标调整到图像中央附近，拍摄正面图像，并绕目标飞行以完成多视角拍摄。

项目将具体无人机控制与 Agent 任务逻辑分离：

- Robot Server 运行在无人机 SDK、仿真器或 ROS 系统一侧。
- Robot Client 负责将 Agent 连接到指定的 Robot Server。
- Agent 负责搜索、检测、视角选择、跟踪、接近、居中和环视拍摄。

目前已注册的 Robot 后端包括 `owl`、`tello` 和 `ue`。

## 快速开始

必须先启动 Robot Server，确认服务可访问且无人机准备完成后，再启动 Agent。

### 1. Robot 端

对于 OWL 真机，在无人机电脑上的仓库目录中运行：

```bash
chmod +x run_owl_console.sh
./run_owl_console.sh
```

该脚本会加载 ROS 环境，在 `8765` 端口启动 OWL Robot Server，并打开命令行控制台。通常的人工操作顺序为：

```text
init
takeoff
health
```

Agent 会识别无人机已经起飞，不会重复执行起飞。

也可以将 `src` 加入 `PYTHONPATH`，通过 `--robot` 直接选择后端：

```bash
export PYTHONPATH="$PWD/src"
python3 -m robot.server --robot owl --host 0.0.0.0 --port 8765
```

将 `owl` 替换为 `tello` 或 `ue` 即可切换 Robot：

```bash
python3 -m robot.server --robot tello --host 0.0.0.0 --port 8765
python3 -m robot.server --robot ue --host 0.0.0.0 --port 8765
```

### 2. Agent 端

在 Windows 上使用 `run_agent.bat` 启动 Agent v17。

文本提示词示例：

```powershell
.\run_agent.bat --use_da3 --det yolo --text "bottle" --robot owl --server-host 192.168.2.20 --config owl\v17_2.yaml
```

SAM3 视觉样例示例：

```powershell
.\run_agent.bat --use_da3 --det sam3 --img assets\bottle.jpg --box assets\bottle.txt --robot owl --server-host 192.168.2.20 --config owl\v17_2.yaml
```

`--box` 指向一个文本文件，文件中包含四个用空格分隔的 `xyxy` 像素坐标。

切换 Robot 时，需要保证 Robot Server 和 Agent 使用相同后端：

```powershell
--robot owl
--robot tello
--robot ue
```

当 Agent 和 Robot Server 运行在同一台电脑时，使用 `--server-host 127.0.0.1`。`--config` 路径相对于 `src/agent/config` 解析。运行日志和中间可视化统一写入项目顶层的 `logs/`，该目录不会被 Git 上传。

> 真机实验应从安全区域开始，并确保能够随时人工接管。

## 目录结构

```text
TJK-UAV/
├── assets/                  参考图像和视觉样例框
├── logs/                    运行日志和可视化（Git 忽略）
├── src/
│   ├── agent/
│   │   ├── config/          按版本和 Robot 划分的 YAML 配置
│   │   └── tjk/             按版本保存的 Agent 实现
│   ├── robot/
│   │   ├── controllers/     控制逻辑、坐标转换和安全判定
│   │   ├── hardware/        原始 SDK、ROS 或仿真通信
│   │   └── server.py        统一 HTTP Robot Server 和控制台
│   ├── robot_client/        各 Robot 对应的 Agent 侧 Client
│   ├── third_party/         检测、跟踪和深度模型适配器/服务
│   └── utils.py
├── run_agent.bat            Windows Agent 启动脚本
└── run_owl_console.sh       OWL Robot Server 启动脚本
```

## 使用 Codex 安装和定制

不同无人机的 SDK、ROS 工作空间、模型和 Agent 依赖差异很大，因此本仓库不规定统一的依赖安装方案。通常可以让 Codex 阅读现有实现，并只安装目标平台真正需要的内容。

### 1. 定制 Robot

向 Codex 提供新无人机的 SDK 或 ROS 文档、可用 topic/service、坐标系、相机接口、位姿来源、运动指令以及运行环境。要求它参考已有的 `owl`、`tello` 和 `ue` 实现，并完成：

1. 新建 `src/robot/hardware/<robot>.py`，只负责原始硬件、SDK、ROS 或仿真通信。
2. 新建 `src/robot/controllers/<robot>.py`，负责坐标转换、安全检查、阻塞运动完成判定和运动容忍误差。
3. 新建 `src/robot_client/<robot>.py`，实现统一的 Agent 侧接口。
4. 在 Robot Server、Agent 的 Client 构造逻辑和 `--robot` 选项中注册新名称。
5. 在执行 Agent 任务前，验证图像、位姿、健康状态、起降、相对 XYZ/yaw 运动和完成判定。

可以直接向 Codex 输入：

```text
阅读 src/robot、src/robot_client 和现有 OWL 适配，根据我提供的
SDK/ROS 文档新增 <robot-name> 后端。hardware 只负责原始通信，控制、
坐标转换和安全逻辑放在 controller，实现统一 client 接口并注册
--robot <robot-name>。不要修改无关后端，最后验证 Robot Server 接口。
```

### 2. 定制 Agent

向 Codex 说明任务流程、目标 Robot、检测器/跟踪器输入、期望输出和可用运行环境。Agent 需要实现哪些功能、采用什么模型以及安装哪些依赖，可以由 Codex 根据任务自行设计。要求它：

1. 阅读 `src/agent/tjk/` 中的最新实现，并复用统一 Robot Client 接口。
2. 需要保留实验历史时，新建一个版本化 Agent 文件，不覆盖旧版本。
3. 在 `src/agent/config/<robot>/` 下新增对应 YAML 配置。
4. 仅在新 Agent、模型服务或输入形式需要时修改启动脚本或 CLI。
5. 将日志和可视化结果保存在项目顶层 `logs/` 下。

可以直接向 Codex 输入：

```text
阅读最新 Agent 及其 YAML 配置，为 <任务描述> 新建一个 Agent 版本，
使用 <robot-name>。你可以自行设计任务阶段，并选择、安装需要的模型和
运行依赖。保留旧 Agent，新增对应 Robot 配置；必要时更新启动脚本，
真机运行前先使用模拟输入验证完整流程。
```
