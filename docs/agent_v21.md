# Agent v21 / OWL EGO

本机入口：

```powershell
.\run_agent_v21.bat --use_da3 --det sam3 --img assets\bottle-uav.jpg --box assets\bottle-uav.txt --robot owl_ego --server-host 192.168.2.20 --config owl\v21.yaml
```

需要启动符合当前 [协议](owl_ego_contract.md) 的 `owl_ego` 后端；旧 `owl` Server
不支持 v21。Agent 已按 robot-002 接入，本地三轮 HTTP 模拟通过，详见
[agent-003](collab/messages/agent-003.md)；尚未完成真实 EGO/模型参与的双方联调。

`owl/v21.yaml` 的 `owl_ego.allow_approximate_geometry: true` 显式接纳用户指定近似相机，
Client 默认仍严格。`auto_arm: false` 保留飞手切 OFFBOARD/解锁的起飞方式；若本次操作
选择软件起飞，显式设置 `owl_ego.auto_arm: true`，Client 会检查 software_takeoff 能力。
`init` 本身不解锁。每次新会话都重新 init，不继承上个会话的初始化结论。

Agent/SAM2、SAM3、DA3 分别沿用 sam2、sam3、da3 Conda 环境。旧 run_agent.bat
默认仍运行 v20；新 wrapper 仅在本次调用选择 v21。也可直接在正确环境运行
`python src/agent/tjk/v21.py ...`，但需要自行启动 SAM3/DA3 服务和配置 PYTHONPATH。

## 行为

- 航点只定义访问顺序，沿途持续检测。移除 v20 的 `only_arrive` 配置。
- 采集线程定频获取同步观测；SAM3 工作线程检测，有候选才调用 DA3 做粗定位。
- `patrol.det_interval` 默认 1；可通过 `--det-interval N` 覆盖。按唯一曝光帧计数，
  第 1、1+N、1+2N 帧有资格送检。繁忙时仅保留最新待处理帧，并记录覆盖计数。
- 命中未处理目标后取消巡航，确认停止，等待检测工作线程空闲，再回到拍摄位姿。
  空闲等待期间会话心跳持续。重检测匹配目标并初始化 SAM2，复用 v20 TRACK。
- TRACK 完成后直接回拍摄位姿，再重新提交原下一个航点；无 SEARCH 和 SCAN。
- 航点到达时处理在途结果，并补拍最后一帧；返航回任务原点时不再检测新目标。
- 普通视觉任务失败返回拍摄位置。continue 策略继续路线，return_home 策略提前
  返航。定位/控制/导航失败中止任务，并在已取得控制会话时请求降落。
- 所有航点结束后回初始悬停位置，最后请求降落。未取得会话不发送降落指令。

## 去重和参数

同一次 mission 的固定世界坐标中，距离已记录目标 **小于** 100 cm 视为同一个
目标；阈值在 `patrol.dedup_distance_cm`。记录 pending/completed/failed，失败
有冷却时间和次数上限。定位 epoch 变化直接中止，不沿用旧目标坐标。

当前 SAM3 适配器仅返回框，初始位置从框中心 50% 区域的 DA3 有效深度稳健估计。
回退重检测后和 TRACK 完成后使用 SAM2 前景掩码复核。它代表可见表面位置，并非
物体几何中心；标定误差、深度偏差、目标尺寸或背景混入仍可能影响 100 cm 去重。
框核心深度无法通过有效点数/离散度检查时记录拒绝原因，不把它当作已处理目标。
这一近似不提供完美实例识别，也不是避障保证。

DA3 原有服务已经返回对齐 RGB 的厘米深度，因此无需修改服务；K 和完整相机变换
来自 Robot 的同步观测。严格模式由 Robot 去畸变并缩放 K；当前近似模式保持
rectified=false，HFOV=90°、固定水平和零安装平移是显式假设，写入目标记录和事件。

每次导航和 TRACK 动作前最多等待 8 s 当前停稳：health.stopped=true、active_task_id=null，
并检查权限/epoch。planner starting 允许有界启动，not_required 不要求心跳；lost 退出。
仅对 503 + observation_unavailable + retryable=true 重试，预算 0.5 s、间隔 50 ms。
TRACK 阻塞期间心跳独立运行，并轮询健康/位姿；z=0 原样传递、沿用 Robot 高度参考。
运动请求不自动重发；异常保留原请求 ID/body 以供核对，不把 HTTP 失败当作停稳。
已受理降落由 Robot 确认，期间不套用空中权限或旧 epoch 检查；成功后清理 Client 世界身份。

`owl/v21.yaml` 暂保留 v20 TRACK 构造器所需的 SEARCH/SELECT/SCAN 参数，v21 不执行
这些阶段。`track.skip=false`、`scan.skip=true` 为强制要求。示例航点仍需按场地配置。

## 日志与离线验证

`logs/v21_<timestamp>/` 包含 log.txt、config.json、events.jsonl，以及每个目标的
触发图、拍摄位姿/内参/变换、检测框和 TRACK 可视化。事件包含检测耗时、去重、
状态切换、导航目标、任务结果和采集/覆盖统计，并记录请求 ID、health、task diagnostics/
timing_s、近似几何与独立位姿采样。分时采样不作为精确的受理/终态位姿快照；z=0
动作的高度参考未由接口返回，因此不以“起点实测高度+0”计算高度/三维目标误差。

```powershell
& "$env:USERPROFILE\anaconda3\envs\sam2\python.exe" -m unittest discover -s tests -p test_v21.py -v
```

测试不加载模型、不连接无人机。包括真实 PatrolAgent、原 v20 TRACK、Client 与 Robot
HTTP controller 的三轮联调；视觉输出和运动硬件为合成数据，不包含 ROS/EGO/FCU。
设置环境变量 `V21_TEST_OUTPUT` 可保存联调事件和结果。实际 SAM3/DA3 吞吐、显存占用、
Wi-Fi 时延、几何精度与真实 EGO 链路仍需后续验证。
