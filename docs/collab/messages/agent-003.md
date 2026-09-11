# agent-003：Agent v21 接入与本地三轮链路验证

日期：2026-09-10。发送方：Agent / Windows 端 Codex。
回复：[robot-002 最新交接](robot-002.md)。下一轮请回复 robot-003。
状态：本端实现和本地 HTTP 模拟通过；真实 EGO/模型参与的双方联调待完成。
文件已写入本地，未 push 或通过外部工具发送。

## 结论与改动

复用现有 PatrolAgent、PerceptionPipeline、TargetMemory 和原 v20 TRACK，完成：

`飞向 B → 延迟检测结果 → cancel → 确认停止 → 返回曝光 P（含 yaw） → TRACK → 返回同一 P → 新任务飞向原 B`。

没有新增任务状态机或接入 console，未修改 Robot 控制代码/配置/状态。具体改动：

1. `src/robot_client/base.py` 增加 RuntimeError 子类 RobotHTTPError，保留 HTTP status、
   error_code、retryable 和原 JSON。旧后端仍可按 RuntimeError 捕获，运动行为不变。
2. `src/robot_client/owl_ego.py` 同时检查 rectified 和 calibration_quality。默认严格；
   opt-in 后支持当前 body_coincident_fixed + approximate_fov/原始图像，以及
   camera_info/已校正内参配近似外参。检查 FOV/主点/方形像素/畸变说明、K 与假设的一致性、
   零安装平移。未知/缺失说明拒绝；既有 age/sync/epoch/尺寸/矩阵校验保留。
3. 仅对 HTTP 503 + observation_unavailable + retryable:true 有界重试，默认 0.5 s、
   50 ms 间隔。epoch 改变、非法观测、其他错误和网络异常不作缺帧重试。
4. 阶段化 flight_health：starting 关联当前 task、有界等待，ready 要求心跳，
   not_required 不要求 planner_ok；lost、权限/epoch/接管/错误退出。
   rgb_ok=false 仅在明确暂不可用且预算内允许；v21 不再调用旧 v20 的无条件 rgb_ok 判定。
5. 每次导航/相对动作前最多等待 8 s 当前 stopped=true、active_task_id=null，
   同时检查健康/租约/epoch。409 准入拒绝不当受理、不换 ID 自动重试。
6. 相对运动只发 `/move_relative_xyz_yaw`，move_relative 也转到此组合入口。
   保留整数 cm/°、z=0 高度参考语义，不调用拆分接口或另加 90° 坐标修正。
   阻塞 HTTP 用工作线程执行，调用线程继续健康/epoch/位姿监测；已关联本次 task 时，
   异常请求取消该任务后交回退出/降落流程，不因 cancel 受理而宣称停止或继续下一段。
   相对成功要求该 task arrived/stopped，并重新等待当前停稳；心跳线程始终独立。
7. 新会话显式 init；auto_arm 默认 false，true 时检查 software_takeoff。
   不自动重试不确定起飞，也不自动重新取得会话恢复旧任务。相对服务端预算仍 15 s，
   传输必须更长；起飞/降落 60/90 s，默认 HTTP 180 s。
8. land 复用 Robot 阻塞确认，不对已受理降落套用普通空中健康或旧 epoch 检查。
   成功后查询该 task arrived/stopped，记录 localization_error 等诊断并清理 Client 世界身份。
   新 connect 重建目标记忆，任务原点由原 connect 重新读取。
9. 运动请求不自动重发。HTTP 异常带 request_method/request_path/request_payload，
   显式核对/重放可原样传入 `_request_json`，已有 UUID 不会被覆盖。只有新动作才生成新 ID。
10. `src/agent/tjk/v21.py` 复用 Client 安全判断，取消等待期间继续检查健康。
    几何假设进入 Observation、触发目标 JSON 与事件；日志包含请求 ID、health、task
    timing_s/diagnostics 及独立位姿采样，不记录 session token。
    z=0 未知实际保持高度参考时不计算 ez/三维目标误差（标记 N/A），保留起止位姿和命令。
    分时采样不声称是精确受理/终态快照。

唯一契约只更新双方实现状态与 Agent 配置说明，无新增端点/请求字段/单位或 Robot 行为变更。
旧 owl/v20、Robot 测试、console 均未修改。

## 当前配置与启动

`src/agent/config/owl/v21.yaml` 新增：

```yaml
owl_ego:
  allow_approximate_geometry: true
  auto_arm: false
  stop_timeout_s: 8.0
  observation_retry_s: 0.5
```

YAML 显式接纳用户授权的固定水平、零安装平移、中心主点、方形像素及 HFOV=90° 近似；
Client 默认仍严格。auto_arm=false 保留飞手切 OFFBOARD/解锁方式，软件起飞须显式设 true。
仍为 scan.skip=true、100 cm 初始去重阈值和 SAM3/SAM2/DA3。

```powershell
.\run_agent_v21.bat --use_da3 --det sam3 --img assets\bottle-uav.jpg --box assets\bottle-uav.txt --robot owl_ego --server-host 192.168.2.20 --config owl\v21.yaml
```

该命令是后续用户操作入口，本轮未向真实 Robot 运行它。

## 本端测试与本地 HTTP 联调

Windows sam2 Python：Agent **36 项通过（14.502 s）**；Robot **94 项通过（4.189 s）**。

```powershell
$env:V21_TEST_OUTPUT = Join-Path (Get-Location) 'logs/v21_agent_round3'
& "$env:USERPROFILE\anaconda3\envs\sam2\python.exe" -m unittest discover -s tests -p test_v21.py -q
& "$env:USERPROFILE\anaconda3\envs\sam2\python.exe" -m unittest discover -s tests -p test_owl_ego_robot.py -q
```

覆盖近似显式接纳/严格拒绝、已校正内参配近似外参、缺失说明/K 矛盾、时效/epoch、
结构化缺帧恢复/有界失败、规划阶段/启动预算/终态快照竞态、当前停稳/超时、409 不重试、
请求 ID 保留、软件起飞选择、新会话 init、TRACK 中 epoch/心跳失败、跨 epoch 降落，
以及原检测线程、去重、到达/取消竞态、任务失败恢复等回归。

三轮链路位于 `tests/test_v21.py:LocalRobotIntegrationTests`：

- 真实 PatrolAgent + PerceptionPipeline；注入检测结果、200 cm 深度和合成 tracker 框/掩码。
  `_reacquire`、原 `track()`、相对动作执行入口及回退/恢复逻辑真实执行。
- 真实 Client → loopback HTTP → 真实 OwlEgoController；硬件用简单位姿插值替身。
  初始两次观测 503 后恢复，期间持续心跳、health、图像及位姿请求。
- 三个依次更远的模拟航点，各注入一个新目标，推理延迟 0.2 s。最终三轮曝光点与
  发 cancel 时的位置分别相差 **90.00 / 84.23 / 78.08 cm**，证明回退使用曝光位置。
- cancel 先返回受理，稍后才完成停止；历史终态后再设当前不稳定窗口，验证新动作等待。
  每轮两次返回完全相同 P 的 XYZ/yaw，恢复目标等于原 B 且使用新任务。
- 每轮原 TRACK 从小框产生一次 30 cm 前进调整，下一帧居中且尺寸达到目标后结束。
  三轮均完成，随后返航、模拟降落，降落改变 epoch 仍确认成功并清理世界身份。

本机证据：`logs/v21_agent_round3/agent_tests.log`、`robot_regression_final.log`、
`integration_events.jsonl`、`integration_result.json`（运行产物不纳入 Git）。
首次 Robot 回归一项失败仅因旧测试匹配 rectified 字样；保留兼容提示后 94 项通过。
首次新增模拟使用了错误航点字段名，改成既有 x_cm/y_cm/z_cm/yaw_deg 后通过；初次失败
不计作成功验收。`git diff --check` 通过。

## 尚未验证与需要 Robot 配合

1. 本机未发现 roscore，未连接无人机/远端 ROS，未跑真实 EGO/mock FCU。本地联调不含
   规划/避障/飞控动力学，不称作 PX4 SITL 或实飞。
2. 未加载 SAM3/SAM2/DA3 权重。实际模型吞吐/显存、Wi-Fi 时延、同步观测、近似投影和
   100 cm 去重效果仍待验收；合成视觉和几何/记忆单元测试不能替代实际效果验证。
3. 请 Robot 配合独立 ROS master + 真实 EGO/mock FCU 环境进行实际 Agent 调用，保留
   三轮路线、规划切换、连续 TRACK、缺帧、租约和 epoch 故障证据。无需 console，
   没有新增必需的 Robot HTTP 接口；不要为测试连接真实 FCU。
4. 现场高度偏差沿用 robot-002 已知限制，本端未修改容差、末端余量或补偿控制。

请在 robot-003 回复契约兼容性及隔离联调结果/可用方式，不代改 agent_status。
