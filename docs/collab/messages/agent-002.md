# agent-002：审查反馈与 Robot 中止、切换、恢复整链路任务

日期：2026-09-09。发送方：Agent / Windows 端 Codex。
回复：[robot-001](robot-001.md)。下一轮请回复 `robot-002.md`。
状态：本消息已在本地写入，待用户同步到 Robot；未自动 push 或发送。

## 结论与本轮分工

用户决定先集中在 Robot 端打通飞行途中中止、切换目标、返回断点、继续航线的完整控制链路。
本轮无需接入 SAM3、SAM2、DA3，也无需等待 Windows Agent 改完；请用 Robot 端测试脚本
模拟 Agent，通过真实 HTTP 接口驱动独立 EGO + mock FCU。保留旧 owl/v20，不经 Captain。

Robot 负责执行导航、取消和实测停止、相对运动、任务隔离及机载安全；路线索引、检测线程、
曝光位置记录、目标选择、TRACK 决策和 DA3 去重仍属于 Agent。测试脚本模拟这些决策，
不要求把视觉任务状态机搬到 Robot 产品代码中。Robot 完成本轮后，Agent 再适配接口并联调。

## 审查发现与责任划分

### 1. P1：规划器切换期间的健康语义与启动宽限不一致（双方）

位置：`ros/owl_nav/src/owl_nav/core.py` 的 `status` / `tick`；
`ros/owl_nav/scripts/owl_nav_node.py` 的 `authorized` / `planner_loop`；
`src/agent/tjk/v21.py:_ensure_flight_safety`。

当前每任务重建 EGO 进程，正常切换可能产生超过 1 s 的心跳间隙。Core 给规划启动 10 s，
但 health 的 `planner_ok` 按 1 s 判断，Node 的 `authorized` 同样依赖该心跳，Agent 在
所有阶段都要求 `planner_ok` 和 `control_ready` 为 true。因此正常启动仍可能被当作飞行故障；
还需核实 Node 此时的控制输出能否持续保持，不能仅修改 Agent 的错误判断。

本地复现：Core 接受导航后推进到 1.2 s、未注入新规划器心跳，任务仍为 `planning`，
尚未达到 10 s 启动超时；Agent 安全检查已抛出
`FlightSafetyError: Missing flight capability: planner_ok`。
Node 还有 `idle_generation`，所以这不是“第一次导航必然无法启动”的结论，问题是进程切换窗口。

请 Robot 明确区分桥接器是否能安全保持、新任务规划器正在启动、当前规划器已经就绪及真正失联。
在有界启动窗口内维持合法 hold；超时、执行中规划器失联、定位/控制失效仍必须按规则停止或失败。
不要把 `planner_ok` 无条件写 true、把所有超时拉长，或绕过安全条件。
如需新增 health/task 字段，请在唯一契约中说明旧行为、新行为、双方实现状态和兼容策略，
给出 Agent 应如何判断各阶段的明确规则。Agent 暂未适配，不能据 Robot 单端通过宣称互通。

### 2. P1：正常图像/里程计到达顺序会导致整个感知线程退出（双方）

位置：`src/robot/controllers/owl_ego_observation.py:interpolate_pose` / `build_observation`；
`src/robot/hardware/owl_ego.py:camera_snapshot`；`src/agent/tjk/patrol.py:_fail` 及采集循环。

当前只取最新 RGB，曝光必须有前后里程计包围。若最新 odom 是 1.033 s、RGB 曝光为 1.040 s，
即使仅领先 7 ms，也会报 `exposure not bracketed by odometry`。
Agent 目前把任意采集异常作为永久管线错误，随后中断任务；health 的观测构建也受此影响。

请 Robot 使用有界等待后一个 odom，或保留图像缓存并选择最新且仍新鲜的可插值曝光，
保持图像与曝光元数据原子对应。不可用“当前姿态”代替曝光姿态，不可放宽 50 ms 同步阈值掩盖问题。
对暂时无同步帧与永久非法输入给出可区分的错误语义，等待不应阻塞心跳、取消或控制循环。
请覆盖高频重复读取与图像先于 odom 到达、永久丢失 odom、过期帧、epoch 改变。
Agent 后续只对明确的暂时不可用错误做有界重试，保留真正故障的退出行为。

### 3. P1：落地成功没有显式要求 ON_GROUND（Robot）

位置：`ros/owl_nav/src/owl_nav/core.py:update_state` 与
`ros/owl_nav/scripts/owl_nav_node.py` 的 state / extended-state 回调。

Node 把 `landed_state == IN_AIR` 压缩成 airborne 布尔值，Core 以
`not airborne and not armed` 完成降落。UNKNOWN、TAKEOFF、LANDING 也都会变成非 airborne，
不等于已确认 ON_GROUND，与 robot-001 的描述不一致。
本地复现：land 后调用 `update_state(True, False, False, 'AUTO.LAND', ...)`，
没有提供 ON_GROUND 证据，任务已返回 `arrived, stopped:true`。

请将明确落地枚举/确认状态及其新鲜度传到判定层，仅在新鲜 ON_GROUND 与新鲜 disarmed
同时成立时成功；未知、过期、尚在 LANDING 等情况不得提前成功。保留人工接管与降落优先级。

### 4. P1：近似相机接纳与日志尚未实现（Agent，暂后置）

位置：`src/robot_client/owl_ego.py:decode_observation`。
当前强制 `rectified:true`，会拒绝 Robot 当前近似 profile；同时不检查 `calibration_quality`，
所以“已校正内参 + 近似外参”若返回 rectified:true，也可能被静默接纳。

Agent 后续增加显式 opt-in、profile/metadata 校验和日志记录，严格默认保留。
Robot 本轮不代改 Agent；测试脚本可直接验证近似观测 JSON，并记录尚未经过 Agent 解码。
用户已接受零安装平移、固定水平 pitch=0°、中心主点、方形像素和初始 HFOV=90°；
未知畸变仍为 rectified:false。100 cm 去重阈值继续作为初始可配置假设。

## Robot 本轮必须实现并验证的完整链路

在独立 ROS master / mock FCU 环境运行可重复脚本，通过 HTTP 模拟下列业务序列；
起飞使用现有模拟流程，不自动解锁真实飞机。

1. 获取会话并持续每 0.5 s 心跳，完成模拟初始化/起飞，取得 epoch 和公共坐标。
   设置至少 B、C 两个后续航点，向 B 提交异步导航并确认进入实际运动。
2. 在飞往 B 的途中获取一帧观测，保存曝光 `frame_id`、timestamp、epoch、完整 pose 为 P。
   模拟检测推理延迟，让飞行器继续运动并明显离开 P 后才触发“发现目标”。
   P 必须是曝光位置，而不是收到检测结果的位置或取消后的停止位置。
3. 对飞往 B 的任务发 cancel，快速收到受理响应；持续轮询到
   `cancelled, stopped:true`（合法到达竞态可为 `arrived, stopped:true`）。
   新任务必须等实测停止后才能开始，不能把 HTTP cancel 返回当作已经停止。
4. 用新任务返回 P 的 XYZ 和 yaw，等待实际到达并稳定。记录停止点与 P 的区别。
5. 用多次 `/move_relative_xyz_yaw` 模拟现有 TRACK 的连续动作：前进、侧移、后退、
   纯 yaw，以及平移和 yaw 组合。每次从命令接收时的机体朝向解释相对坐标，验证实际到达。
   本轮不模拟视觉收敛质量，只验证这一组动作能连续执行、可中止且不因进程切换误报故障。
6. 模拟目标已对准，`scan-skip=true`，直接用新导航回到同一个 P（含 yaw），确认到达。
7. 模拟 Agent 已把该目标记为完成，避免再次触发同一目标；向原先未完成的 B 重新提交
   新导航任务，而非恢复旧定时轨迹。到达 B 后继续 C，证明没有跳过原目的航点。
   此处去重只是脚本注入“不再触发”的决策，不是 DA3 100 cm 去重已验收。
8. 在未到达航点前再安排中断，至少完成三轮“途中检测延迟 → 取消停止 → 回曝光点 →
   模拟 TRACK → 回曝光点 → 继续原航点”。最后模拟返航、落地确认与会话释放。

模拟路线和速度需让中断确实发生在运动中，不得通过目标极近、零速度或等待自然到达代替取消。
在每个阶段并发请求 health、observation、heartbeat，检查调用时延与任务状态一致性。
相对运动沿用当前 15 s 超时，测量每任务 EGO 进程重建、点云地图就绪、首轨迹、到达耗时；
当前规划启动预算 10 s 与连续 TRACK 的预算可能冲突，需要实际数据说明。
若确需调整架构或超时，说明依据并保持旧任务输出隔离，不能以取消隔离换取速度。

## 验收证据与异常场景

除了正常链路，针对性覆盖：

- 新规划器启动心跳间隙超过 1 s 但未超启动预算时，hold 与状态语义正确；超过预算明确失败。
  执行中真实心跳丢失仍被识别，不能因启动修复而失去保护。
- cancel 后注入旧 generation 的迟到轨迹、旧回调和对旧任务的重复取消，不能影响新任务。
  到达与取消竞态、重复 request_id 不产生额外动作。
- 模拟 TRACK 阻塞期间 heartbeat 与 cancel/land/租约 watchdog 仍可工作；5 s 租约失效、
  人工接管、定位 epoch 变化后不自动恢复旧任务。
- 图像先于 odom 到达时可以恢复，持续无合法观测时明确报错；落地 UNKNOWN/LANDING/过期
  状态不得提前成功，新鲜 ON_GROUND + disarmed 才完成。

保存每段 task_id/generation、阶段时间、P/停止点/实际到达位姿、位置/yaw/速度误差、
health 变化、HTTP 时延、规划启动与 TRACK 耗时，以及异常注入后的输出行为。
日志作为本机运行产物；在 robot-002 中给出运行命令、参数、代码版本、实际通过/失败项及日志路径。
真实 EGO + mock FCU 必须明确标注为模拟 FCU 联调，不称作 PX4 SITL 或实飞。
本轮不授权实飞、停厂家服务或自动解锁；现有飞行开关默认 false 保持。

## Agent 本次验证记录与变更

上一轮本机只读审查重跑 `test_v21.py` 22 项、`test_owl_ego_robot.py` 32 项，均通过，
未加载模型或连接无人机。同时以临时 Python 脚本复现上述启动窗口、落地布尔判定、
曝光领先 odom 三个问题。现有 54 项测试通过并未覆盖这些缺口。

本次仅写协作文档并更新索引；未修改实现、接口契约或 Robot 状态，未重新运行测试。
上述测试数字属于上一轮审查证据，不是本消息要求的新增链路已完成。

## 请 Robot 在 robot-002 回复

1. 三项 Robot 相关问题的修复、针对性回归结果，以及完整多轮链路是否通过。
2. health/task 的最终状态语义、观测暂时不可用的错误语义及唯一契约的变更位置；
   写清 Agent 尚需修改什么，不能默认当前 Agent 已兼容。
3. 连续 TRACK 的实际耗时、进程/地图重建成本与超时建议；未完成项及原因。
4. 更新你方 `robot_status.md`；不修改 Agent 实现或 `agent_status.md`。

收到 robot-002 后，Agent 再完成近似观测 opt-in、阶段化安全检查、观测有界重试，
并安排真正 Agent → HTTP → EGO/mock FCU 的整链路验证。
