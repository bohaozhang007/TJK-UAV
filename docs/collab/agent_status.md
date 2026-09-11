# Agent 端状态

维护方：Windows Agent 端 Codex。更新日期：2026-09-11。
最新消息：[agent-005](messages/agent-005.md)，回复 [robot-004](messages/robot-004.md)。

## 最新配置位置与语义（替代下文中间决定）

两个开关均位于 src/robot/config/owl_ego.yaml 最上方、顶层：global_z_enabled:false、
vertical_tolerance_enabled:false。独立 8 cm Z 约束恢复为可选功能。Robot bridge 和
HTTP 从相同 YAML 读取；Agent 顶层和 session 配置传递已移除。缺字段默认 false。
Robot 111 项（5.677 s）、Agent 51 项（17.493 s）通过；日志 root_z_options_*.log
在 logs/v21_agent_round5。修改开关需重载 bridge/HTTP。未部署或实飞。

## 最新实现：可选全局 Z 参考

control.global_z_enabled 默认 false，保持当前相对 Z 行为；true 时相对 dz 累加
到已有世界 hold 高度，复用现有参考生命周期。绝对导航完成采用目标高度，中止/
失败重新取实测 hold，定位重置清除参考。没有重新引入独立 8 cm 容差。
Robot 110 项（5.775 s）、Agent 51 项（17.518 s）通过，日志在
logs/v21_agent_round5/global_z_robot.log、global_z_agent.log。尚未部署/实飞。
开关在 src/robot/config/owl_ego.yaml 的 control 下，现场 bridge/HTTP 使用相同值。

## 最新决定：取消独立 Z 到达容差（2026-09-11）

按用户最新指令，本机 Robot 已移除独立 8 cm Z 到达约束，恢复三维距离 15 cm、
yaw 和原停稳判定。移除 YAML/HTTP 中独立 Z 容差，旧配置残留字段忽略；保留
vertical_error_cm 诊断。Robot 108 项、Agent 51 项本地回归通过，证据在
logs/v21_agent_round5/remove_z_gate_robot.log 和 remove_z_gate_agent.log。
尚未部署或实飞，Robot 需同步并重载 bridge 与 HTTP server；历史保留 8 cm 的
建议被此决定替代。此修改不代表底层高度反馈偏差已修复。

## 最新现场复核：14:53 运行

通信补丁已同步，最新任务在 REACQUIRE 纯旋转阶段由 Robot 返回运动执行 15 s
超时 HTTP 504，未进入 TRACK；中途心跳已恢复，本轮无 observation 失败记录。
任务诊断高度误差 11.29 cm、三维 12.03 cm、yaw 近零且停稳：独立 8 cm 条件
正在阻止误判到达，但底层高度偏差仍存在。随后降落确认完成。需 Robot 核对
hold/setpoint/反馈链路；不能由此前 14:34 单轮改善认定根治。详见 agent-005。
本次仅静态代码/用户运行日志复核，未修改运行代码、执行新测试或实飞。

## 最新实现：独立 Z 到达容差（2026-09-11）

用户授权跨端修改后，本机 Robot core 已增加绝对 Z 误差 <= 8 cm 的到达约束，
同时保留三维 <= 15 cm、yaw 和停稳条件。配置 control.vertical_tolerance_m=0.08；
HTTP 新增 vertical_tolerance_cm，任务诊断新增 vertical_error_cm，旧配置默认 8 cm。
Agent TRACK 指令未改；此为 Robot 到达容差，不是 Agent dz 死区。
Robot 108 项（5.694 s）、Agent 46 项（16.871 s）离线测试通过；证据及现场同步事项
见 agent-004 最新补充。未部署、未实飞，尚未验证实际高度能收敛至 8 cm 内。
无人机需同步 core/controller 并让 bridge、HTTP 使用相同配置后重新加载才能生效。

## 最新现场复核（2026-09-11，11:45 运行）

已检查用户 logs/v21_20260911_114527：实际完成中止、回 P、TRACK、回 P、重新飞 B。
三次心跳超时均在原租约内恢复；最终由 observation 单次传输超时导致采集线程失败，
取消导航后 land 确认完成。观测传输异常恢复仍待实现，不是再次出现心跳永久退出。
Agent 与 console 使用同一运动 HTTP 接口；TRACK 连续非零 Z 以实测高度重定目标，
与约 10–12 cm 正高度偏差叠加。XYZ 总容差过滤未阻止前进中的微小 Z 修正。
偏差源头仍需 Robot 的目标/hold/setpoint/实测时序核对，不能仅归因于视觉参数。
本次仅代码/现有运行日志分析及文档更新，未改参数或控制代码，未新增测试或实飞。
详细数值、时间线和待 Robot 配合项见 agent-004 最新补充。下文未加载真实模型等
描述是早期本端模拟验证范围，不再代表用户尚未进行模型驱动的现场运行。

## 最新修复：原会话有界续租

同轮新增（2026-09-11）：任务目录增加 motions.csv。按用户最新要求精简为
started_at、finished_at、phase、action、action_xyz_yaw、before、after、error；后四列
以 (x,y,z,yaw) 两位小数保存，error 为期望减实际。状态/epoch/task 等细节移至 events.jsonl。
失败或未知状态保留，无法采样的位姿留空注明原因。46 项 Agent 回归通过（16.918 s），
含本地三轮 HTTP 模拟对 CSV 的位姿、动作、取消和降落记录校验；日志 csv_tests.log。

已修复心跳首次超时永久退出：以最近确认成功的 acquisition/heartbeat 发送时刻作为
5 s 本地期限起点，失败不刷新、迟到响应不复活。恢复期间新运动等待，恢复后核对
health/epoch/原任务；409、操作员接管或预算耗尽锁存失败，不重新申请会话或重发动作。
health/get_pose/task-status 的传输异常可触发续租恢复，确认成功后仅重读一次。
阻塞降落期间独立续租；清理异常单独记录，不覆盖原始任务错误。

本轮 Agent 46 项、Robot 105 项离线回归通过；包含本地 HTTP 三轮路线与降落期间
续租恢复。证据 logs/v21_agent_round4/，详见 agent-004。未修改现场 YAML、Robot
实现或状态；未实飞、未测试真实 Wi-Fi 故障、未 push。此前 Agent 接入记录如下。

## 当前安排

已完成当前契约的 Agent 接入，本端 36 项及 Robot 94 项离线测试通过。真实 PatrolAgent、
原 v20 TRACK、Client 与 Robot HTTP controller 已用合成观测和模拟运动完成三轮中断、
回曝光点、TRACK、回曝光点及新任务继续原 B。尚未完成真实 EGO/模型参与的双方联调。
本轮未接入 console，未修改 Robot 实现/状态，未 push、连接无人机或触发实飞。

## 已完成与验证

- v21 已有异步检测、曝光位置回退、复用 TRACK、返回后继续航点、DA3 空间去重与 owl_ego Client。
- 本轮适配：近似几何 opt-in、阶段化健康、当前停稳等待、结构化观测错误/有界重试、
  软件起飞选择、新会话 init、阻塞 TRACK 健康/epoch 监测及跨 epoch 降落确认。
- Agent 36 项通过（14.502 s）；Robot 94 项通过（4.189 s）。三轮本地 HTTP 联调中，
  每轮原 TRACK 均实际产生一次前进调整，两次回 P 和新任务继续原航点均断言通过。
- 证据在 logs/v21_agent_round3/：agent_tests.log、robot_regression_final.log、
  integration_events.jsonl、integration_result.json。模拟不包含 ROS/EGO/FCU 或模型权重。

## 当前配置假设

沿用 owl/v21.yaml、SAM3 + SAM2 + DA3、owl_ego；扫描跳过，去重半径初始 100 cm。
用户已接受固定水平 pitch=0°、零安装平移、中心主点、方形像素及初始 HFOV=90°。
这些为显式近似，未知畸变不得标记成已校正。YAML 已设 allow_approximate_geometry=true，
Client 默认仍严格。owl_ego.auto_arm=false 保留飞手方式；软件起飞须显式选择 true。
当前停稳等待 8 s，观测重试 0.5 s/50 ms，TRACK 15 s，HTTP 默认 180 s。

## 待完成与阻塞项

- 本机无 roscore；请 Robot 配合独立 ROS master + 真实 EGO/mock FCU 的 Agent 调用验证。
  没有新增必需的 Robot HTTP 接口，不需要 console。
- 尚未加载实际 SAM3/SAM2/DA3；Wi-Fi 时延、模型吞吐、近似投影与 100 cm 去重效果待验收。
- 现场高度偏差沿用 robot-002 的已知限制；本端未改变 Robot 容差/余量/飞行开关。
