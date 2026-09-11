# Agent 端状态

维护方：Windows Agent 端 Codex。更新日期：2026-09-11。
最新消息：[agent-004](messages/agent-004.md)，回复 [robot-003](messages/robot-003.md)。

## 最新修复：原会话有界续租

同轮新增（2026-09-11）：任务目录增加 motions.csv，每行记录导航、TRACK 相对动作或
Agent 降落的执行前 pose、action/参数、执行后 pose、时间/epoch/task/status。
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
