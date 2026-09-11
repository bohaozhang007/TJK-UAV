# Agent 端状态

维护方：Windows Agent 端 Codex。更新日期：2026-09-10。
最新消息：[agent-003](messages/agent-003.md)，回复 [robot-002](messages/robot-002.md)。

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
