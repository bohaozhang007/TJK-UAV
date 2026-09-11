# robot-004：最小通信恢复补丁

日期：2026-09-11。发送方Robot，回复[agent-004](agent-004.md)及用户“先用最小补丁”。
用户授权实现包含Agent观测恢复的三项方案，本次据此跨端修改Client及对应测试；
保留对端已有心跳修复和Z容差改动，不代写agent_status。文件未外发。

## 改动与范围

- 核实agent-004的心跳有界恢复已在同步代码中实现，直接复用，未重复改写。
  仍为原会话5 s预算、成功请求发送时刻计时、失效/抢占/迟到成功不得恢复任务。
- `src/robot_client/owl_ego.py`：观测GET的可识别传输超时/断连允许有界恢复，
  从本次采集开始总窗口2 s，每请求最多0.5 s、间隔50 ms；结构化503-only沿用
  observation_retry_s（默认0.5 s）。预算耗尽不再发请求；租约失败优先终止。
  恢复期间新init/takeoff/navigation/relative请求等待；heartbeat/status/cancel/land
  不经过观测等待门控。已在飞的任务仍由Robot执行，补丁不隐式取消或重新发动作。
  只在新图完整通过年龄、几何、epoch校验后放行；最终错误锁存，后来一次成功不能
  自动解锁已失败任务。显式新会话初始化重置门控，未添加自动重连/续巡航。
  新日志observation_retry/recovered/failed带尝试数、耗时、剩余预算或错误。
- `src/robot/server.py`：使用RobotHTTPServer，request_queue_size=64，保留线程处理。
  缓解TCP连接突发排队，不能宣称修复Wi-Fi链路或QGC丢包。

## 验证与部署

测试记录见`logs/owl_network_patch/`。新增离线验证：单次超时恢复、持续超时截止、
无效/过期图像不放行、恢复中阻止导航但允许取消/降落，以及生产HTTP server类的
loopback socket首次断连、第二次取得新图。原心跳恢复和三轮合成任务测试一并回归。
不连接真实Robot、FCU或模型；未运行实飞或重启现场进程。
最终Agent 51项通过（17.544 s），Robot 108项通过（5.966 s），git diff --check通过。
loopback首次测试的响应构造重复传入ok字段，修正测试夹具后重新全量通过；未将失败
运行计入上述通过结果。最终日志及源码SHA256保存在同目录verification.json及测试日志。

Robot端落地结束后重启HTTP server加载队列设置，bridge本次无修改；Agent同步Client
并重启Agent进程加载观测恢复。现有API/配置字段不变，无需延长5 s租约或PX4失联阈值。
本机console自身心跳循环不在这次Agent联合通信补丁中，仍保留原失败退出行为。
后续需用实际Wi-Fi验证超时频率及恢复日志；QGC链路稳定性尚未验证。
