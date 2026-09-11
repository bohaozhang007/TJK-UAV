# robot-004：最小通信恢复补丁

日期：2026-09-11。发送方Robot，回复[agent-004](agent-004.md)及用户“先用最小补丁”。
用户授权实现包含Agent观测恢复的三项方案，本次据此跨端修改Client及对应测试；
保留对端已有心跳修复和Z容差改动，不代写agent_status。文件未外发。

## 最新补充：纯旋转高度偏差，先补诊断（2026-09-11）

### 15:09实飞录制复核：四段诊断已加载（后续结论）

用户只进行了纯旋转。复核`logs/owl_diagnostics/agent_20260911-150945/flight.bag`及
`logs/owl_live/20260911-150950-b0b468/events.jsonl`，新增同目录
`rotation_height_chain.csv`、`rotation_height_analysis.json`（源bag SHA256、任务详情）。
本次只读分析，未修改控制代码/参数、未重启服务或飞行。

三次受理的旋转分别为+30°、−30°、−30°：

| 次数 | 目标Z | 结果 | 终态Z误差 |
| --- | --- | --- | --- |
| 1，15:10:04 | 86.871 cm | 约2.42 s arrived | 3.689 cm |
| 2，15:10:25 | 86.871 cm | 15 s阻塞请求超时触发停止，随后cancelled | 取消前约11–13 cm，停止后旧目标误差15.051 cm |
| 3，15:10:45 | 98.339 cm | 约8.08 s arrived | 7.9986 cm，接近8 cm门槛 |

第二次取消前，goal/hold/output/published Z全部保持0.8687057197 m；Z速度和加速度
前馈为0。15:10:29–40实测约0.98–1.00 m，反馈vz中位−0.15233 m/s，而500 ms
位置差分中位+0.000978 m/s。第一次完成后15:10:08–25无旋转运动的悬停段，实测
Z从91.999升到97.622 cm，hold/输出仍是86.871 cm，证明初始偏差并非旋转专有问题。

15:10:40.967首次记录第二次任务stopping，hold/输出改为0.9833901525 m，任务旧目标
仍是0.8687057197 m。该变化发生在超时取消之后，符合invalidate捕获实测位置保持的
现有语义；不是之前偏高的原因，但会把已有偏差带入后续参考。第三次z=0继承98.339 cm，
实测一度109.719 cm，终态刚进入8 cm容差。第三次成功不代表回到最初86.871 cm目标，
也不代表已持续收敛。不要通过重复超时、取消再试来判断高度偏差已经解决。

15:10:58执行land，15:11:04.996 console确认arrived；记录有降落期间lio/world错位
localization_error，已受理降落仍完成。不能把这一降落阶段错误反推为空中初始偏差原因。

结论：同周期四段记录排除了第二次取消前Robot主动抬高参考这一解释；最终ROS输出
正确而反馈高度/速度不一致。停止保持又使后续目标上移，是后续参考变化的明确来源。
不改固定补偿、不把非零dz改为旧hold累加、不贸然改变取消保持语义。下一步需要本次
FCU ULog对齐内部目标、估计位置/速度、重置计数、控制状态；不需要继续重复纯旋转
来证明已经充分记录的现象。高度问题仍未修复，正负Z及稳定收敛两组验收尚未完成。

用户要求保持8 cm垂直到达容差、不固定减11 cm、不改变非零dz以实测为起点的语义。
本次仅分析已有Robot录制、核对厂家MAVROS源码和增加只读诊断；未改控制律、FCU参数、
飞行配置或厂家代码，未重启实机服务。此前通信补丁不解释本次高度偏差。

证据：`logs/owl_diagnostics/agent_20260911-145332/flight.bag`；现有分析器已导出，
新增`height_chain.csv`、`height_chain_analysis.json`，保留源bag SHA256及对齐局限。
纯旋转任务nav-830d42e3c374425b972b083d626c5eb4于14:54:02.169首次出现。
取14:54:07–14:54:17末段301个odom样本，按最近已收到status/setpoint对齐：

| 项目 | 结果 |
| --- | --- |
| 任务目标Z | 固定0.952324533 m |
| hold Z | 旧录制未记录，不能用代码推断冒充实测证据 |
| 发给MAVROS的Z | 固定0.952324533 m，与转换回world的值一致 |
| 同world实测Z | 中位1.064082146 m，范围1.051779–1.071408 m |
| Z速度/加速度前馈 | 均为0 |
| MAVROS控制字段 | coordinate_frame=1，type_mask=2048，仅忽略yaw_rate；位置、速度、加速度字段有效 |
| world竖直反馈速度 | 中位−0.133039 m/s |
| 500 ms位置差分竖直速度 | 中位+0.000125 m/s；是窗口差分，不当作真实瞬时速度 |
| 最近setpoint接收年龄 | 中位12.763 ms，最大48.457 ms |

该bag的setpoint连接callerid仅/owl_ego_bridge，末段健康审计conflicting_publishers=[]、
OFFBOARD、epoch不变、frame_alignment_ok=true。到14:54:17.225，yaw误差0.205°，
Z误差11.344 cm，仍超过8 cm，停止诊断已稳定。15 s相对请求等待后进入取消/降落，
原任务最终记录preempted by landing；14:54:23.896降落arrived。
因此不是yaw不对准，也不能把本次归为EGO轨迹到期；纯旋转无需EGO位置轨迹。

厂家源码核对：Frames.setpoint只旋转XY；local_position.cpp先NED→ENU再加init_pose，
setpoint_raw.cpp对位置减init_pose再ENU→NED。共享静态高度原点在误差中抵消，未发现
Robot层把11 cm加入输出的证据。此为源码核对，未取得当次FCU内部目标及init_pose运行
历史，不能以此证明整个FCU反馈链路无问题。同期LIO/vision_pose Z中位1.171220 m，
FCU local_position/pose中位1.064144 m，不能将两者直接替换计算到达误差。

结论更接近“最终ROS输出正确，实测稳定偏高”。反馈vz和位置变化明显不一致，与既往
FCU日志的估计速度偏差现象相似，但本次无FCU ULog，不能确定是估计融合、传感器、
控制器内部目标还是其他FCU环节，更不能直接增设补偿控制器或切换控制掩码。

### 只读诊断补丁及后续验证边界

Node在同一控制锁、同次tick输出发布后记录`control_sample`，包含task_id/status、epoch、
goal_z_m、hold_z_m、output_z_world_m、published_z_m、measured_z_world_m、Z前馈/反馈速度、
ROS设定值与odom时间戳、odom接收年龄、FCU模式、掩码、坐标profile和发布者审计年龄。
`control_sample_age_s`明确最后输出样本的新鲜度；不再输出时仍保留历史样本及年龄，
不得将它当作当前仍发布的控制量。goal为空表示该发布时刻没有活动任务。
诊断是同周期缓存快照，不宣称odom与发送时刻物理同步，也不证明FCU已接受该消息。
现有`/owl_ego/status`录制及分析器保留完整字段，无需新接口。

109项Robot回归通过（5.992 s），新增测试确认四个高度字段不混淆、不修改core状态、
快照不会被后续数组更新改写；node编译检查及git diff --check通过。
证据同录制目录diagnostic_tests.log。未进行实机两组动作验收，不能宣称高度已修复。

落地后重启bridge加载诊断，后续录制先验证纯旋转/水平移动时高度参考不变、实测收敛
8 cm内，再验证正负Z每次收敛后继续。任一步不能收敛就结束该组，不用放宽容差跳过。
若同周期四段仍显示最终输出正确而实测偏高，需对应FCU内部setpoint、local_position、
估计器及控制状态的当次ULog，才能决定底层修复；本轮未主动触发这些飞行动作。

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
