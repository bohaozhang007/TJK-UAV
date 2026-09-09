# Robot 端状态

维护方：OWL 无人机端 Codex。更新日期：2026-09-09。
最新回复：[robot-002](messages/robot-002.md)，对应 [agent-002](messages/agent-002.md)。

## 当前结论

本轮 Robot 修复完成，真实 EGO + mock FCU + Robot HTTP 的三轮中断/恢复链路及
七类故障注入均通过。**这不是 PX4 SITL、实飞或实际 Agent/模型整链路验收。**
Agent 近似观测 opt-in、阶段化安全检查和观测有界重试仍待对端实现。
未修改 Agent/Client/模型实现、Agent 状态或此前消息。旧 owl/v20 保留。

## 用户最新决定与实际配置

| 项目 | 当前处理 |
|---|---|
| 定位/避障 | 继续使用现有 FAST-LIO 点云；无需 DA3 参与机载避障 |
| 相机中心 | 按用户要求近似等于曝光时无人机位置，安装平移取零 |
| 云台 | 用户指定忽略安装俯仰，按相对机体水平朝前（pitch=0°）建模；不发送云台控制命令 |
| 朝向 | 完整曝光机体姿态 × 固定安装旋转 × optical→body 轴转换，不需要 TF |
| 主点 | cx=W/2、cy=H/2 |
| 焦距 | fx=fy=W/(2tan(HFOV/2))；HFOV=90° 是本次显式初始假设，不是厂家参数 |
| 畸变 | 未知，保持原始 RGB 并缩放，不伪造 D 或宣称真实去畸变 |
| 观测标记 | rectified=false、calibration_quality=approximate、geometry_assumptions |
| 严格模式 | camera_info + tf 仍可选，真实标定校验仍保留 |
| 飞行配置 | flight_enabled/failsafe_validated/sensors_validated 均 false；未实飞 |

仓库配置：`src/robot/config/owl_ego.yaml`。工作空间的已有 `owl_ego.yaml` 不自动覆盖；
本次另生成 `/home/visbot/owl_ego_ws/owl_ego_approx.yaml`，保留实际 planner 路径与 SHA256，
几何设置使用当前仓库模板。后续可显式选择该配置。

## 本轮完成

- 规划器启动与 hold/control 授权分离，心跳严格绑定 generation；正常启动保持，
  执行中失联立即失败。启动预算仍 10 s，心跳超时仍 1 s，租约仍 5 s。
- 修复真实 EGO 早于 FSM 初始化收目标的竞态：私有 odom + DataDisp 初始化握手，
  观察地图输出后再发目标。逐任务独立进程与旧输出隔离不变，上游源码未改。
- RGB 缓存与最长 80 ms 等待，曝光前后 odom 插值，50 ms 同步阈值不变。
  暂不可用/epoch 变化/非法几何分别为明确的 503/409/422 错误。
- 降落必须有新鲜 ON_GROUND 与新鲜 connected/disarmed；未知/过期/降落中不提前完成。
- 增加任务 generation、计时和实测诊断，规划器日志与退出清理。

## 本轮验证

- Robot 43 项、原 Agent 22 项离线回归通过。日志：
  `logs/v21_robot_round2/robot_tests.log`、`agent_regression.log`。
- 独立 master 11421，真实 EGO + mock FCU：三轮“运动中保存曝光 P → 推理延迟 →
  cancel/实测停止 → 回 P → 五个连续 TRACK 动作及额外 TRACK 取消 → 回 P →
  以新任务继续原航点”，最后返航、落地、释放会话，全部通过，约 153.98 s。
- 三轮停止点距 P 为 25.30 / 22.33 / 22.46 cm，确认停止耗时约 0.55–0.62 s。
  成功 TRACK 耗时 1.619–4.597 s，保留 15 s 超时；首轨迹约 1.021–1.885 s。
- health/观测/心跳并发监测无错误；最慢约 59 / 55 / 128 ms，cancel 最慢约 34 ms。
  水平近似观测中，RGB 故意领先 odom 7 ms，仍正常恢复；age/sync 阈值均未放宽。
- 独立 master 11422：>1 s 启动保持、>10 s 启动失败、执行中心跳丢失、5 s 租约失效、
  epoch 重置、人工接管、land 抢占/严格落地确认，七类均通过。
- 迟到旧轨迹、重复取消旧任务、旧回调/心跳、幂等与到达竞态均有覆盖。
- 详细 source SHA、参数、全部任务/P/停止点/实测误差、HTTP 时延和 EGO 日志：
  `logs/v21_robot_round2/final_route/`、`logs/v21_robot_round2/final_faults_retry/`。
  中间失败与定位过程保留，说明见 robot-002，不将失败运行算作验收通过。
- 使用 EGO commit `9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010` 的既有独立 aarch64 构建，
  上游 checkout 干净。没有接入真实 master、停止厂家服务、控制真实云台或解锁/实飞。

## 等待 Agent

按[契约本轮补充](../owl_ego_contract.md#2026-09-09-robot-round-2-planning-transitions-and-observation-availability)
适配 planner_state/hold_ready/active_task_id；不再全阶段强制 planner_ok=true。
对明确 observation_unavailable 做有界重试，epoch/非法几何不得当缺帧重试。
近似 profile 须显式 opt-in，并同时检查 rectified/calibration_quality、记录假设。
请回复 agent-003 并更新 Agent 自己的状态，再安排实际 Agent→HTTP→EGO/mock FCU 联调。

## 尚待实机验证

厂家控制互斥及自动重启、FAST-LIO world 与 MAVROS local world 一致性、定位 reset 接线、
PX4 failsafe/RC 接管/制动距离/起降，以及实际相机投影、DA3 尺度和 100 cm 去重效果。
地图输出里程碑不是地图完整性或实机场地适用性的确认。飞行开关保持 false。

本轮完整交接见 [robot-002](messages/robot-002.md)；最初部署与相机排查保留在
[robot-001](messages/robot-001.md)，无需另建附件或 archive。
