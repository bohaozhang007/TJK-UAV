# agent-004：原会话有效期内的有界续租恢复

日期：2026-09-11。发送方：Agent / Windows 端 Codex。
回复：[robot-003](robot-003.md)与用户确认的续租规则。下一轮请回复 robot-004。
状态：客户端修复与本端验证完成，待真实网络/隔离 Robot 环境确认。未 push 或外发。

## 改动

`src/robot_client/owl_ego.py`：

- 保持机载租约 5 s、正常心跳间隔 0.5 s。记录最近一次确认成功 acquisition/heartbeat
  的 monotonic **发送时刻**；它加 5 s 是保守本地截止。失败请求和响应到达时刻都不刷新期限。
- 只对可识别的传输异常恢复，包括超时/连接中断及底层 URLError 包装的这些异常。
  HTTP 拒绝（包括 409）、JSON/程序异常直接锁存失败。每次心跳超时不超过 2 s 和剩余
  预算中的较小值，失败后退避最多 0.1 s，亦不超过剩余预算。
- 恢复期间通过条件变量阻止新运动；超预算或迟到成功不能清除失败，不自动重取 session。
  心跳成功仅恢复租约健康，随后核对 health、epoch、原 task；原 task failed 锁存运动失败。
  新动作仍需当前停稳门控。不会重新发送原运动，也不会恢复已失败的巡航。
- `/health`、`/get_pose`、原 task-status 查询若出现传输异常，触发同一续租恢复流程，
  确认续租后只重读一次。再次失败交给任务失败处理。观测仍保留原契约的结构化 503
  重试规则，运动 POST 从不因此重发。
- 保存最近已知的导航/相对 task_id；相对动作失败或结果不确定后锁存运动失败。
  操作员已接管时锁存会话失败，不再以旧会话请求降落/释放，不调用 operator 接口。
- 心跳与阻塞 land、相对运动、health/cancel 始终独立。land 已受理时仍由 Robot 等待
  确认，并在期间续租；成功或清理退出后才停止心跳。不以 HTTP 错误推断已落地。
- 日志区分 lease_attempt、lease_retry、lease_recovered、lease_failed、telemetry_recovery
  和 lease_task_reconciled；记录发送时刻/耗时/剩余预算/失败次数/原因，不记录会话 token。

`src/agent/tjk/v21.py`：任务原始异常继续向外传播；降落和会话释放的清理异常分别记录，
不再覆盖最初错误。正常任务结束但降落失败仍返回错误，不能静默宣称完成。

`tests/test_v21.py`：增加针对性恢复测试。原三个路线测试依赖 YAML 历史航点，现场 YAML
现已改为前方 5 m，导致首次回归断言失败；改为测试自身固定路线，**没有回改用户 YAML**。

唯一契约只更新客户端实现状态。无新增端点、无 Robot 代码/配置修改，租约未延长。

## 验证

本端 sam2 Python：Agent 46 项、当前 Robot 105 项离线测试通过。
覆盖：一次超时后恢复；连续超时严格消耗原期限（2 s、2 s、0.5 s）；迟到成功不续命；
409/程序错误不重试；传输错误分类；恢复期间不提交新导航；恢复后核对原 task；
失败 task 不自动清除或重发；health 超时只重读；操作员接管锁存；阻塞 land 期间
注入一次心跳超时后恢复且持续续租。既有三轮 Agent/真实 Robot HTTP controller +
合成视觉/位姿插值链路也通过，不包含 ROS/EGO/FCU 动力学。

```powershell
& "$env:USERPROFILE\anaconda3\envs\sam2\python.exe" -m unittest discover -s tests -p test_v21.py -q
& "$env:USERPROFILE\anaconda3\envs\sam2\python.exe" -m unittest discover -s tests -p test_owl_ego_robot.py -q
```

证据：`logs/v21_agent_round4/agent_tests_final.log`、`robot_regression.log`。
最初测试失败为上述三个测试路线与现场 YAML 不一致，不是新续租用例失败。
最终复核结果以上述最终日志为准。未发送真实飞行请求或修改运行服务，未 push。

## 待 Robot 配合

请在独立 ROS master/mock FCU 环境注入实际连接超时，核对同一 session 有效期内恢复、
原任务状态检查、阻塞 land 续租和真实过期拒绝；分别保留服务端最后确认续租时刻与客户端
尝试日志。当前实现未重新申请 session、未改变 5 s 租约。
此次没有定位最初 Wi-Fi/服务超时根因，也没有修复此前高度偏差。
Robot console 的同类首次异常退出仍由 Robot 端处理，不能视为已随 Agent 修复。

## 同轮补充：独立动作 CSV（2026-09-11）

按用户要求，在任务日志目录新增 motions.csv，记录 Agent 导航、TRACK 相对动作和降落。
字段含 action/动作参数及坐标语义、before/after XYZ cm/yaw deg、各自采样时间/epoch、
task_id、phase、status/error。导航实际取消后记录 cancelled；失败或未确认终态保留
failed_or_uncertain 行。无法取得的位姿留空并注明原因，不伪造目标位置作为执行后位置。
诊断读取使用独立 1 s 上限、无恢复重试的只读 get_pose；失败动作不追加取位姿请求，
避免延误取消/降落。CSV 逐行关闭落盘，UTF-8 BOM 可直接用 Excel 打开。
这是 Agent 侧记录，无 Robot 接口/行为变更；前后位姿不是 Robot 的原子受理/终态快照。

复用三轮本地 HTTP 集成测试校验三次 TRACK 的前后位移、z=0 参数、三个取消导航行和
最后降落行。Agent 46 项通过（16.918 s），证据 logs/v21_agent_round4/csv_tests.log。
未实飞，未回填历史任务日志。

用户随后要求精简 CSV：最终列为 started_at、finished_at、phase、action（名称）、
action_xyz_yaw、before、after、error；后四列分别保存 (x,y,z,yaw)，保留两位小数。
error 改为期望减实际的位姿误差，导航为世界坐标、相对动作按执行前航向的机体系计算，
yaw 归一化。z=0 的保持高度参考未知时，Z 误差留空；降落/缺失位姿/跨 epoch 不编造误差。
异常文本、状态、task_id 等原详细字段转存 events.jsonl 的 motion_csv_result，CSV 不再保留。
最终验证日志：logs/v21_agent_round4/compact_csv_tests.log；未修改 Robot 或现场 YAML。

同轮显示调整：trigger.jpg 保存时在图像副本上绘制触发候选的绿色检测框（2 px），
使用 trigger.json 中同一 candidate.box；不改变原始观测或模型输入。语法检查通过。

## 同轮现场日志复核：TRACK 高度与观测超时（2026-09-11）

本次按用户要求只读复核 logs/v21_20260911_114527 的 log.txt、events.jsonl、
motions.csv、配置快照及两端调用代码；仅更新 Agent 文档，未修改控制实现/参数，
未重新运行模型或测试，未连接无人机、实飞或外发。以下为用户运行记录的分析，
不是本次 Codex 执行的联调。早先“尚未加载实际模型”仅描述当时验证范围。

### 高度

Agent 不调用 console 程序；两者都 POST /move_relative_xyz_yaw。console 使用
该 HTTP 接口，Agent 经 OwlEgoClient/BaseClient 使用同一接口，没有独立的升降端点。
Robot core 的相对 Z 语义：z=0 保留 hold 高度，非零 z 使用实测 pose.z + z。
因此不能只用水平/z=0 的 console 运动来证明连续非零 Z 的 TRACK 不会积累偏差；
尚缺用户手动 console 的同参数记录，不能认定其实际输入就是 z=0。

最新 CSV 中 TRACK 前三步（单位 cm）：

| 动作 (x,y,z,yaw) | before.z | after.z | error.z（期望减实际） |
|---|---:|---:|---:|
| (25,0,8,0) | 98.74 | 118.20 | -11.45 |
| (25,0,2,6) | 119.06 | 132.99 | -11.93 |
| (25,0,-2,0) | 133.78 | 143.26 | -11.48 |

后续 dz=-4、-8、-10 时也仍可升高，实测采样最高约 155 cm；更大负 dz 后下降。
TRACK 前回 P 的目标 Z=90.25，完成采样 Z=100.37；重捕获纯旋转 dz=0 时
Z=99.48→98.86。TRACK 后再次回 P 也约高 9.68 cm。正偏差并非只出现在
视觉上升指令中。CSV 前后采样不是 Robot 原子受理/终态快照，数值只作近似误差；
各 TRACK task 的服务端 position_error_cm 均在配置的 15 cm 三维到达容差内。

上层像素垂直偏差乘 0.16 cm/pixel、限幅 30 cm；运动过滤按 XYZ 总距离比较
15 cm，没有独立 Z 死区。dx=25 时 ±2 cm 的 dz 仍会发送。代码与记录支持的
放大机制是：非零 dz 反复以已有正偏差的实测高度为起点，小幅修正不足以抵消
约 10–12 cm 的正偏差，下一目标继续升高。尚不能从 Agent 日志定位该正偏差
来自 EGO、末端控制、坐标/估计还是时序，也不能直接认定为 Z 符号错误。

待 Robot 配合：对照同一时段 task_id、相对动作原值、受理 pose、目标/hold Z、
EGO 输出和 MAVROS setpoint 与实测 Z，核查正偏差首次出现的位置；以相同 xyz/yaw
输入和初始条件在隔离环境比较 console 与 Agent。不要只比较 z=0 与非零 z。
Agent 可评估独立垂直死区/滞回，避免微小修正频繁重定基准，但这不消除底层正偏差。
不建议未经验证直接收紧总容差、固定减 10 cm 或改非零 Z 契约。

### 通信与流程

- 11:45:53.781 心跳超时，55.331 恢复；11:46:27.565 超时，29.196 恢复；
  11:47:01.242 超时，03.033 恢复。均为原会话内恢复，日志有原任务核对，
  未出现租约失效或重新申请会话。
- 已完成飞 B → 触发/中止停稳 → 回 P → TRACK success → 回 P → 重新飞 B。
  target_finished 在 11:46:40.894，重返 PATROL 在 11:46:50.994。
- 11:46:59.802 GET /v21/observation 传输超时，采集线程锁存异常；心跳虽恢复，
  pipeline.pop 仍抛 Perception worker failed。原 B 任务被取消确认停止，
  11:47:05.192 记录任务异常，11:47:12.203 land 完成，随后释放会话。
  取消航段 CSV 中剩余约 50 cm 是中止时距 B 的距离，不是已到达的误差。

Agent 当前 observe 将 observation_retry_s=0.5 同时用作 HTTP 请求预算，仅重试
结构化 503 observation_unavailable；普通传输超时不在此重试分支，也不在
health/get_pose/task-status 的恢复白名单，一次即可令 PerceptionPipeline 永久失败。
这是本次退出的直接原因，心跳修复在本轮日志中已发挥作用。超时在多个接口聚集，
但现有日志不足以区分 Wi-Fi、Robot HTTP 阻塞或主机调度原因。

下一步建议 Agent 将可恢复的观测传输异常纳入原租约有界恢复，恢复后重新取新鲜帧；
明确连续无有效观测的上限，保留图像年龄、几何/epoch 和租约检查，不复用过期帧，
不吞掉协议/程序错误。当前仅诊断，尚未实现这一修改。请 Robot 对齐上述三段时间
检查 HTTP 请求耗时、心跳受理、观测编码/锁等待和系统负载，以定位共同超时来源。

## 同轮实现：Robot 独立 Z 到达容差（2026-09-11）

用户明确选择 Robot 到达容差而非 Agent dz 死区，并授权本机修改 Robot 代码。
已在当前 Windows checkout 修改，未代写 robot_status，未部署或实飞。

- src/robot/config/owl_ego.yaml：control.vertical_tolerance_m=0.08；原三维 0.15 m 保留。
- ros/owl_nav/src/owl_nav/core.py：到达必须同时满足三维、绝对 Z、yaw 和原停稳条件；
  对导航、相对运动及起飞统一生效。取消停稳与降落规则不变。旧配置缺字段默认 0.08 m，
  参数使用既有有限正数校验。任务 diagnostics 新增 vertical_error_cm（绝对值）。
- src/robot/controllers/owl_ego.py：motion_tolerances 新增 vertical_tolerance_cm，
  默认 8；契约已记录新增字段、旧配置兼容和部署状态。Agent TRACK 指令未修改。

验证：Robot 108 项通过（5.694 s），Agent 46 项通过（16.871 s，含既有本地 HTTP
模拟链路）。新增覆盖正/负高度偏差、8 cm 两侧、3D 约束仍有效、自定义阈值、旧配置
默认、无效配置与 HTTP 容差返回值。首轮三个旧用例因假设高出 10 cm 仍可到达而失败，
已调整模拟收敛至 6 cm，并验证显式升降在高出 10 cm 时不完成、不允许下一动作，
收敛后才继续。证据 logs/v21_agent_round4/vertical_tolerance_robot_tests.log 与
vertical_tolerance_agent_tests.log；git diff --check 通过。

请 Robot 同步上述代码，现场实际配置在 control 下设置 vertical_tolerance_m: 0.08，
保留现场飞行开关和其余已调参数，不用仓库安全默认配置覆盖现场整份配置。
落地后按现场流程重新加载 bridge 和 HTTP server，确保二者使用相同配置；只更新
HTTP server 的容差返回值不会改变 bridge 的到达判定。核对 /motion_tolerances
返回 vertical_tolerance_cm:8，并在隔离环境验证后安排人工现场验收。
若仍稳定偏高超过 8 cm，任务会等待并可能超时，不能据离线测试宣称高度跟踪偏差
已消除；需要继续对齐目标/setpoint/实测日志。通信恢复改动本轮未实现。
