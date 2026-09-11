# Robot 端状态

维护方：OWL Robot 端。更新：2026-09-11。
最新回复：[robot-004](messages/robot-004.md)，对应 [agent-004](messages/agent-004.md)。

## 当前结论

输入时序继续排查：708条LIO→FCU高度一致，但FCU采样时间近似接收时间，丢失中位
37.91 ms的采集延迟；与固件时间同步未收敛回退一致。三路MAVLink配置均Normal，
默认无FCU主动TIMESYNC，是明确待核对链路配置线索。未改流频率或参数，不能以
38 ms延迟直接解释全部静态高度偏差；需地面验证正确链路的双向同步后再评估融合。
证据15:09录制fcu/ev_timing_summary.json及robot-004后续审计，无新增实飞。

对应ULog已分析：logs/log_162_2026-9-11-15-11-04.ulg与15:09录制307个速度样本匹配，
ROS/FCU高度原点差为0。FCU内部目标未抬高，位置环确实输出下降速度；估计下降速度
却显著大于位置变化。526个低变化样本偏高10.441 cm，估计vz/Kp对应10.427 cm，
支持反馈不一致导致偏高平衡的解释。EV高度与气压高度同时融合、EV速度未融合，
传感器/融合时序根因仍未确定。未改FCU/控制参数；证据见15:09录制目录fcu/及robot-004。

15:09纯旋转复核：四段诊断已实际加载。三次旋转为成功/15 s超时取消/成功。
第二次取消前goal=hold=output=86.871 cm，实测约98–100 cm；悬停段也有抬升，
反馈vz与位置差分不一致。取消后实测保持将hold更新为98.339 cm，第三次继承该参考，
终态Z误差7.9986 cm勉强满足8 cm，不代表初始高度恢复或偏差消失。未修改控制逻辑。
证据logs/owl_diagnostics/agent_20260911-150945/rotation_height_analysis.json及同目录CSV。
下一步需本次FCU内部ULog，暂不重复纯旋转或添加补偿；详见robot-004最新实飞复核。

最新高度排查（2026-09-11 14:53录制）：纯旋转目标Z与MAVROS输出均固定95.232 cm，
14:54:07–17实测中位106.408 cm；Z速度/加速度前馈为0。world vz中位−0.133 m/s，
位置差分近0，反馈不一致仍需FCU内部日志定位。旧录制缺hold，不宣称四段已完全闭环。
保留8 cm容差及非零dz语义，仅新增同周期control_sample诊断，包含goal/hold/输出/实测
及时间戳、掩码、前馈/反馈速度。109项Robot回归通过，未改控制律/实机参数或重启服务。
证据logs/owl_diagnostics/agent_20260911-145332/height_chain_analysis.json及同目录CSV。
两组动作的实际收敛验收尚未完成；高度偏差尚未修复，详见robot-004最新补充。

最小通信补丁（2026-09-11）：用户授权跨端实现，复用已同步的Agent有界心跳恢复，
新增观测传输错误2 s恢复窗口、每请求最多0.5 s、50 ms间隔；恢复时阻止新运动，
取消/降落及心跳独立，最终失败锁存且不自动续任务。图像时效/几何校验不放宽。
Robot HTTP等待连接队列设为64。详见robot-004；本轮未重启真实进程或飞行。
Agent 51项（含真实loopback断连恢复）及Robot 108项回归通过，证据logs/owl_network_patch/。
下面“Agent心跳待修”是早期记录，已被agent-004的实现及本次回归取代。

心跳恢复排查补充（2026-09-11）：已收到agent-003；当前同步Agent代码的心跳循环
遇任意异常即永久退出，与用户补充的09:46:43.007心跳超时及随后租约失效时间线吻合。
契约补充原会话有效预算内有界重试、恢复期间暂停新运动、失效后禁止自动恢复任务。
Robot已有有效会话续租/过期拒绝行为，无需延长5 s租约；Agent实现待对端修复验证。
本机console有相同首次异常退出写法，亦未修复。详见robot-003；本轮仅静态检查及文档，
未运行新的隔离联调，不将agent-003的本端模拟结果当作双方完整验证。

**最新实飞日志复核（2026-09-11 09:44–09:47）：接入/中止可用，降落任务因租约过期失败。**
用户提供运行场景；本轮只读分析现有录制，未发飞行指令或修改运行配置。
09:46:13控制拥有者由operator变为agent；前向导航被取消并确认停止，随后返回类导航及
六次运动均arrived。09:46:44.421 Agent会话的land受理，任务0.893 s后因
`control lease expired`失败；原始FCU记录09:46:45.347为AUTO.LAND，09:46:49.754
ON_GROUND，09:46:52.345 disarmed，实际已落地。缺少Windows Agent心跳/异常日志，
只能确认5 s内没有成功续租，不能确定是客户端提前停心跳、阻塞还是网络原因。
Agent须在阻塞land及落地等待期间持续独立心跳，确认完成后再close/release。
TRACK期间目标Z并非固定：99.53→117.33→129.77→138.24→141.31→136.90 cm，
降落前实测最高152.26 cm。非零相对Z按实测起点计算，叠加高度跟踪正偏差会使后续
目标继续升高；仍需结合Agent原始动作及视觉日志分析，不能宣称底层高度偏差已修复。
未观察到TRACK后返回P/继续B的任务，不能认定完整路线通过。
证据：`logs/owl_diagnostics/agent_20260911-094506/joint_flight_analysis.json`，
详细时间线及对端待查项见[robot-002](messages/robot-002.md)。录制已证明新接入能力实际运行，
下面“未重启/未实飞”仅为当时实现验证的范围，不再代表当前部署状态。

**最新联合调试模式（2026-09-11）：console起飞，Agent自动接入，本机land抢占。**
用户不需要显式交接：console init/takeoff后保持打开，Agent在起飞停稳且无任务、
当前权限/传感器正常时用原POST /v21/session取得独立运控租约。保留高度参考和epoch，
console旧心跳结束且不替Agent续租；Agent失联5 s仍停止。本机console land通过受限
操作员接口原子撤销Agent会话，抢占导航/阻塞相对运动并复用AUTO.LAND；旧请求不能
取消降落或释放新租约。遥控接管/新鲜遥测要求及实际落地判定保留。

Robot实现与接口契约已更新，Agent实现/状态未改。Agent需按已起飞模式接入，跳过
自身起飞，但保留正常任务结束或明确恢复时的/land；用户已确认两端均可降落。
Agent降落须保持会话/心跳至确认落地再release。识别operator_supervised/control_owner及降落抢占，
停止任务、不自动重新获取会话；具体见robot-002最新开头。尚未重启真实bridge/server
或实飞；落地后重启bridge、HTTP server、console才能加载这次两端协作能力。

本次验证：105项Robot、38项客户端/console回归通过；独立master11428真实EGO＋mock
FCU三轮自动接入/操作员降落全部通过，覆盖导航抢占、Agent断心跳后降落、阻塞相对
运动抢占。master11429九类故障回归通过，monitor_errors=[]。最终运行代码哈希与
三轮记录一致，16个HTTP端点清单与实现一致。证据 `logs/owl_operator_agent/verification.json`。
这些是实际console＋HTTP Agent模拟器，不冒充Windows Agent/模型联调或本次实飞。

**当前阶段：Robot运控接口交接完成，进入Agent适配与联调（2026-09-10）。**
用户确认起飞、基础运控和中止已可用，并反馈末端余量改为5 s后的中止测试通过；
此处为用户现场反馈，不冒充本次重新执行实飞。唯一契约已按现有实现重整，去除
操作者命令/测试路线说明；[robot-002开头](messages/robot-002.md)列出当前接口、
Agent具体适配位置与验收顺序，后续旧状态/记录均以该最新交接为准。

端点/请求形状和协议1保持：重点适配近似几何opt-in、阶段化planner健康、当前停稳
门控、观测结构化错误/有界重试、可选软件起飞auto_arm:true、跨epoch已受理降落及
z=0沿用高度参考。Agent代码及agent_status未修改，尚无双方模型驱动完整联调结果。
本轮Robot离线回归94项通过（4.418 s），未操作真实服务/FCU。历史高度跟踪偏差仍是
已知精度问题；接口交接可进行，不宣称零高度误差或新一轮完整实飞已验收。

Console方向键修复（2026-09-10）：交互输入分支加载Python readline，支持本次会话
上下键历史及左右键编辑；缺少模块时提示并保留普通输入，命令文件模式不变。
实际console伪终端验证上下键/左右键通过，无HTTP请求或飞行动作。落地结束会话后
重新打开console生效，无需重启bridge/server；历史不跨console进程保存。

用户确认的现场时间调整（2026-09-10）：已将
`/home/visbot/owl_ego_ws/owl_ego_live.yaml` 的 `control.trajectory_grace_s` 从2改为5 s。
仅增加轨迹/航向参考结束后的收敛余量，满足到达条件仍立即完成；其他参数不变。
配置解析及单项差异已核验，未重启真实bridge、未实飞；落地后重启bridge加载。
此次不代表高度偏差已修复，接口无新增适配要求。

最新实飞日志复核（2026-09-10 20:40–20:43）：五次手动相对运动、中断并确认停止、
首次返回P、左右TRACK及90°旋转均完成；再次返回P失败，未恢复B，随后land完成。
失败前最后记录（距console报错0.109 s）：三维位置误差17.820 cm > 15 cm，yaw误差
1.906° < 5°，同次health.stopped=true（稳定0.527 s）。返回任务当时已执行7.032 s；
最后约0.47 s位置误差19.926→17.820 cm，仍在减小。直接触发为到期未满足到达条件，
日志可见的未满足项为位置；不能套用旧yaw超时结论，也不能保证单纯延时一定成功。
P目标Z96.559 cm，TRACK结束实测Z依次110.443、105.823、105.176 cm，高度偏差仍在。
本次console未记录失败返回段的XYZ，实时bridge状态读取超时，不能把最终17.82 cm
全部归于Z。证据 `logs/owl_live/20260910-204043-080b93/return_failure_analysis.json`。
本次仅复核日志及更新协作记录，未改控制代码/配置、未操作实机；接口无新增适配要求。

旧v20对照及降落状态修复（2026-09-10）：旧Robot每次相对运动目标Z=实测Z+dz，
新版z=0保留原参考；两版位置容差同为15 cm。同一末次pose按旧几何算位置误差3.68 cm，
不代表旧版速度门控/实际飞行也必通过，不能恢复逐次高度重置来掩盖偏差。
20:15飞控日志flight_150.ulg已只读取回并CRC3096677744匹配：失败前约0.9 s指令高度
固定88.269 cm、竖直v/a前馈均0，实测105.22–105.73 cm；NED vz中位0.19971 m/s，
z_deriv中位0.02624 m/s。固定段不是持续上升指令导致，底层估计异常根因仍未定。
FCU日志证实第一次land已进入AUTO.LAND并落地，新版因定位reset误报任务失败。
已修复：保留已受理land，撤销world输出/初始化并记录localization_error；console等待
land不再因epoch/odom变化提前退出。94项Robot、33项客户端、独立master11424真实
EGO/mock FCU降落待应答期间持续LIO错位注入及master11426九类故障回归通过。
证据 `logs/owl_v20_comparison/`。未加载真实服务；需落地后重启bridge/server/console。
高度偏差与降落时实际转向未解决，未改飞控参数或实飞。Agent需按契约观测跨epoch
降落并在下次运动前重新初始化；未代改另一端实现或完成双方联调。

最新实机手动右移失败已定位（2026-09-10 20:16）：任务保持Z目标88.269 cm，开始
实测102.224 cm，末次采样104.879 cm，比目标高16.610 cm；末次水平误差仅2.55 cm。
最终任务位置误差16.713 cm超过15 cm，yaw误差0.348°，到期前health.stopped=true。
因此主因是高度偏差超出到达容差，不是wait命令或yaw预算；HTTP约6.44 s返回失败。
此前前移50 cm成功（误差11.54 cm），随后悬停高度仍从99.43升至102.22 cm。
末次采样较命令起点仅+2.65 cm与超过目标16.61 cm并不矛盾，参考基准不同。
之后land任务受lio/world mismatch（0.109 m/10.60°）中断，不能把该次HTTP失败当作
已完成降落；只读检查已确认当前新鲜ON_GROUND，后续另一次land任务arrived。
证据 `logs/owl_live/20260910-201457-48feef/right_move_failure_analysis.json` 及
bridge_after_failure.json。本轮未改时间/位置阈值、未控制实机；高度和降落异常待解决。

新console手动相对运动已接入（2026-09-10）：`move_rel_xyz_yaw X Y Z YAW`，以及
`move_rel_xyz X Y Z`（yaw=0），均为整数cm/°，接受move_relative同义命令。
复用原HTTP接口/15 s超时/心跳/停止等待/取消降落；手动XY/yaw相对当前机体，非保存P。
新增约5 Hz位姿采样及起止Z、采样最高抬升统计，启动打印events.jsonl路径；失败也留摘要。
31项客户端回归、独立master11424真实EGO/mock FCU五次手动运动及降落/ROS录制分析
全部通过，证据 `logs/owl_console_relative/verification.json`。注入高度/速度偏差与yaw
滞后时，三次平移采样最高抬升6.26/5.01/5.52 cm，结束高度未逐次累加；这验证诊断
能捕获瞬态，不代表高度问题已修好。实际服务/飞机未操作，本次重开console即可加载命令。

换电池重启后的自动交接已实现（2026-09-10）：新增 `run_owl_ego.sh prepare`，确认
飞控已连接、状态新鲜、落地且未解锁后，仅停止 `/captain`、`/mavros_controller`，
复查1 s无运动发布者，再运行严格preflight。`bridge` 启动也自动执行交接，失败不启动；
`check` 保持只读。存在其他控制器/已有owl bridge时拒绝自动处理。
用户授权范围已记入AGENTS；未改厂家开机项或停止真实节点。7项单元测试及独立ROS
master11427 mock节点验证通过，覆盖拒绝已解锁/未知控制器、定向停止、保留遥测与幂等。
证据 `logs/owl_prepare/`。本项只解决开机控制权冲突，下面的高度问题仍未解决。

最新排查/修复（2026-09-10，用户确认对照旧 owl/v20＋Captain）：

- 18:30 实飞的中断、返回P、三次TRACK、再次返回P、恢复B及降落全部完成；
  但高度未通过验收。飞控连续日志显示返回P/恢复B有约10 cm瞬态上冲；固定高度片段
  高于参考中位11.24 cm，NED vz估计中位0.13455 m/s，而高度变化率中位0.01397 m/s。
  新日志只读下载并与机上CRC674210575匹配，证据 `logs/owl_altitude_compare/`。
- 找到并修复绝对导航启动时把等待hold的Z重置为偏高实测Z的问题。所有普通导航现在
  等待EGO期间沿用旧hold高度；目标不变，EGO启动后仍执行原3D轨迹。取消/失败仍捕获
  实测停止位置，未夹平EGO轨迹、未加入高度补偿或放宽配置。
- 修正用户路线：返回P → P左侧1 m → P右侧1 m（两目标相距2 m）→ 顺时针90° →
  返回P并恢复原yaw → B。左右位置和航向均锚定保存P，复用绝对导航接口；第三次仍为
  原地relative yaw。此前“左移1 m再右移1 m回P附近”的理解作废。
- Robot91项、客户端26项通过；独立master11424实际console全链路通过，左右目标
  相距精确200 cm；master11425真实EGO/mock FCU连续3轮通过，监测错误0。
  注入+10 cm高度跟踪偏差、−0.124 m/s竖直速度偏差及yaw滞后；
  `verified_route.json`、`three_rounds/result.json`保存结果。
  master11426九类故障回归也全部通过，证据 `faults/`；均非PX4 SITL或实飞。
- **剩余问题明确未修复**：有偏差mock的平移段仍出现约5–6 cm瞬态上冲，三次TRACK
  结束高度稳定在115.94–115.96 cm（P参考105.96 cm），无逐段累加。EGO从实测位置
  起步及FCU底层速度/高度偏差仍需解决；不能将上层补丁或模拟通过当成真机高度安全。
  当前15 cm到达容差允许约11 cm偏差通过，缩小它只会拒绝完成，不会修正高度。
  厂家坐标转换与当前Z符号一致；没有同条件旧v20飞行日志，尚不能归因于新旧控制器差异。
  本轮未修改现场配置/飞控参数、未重启真实服务、未发送实飞控制命令。

以下为此前各阶段记录；发生冲突时以以上最新结论和 robot-002 最新补充为准。

返回P航向超时修复已实现（2026-09-10）：EGO任务按位置轨迹结束与预计限速yaw完成
时刻中的较晚者，再加2 s跟踪余量和0.5 s稳定窗口判超时；yaw预算只计算一次，重规划
不重新增加转向预算。位置轨迹结束后保持其端点、速度/加速度前馈置零，继续限速yaw。
5°/15 cm实际到达容差和停止确认、租约/规划器心跳/总超时/取消保持等保护保留。
客户端将后台执行错误明确显示Robot motion failed，不再统称Flight authority unavailable。
最终版Robot90项/客户端25项、实际console真实EGO/mock FCU转向滞后完整路线通过：
返回P约6.12 s完成，yaw误差0.79°并实测停稳，之后恢复B并降落。
证据logs/owl_yaw_deadline/；尚未加载真实bridge/server或实飞，原高度偏差问题仍未解决。
最终版独立master11425真实EGO/mock FCU（同样三种偏差/滞后注入）连续3轮路线及
最后降落通过，监测错误0；证据logs/owl_yaw_deadline/three_rounds_final/result.json。
初版three_rounds/仅为中间版本回归，最终验收证据使用three_rounds_final/。

最新现场失败（2026-09-10 18:11）：新路线完成左右移动和90°旋转，返回P恢复原航向时
报trajectory expired before measured arrival。最后状态XYZ误差10.28 cm（≤15 cm），
yaw误差7.55°（>5°），仍约13.51°/s转动；请求后4.93 s失败。当前轨迹到期判定只用
位置轨迹末尾+2 s宽限，未为独立限速yaw完成留足预算。任务失败后invalidate清空轨迹/
active并重置停止窗口；控制权/定位/hold健康字段仍正常，用户随后land完成。
本轮仅诊断，未修改到期逻辑或重启真实服务。证据最新robot-002及
logs/owl_live/20260910-181044-f14986/return_yaw_failure.json。真机整链路仍未通过。

最新用户路线（2026-09-10）：console/test默认B改为起始朝向前方4 m，P仍为距起始
悬停点约1 m时记录的实测pose；记录P后延时由0.8 s改为1.0 s，发送取消并等待停止，
返回P→左移1 m→右移1 m（回P附近）→顺时针转90°→返回P并恢复P航向→新任务飞B。
三个TRACK动作z均为0，沿用高度参考修复；到B后悬停，land仍需显式输入。
这是路线修改，不代表先前高度控制问题已真机验收；未修改现场配置或发送实飞命令。

新路线验证：客户端24项回归通过；独立master11424实际console/真实EGO/mock FCU（同时
注入+10 cm高度偏差及−0.124 m/s速度偏差）全路线与降落通过，3次TRACK准确符合新列表。
本次记录P到发cancel为1.083 s，B距起点400 cm；证据logs/owl_route_4m/verified_route.json
及console_verified/。第一次启动提示误读CLI参数导致退出，已修正并以完整console重跑通过。

最新高度累加修复已落地（2026-09-10）：relative z=0沿用既有hold高度，规划启动期间
也保持该高度；纯yaw复用直接转向，显式z仍为实测高度+偏移。取消/失败重新捕获实测
hold、epoch清空参考。console五次TRACK统一按返回P的目标高度验证，避免跟着偏差走。
本次17:34飞控ULog已只读取回并CRC校验通过；稳定片段报告NED vz中位0.10675 m/s，
高度变化率0.00519 m/s，MPC_Z_P=1.2，实测高于参考中位8.56 cm，与位置环公式吻合。
证据logs/owl_altitude_fix/fcu_altitude_summary.json；估计器/传感器底层原因仍未定。
86项Robot/24项客户端、真实EGO/mock FCU三轮完整路线及九类故障回归通过；新增
+10 cm实际高度跟踪偏差与−0.124 m/s速度偏差同时注入。实际console五段结束高度
1.274–1.275 m，不再累加，证据logs/owl_altitude_fix/console_altitudes.json。
但运动过程模拟仍有约5–6 cm瞬态上冲，原本约10 cm悬停偏差尚未修复；暂不直接复飞
整套test。修复尚未加载真实bridge/server或真机验收，未改现场飞行开关或飞控参数。

**最新实飞异常（2026-09-10 17:34–17:35）：暂停同版test真机复试。**
用户本轮流程返回成功并已降落，但五次TRACK的dz全部为0，实测world Z从1.101 m升到
1.602 m，累计+50.05 cm（包括纯yaw动作）。当前relative以实测Z重建目标，单段高出
目标约9–11 cm仍满足15 cm到达容差，下一段再次吸收该偏差，形成高度逐段累加。
已用实际FlightCore加固定+10 cm高度偏差离线复现；此前mock只模拟twist偏置，遗漏了
持续位置跟踪偏差，模拟通过不能证明这版真机安全。证据logs/owl_altitude_ratchet/。
本轮仅分析与复现，未修控制代码或重启真实服务；原始单段高度偏差根因仍未确定。
下一步修复应保持无Z指令时的既有高度参考，避免再次取偏高的实测Z作为新目标，并验证
取消重新捕获hold、纯yaw、显式升降与坐标epoch行为；需增加偏置位置的连续动作回归。

最新简化（2026-09-10）：复用原取消/保持与任务切换，停止、到达和下一段准入共用
最近0.5 s位置/航向稳定窗口（至少5条新样本，XYZ范围范数≤5 cm、yaw范围≤2.5°）。
删除重复到达计数；原始twist只作诊断，停止不再被已录得的−0.124 m/s竖直偏差卡住。
取消与新运动前的客户端等待统一8 s；不是把stopped无条件设true，也不代表物理零速度。
不增加新配置模式，不把硬件/EKF排查作为此链路前置；上游速度异常尚未修复。
本次Robot82项/客户端24项、真实录制回放、真实EGO/mock FCU三轮完整路线、实际console
及九类故障回归全部通过，证据logs/owl_simple_stop/；Agent尚未确认新停止语义。
需落地后重启bridge/server/console加载，未代用户重启或实飞。

用户真机起飞发现顺时针 90° 偏航，本方确认并修复厂家 MAVROS world/map 转换遗漏。
随后完成坐标、点云、停止与降落安全边界复审；本次又修复起飞参考领先过多、
落地后 console init 未重新初始化及转动中 map 错时配对，模拟回归通过。
**修复后的真机飞行尚未验收，不能把模拟通过或配置 true 当作安全保证。**
未修改 Agent 实现/状态或厂家源码，未发送真实解锁/模式/运动命令。

## 当前配置

- 定位为厂家 `/mavros/local_position/odom` 的 world；FAST-LIO 直接向
  `/ego_planner_node/grid_map/cloud` 发布同一 world 点云。名称来自 launch remap，
  不依赖旧 EGO 转发。DA3 不参与机载避障。
- 显式 `mavros_frame_profile: owl_vendor_world`：发送前向量和 yaw 转为 map，
  MAVROS 再转换成飞控坐标；公共 cm/deg、前右上/顺时针语义不变。
- 现场用户已将 live.yaml 三项飞行/验证开关设 true；仓库默认仍 false。
  本方增添 frame profile、max_tracking_error_m=0.5、起飞参考领先上限0.2 m
  及无上升进展超时10 s，保留用户原飞行开关。
- world Z 导航目标范围 [0,2.7) m，由 EGO 虚拟顶面及膨胀半径限制；
  起飞高度相对接收命令时位置 +1 m。不是高度离地标定。
- 相机固定水平近似、零平移、中心主点、方形像素/HFOV90°；未知畸变保持
  rectified:false、calibration_quality:approximate，仍需 Agent 显式接纳。

## 已修复与入口

- world/map 全向量转换、厂家线速度/角速度语义、四种朝向相对运动检查。
- map/LIO 与控制 odom 持续配对，错位/失效撤销输出；health 新增
  frame_alignment_ok/error，必须关注 epoch 与 manual_takeover。
- 坏点云/重复时间戳不能续期；无效定位立即撤权；取消旋转保持实测 yaw。
- land 等待 AUTO.LAND 期间保留 hold，模式确认后停止 setpoint；接管不可自动恢复。
  点云丢失不取消降落，navigation cancel 不取消 landing；落地仍须新鲜明确状态。
- 轨迹参考跟踪误差 >0.5 m 失败/保持；导航高度越界拒绝，降落保留可用。
- takeoff/纯 yaw 独立于任务 EGO；软件起飞显式请求模式/解锁，普通调用仍兼容。
- `run_owl_ego_console.sh`：init→takeoff→test→land；test 在 B 结束保持。
  stop/quit 不自动降落、不是电机急停；确认落地静止后再输入 init 即可建立新会话。
- 新只读入口 `OWL_EGO_CONFIG=... ./run_owl_ego.sh frames`。

## 本轮证据

`logs/owl_safety_audit/` 保存实际源文件、任务、姿态、时延与故障日志。
8 s 真机只读 map/LIO 对照通过；初次采样 LIO 最大差 1.222 cm/0.155°，
新 frames 脚本再次采样最大差 0.949 cm/0.205°。这仅是静止一致性检查。

真实 EGO + 厂家坐标 mock FCU 的三轮中断恢复、九类故障、实际 console 软件起降
及 0.7 s AUTO.LAND 延迟保持通过。额外障碍墙绕行，mock 最小点间距 0.4853 m。
证据目录与完整复现命令/最终回归结果见 robot-002 最新补充。
不是 PX4 SITL；厂家服务/真实 master 未被这些控制测试接管。

## 未完成与限制

- 修复后的真机保持朝向、XYZ/yaw 跟踪、制动距离与机体净空未验收。
- 静止坐标一致性不能验证全航域定位尺度/复位、障碍物覆盖、雷达盲区、细线/玻璃、
  动态障碍或实际碰撞余量。起飞垂直段和 AUTO.LAND 不经过 EGO 避障。
- 真实 OFFBOARD-loss、RC 接管/急停行为仍需现场证据，配置 true 不等于已测通过。
- Agent 近似相机 opt-in、阶段化健康检查、观测暂不可用重试，以及真实 Agent→Robot
  联调仍待 Agent 完成；本方不代改。保持旧 owl/v20 可用。

历史部署/几何决策见 robot-001；本轮全部实现、异常与测试汇总在 robot-002，
无需独立 handoff/camera-check/implementation 或 archive。

最终代码：Robot 69、console/客户端 16、Agent 22 项回归通过，共 107 项；
最终三轮/实际 console 证据为 `final_three_rounds/`、`final_console/`。
AUTO.LAND 服务人为延迟 0.7 s 时 32 个保持 setpoint 最大间隔 25.49 ms。
最终版本九类故障重跑全部通过，证据 `logs/owl_safety_audit/final_faults/`。

相机最新状态（2026-09-10）：厂家 visbot_media_g 在15:34:55以 -11 崩溃，已单独
恢复原图像 launch；10 s RGB/CameraInfo 各99条、1280x720/10 Hz，check errors:[]。
证据 logs/owl_camera_recovery/；preflight 缺图像时重复 KeyError 已修，原超时错误保留。
厂家崩溃内部根因仍未知，短时恢复不代表长期稳定性已验收；相机进程当前保持运行。


最新起飞故障修复（2026-09-10）：旧参考按时间爬升，未限制与实测高度的领先距离；
现在上限0.2 m（且不超过总跟踪限制的一半），总跟踪保护0.5 m不变，无进展10 s仍失败。
map 使用同采集时间戳匹配，LIO仍50 ms；只读240组map/238组LIO检查通过，
LIO最大差1.206 cm/0.102°。修复真实EGO一次性FSM通知偶发丢失的初始化超时，
使用当前独立进程自身INIT→WAIT_TARGET日志作为补充证据；不按时间放行。
Robot 76 项、console/客户端18项最新回归通过；未重复运行未改动的Agent回归。
独立master11424、真实EGO+mock FCU完整console路线及连续两次起降通过，
包括解锁后2.5 s不爬升、map延迟60 ms、降落时86°/s模拟偏航、AUTO.LAND应答延迟0.7 s。
证据 logs/owl_takeoff_failure/final_retry/；本次未重启真实bridge/server，需地面重启加载修复。
现场日志含修复前一次起飞arrived，不能算本次修复已真机验收；完整真机链路仍待验证。


最新停止门控修复（2026-09-10）：用户起飞arrived约4 s后发test，现场连续三次
health.stopped=false，旧客户端仍发导航导致409。已在test起点、各航点和每次TRACK前
加入最长8 s的当前停止等待，心跳/健康检查持续；超时不提交，机载阈值不变。
health新增stop_diagnostics，解释当前速度与稳定时长；历史任务stopped不代表永久停止。
Robot78项、客户端22项通过；独立master11424真实EGO+mock FCU注入起飞完成后2 s漂移，
客户端等3.07 s稳定后才发B，完整test与land通过；证据logs/owl_stop_wait/final_console/。
真实机起飞→test→降落完整链路仍未验收；地面重启bridge/server/console加载本次修复。


最新实飞证据（2026-09-10 16:33）：B导航已受理，前进约1.019 m记录P，再至约1.464 m
发送取消。此后持续stopping，未执行返回P/TRACK；初始制动后测得速度中位0.147 m/s，
超过0.1 m/s停止阈值，最终用户降落完成。证据logs/owl_live/20260910-163310-96cfe1/，
已导出pose/action/停止诊断CSV。实飞整链路尚未通过；缺少取消后的连续XYZ和原始各轴
twist/setpoint，尚不能判断速度偏差或真实微动，不应直接放宽门控。


停止诊断补丁已就绪（2026-09-10）：新增run_owl_ego.sh record/analyze，独立只读录制
连续odom/原始各轴速度、map/LIO及local/body速度、setpoint、任务和飞控状态；离线导出
多窗口位置差分与twist对照CSV。console补记请求体/命令，每2 s提示停止等待原因。
无控制算法/速度/门槛改动，只需地面重开console并新增录制终端；使用方法见robot-002最新节。
诊断3+客户端23项通过；真实EGO/mock FCU完整路线、录制封存、分析通过，证据
logs/owl_diagnostic_patch/final_integration/。现场短时只读记录验证实际速度话题可采集。
上次实飞停止异常底层原因仍未确定，等待用户新一轮同步数据；未代用户实飞或改阈值。


最新录制分析（2026-09-10 17:00）：stop_check_01 中test尚未提交导航即停止等待超时。
242条样本Z速度中位−0.12353 m/s，与8.03 s内高度净增1.327 cm不一致（vz积分下降
99.328 cm）；多窗口位置差分速度中位约0.009–0.013 m/s。同stamp MAVROS各速度话题
vz与odom完全一致，异常在Robot桥接前已存在，指向上游竖直速度估计/融合问题。
目标固定，另有约10.4 cm高度目标偏差；具体EKF/传感器根因需飞控日志，未放宽阈值。
图表/统计/样本见logs/owl_diagnostics/stop_check_01/hover_analysis.*及hover_window.csv。


飞控日志已自动取回（2026-09-10）：logs/owl_estimator_audit/09_00_17.ulg，PX4 1.15.3，
只读下载且CRC匹配。飞控自身NED vz中位+0.124 m/s；EV位置/高度融合正常启用，
EV输入速度NaN，不能仅开启EV速度融合。飞行振动明显增大、无clipping，因果尚未定。
下一步核对飞控固定/减振及估计器输入；尚未改飞控参数、阈值或运行新的实飞。


最新用户方向：优先完成可中断/恢复链路，全面硬件/EKF排查不作为前置。
EGO本身支持mandatory_stop定点保持；现有Robot取消已生效，卡点是停止判定。
该方向现已实现，替代上文历史排查阶段的“阈值不变/等待进一步硬件排查”安排。
