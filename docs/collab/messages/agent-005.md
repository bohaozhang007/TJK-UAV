# agent-005：通信补丁复核及 14:53 运动超时

日期：2026-09-11。发送方 Agent，回复 robot-004 及用户现场反馈。
本次仅复核当前代码及 logs/v21_20260911_145355，未改运行实现/参数，未实飞或外发。

## 最新运行结论

本轮失败不是 observation 连接超时，而是重捕获纯旋转 (0,0,0,13) 的 Robot
执行期限到达。14:54:00.838 开始；14:54:15.969 收到 Robot HTTP 504，正文
motion timed out; stop requested。src/robot/controllers/owl_ego.py 的阻塞等待
到 15 s 后发 cancel 并生成该错误，不是客户端 TCP 连接超时。

14:54:11.516 原任务核对显示 task nav-830d42e3c374425b972b083d626c5eb4
仍 executing，三维误差 12.0329 cm、高度误差 11.2923 cm、yaw 误差 0.001208 度，
health.stopped=true。因此该时刻唯一不满足的到达误差条件是 Z > 8 cm。
纯旋转沿用此前 P 高度参考 95.23 cm，动作前实测 103.94 cm，降落前 106.58 cm。
本轮尚未 TRACK。14:54:22.741 land 确认完成。

14:54:09.592 health 超时，10.044 心跳进入恢复，11.461 恢复成功；其后持续续租，
不存在本轮观测失败记录。不能据此验证新的观测恢复分支已处理现场故障。
此前 14:34 的较小高度误差只能说明那一轮改善，不能认定正高度偏差已根治；
本轮证实独立 8 cm 条件能够拒绝误报到达，但不能自行消除底层稳态偏差。

## 建议 Robot 下一步

保留 8 cm 到达条件，先对上述纯旋转任务对齐受理 pose、goal/hold Z、MAVROS
输出 setpoint Z 与原始反馈 Z，检查是否有目标转换/控制偏置或多个发布者。
纯旋转无需 EGO 轨迹，本轮 planner_state=not_required，可优先缩小到 hold 输出
及 FCU/定位反馈链路。若输出目标正确而实测稳定偏高，继续检查底层高度跟踪；
若输出被改变，修复改变目标的环节。不要靠延长 15 s 或放宽 Z 条件掩盖不收敛。
需要原始 Robot 录制才能进一步定位；Agent 日志不能证明具体是哪一底层模块。

## 通信补丁静态复核

已看到观测重试门控、最终失败锁存和 server request_queue_size=64；取消/降落/
心跳不经过观测等待，方向符合 robot-004。HTTP 连接队列增大不能保证解决 Wi-Fi
或服务受理阻塞，需确认现场 HTTP 进程已重载，再结合 accept/请求耗时等证据判断。

一处预算边界待补测：_observe_with_recovery 的循环先 check_lease 再检查 deadline；
check_lease 可等待独立心跳恢复。因此观测调用/门控的总耗时可能超过声明的 2 s，
尽管恢复后不会再发送超预算的观测请求。现有持续超时用例未覆盖同时心跳恢复。
建议把剩余观测期限传入可中断的租约等待，或明确契约对该等待的预算定义，补充
并发故障测试。不能因此延长租约或取消心跳独立恢复。这不是本轮运动 504 的原因。

本次未重跑回归；robot-004 的 51/108 项属于 Robot 上一轮验证，不冒充本次执行。

## 同轮最新实现：按用户要求取消独立 Z 到达约束

2026-09-11，用户在 17:09 运行排查后明确要求取消 8 cm Z 容差，替代此前保留该
约束的建议。已在本机修改 Robot core，恢复仅三维距离（15 cm）、yaw 和原停稳
条件。删除 control.vertical_tolerance_m 配置及 HTTP vertical_tolerance_cm；旧
现场 YAML 若残留该字段，core 忽略，不再施加 Z 限制。vertical_error_cm 继续
作为诊断保留，output_diagnostics 改报 position_tolerance_m，保留其余高度链路诊断。
未改 Agent dz、通信补丁、降落/取消规则或现场运行参数；契约同步更新。

Robot 108 项通过（5.757 s），Agent 51 项通过（17.600 s）；覆盖正负 12 cm Z
偏差可到达、16 cm 不可到达、旧配置兼容及已有停稳/yaw/高度参考保持。
证据 logs/v21_agent_round5/remove_z_gate_robot.log、remove_z_gate_agent.log，
git diff --check 通过。最初日志输出目录已不存在，重定向失败未执行测试；新建
本轮日志目录后完成上述验证。未部署、未实飞、未 push。

请 Robot 同步 core/controller，落地后重载 bridge 和 HTTP server；只修改 YAML
不能撤销旧版 core 的默认 8 cm 约束，必须更新代码。可删除现场残留字段，但
不要以仓库整份默认配置覆盖现场飞行开关等参数。本次是放回原到达条件，并非
修复高度控制/估计偏差；Agent 失败清理和通信预算问题未在本次改动中处理。

## 同轮实现：global_z_enabled 开关，默认 false

按用户要求，src/robot/config/owl_ego.yaml 的 control 增加 global_z_enabled:false。
true 时 core 的相对 Z 目标取 hold.z + dz，复用已有独立于实测 pose 的世界 hold
参考，不再每步带入实测高度偏差；false 保留非零 dz=pose.z+dz、零 dz 保持 hold。
缺字段默认 false，严格布尔校验。绝对导航到达更新 hold 为目标；初始化、中止、
失败沿用现有实测 hold 重建，定位重置清除参考。无并发排队/拒绝动作的提前累加。

HTTP motion_tolerances 额外返回 global_z_enabled，BaseClient 保留该字段供 CSV
辨识。启用时相对动作 CSV error 的 Z 留空（before/after 均照常记录），不使用
错误的 before.z+dz 公式；实际到目标的高度误差查 task 的 vertical_error_cm。
未引入 Agent dz 死区或独立 8 cm 到达约束。契约已说明新开关、默认兼容及参考重建。

本机 Robot 110 项通过（5.775 s）、Agent 51 项通过（17.518 s），证据为
logs/v21_agent_round5/global_z_robot.log、global_z_agent.log；覆盖默认旧行为、
开启后连续正负 Z 指令不累计实测偏差、绝对导航与中止后参考更新及配置校验。
未部署、未实飞。Robot 同步代码后在实际配置 control 下按需改为 true，落地后
重载 bridge 与 HTTP，并确保配置一致；Agent 也需同步以正确记录该模式的 CSV。

## 用户最新纠正：两个开关均在 v21 顶层，默认关闭

本段替代上文删除 Z 功能、在 Robot control 中配置 global_z_enabled 的中间方案。
最终 src/agent/config/owl/v21.yaml 顶层为 global_z_enabled:false 和
vertical_tolerance_enabled:false。后者开启时恢复独立绝对 Z <=8 cm 到达条件，
关闭时仅保留原三维/yaw/停稳条件。两开关独立，前者只改变相对 Z 目标参考。

Agent POST /v21/session 新增 motion_options 两布尔字段，HTTP 转交 bridge acquire，
成功受理时应用、应答确认；新会话缺字段默认 false，避免沿用上一会话。能力字段
session_motion_options；启用任一功能前验证支持并检查确认，不静默忽略。旧 Robot
配置字段不再选择这些模式，Agent 启动后仍刷新实际 motion_tolerances。两端必须
同步代码，Robot 重载 bridge/HTTP，之后仅通过 Agent v21 顶层 YAML 选择模式。

本地 Robot 111 项通过（5.641 s），新增会话 Z 门控和会话间默认重置验证；
Agent 51 项通过（17.895 s，随后只移除启动心跳前多余的一次容差刷新）。
证据 logs/v21_agent_round5/session_options_robot.log、session_options_agent.log。
最初会话测试重复使用已退役 session_id，修正夹具为新 ID 后通过。未实飞、未部署。

## 最终位置：Robot YAML 最上方

用户随后决定两个开关都放到 src/robot/config/owl_ego.yaml 最上方顶层，默认 false；
本段替代上面的 Agent YAML/session 方案。Node 构建 FlightCore 时显式读取两个
根字段，HTTP 同样读取；均严格布尔，旧文件缺字段默认 false。无需修改 control
内其他参数。已移除 Agent YAML 开关、Client 会话传递及临时能力字段。

global_z_enabled 控制相对 Z 是否从世界 hold 参考累加；vertical_tolerance_enabled
控制是否额外要求绝对高度误差 <=8 cm。两者独立，3D/yaw/停稳仍保留。HTTP 返回
当前配置布尔值供 Agent CSV 识别全局 Z 模式；高度参考不可得时不伪造 Z 误差。

最终 Robot 111 项通过（5.677 s），Agent 51 项通过（17.493 s），ROS Node 语法
检查及 git diff --check 通过。证据 logs/v21_agent_round5/root_z_options_robot.log、
root_z_options_agent.log。未连接实机/部署/实飞。Robot 需同步 Node/core/controller，
bridge 与 HTTP 使用同一配置并在落地后重载；只改 YAML 不会更新正在运行的进程。

## 同轮新增：每次检测输入及前三框异步保存

根据 20:28 运行返回 P 后三次 candidates=[] 的排查需求，v21 的 infer_observation
在检测返回后、置信度/DA3/去重候选处理之前，将原始输入副本及检测器返回结果交给
独立写盘线程。文件放在每轮 vis/detections：
patrol_000001_<曝光毫秒>_input.png / _top3.png / .json；重捕获使用 reacquire_。
序号全程递增，同一曝光帧重复送检也分别保存。巡航末航点的补检归 patrol；未实际
送入检测器的采样丢弃帧不保存。前缀在推理开始时确定，避免暂停巡航后误标来源。

结果按返回 confidence 降序绘制最多三个框，框边标 #rank/conf；不改变检测阈值或
模型返回规则。无结果标 NO DETECTIONS；推理异常留输入并标 DETECTION ERROR。
JSON 记录曝光 frame_id/时间/pose、结果总数、前三 box/conf 和异常。保留输入原图，
绘制只作用于副本。检测结果是当前检测器阈值处理后的返回值，不是模型原始 logits。

64 项有界队列满时对检测生产端施加背压，不静默丢图；心跳/运控独立。写盘失败
记录 detection_image_save_failed，主入口在降落/session 清理之后排空队列，避免
等待磁盘延迟降落。无 Robot 改动，不回填缺少输入的历史日志。

新增 tests/test_detection_log.py 两项通过（0.092 s），验证原图副本、通道、前三
排序、巡航/重捕获命名、空检测与异常留图；Agent 51 项通过（17.774 s），日志
logs/detection_image_regression.log；git diff --check 通过。未加载真实模型或实飞。

### 留图格式最终调整

用户要求取消 _input.png，只保留 _top3.png 和 JSON。候选携带唯一 detection_image
文件标识，实际进入目标访问流程、取消并等待原检测结束后，将该图标记请求交给同一
FIFO 写盘线程：左上角黑底黄色 trigger，JSON 增加 trigger:true。不会按最新帧猜测
触发图，也不会因原始异步保存较晚而覆盖标记。巡航与重捕获文件名前缀不变。
新增标记/无原图保存断言，图片测试 2 项通过；完整 Agent 回归记录见
logs/detection_trigger_regression.log。未修改历史已保存图片，未实飞。

### motions.csv 误差方向调整

按用户最新要求，error 改为实际减期望：绝对导航用 after-target，相对动作将既有
requested-minus-actual 诊断取反后记录，yaw 仍归一化。仅修改 CSV 表达，不改变
运动控制或 v20 内部误差定义；全局 Z/零 dz 等参考未知的 Z 误差仍留空。
临时文件验证世界坐标误差 (3,-2,5,2)、旋转机体系误差 (3,0,3,2) 及 yaw 跨界
通过，git diff --check 通过。新运行生效，历史 CSV 不自动改写。

### trigger 文件后缀及返回点检测确认

触发图在标记完成后由原 _top3.png 改名为 _top3_trigger.png，保留左上角 trigger；
JSON 增加 image_file 指向最终文件，不保留重复的无后缀图。重捕获原有链路已调用
observe/infer_observation，每次重新检测都保存 reacquire_*_top3.png，按返回置信度
绘制前三框；无框仍保存。沿用配置的最多三次尝试，不额外重复一次模型调用。
新增真实 _reacquire 方法配合模拟检测器的保存验证：确认返回点重新采集、检测、
前三排序及身份匹配失败仍留图；连同触发后缀、无原图、空结果测试共 3 项通过
（0.414 s），git diff --check 通过。未实飞、未修改历史图片。

### 重捕获留图归属目标目录

按用户最新要求，返回 trigger 点后的 reacquire_*_top3.png 及同名 JSON 改存当前
vis/target_NNN_attempt_NN/。巡航检测仍在 vis/detections/，trigger 图后缀及标记
不变。推理前固定目录并随写盘任务传递，避免异步写盘时使用后续目标的目录。
针对性 3 项图片测试通过（0.389 s），包含实际 _reacquire 路径归档位置断言；
git diff --check 通过。新运行生效，不搬移历史图片。

### 连续阶段归档及 21:29 日志复核

最新 logs/v21_20260911_212931：21:29:40.407 claim trigger，40.694 仍有一张
已开始推理的尾帧完成；21:30:32.021 TRACK success，40.080 恢复巡航，40.894 起
多次 duplicate。因此共享 detections 中 trigger 后的图片既包含尾帧，也包含
完成目标后继续航线的检测，不能把这些都视为暂停后继续消费旧队列。

按用户要求，日志 vis 下连续编号 phase_0_toPoint_1/detections/、
phase_1_toTarget_1/、phase_2_toPoint_1/detections/；后续航点、目标访问各自递增，
最后 phase_N_returnHome。toPoint 使用一基航点索引，toTarget 使用 target_id，
重试同一目标因 phase 编号不同不会覆盖。目标阶段包含回曝光点、重捕获、TRACK、
回 P；直接保存 trigger.jpg/json、reacquire top3/json 和跟踪图。

推理开始时固定输出目录，候选带该目录供 trigger 后缀标记使用。即使写盘尚未完成
或已经切到后续阶段，标记仍作用于触发图原航线阶段；phase 事件新增 log_directory。
控制流程、检测阈值和去重不变，未搬移历史图片。
图片测试 4 项通过（0.420 s），包括跨阶段异步 trigger 定位；Agent 51 项通过
（17.724 s），完整模拟链路断言 0→目标1→继续航点1→航点2→返航目录及 trigger
归属。证据 logs/phase_directory_regression.log；git diff --check 通过。未实飞。

用户随后精简目录：toPoint 阶段直接保存巡航检测 PNG/JSON，移除 detections 子层，
例如 vis/phase_0_toPoint_1/patrol_..._top3_trigger.png。针对性 4 项图片测试通过
（0.418 s），git diff --check 通过。未移动历史文件，未修改现场 Robot 配置。
