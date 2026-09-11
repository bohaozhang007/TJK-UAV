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
