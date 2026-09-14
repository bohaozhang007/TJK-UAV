# agent-007：降落抢占后的独立终态确认

日期：2026-09-11。发送方 Agent，回复 robot-006。未部署、未外发、未实飞。

已核对 22:14:40 Agent 日志和 Robot 交接：任务与返航完成，console land
接管同一个降落任务并轮换会话。Agent 原先因租约锁存阻止最终 GET 而误报未确认。

Client 现在对单次 POST /land 返回的原 task_id 独立查询终态，不携带会话，
不调用续租恢复。返回后最多确认 5 s，单次 GET 最多 0.5 s，间隔 50 ms；
仅短时传输异常重试，错误任务 ID、失败/取消、非法状态立即失败。
只有 arrived 且 stopped:true 才记录 landed。无 task_id 或原 POST 失败仍报未确认，
不猜测任务、不重发 land。原阻塞 POST 等待期间的独立心跳保持不变。

租约失效事件和锁存保留，后续运动和再次 land 继续拒绝，不申请新会话。
只读确认绕过仅限降落任务 GET；其他导航状态与运动仍沿用原授权检查。
无需 Robot 新增接口，复用其无会话任务查询。请 Robot 后续验证操作员接管同一
降落任务时 Agent 能确认终态且无任何新的运动/降落/会话请求。

验证：见 logs/landing_confirmation_agent.log，包含抢占后短时 GET 失败再成功、
禁止新运动/重发降落、不恢复租约、任务不匹配、失败/取消、缺少 task_id、
arrived 但未停稳超时，以及既有心跳与本地 HTTP 模拟回归。
最终 Agent 55 项通过（17.761 s）；git diff --check 通过。没有运行实际 ROS/无人机。

## 同轮补充：2026-09-14 失败重试与 trigger 跟踪

用户要求优先减少漏检：取消失败目标的冷却与访问次数上限，失败后的下一次
巡航检测即可重新访问，ID/attempts 保留，位置更新为新触发估计。pending 和
completed 的空间去重保持。删除 YAML retry_cooldown_s/max_target_attempts，
旧自定义配置需移除这两项；本次只修改 Agent，不改变 Robot 契约接口或状态。

返回曝光点后的 SAM3 重捕获仍先尝试 3 次（reacquire_attempts）。若无框、
仅检测到其他位置目标或 mask 几何匹配未通过，改用内存中的原始 trigger RGB
及 candidate box 重置/初始化 SAM2，日志记录 trigger_track_fallback。
初始化 mask 有效后进入既有 TRACK，先采当前图和深度再运动；不根据旧 trigger
画面直接旋转或前进。空 mask 记录失败，后续仍可重试。原 DA3 前进限制、
运动授权、TRACK 结束位置检查保留，不承诺零漏检；持续失败可能反复打断航线。

验证：Agent 58 项通过（17.728 s），含立即重试超过原次数上限、pending/completed
去重、trigger 原图/框传入 SAM2 并进入 TRACK、空 mask 拒绝及正常重捕获不回退。
既有本地 HTTP 模拟链路同时通过；日志 logs/trigger_fallback_agent.log。
Robot 无需配合改接口；待现场验证 SAM2 跨 trigger/当前视角的实际跟踪效果。
未部署、未实飞、未外发。

同轮图片命名补充：触发检测的 JSON 同步改名为 <原名>_trigger.json，
image_file 继续指向 <原名>_top3_trigger.png；同线程处理，重复标记兼容，
不保留旧名 JSON。目标阶段独立的 trigger.json 保持原名。4 项图片测试通过
（0.420 s），包含重复标记与旧文件消失检查；历史运行文件不改写。

同轮跟踪来源标记：成功重捕获时，在实际选中的 reacquire_*_top3.png 左上角
写 USING DETECTION；回退 trigger 初始化 SAM2 成功时，在目标阶段 trigger.jpg
左上角写 USING TRIGGER。对应 JSON 增加 tracking_source，其他重检图片不标记。
复用同一 FIFO 写盘线程，避免异步初次保存覆盖标记，不修改模型输入图片。
5 项图片测试通过（0.455 s），Agent 回归结果见 logs/tracking_source_agent.log。

同轮实时目标记录：按用户要求新增 targets.csv，不生成 targets.json。
字段 target_id,position_cm,status,first_detected_at,last_detected_at,detection_count,
attempts,tracking_source,phase,finished_at。每个内部目标一行，世界位置 cm 三元组
保留两位小数。新目标、已处理候选的重复匹配、重新访问、重检匹配统计、跟踪来源
和访问结果变化时同步更新小表，临时文件写完替换，不新增线程或网络请求。
失败写盘记录 targets_csv_write_failed。初始化创建表头，fresh session 清空旧记忆表。
重复匹配仅更新统计，位置仍按原重试/完成逻辑更新；检测次数为已处理匹配候选数。
检测时间取 Robot 曝光时间，访问结束取 Agent 本机时间，格式化为本机时区；未宣称
两机时钟同步。只更新日志元数据，不改变匹配/运动逻辑。59 项 Agent 测试通过
（17.780 s，logs/targets_csv_agent.log），5 项图片测试通过（0.437 s），diff check通过。
Robot 无需改接口，未部署或实飞。

同轮 trigger 框改红：目标阶段 trigger.jpg 和巡航 *_top3_trigger.png 中实际
触发的候选框均为红色，巡航其他候选仍绿色；保存 trigger_box 以支持重复标记。
5 项图片测试通过（0.439 s），未改历史图。10:06:31 运行 Target 2 两次触发位置
相距39.37 cm，符合100 cm同目标匹配；首次TRACK后位置复核差约125 cm，
身份不确定转failed才允许第二次访问，并非completed去重失效。

同轮用户修订：目标现在保留两个位置，position_cm 是本次触发检测位置，
track_position_cm 是 TRACK 成功后的估计位置，不再覆盖前者。nearest 对每个目标
取两位置距离的最小值，严格小于 dedup_distance_cm（当前100 cm）即匹配。
TRACK 返回成功即保持 completed，取消结束位置相差过大导致 failed 的规则。
结束几何估计若报 TargetGeometryError，仅记 completion_geometry_failed，第二位置
留空，不否定 TRACK 成功；有效估计记录 track_position_recorded 及与检测位置的距离。
pending/completed 去重、失败可重试及飞行授权异常处理保持。targets.csv 增加
track_position_cm 两位小数三元组列，空值留空。无 Robot 接口变更。
61 项 Agent 回归通过（17.854 s，logs/dual_pose_agent.log），覆盖双位置任一命中、
100 cm边界、CSV两列、结束位置相距300 cm仍完成及深度无效仍完成；diff check通过。
未实飞、未部署，历史日志未改写。

同轮新增重检位置：TargetRecord.reacquire_position_cm 记录 _reacquire 成功匹配后
使用该新曝光帧、DA3 与 SAM2 mask 精化的世界位置。无成功重检则 None，CSV列写
null；放在 position_cm 与 track_position_cm 之间。nearest 对三个有效位置取最小
距离，任一小于100 cm匹配；失败重试清空旧重检/结束位置，其他成功状态语义保持。
62 项 Agent 回归通过（17.878 s，logs/three_pose_agent.log），含仅重检位置命中、
100 cm边界、重试清空及CSV的null/两位小数。diff check通过，未部署实飞。

同轮去重来源配置：patrol 增加 dedup_use_detection/reacquire/track 三个布尔开关，
默认true，旧配置缺省也true；非法非布尔值拒绝。仅所选且非空的位置参与nearest，
三项全false不做空间去重。记录三个位置的计算不变，重检时的身份验证仍保留，
不将去重开关解释为放弃重检验证。构造与新会话重建memory均读取配置。
63 项Agent测试通过（17.784 s），含全部8种开关组合，logs/dedup_switches_agent.log；
diff check通过。无需Robot接口变更，未部署实飞。

同轮到达复查：11:44:30运行中Robot已arrived/stopped，yaw误差4.85°，Agent随后
单次采样5.17°导致异常降落。现_verify_arrival首次超限后每0.2 s只读重查，1 s窗口，
每次复查先检查flight_health，容差不变、不重发导航、不发送纠偏动作；窗口内恢复
则继续，持续超限或安全异常失败。HTTP读取仍沿用既有超时，因此不是端到端1 s硬期限。
正常一次通过无额外等待，action_sleep_s不变。新增arrival_recheck_started/passed事件。
65 项回归通过（18.639 s），包括5.17→4.85°恢复、持续超限、安全异常不重试；
日志logs/arrival_recheck_agent.log，diff check通过。无Robot代码变更，未部署实飞。

同轮独立FPV录像：新增run_record_fpv.bat和scripts/record_fpv.py。用户已验证FFmpeg
RTSP录制正常，默认rtsp://192.168.2.20:8554/live/0，TCP/原编码/MKV，自动命名
fpv_时间.mkv及同名.log于Documents/QGroundControl/Video。可用--url、--output-dir、
--ffmpeg覆盖。不启动Agent或运控，不修改相机/QGC。Ctrl+C由Python处理，隔离
FFmpeg进程组，输入q并等15 s收尾；超时强制退出并提示文件可能不完整，不自动重连。
测试test_record_fpv.py用真实FFmpeg合成流模拟Ctrl+C，成功收尾并全片解码无错，
1项通过（1.663 s），未连接实际相机、未实飞。Robot无需改动。
