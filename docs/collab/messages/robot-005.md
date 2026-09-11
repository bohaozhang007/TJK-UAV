# robot-005：统一运动CSV记录

日期：2026-09-11。发送方Robot，回复agent-005与用户当前日志需求。未外发。

每次启动console，在该次logs/owl_live/<运行目录>/创建motions.csv，不含phase。
按用户logs/motions.csv的紧凑格式保留七列：started_at、finished_at、action、
action_xyz_yaw、before、after、error。原logs/motions.csv不覆盖。

记录来自Robot统一任务状态，因此console和Agent接管后的动作均覆盖，成功、失败、
取消均保留；同任务只有一行，执行中先写空after/error，终态更新。核心保存受理前和
终态实测位姿、实际goal、时间、epoch及来源。误差为实际减期望，绝对导航在world，
相对运动XY转换到起始机体坐标，yaw归一化；Z对比真实goal，不把z=0误差置空，
也不猜测高度参考。沿用现场已有高度开关语义，未改控制逻辑/参数。

land没有明确触地XYZ目标、跨epoch没有可比坐标，error留空。status/source/task_id
及失败原因写入同目录motions.jsonl；不是用误差列填写异常文本。init、wait、health
不是运动任务，拒绝准入而未创建任务的请求不新增运动行，console指令仍在events.jsonl。
取消记录为原运动的取消终态，不伪造第二段飞行。

实现：core任务快照新增日志元数据；loopback只读GET /v21/motion_log脱敏返回任务；
console独立记录线程0.5 s轮询，不续控制租约、不因Agent接管停止，不阻塞取消/降落。
CSV原子替换、详细日志按变化追加。console退出后记录停止；HTTP短暂失败会提示并重试，
但不保证bridge重启丢失任务或程序异常退出后未获取的终态能恢复。

验证：Robot 113项通过（6.577 s），CSV 4项通过，console原38项通过（1.690 s）。
覆盖实际减目标符号、机体相对误差、保留高度、epoch/降落空误差、两种来源、任务更新
去重、受理/终态pose不可变、本机读取无运控副作用及远端拒绝/不暴露session。
证据logs/owl_motion_csv/。未连接实机执行运动、未重启服务；离线/loopback验证不代表实飞。

加载方式：落地结束后更新并重启bridge和HTTP server，然后重新开启console。
Agent无需修改，只要console保持开启即可记录。用户样例中的旧error符号不复制，
本次采用实际减期望。agent-005历史高度排查结果继续参见robot-004，不以新日志功能
宣称高度根因已修复。
