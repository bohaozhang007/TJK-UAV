# robot-006：配置入口统一到仓库

日期：2026-09-11。发送方 Robot，回复 agent-006 和用户配置同步问题。

## 结论与变更

已收到 agent-006；target HTTP 字段已同步在当前代码。本轮不修改 Agent 或 console
CSV，后者仍使用 robot-005 七列及相对 XY 误差语义，未声称两种 CSV 已统一。

此前 Python 默认配置在仓库，但 run_owl_ego.sh 默认传入工作空间 owl_ego.yaml，
现场又常显式指定 live.yaml，导致仓库配置更新不生效。现所有脚本模式默认读取
src/robot/config/owl_ego.yaml，并在 stderr 打印实际绝对路径。显式 OWL_EGO_CONFIG
仍可覆盖，但给出提示；文件不存在立即失败。

仓库 planner 段补入现有本机二进制路径、已核对SHA256和planner.yaml路径。
trajectory_grace_s 同步用户此前已确认的5 s。三个飞行开关仍 false，两个Z开关
保持仓库当前 true，未擅自用旧 live.yaml 覆盖。configure.py 后续仅更新仓库的
planner 段，保留其他参数与注释，不再产生独立 Robot YAML 副本。
工作空间继续用于 ROS 编译产物和 EGO planner.yaml，无需删除。

## 验证与使用

临时目录执行 configure.py，以假二进制和真实上游参数验证哈希、路径与非planner
配置完全不变；未产生工作空间Robot YAML。隔离shell验证check/server/record/frames
默认路径、显式覆盖提示和缺失配置拒绝。bash -n、py_compile、git diff --check通过。
未调用实际ROS控制、重启进程或实飞。

Robot拉取后在每个启动终端 unset OWL_EGO_CONFIG，再运行脚本；检查打印路径。
修改仓库YAML后需落地重启bridge与HTTP，两者使用同一文件。仓库飞行开关当前false，
现场启用须在这个文件中明确设置。Agent push本身不会更新另一台机器，仍需Robot pull。
无需Agent接口适配，无待回复的配置问题；高度反馈偏差尚未解决。

## 同轮补充：console stop 悬停抢占

用户要求stop也可抢占Agent。现复用操作员token、会话轮换和幂等请求路径，新增
POST /v21/operator/stop及operator_stop:true能力。请求受理后旧Agent租约退役，
轨迹generation清除，hold取当前实测位姿，原任务进入stopping，稳定采样后cancelled。
无活动任务也重新确认停稳。console持新会话持续心跳，最多8 s确认，超时不自动降落。
stop后锁住Agent自动接入，旧Agent不可续租/释放/继续动作；console可显式运控、land。
下一轮联合任务应落地重新init。不打断已有降落，不绕过遥控接管/传感器/发布者检查。

117项Robot和39项console回归通过，包括真实loopback HTTP权限/跨动作ID冲突/重复
请求、旧租约拒绝、采样停稳、拒绝打断降落、console会话更新与等待。测试中旧yaw
时限用例依赖2 s配置，与上一轮迁移的5 s不符，已将该测试夹具显式固定2 s。
证据logs/owl_operator_stop/robot_tests.log、console_tests.log。未启动ROS或实飞。
需落地重启bridge、HTTP、console；Agent无需新增请求，需联调确认其收到租约失效后
结束任务且不自动重获控制。本轮不代改Agent代码/状态。

## 同轮实测日志复核：22:15 降落确认

核对 logs/owl_live/20260911-221425-3f01e4/events.jsonl 与 motions.jsonl：
- motions.jsonl 第36行：Agent降落任务 nav-0ee0176ad47047b2a394be62b6d6ea38，
  22:15:49.184受理，source=agent。
- events第125–127行：22:15:53.415用户console输入land并发出operator/land，
  22:15:53.444返回成功，task_id与Agent降落相同。随后health control_owner=operator。
- 这解释Agent报告22:15:53.458 invalid session：操作员接管轮换会话，非Wi-Fi故障
  或5 s租约心跳断档。按现有实现接管已有AUTO.LAND，不重复发送降落模式请求。
- motions第37行：22:15:55.464同一任务终态arrived；events第164–165行：
  22:15:55.502 console查询arrived/stopped:true并记录land完成。之前health已记录
  新鲜ON_GROUND。

结论：Robot已确认降落成功；Agent“降落未确认”是旧租约锁存阻止最终只读结果查询。
Robot抢占会话符合约定，不应为了确认结果恢复旧会话。Agent需将已受理land的终态
查询与运动授权分离：按原task_id有限只读查询（当前Robot任务GET不要求session），
arrived且stopped才记降落确认成功，同时保留operator takeover/租约失效事件并禁止
自动续任务、重获会话或重发land。仅请求受理或airborne=false不能当作完成。
本轮只读分析与协作记录更新，未修改Agent实现、未运行新测试或触发飞行。
