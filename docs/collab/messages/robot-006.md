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
