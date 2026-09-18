# robot-007：OWL 起飞相机检查与统一传感器外参

日期：2026-09-18。发送方 Robot，回复 agent-007 及用户本轮标定应用要求。

收到 agent-007 的独立降落终态确认及同轮补充；本轮不改这些行为，未进行真实
操作员接管/降落联调，不将既有 Agent 离线结果当作现场验收。

按 owl.txt 的 bridge/server/console 路径实施。bridge 启动读取相机/雷达/飞控IMU
外参，缺失或非法矩阵拒绝启动；订阅图像与云台IMU。init、takeoff受理前及软件起飞
准备过程（包括OFFBOARD/ARM前）检查图像、反馈消息双时钟新鲜度<=0.5 s，pitch在
18–21°。图像分辨率必须匹配内参。异常不会发出新的解锁请求；land/stop不受此门禁。
不自动调整云台角度，不将云台消息中的角速度/加速度视为有效数据。

通用 GET /sensor_geometry（别名 /v21/sensor_geometry）对所有Robot后端开放，
未配置返回503 sensor_geometry_unavailable，绝不返回默认零位姿。OWL配置已填实测
K/D、暂定雷达→相机外参及固定FAST-LIO雷达→IMU外参。统一矩阵单位米、方向
target_from_source；body_from_imu=I为待独立验证的里程计参考点假设。
替换机型应提供自身sensor_geometry；OWL pitch策略只由owl_fixed_gimbal启用。
其他现有机型未补造外参，也未声称其起飞控制已迁移到OWL bridge。

观测真正执行畸变校正、按输出尺寸缩放K，并按曝光时刻机体完整姿态旋转相机杆臂。
rectified=true但calibration_quality仍approximate；本次箱子边缘约10–22像素误差
未消失。没有实现雷达深度图/像素查3D接口，本轮只落实几何与起飞前置检查。

## Windows Agent 待适配（未代改Agent代码）

当前decoder只接受body_coincident_fixed，会拒绝新默认观测。请在保留严格默认和
allow_approximate_geometry显式许可的前提下，支持并验证：
- geometry_assumptions.intrinsics=configured_calibration，distortion=corrected；
- extrinsics=sensor_geometry，camera_translation=configured_sensor_geometry；
- profile_id=owl_20260918_candidate，limitations日志留档；
- rectified=true但quality=approximate，不能仅凭rectified升级为calibrated；
- 起飞/任务准备读取通用sensor_geometry，对未知机型或缺失配置明确拒绝。

单点世界变换继续直接使用world_from_camera_optical_cm，勿再次加杆臂或pitch。
矩阵米单位仅用于sensor_geometry；观测矩阵平移仍cm。cloud若已在world中不能
再次套lidar→IMU。需要对端确认新格式读取与观测消费，尚未双端联调。

生效方式：落地后按owl.txt重启bridge和server；保持console起飞后Agent接入流程。
OWL_EGO_CONFIG覆盖会使用外部配置，必须迁移新增几何字段；不静默回退旧几何。
本轮未重启节点、未解锁、未起飞、未修改厂家ROS工作空间、未外发消息。

## 本轮验证

`python3 -m unittest discover -s tests -p 'test_owl*.py' -q`：170项通过。
`python3 -m unittest discover -s tests -p test_sensor_geometry.py -q`：8项通过。
包含矩阵方向/杆臂/缩放内参、缺失几何通用HTTP响应、pitch边界/越界、双时钟过期、
无效四元数、分辨率、实际bridge命令受理前拦截及OFFBOARD/ARM前再次拦截、land不受影响。
py_compile及git diff --check通过。隔离ROS smoke的模拟相机/外参/云台消息夹具已更新，
本轮仅编译检查，未重跑完整ROS smoke，不将单元测试称为真实FCU验证。
只读现场订阅：图像1280x720、frame=camera_link；云台frame=gimbal，pitch=20.00337°，
新门禁函数ready=true。未向飞控发送控制命令；运行中的bridge/server仍需重载代码。

## 同轮补充：传感器常驻消费与心跳监测

2026-09-18 新增 run_sensor_heartbeat.sh 与 scripts/owl_ego/sensor_heartbeat.py，
独立于bridge/server运行；owl.txt增加ter0。默认持续消费云台Image与Livox CustomMsg，
queue_size=1，接收完整序列化数据，只读取Header时间戳和计数，不保存/解码大数组。
每秒更新logs/sensor_heartbeat/status.json；默认每10秒及状态变化时输出日志，
heartbeat.log轮转上限1MiB、2个备份。每用户单实例锁；Ctrl-C释放订阅。
支持--duration有限试跑（结束时健康返回0、异常返回2）；默认持续运行。
监测NO_DATA、STALE_STAMP、FROZEN_STAMP、INVALID、LOW_RATE，时间源包括单调时钟和
ROS源时间；默认超时2秒、最低2Hz、初始等待10秒。只表明数据流活性，不证明图像
内容/点云质量，也不能修复过热或驱动故障。不发布飞行控制、不自动重启驱动。

离线5项测试通过，覆盖断流/恢复、旧/未来时间戳、ROS时钟冻结、格式异常、低帧率、
最新状态文件覆盖；bash -n、py_compile通过。12秒现场只读试跑：相机约10Hz，雷达
0帧，正常以退出码2报告异常。独立rostopic hz再次核验结果见本轮用户回复。
当时雷达已有laserMapping订阅，相机已有ros_tracker订阅，不能确认故障原因是
缺少消费者。驱动进程仍存在不等同于数据正常；本轮未自动重启或修改厂家服务。
独立rostopic hz确认/livox/lidar与/livox/imu均无新消息。随后已启动独立后台消费者
（本次PID 129816），status.json确认running=true，相机约9–10Hz、雷达NO_DATA。
第二次启动被单实例锁拒绝。仅此新增消费者在后台运行，未设置开机自启。
停止本次实例可kill -INT 129816；后续前台运行按owl.txt的ter0，Ctrl-C退出。

## 同轮补充：标定结果独立留存

用户将迁走gimbal_calibration以释放空间；仅将结果保存到仓库calibration/owl_20260918/：
camera_intrinsics.yaml保留ROS格式相机K/D/R/P，sensor_extrinsics.yaml保留原雷达→相机
R/t、雷达→飞控IMU固定外参以及派生相机/IMU/body变换、方向/单位/适用反馈角和精度限制。
无独立雷达内参标定结果，不虚构参数；body/IMU重合仍是参考点假设。两文件共4139字节，
未复制图像/点云/rosbag/报告数据。已核对保存的K/D、R/t与原结果及运行配置一致，
组合矩阵方向和相机位置一致；配置source说明已改仓库路径，运行不依赖原数据目录。
原gimbal_calibration未移动或删除。
