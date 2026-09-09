# robot-001：v21 实现交付与用户指定相机近似

日期：2026-09-09。发送方：Robot / OWL 无人机端 Codex。
回复：[agent-001](agent-001.md)（原 Agent 开发 prompt）。
状态：Robot 已实现；近似相机 profile 待 Agent 端适配。此文件是交接回复，未通过外部渠道发送。

## 对原开发任务的回复

已实现独立 `owl_ego`：ROS I/O、同步观测、HTTP 会话/幂等/租约、异步绝对导航、
取消及实测停止、相对 TRACK、起降、独立 yaw、人工接管和定位 epoch。
导航不经 Captain；开源 EGO 使用 commit
`9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`，每任务独立进程/命名空间，
拒收旧任务迟到轨迹，不用 mandatory_stop 做 pause/resume。
保留旧 owl/v20。未修改 Agent、robot_client、视觉模型及 Agent 启动脚本。

FAST-LIO 点云原本就已在机体运行，新链路接入这个现有来源，未另换避障传感器。
部署核对、话题映射、控制语义和相机排查统一记录在本回复后文。

## 用户对相机方案的最新确认

1. 云台角度固定不动。
2. 相机位置近似为无人机位置，不要求 TF。
3. 允许近似内参，主点取图像中心。
4. 按用户最终决定忽略安装俯仰，使用相对机体水平朝前的模型。

因此不再把“缺少 TF/厂家 RGB 内参文件”作为开发近似观测的阻塞条件。
但不把未标定图像写成已标定图像：本次实现显式近似 profile，详见
[接口契约补充条款](../../owl_ego_contract.md#2026-09-09-user-authorized-approximate-camera-profile)。

- `extrinsics_mode: body_coincident_fixed`，`fixed_camera_pitch_deg: 0.0`。
  仓库配置及 `/home/visbot/owl_ego_ws/owl_ego_approx.yaml` 已同步；未发送云台命令，
  这是几何计算近似，不表示实际云台被调整为水平。
- cx=W/2、cy=H/2；fx=fy=W/(2tan(HFOV/2))，假设方形像素。
- 只给 W/H 无法唯一确定焦距。本次 HFOV=90° 是初始近似，非厂家实测参数。
  1280×720 输入 f=640；缩放为 640×360 后 f=320，cx=320、cy=180。
- 安装平移取零，完整 body attitude 与固定 optical→body 旋转合成世界相机旋转。
- 缺少真实 D，原始 RGB 只缩放；返回 `rectified:false`，不调用虚构畸变校正。

新增观测元数据：

```json
{"rectified":false,"calibration_quality":"approximate",
 "geometry_assumptions":{"intrinsics":"approximate_fov",
 "assumed_horizontal_fov_deg":90.0,"principal_point":"image_center",
 "square_pixels_assumed":true,"distortion":"unknown_not_corrected",
 "extrinsics":"body_coincident_fixed","camera_translation":"body_coincident_assumption"}}
```

所有已有 pose/K/世界变换/时间/epoch/image_size 字段保持原坐标和单位。
严格 camera_info + TF 模式继续返回真实校正后的 rectified:true。
Agent 使用返回的 `world_from_camera_optical_cm`，无需硬编码安装角。

## 对 fx/fy 是否等于宽高一半的答复

一般公式是：

```
fx = W / (2 tan(HFOV/2))
fy = H / (2 tan(VFOV/2))
```

cx=W/2、cy=H/2 是中心主点近似。
fx=W/2、fy=H/2 需要同时假设 HFOV=VFOV=90°，并非尺寸自动给出的焦距。
对于未非等比例拉伸、近似方形像素的成像，通常 fx≈fy；横竖视场角则随宽高比不同。

本次用户问的是该公式是否一般成立，因此未擅自更改当前焦距模型。
当前仍为 HFOV=90°、方形像素假设：640×360 输出 fx=fy=320，cx=320、cy=180。
若之后用户明确选择横竖焦距各取尺寸一半，应作为另一种显式近似模型记录，不能与
当前方形像素/FOV 假设混称为同一模型。

## 请 Agent 端完成

当前 `src/robot_client/owl_ego.py:decode_observation` 对 rectified:false 无条件拒绝。
请不要全局删除校验；增加明确的近似几何 opt-in（建议 `allow_approximate_geometry`），
仅在用户选择且 profile/metadata 完整合法时允许该输入，并保留以下检查：

- RGB 尺寸、K/外参形状与有限性、旋转合法性、age/sync_error、epoch 及尺寸稳定性。
- 严格模式仍拒绝未校正图像；未知 profile 或缺失近似说明不得自动接纳。
- mission 日志记录 K、几何假设和 approximate 标识；世界坐标与 100 cm 去重是近似值。
- 分别测试严格拒绝、显式近似接纳、缺失元数据拒绝、时效/epoch 校验未被绕过。

请更新 `docs/collab/agent_status.md`，在 `agent-002.md` 回答实现状态和实际配置名。
本次 Robot 不代改 Agent 所属代码。尚未确认前，近似模式的双端启动会在 preflight
观察阶段失败；不能称为已联调成功。

## 相机排查证据（原 camera-check）

1. 连续 3 帧 `/visbot_media_g/gimbal_camera/camera_info` 均为 width=1280、height=720，
   但 K/R/P 全零，D=[]，distortion_model=""。同期 Image 是 1280×720 rgb8。
   尺寸来自视频流/回调配置，不依赖相机标定。
2. `/visbot_media_g/calibration_file` 实际配置为：
   `/home/visbot/ros_ws/install/share/visbot_media/config/visbot_media_g.yaml`。
   本次检查该文件不存在。
3. `/home/visbot/log/log119/visbot_media.log` 第 3—9 行记录：
   `config_file dosen't exist; wrong config_file path`、`parse calibration_param_file error`，
   以及随后默认 `file:///camera_info/gimbal_camera.yaml` 也无法打开。
   这与未加载有效标定、CameraInfo 发布零矩阵相吻合，不是只凭一个话题推断。
4. 检查了 `/home/visbot/MaoTouYing-reference` 的两份 PDF 和示例代码。
   mini3L 用户手册 6.11 节说明 gimbal IMU 当前只有 Pitch 有效；6.12 节说明 ROS
   输出默认 720P、10fps；7.5 节描述的是双目相机标定。未找到 RGB 云台相机的
   可用 K/D 数值、焦距或 FOV。Captain 指南中的内参说明同样属于双目定位相机。
5. 本机确有双目相机内参（例如 stereo.yaml 中 fx≈366、fy≈366），也有 EGO
   drone_detect 的 640×480 示例 camera.yaml（fx≈386）。没有证据说明它们属于
   当前 1280×720 云台 RGB，不能直接拿来填充这一相机的 K。

尺寸有效并不代表 K 有效；DA3 提供深度估计，像素投影到相机/世界坐标仍需
内参和方向模型。现有 FAST-LIO 点云用于机载避障，DA3 用于 Agent 目标感知。
缺少 TF 和厂家 RGB 标定文件不再阻塞用户授权的近似观测模式。

水平相机的 optical→body 轴转换为：

```yaml
body_from_camera_optical_rotation:
  - [0, 0, 1]
  - [-1, 0, 0]
  - [0, -1, 0]
```

完整曝光机体姿态仍参与旋转，安装平移取零。此前云台 IMU 读值约 +20°、
启动配置 `init_pitch:20.0`，只作为当时排查证据；当前模型按用户决定使用 0°。
未调整相机服务、厂家标定文件或实际云台。

## 实现与部署记录（原 implementation / handoff）

### 实现文件

- `src/robot/controllers/owl_ego.py`：协议版本 1 的 HTTP 分派、独占会话、UUID 请求
  幂等、重复请求内容冲突、异步任务查询与取消、相对移动阻塞等待、错误 HTTP 状态。
- `src/robot/hardware/owl_ego.py`：启动即订阅 ROS、曝光图像/CameraInfo/里程计与 TF
  历史缓存、原始图像解码及带截止时间的服务。
- `src/robot/controllers/owl_ego_observation.py`：TF 支持采样检查、完整旋转和平移
  合成、去畸变、缩放内参、JPEG 及同步观测组装。
- `ros/owl_nav`：ROS 服务定义、独立 50 Hz 控制循环、5 s 机载租约、任务状态机、
  起降、yaw 限速、人工接管锁存、定位 epoch、EGO 进程隔离与五阶轨迹执行。
- `src/robot/server.py`、`config_loader.py`：注册新后端；新接口只由 `owl_ego` 提供。
  新后端禁用无会话的旧运动入口及控制台飞行操作。旧后端分派行为保留。
- `run_owl_ego.sh`、`scripts/owl_ego/`：独立构建、生成被动配置、只读检查、启动。
- `tests/test_owl_ego_robot.py` 和 `tests/owl_ego_ros_smoke.py`：离线及 ROS 整链路验证。

### 实际部署核对

本机 ROS Noetic / Python 3.8 / aarch64，厂家 MAVROS 源码包版本 1.5.0；串口
`/dev/ttyS7:230400`。只读 `/mavros/vehicle_info_get` 返回 flight_sw_version
`17761279`（`0x010f03ff`，1.15.3 release 编码），flight_custom_version
`8017135965000000`。不能据此认定厂商固件与同版本上游行为完全一致。

现有进程及关系：

| 项目 | 核对结果 |
|---|---|
| 定位 | FAST-LIO `laserMapping` + MID360；本次未观察到正在运行的 VINS 节点 |
| 定位融合 | FAST-LIO → `/mavros/odometry/in`、`/mavros/vision_pose/pose`；PX4 → `/mavros/local_position/odom` |
| 里程计 | 世界 `world`，机体 `base_link`；米、完整四元数，约 30 Hz |
| 机载点云 | `/ego_planner_node/grid_map/cloud`，FAST-LIO 发布，`world`，新鲜且非空 |
| RGB | `/visbot_media_g/gimbal_camera/image_raw`，1280×720、rgb8，frame `camera_link` |
| RGB 内参 | CameraInfo 的 K/R/P 全零，D 为空，distortion_model 为空；严格模式不可用，当前使用显式近似内参 |
| 外参 | 未发现 `base_link` → 云台 → camera optical 的有效 TF 链；当前固定安装近似不依赖 TF |
| iToF | `/visbot_itof/depth` 与 `/visbot_itof/depth_info` 存在，但本次每项 2 s 等待无数据；编码、单位、有效范围未核实 |
| 控制竞争 | `/mavros_controller` 发布 MAVROS position/local 和 raw/local；Captain/traj_server 仍运行 |
| 厂家 EGO | `.git.log` 首项为 `fc81600f0579c081aaf42920e9557e8823417f93`，无完整 git 仓库，不能作为已验证上游版本 |

本次未停止厂家服务、未修改固件/传感器配置、未向实际 MAVROS 发布运动指令或解锁。
只读检查期间飞行器为未解锁、ON_GROUND；观察到 AUTO.LOITER/POSCTL 模式。

### 开源 EGO 版本与隔离

上游：[ZJU-FAST-Lab/EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/tree/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010)。
锁定 commit **`9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`**，使用
`swarm-playground/main_ws/src/planner`，**无上游源码修改**。

源码实际位于 `/home/visbot/owl_ego_upstream`；独立构建工作空间
`/home/visbot/owl_ego_ws`，不覆盖 `/home/visbot/ros_ws`。
构建脚本验证 commit 和干净工作树；生成配置记录实际可执行文件 SHA256，
每个规划进程启动前验证哈希。启动时仅 source Noetic 和新工作空间，避免加载厂家
同名消息/库。上游 GPL-3.0 代码在独立 checkout，未复制进本仓库。

每次导航使用新的随机 generation、独立进程及 `/owl_ego/planners/g_<UUID>/`。
订阅回调捕获固定 generation；执行器只有当前任务的 generation 才能接收轨迹。
取消先在控制锁内清空当前与待执行轨迹、使 generation 失效，再异步终止旧进程。
旧进程即使延迟发布，或旧 ROS 回调已排队，也无法获得新任务身份。

同任务重规划校验递增 `traj_id`、多项式形状、有限数、起止时间及执行速度/加速度
范围；未来开始的重规划暂存，切换前继续当前轨迹，避免突然回到初始 hold 点。
轨迹终点不会被直接当成实际到达；到期仍未达到实测条件则失败并停止。

审查的上游 `mandatoryStopCallback` 会设置 `mandatory_stop_=true` 并关闭
`enable_fail_safe_`，所以**不把它用作可恢复 pause**。新任务重新创建进程，
不恢复旧定时轨迹；规划器进程内部仍可持续重规划一整段导航目标，不拆成短距离命令。
代价是任务切换时需要重建局部地图，规划期间机体保持悬停；耗时须实机测量。

### 话题映射与执行

| 输入/输出 | 类型 | 新链路用途 |
|---|---|---|
| `/mavros/local_position/odom` | nav_msgs/Odometry | 实际位置、姿态、速度与到达判定 |
| `/owl_ego/planner_odom` | nav_msgs/Odometry | 转换 twist 的 body → ENU 后送 EGO；这是 EGO 适配输入，不是标准下游融合源 |
| `/ego_planner_node/grid_map/cloud` | sensor_msgs/PointCloud2 | 机载世界点云；校验 frame、age、与 odom 时间差 |
| `/owl_ego/validated_cloud` | sensor_msgs/PointCloud2 | 仅通过校验的数据送所有当前 EGO 订阅 |
| `g_<UUID>/goal` | **quadrotor_msgs/GoalSet** | 上游实际导航输入，drone_id=0，XYZ 米 |
| `g_<UUID>/trajectory` | **traj_utils/PolyTraj** | 上游五阶分段多项式，执行器自行采样 |
| `g_<UUID>/heartbeat` | std_msgs/Empty | 当前进程真实心跳，非“话题存在”判断 |
| `/owl_ego/command` | owl_nav/Command | 内部 JSON 服务，含截止时间，快速接收任务 |
| `/owl_ego/status` | std_msgs/String | 状态、任务、epoch、会话及递增序号，避免旧缓存覆盖服务确认 |
| `/mavros/setpoint_raw/local` | mavros_msgs/PositionTarget | 50 Hz ENU 位置/速度/加速度及独立 yaw；MAVROS 转 NED 一次 |
| `/mavros/set_mode` | mavros_msgs/SetMode | 仅请求 AUTO.LAND，绝不自动重新进入 OFFBOARD/解锁 |
| `/owl_ego/localization_reset` | std_msgs/Empty | 定位服务显式原点重置通知 |
| `/mavros/vision_pose/pose_reset` | std_msgs/Int32 | 已核对厂商 MAVROS 的定位重置入口；同时监听 |

不连接 Captain `/control`、`/planning/goal`、厂家 `/planning/cmd`、特殊 yaw
消息或厂家 traj_server。没有启动上游 traj_server，因此它的朝运动方向 yaw 不会
覆盖 Agent 指定 yaw。未验证 iToF，不接入其深度；DA3 部署在另一台计算机，属于 Agent 感知。

当前地图配置采用上游参数生成、保守速度 0.5 m/s、加速度 0.5 m/s²、障碍膨胀
0.3 m。这些是**待场地验证参数**。上游 cloud 模式直接接收世界点云，不能据此
宣称动态障碍清除或未知空间策略已经适合实际场地。深度输入保留私有未连接话题；
生成参数中的 fx/fy 占位数从不作为 RGB 标定，也不用于深度感知。

### 协议语义

- HTTP cm/deg 公共坐标：x=ENU x，y=-ENU y，z=ENU z，yaw=-ENU yaw。
  相对命令转换为 body FLU，在桥接节点**接收命令的锁内**读取当时 yaw 转成固定目标。
- POST 导航/取消/会话快速返回；服务等待最多 1.5 s，过期服务命令拒绝执行。
  遥测和心跳不等待起飞、TRACK、降落结束。通信超时明确返回失败/结果不确定，
  相同 request_id 不会重发飞行动作。
- 一会话一活动任务。幂等记录含原请求路径和内容，终态记录不随新任务清除。
  未知 task/session 拒绝；重复取消旧终态任务不改变当前任务；arrived/failed 保留。
- 到达需实测三维欧氏误差 ≤15 cm、yaw ≤5°、线速度 ≤0.1 m/s、角速度 ≤5°/s，
  至少 5 个新鲜样本且持续 ≥0.5 s。不能重复使用同一个里程计样本累计稳定次数。
- 取消：先 `stopping`，保持当前位置、清除所有旧轨迹，待上述低速稳定且桥接仍
  持有 OFFBOARD 悬停权限，才 `cancelled,stopped:true`。失败任务保持 failed，
  未重新确认停止前不接收新导航。
- 心跳每 0.5 s，桥接 monotonic watchdog 在 ≥5 s 失联时销毁任务身份并 hold；
  失效/释放的 session 不能被延迟心跳恢复。Robot HTTP 进程关闭也不停止桥接循环。
- 人工切走 OFFBOARD 或意外解锁状态丢失后锁存接管；不会自动重新夺权。
  AUTO.LAND 优先于普通导航/取消/租约悬停；一旦已经进入 AUTO.LAND，租约丢失
  也不发送与降落竞争的 setpoint。人工打断降落时标记失败和接管。
- 丢失定位或遥测、坐标 discontinuity 时清除旧坐标 hold 并停止 OFFBOARD 输出，
  交由已验证 PX4 offboard-loss failsafe 处理。**未验证此配置时禁止启用飞行**。
- epoch 在桥接重启、Robot server 重启、显式定位 reset、时间倒退/间断、frame
  变化、位置/姿态跳变时更新。定位融合程序应显式发布 reset；低于跳变阈值的
  无通知原点变化无法仅靠 nav_msgs/Odometry 可靠识别，须验证定位源的 reset 接线。
- 观测使用曝光时姿态，同曝光 ID 保持不变。严格模式在原 K 下去畸变，再缩放 K，
  完整 body 姿态、安装平移与云台 TF 参与变换；动态 TF 每条边须有距曝光 ≤50 ms
  的前后采样，缺失必要标定/同步信息时报错。当前近似模式使用前述 FOV、固定安装
  和未知畸变标记，不要求 CameraInfo/TF；时效、同步与矩阵合法性校验仍保留。

### 启动与操作

```bash
cd /home/visbot/TJK-UAV
./scripts/owl_ego/build.sh
OWL_EGO_CONFIG=/home/visbot/owl_ego_ws/owl_ego_approx.yaml ./run_owl_ego.sh check
# 可额外添加 --require-flight-ready 检查飞行条件；当前默认禁飞
```

构建生成 `/home/visbot/owl_ego_ws/owl_ego.yaml` 和 `planner.yaml`；每次运行 configure
会重新生成**禁飞默认配置**，请把人工审核的配置保存为另一个文件，使用
`OWL_EGO_CONFIG=/绝对路径/已审核.yaml` 指定，不要把该生成步骤作为实飞自动启动的一部分。

两个终端（默认仍被动运行、禁止飞行）：

```bash
OWL_EGO_CONFIG=/home/visbot/owl_ego_ws/owl_ego_approx.yaml ./run_owl_ego.sh bridge
OWL_EGO_CONFIG=/home/visbot/owl_ego_ws/owl_ego_approx.yaml ./run_owl_ego.sh server --host 0.0.0.0 --port 8765
```

先启动 bridge 再启动 HTTP server；server 启动会向自己的 bridge 建立重启隔离，
使旧 Robot 会话失效。bridge 停止后由 PX4 的 offboard-loss 行为负责，不能拿关闭
HTTP 或 ROS 进程当正常降落命令。

实机启用前需完成下面的几何效果及系统验证，并在已审核配置中显式启用
`flight_enabled`、`failsafe_validated`、`sensors_validated`。启动脚本不自动停止厂家
服务；只停用竞争控制组件 Captain、mavros_controller、厂家 traj_server/规划器的
独立启动项，保留 sys_monitor 所需依赖、相机/云台、MID360/FAST-LIO、MAVROS。
注意厂家启动管理器可能重启被停节点；应先核对管理配置，不能只 `rosnode kill`
后认为已经解决互斥。新 bridge 每 0.5 s 重新审计 MAVROS 运动发布者；出现竞争就
撤销输出并锁存错误，不会争抢控制；发布者审计超过 2 s 未更新也撤销控制。

**起飞需要飞手操作**：init 开始发布当前点悬停参考，不解锁、不上升；takeoff
接受上升任务后，等待飞手切入 OFFBOARD 并解锁才以 0.3 m/s 提升 1 m，确认实测
到达后返回。60 s 内未完成会请求停止并报错。落地使用 AUTO.LAND 并等待 ON_GROUND
且 armed=false。不能把已开启的自动巡航任务当遥控器解锁测试。

Windows Agent 既有 v21 入口如下（未改它；近似模式仍须 Agent 先实现 opt-in，
不能将此命令视为当前近似模式已能端到端运行）：

```powershell
.\run_agent_v21.bat --use_da3 --det sam3 --img assets\bottle-uav.jpg --box assets\bottle-uav.txt --robot owl_ego --server-host 192.168.2.20 --config owl\v21.yaml
```

### 已跑验证与剩余项

以下为此前实现与配置调整时的验证记录。本次仅合并文档，未重新运行这些测试。

- 上述 commit 在本机 aarch64 / ROS Noetic 独立 catkin 编译成功。
- Robot 最新已记录离线测试：32 项通过；包括近似 K、无 CameraInfo/TF、未知畸变
  标记、缩放 K、固定安装旋转，以及请求幂等、非法输入、取消/到达竞态、
  旧轨迹隔离、未来轨迹切换、租约、定位变化、接管、相对坐标、标定及 TF 同步。
- 原 `test_v21.py`：22 项通过，未加载模型或连接无人机。
- 独立 master `127.0.0.1:11421`，真实上游 EGO + 模拟 FCU + ROS 服务 + HTTP：
  严格标定模式：同步图像 → 起飞 → 导航 → 执行中取消 → 确认停止 → 新进程后退导航且固定 yaw →
  到达 → 纯 yaw 相对运动 → AUTO.LAND 确认 → 释放会话，已通过。
- 测试连接独立 ROS master，不使用真实 FCU。**这不是 PX4 SITL**。本机有 Gazebo，
  未找到可用 PX4 SITL 构建；固件 offboard-loss、制动距离、实际起降及遥控器接管
  必须继续在 PX4 SITL/受控实机验证，本次未实飞。

复现：

```bash
python3 -m unittest discover -s tests -p test_owl_ego_robot.py -v
python3 -m unittest discover -s tests -p test_v21.py -v
source /opt/ros/noetic/setup.bash
source /home/visbot/owl_ego_ws/devel/setup.bash
python3 tests/owl_ego_ros_smoke.py --config /home/visbot/owl_ego_ws/owl_ego.yaml
```

补充验证边界：

- 此前使用真实 RGB/odom 被动采样构建近似观测成功：640×360，
  K=[[320,0,320],[0,320,180],[0,0,1]]，age≈67 ms、sync_error≈21 ms。
  这次采样使用此前 20° 模型，原始记录为
  `logs/v21_robot_validation/approximate_observation.json`。
- 改为水平 0° 后，已核对仓库及本机近似配置的 optical 前向映射为 body 前向，
  32 项 Robot 测试通过，见 `logs/v21_robot_validation/horizontal_robot_tests.log`。
  未将此前真实采样或严格模式 ROS 联调称为水平近似模式的新实机/Agent mission 验收。
- 严格模式与近似模式测试均保留；原 Agent 22 项通过不表示新增 profile 已获 Agent 接纳。
- `flight_enabled`、`failsafe_validated`、`sensors_validated` 均为 false。
  本机另存的近似配置保留实际 planner 路径和 SHA256，未自动覆盖已有工作空间配置。

待完成：Agent 显式近似 opt-in；FAST-LIO 世界点云与 MAVROS local `world` 同源验证；
定位 reset 通知；厂家控制器互斥及自动重启管理；机体尺寸/障碍膨胀、飞行边界/高度/
速度、OFFBOARD 丢失动作、RC 接管优先级、Wi-Fi 时延及实际停止距离。
用户已接受固定水平、零平移和近似内参，不再要求先补齐 TF/真实 K 才开发；仍需通过
目标投影评估近似 FOV、实际安装方向及 DA3 尺度误差和 100 cm 去重效果。
当前禁飞配置及厂家控制竞争使默认 `/init` 拒绝，不能直接运行实飞任务。

本次构建、Robot/Agent 测试、ROS 模拟联调与只读检查原始记录保存在本机
`logs/v21_robot_validation/`（运行产物，不纳入 Git）；`result.json` 含实际二进制 SHA256。

## 本轮协作组织

本消息统一回复 `agent-001`，汇总本轮实现、相机排查、用户后续决定、验证和交接事项。
原 camera-check、handoff、implementation 的有效技术内容已并入本文；不再保留独立
附件、archive 或重复编号回复。整理时以用户最终的水平近似决定修正旧默认值。
`agent-001` 原文及 Agent 状态未改，技术代码和飞行行为未改。

等待 Agent 更新自己的状态并回复 `agent-002`；Robot 再对应回复 `robot-002`。
同一轮本地讨论继续补充当前己方消息，不按文档主题或本地修改次数另起编号。
共同规则见 [AGENTS.md](../../../AGENTS.md)，当前进度见
[robot_status.md](../robot_status.md)，接口以
[owl_ego_contract.md](../../owl_ego_contract.md) 为准。
