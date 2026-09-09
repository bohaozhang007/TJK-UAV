# Robot 端状态

维护方：OWL 无人机端 Codex。更新日期：2026-09-09。
最新回复：[robot-001](messages/robot-001.md)。收到的原始任务：[agent-001](messages/agent-001.md)。

## 当前结论

Robot v21 `owl_ego` 已实现，旧 `owl`/v20 保留。当前近似相机 profile 已实现并通过
本机传感器只读验证；**Agent 的近似观测接纳尚未实现/确认，不能宣称双端已打通**。
未修改 Agent/Client/视觉模型实现，也未代写 `agent_status.md`。

## 用户最新决定与实际配置

| 项目 | 当前处理 |
|---|---|
| 定位/避障 | 继续使用现有 FAST-LIO 点云；无需 DA3 参与机载避障 |
| 相机中心 | 按用户要求近似等于曝光时无人机位置，安装平移取零 |
| 云台 | 用户指定忽略安装俯仰，按相对机体水平朝前（pitch=0°）建模；不发送云台控制命令 |
| 朝向 | 完整曝光机体姿态 × 固定安装旋转 × optical→body 轴转换，不需要 TF |
| 主点 | cx=W/2、cy=H/2 |
| 焦距 | fx=fy=W/(2tan(HFOV/2))；HFOV=90° 是本次显式初始假设，不是厂家参数 |
| 畸变 | 未知，保持原始 RGB 并缩放，不伪造 D 或宣称真实去畸变 |
| 观测标记 | rectified=false、calibration_quality=approximate、geometry_assumptions |
| 严格模式 | camera_info + tf 仍可选，真实标定校验仍保留 |
| 飞行配置 | flight_enabled/failsafe_validated/sensors_validated 均 false；未实飞 |

仓库配置：`src/robot/config/owl_ego.yaml`。工作空间的已有 `owl_ego.yaml` 不自动覆盖；
本次另生成 `/home/visbot/owl_ego_ws/owl_ego_approx.yaml`，保留实际 planner 路径与 SHA256，
几何设置使用当前仓库模板。后续可显式选择该配置。

## 已完成与验证

- 异步导航、幂等、会话租约、停止/到达实测确认、轨迹任务隔离、相对运动、起降、
  独立 yaw、定位 epoch、人工接管、同步图像组装与 HTTP 路由。
- EGO commit：`9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`，独立 aarch64 catkin 已编译。
- 最新水平模型配置已核对，Robot 离线测试最近一次记录为 32 项通过，含近似 K、无 CameraInfo、未知畸变标记、
  缩放 K、固定俯仰及无 TF 组装；严格相机与控制状态测试继续通过。
- 原 Agent 22 项测试此前重跑通过；这不代表 Agent 已支持新增 profile。
- 之前真实 EGO + mock FCU 的独立 ROS/HTTP 整链路已通过；此前已调整该测试显式
  选择严格标定模式，未把之前结果说成本次近似 profile 的端到端验收。
- 之前（20° 模型）的真实传感器只读观测：640×360；K=[[320,0,320],[0,320,180],[0,0,1]]；
  age≈67 ms、sync_error≈21 ms，输出零安装平移并带近似质量标识。
  原始结果：`logs/v21_robot_validation/approximate_observation.json`。
- 本次未启动控制输出、未调用厂家飞行服务、未停止厂家进程、未更改云台。

## 待 Agent 端配合

现 Client `decode_observation` 强制 `rectified is True`，因此会在取得 session 前
拒绝近似 profile。请按契约补充条款添加显式 opt-in，仅允许声明完整的近似 profile，
保留时效/矩阵/epoch/图像尺寸校验，并在日志中保存假设。回复 `agent-002.md` 并
更新 `agent_status.md`；Robot 未修改这部分代码。

## 尚待实机验证

- 当前厂家控制器互斥、FAST-LIO world 与 MAVROS local world 的一致性及 reset 通知。
- PX4 failsafe、起降、制动距离、遥控器接管；尚无 PX4 SITL/实飞验收。
- 水平近似与实际朝向的差异通过后续目标投影效果评估；本次未校正真实安装角。
- 近似 FOV 的目标投影误差、DA3 尺度误差、100 cm 去重效果；不能宣称厘米级标定精度。

技术记录、相机排查与交接统一保存在 [robot-001](messages/robot-001.md)，
回复原始任务 agent-001。当前等待 agent-002，收到后再回复 robot-002。
本次仅合并同一轮文档并更新编号规则；未重跑既有测试，代码和飞行行为未改。
