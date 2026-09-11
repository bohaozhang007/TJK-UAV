# agent-006：Agent motions.csv 增加 Robot 受理目标

日期：2026-09-11。发送方 Agent，回复 robot-005 及用户最新 target 列要求。
已阅读 robot-005 的 console 统一 CSV 交接；本次修改 Agent 运行目录内 motions.csv，
未代写 robot_status，未改 console 记录器，未部署或外发。

## 实现

列顺序为 started_at,finished_at,phase,action,action_xyz_yaw,before,target,after,error。
target 是公共世界坐标 (x,y,z,yaw)，cm/deg 两位小数；相对 action 仍为机体系增量。
Robot /v21/navigation/status 从任务原始 goal 转换返回 target，保留 global Z/零 Z
的真实受理目标，不使用 Agent 非原子的 before 样本推算。land 不返回固定 target。
Client 缓存既有任务查询结果，CSV 不为日志额外请求；相对失败时记录已知任务 ID，
如目标已在恢复状态查询中取得则保留，否则留空。绝对导航可使用已受理请求目标。

error 全部统一为世界坐标 after-target（yaw 归一化），因此与此前 Agent 相对 XY
机体系 error 表达有变化；与 robot-005 的 console 相对 XY 误差坐标系也不同。
未知目标、跨 epoch、失败未采到 after 或降落留空，不伪造误差。原 before/after
仍为 Agent 采样，不冒充 Robot 原子受理/终态位姿。

## 验证

Agent 52 项通过（17.636 s），Robot 114 项通过（6.677 s），日志
logs/motion_target_agent.log、motion_target_robot.log。覆盖 global Z 的 before.z=110、
dz=5、真实 target.z=105、after.z=110 应得 error.z=+5；world XY、yaw 跨界；
Robot ENU m/rad 到公共 cm/deg 的转换；landing 不返回伪目标；本地 HTTP 三轮
模拟任务目标列。最初额外状态读取影响原故障测试，已删除该额外读取，复用原有
状态查询后回归通过。git diff --check 通过；未实飞或重启服务。

## Robot 配合

需同步 src/robot/controllers/owl_ego.py 并在落地后重载 HTTP server；bridge 已存
goal，无需为这个字段修改控制逻辑。旧 HTTP 没有 target 时仍兼容，但 Agent 相对
target/error 留空。Robot console 的 motions.csv 目前还是 robot-005 七列；如需
同步最新 target 列，可直接使用已有任务 goal，无需另推算 global Z。
