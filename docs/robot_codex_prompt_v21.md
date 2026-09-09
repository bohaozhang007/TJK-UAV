# 发给无人机端 Codex 的提示词

请在无人机端 TJK-UAV 仓库实现新的 `owl_ego` Robot 后端。Windows 本机已实现
Agent v21 和 `src/robot_client/owl_ego.py`；不要再实现 Agent，也不要改变双方
已经定义的字段。先阅读随本提示词提供的 **`docs/owl_ego_contract.md`**，它是
完整 HTTP、坐标、时间同步和任务状态契约。若无人机仓库没有这个文件，请先
向我索取该文件，不要自行猜接口。也可以参考随附的本机 Client 源码。

## 任务背景

- Agent/SAM3/SAM2/DA3 在 Windows 本机；Robot、ROS、EGO-Planner、MAVROS 在 OWL 上。
- Wi-Fi HTTP 通信，Robot 端口默认 8765，Agent 选择 `--robot owl_ego`。
- v21 沿配置航点连续巡航，飞行途中异步检测。发现新目标后取消巡航并确认停止，
  导航回检测帧的拍摄位姿，复用 v20 TRACK 对准和接近目标，跳过 SCAN，再回到
  拍摄位姿，重新导航到原下一个航点。
- 目标位置估计和 100 cm 去重在 Agent 完成。Robot 只提供同步观测和执行任务。
- 使用开源 EGO-Planner-v2 的规划能力，**新后端不经过 Captain 的导航或控制接口**。
  保留旧 `owl` 后端、v20 以及其他后端。

## 实现范围与分层

1. 新增 `src/robot/hardware/owl_ego.py`：只负责 ROS I/O、传感器缓存和服务调用。
2. 新增 `src/robot/controllers/owl_ego.py`：坐标转换、HTTP 任务状态、请求幂等、
   控制会话租约、导航到达/取消确认和错误分类。
3. 新增 `ros/owl_nav/` 包：开源 EGO 轨迹到 MAVROS 的执行桥接，具有独立控制循环、
   制动/悬停、yaw 限速、起降、人工接管、旧轨迹隔离。可参考仓库
   `ros/i7_nav/scripts/i7_nav_node.py`，但不能照搬 I7 的消息约定、特殊姿态编码、
   标定或参数。尤其审核其仅按消息到达时间接受新轨迹、旧轨迹过期后的处理，
   要补足任务归属与停止恢复保证。
4. 注册 `--robot owl_ego`，新增 Robot YAML、launch 和可独立运行的启动脚本。
   不要修改本机的 `src/agent/`、`src/robot_client/`、DA3/SAM3 代码和 Agent 启动脚本。
5. 扩展 `src/robot/server.py` 暴露契约接口，严格限制为 owl_ego 能力，旧后端保持兼容。

## 先核对实际部署，再接入

- 检查机上 PX4、MAVROS、相机、VIO、iToF、TF、ROS launch 的实际版本和连接关系。
  保留传感器、定位融合和 MAVROS 服务。不要直接停止所有厂家服务。
- 新控制链路启用时，Captain 和厂家 mavros_controller 等不能同时向同一 MAVROS
  运动接口发送竞争指令。把互斥和必要依赖写进启动流程，默认先做只读检查。
- 接入并锁定开源 EGO-Planner-v2 的实际版本/commit，记录改动和话题映射。
  不要假设厂商 `/planning/cmd` 是上游接口。
- 使用机载深度/点云及同步位姿做 EGO 避障，不依赖经 Wi-Fi 返回的本机 DA3。
  核对 depth 图编码、单位、有效范围、内参、机体/相机坐标系和时间戳。

## 必须满足的核心语义

- 导航任务异步返回 ID；图像、状态、心跳、取消不能被长运动锁阻塞。
- Agent 提交固定世界 XYZ/yaw，EGO 规划 XYZ；桥接节点控制指定 yaw。支持纯旋转
  及朝向不变的后退，不能被 traj_server 默认朝运动方向的 yaw 覆盖。
- 取消分为请求接受、制动/停止中、已停止。只有旧任务失效且实际速度满足阈值、
  持续稳定后才返回 `cancelled,stopped:true`。绝不把停止收 HTTP 当成停止飞行。
- 取消时清除旧任务及排队轨迹；恢复必须是新的任务和轨迹。旧规划结果可能在新
  目标提交后才到达，不能只凭接收时间判定新旧任务。要有可靠的任务/轨迹隔离方案。
- 审查 upstream `mandatory_stop` 的恢复行为；不要将它直接当成普通 pause/resume。
  如需小范围修改上游或轨迹执行器，明确记录并做停止/恢复测试。
- 状态查询到达必须验证实际 XYZ、yaw、速度和连续稳定采样；规划器结束不等于到达。
- 会话心跳每 0.5 秒，5 秒失联后机载自主停止/悬停；取消、降落、人工接管和 PX4
  failsafe 的优先级必须明确，不得失联后继续旧任务或抢回遥控器控制。
- 相对运动接口用于复用 v20 TRACK，按契约阻塞到完成，但心跳、降落等仍可抢占。
- 相机观测是同一曝光时刻的 rectified RGB、机体 pose、K 和 camera-optical→ENU
  完整外参；包含机体 roll/pitch、相机安装平移和云台状态。CameraInfo 不等于外参。
  图像 resize/crop 后同步调整 K。缺少标定/TF/同步信息则明确失败，不伪造数据。
- 定位重置或坐标原点变化必须更新 localization_epoch 并中止旧任务。

## HTTP 接口清单

具体请求、响应字段和状态以 `docs/owl_ego_contract.md` 为准：

```text
GET  /v21/capabilities
GET  /v21/observation
POST /v21/session
POST /v21/heartbeat
POST /v21/session/release
POST /v21/navigation
GET  /v21/navigation/status?task_id=...
POST /v21/navigation/cancel
GET  /health
GET  /get_pose
GET  /motion_tolerances
POST /init
POST /takeoff
POST /move_relative_xyz_yaw
POST /land
```

## 验证与交付

先完成代码、mock ROS/HTTP 测试和可用的 SITL 验证，不要为了验证实现而直接起飞。
重点测试：飞行中查询和取消并发、取消与到达竞态、旧轨迹迟到、新任务恢复、重复
请求幂等、非法 session/task、失联租约、定位重置、人工接管、独立 yaw、相机同步。
新增接口不应影响原 owl 后端。

交付：改动清单、协议符合性、实际 EGO commit、ROS 话题映射和启动命令、已跑测试、
尚需实机验证项、缺失的标定或系统参数，以及一份可以发回本机 Codex 的联调摘要。
遇到现有固件/依赖无法满足契约时，明确说明阻塞项和建议，不悄悄返回虚假的成功或
降级成短段飞行。不要部署未验证的飞行配置并自动执行实飞。
