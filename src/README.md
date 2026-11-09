# TJK-UAV2 VLN 开发说明 · 当前位置悬停复检

## 目标坐标与回退诊断日志（2026-11-08）

仍按原命令在 `src/` 运行 `python3 task/main.py --autofocus`，新增记录自动写入本项目 `logs/<运行时间>/log.txt`，使用现有 `[TIMING]` JSON 格式，无需调整 Windows 服务。

| 搜索事件名 | 查看内容 |
| --- | --- |
| `uav.target_localization` | 每个检测的置信度、框、相机坐标、世界坐标、采样无人机位姿、目标到无人机的距离，以及定位/8 m 距离过滤结果 |
| `uav.dedup_match` | 分配或关联的 target_id、与各历史目标最接近的参考坐标、XYZ 偏差、去重阈值，以及新建/重试/已完成跳过等原因 |
| `uav.observation_positions` | 原采样位姿 A、当前悬停位姿 B，以及本次原目标的估算坐标 |
| `uav.recheck_match` | 每个原目标与当前复核候选的坐标、XYZ 偏差、欧氏距离、候选归属、匹配/歧义/无候选的原因 |
| `uav.recheck_unavailable` | 图像采样或请求等操作重试后仍失败；具体异常保留在相邻 warning 中 |
| `uav.recheck_later_missing` | 多目标拍摄期间，再次刷新后原本匹配的目标不再得到确认 |
| `uav.return_for_unconfirmed_targets` | 实际准备回退的目标编号、原目标坐标和返回位姿；对照前面的复核事件可查原因 |
| `uav.capture_location_selected` | 统一选择 A 或 B，完整覆盖判断的结果和所选位置的拍摄编号 |
| `uav.current_capture_batch` | 选择 B 后保留的 A 目标编号、B 独有待拍目标编号及坐标 |
| `uav.current_extra_skipped` | B 独有候选与已确认目标重叠、或历史编号有歧义时暂缓处理的原因 |
| `uav.target_capture_start` / `uav.target_capture_result` | 使用当前复核图或原采样图初始化拍摄，以及函数返回结果和 observed 状态 |

所有坐标单位为米。`position_world_m` 是参与去重的世界坐标，`position_camera_m` 是模型输出转换后的相机坐标；`context.pose.position_m` 是该图片配对的无人机位置，`context.frame_id` 用于对照请求。目标快照的 `_localization_context` 保留其来源图片的编号和位姿。帧编号可能随采集重新启动而重复，需连同时间、phase 和 pose.stamp_s 查看。

`delta_xyz_m = 当前估算位置 - 参考位置`。巡检去重保持原规则：XYZ 三个轴的偏差分别严格小于 1.8 m；日志对每个历史目标列出按最大轴偏差最小的历史参考点。当前位置复核保持原规则：欧氏距离不大于 1.8 m，并且双向唯一匹配。两个判据不同，日志中的 `metric` 会明确区分。

例如，同一个目标在 A、B 处分别估计为 `[5, 0, 1]`、`[7, 0, 1]`，复核日志会给出 `delta_xyz_m=[2, 0, 0]`、`distance_m=2.0`、`outside_distance_gate`，随后可能出现该目标的回退记录。若看到 `ambiguous_candidates`，则是候选不唯一；若看到 `outside_distance_limit`，则是在匹配前被 8 m 距离过滤。不能把所有回退都归因为深度误差。

坐标是模型和位姿共同计算的估算值，不是真值；应结合图像确认是否同一螺栓，并检查位姿/坐标系波动。`captured=true` 表示当前 autofocus 函数返回成功，仍包含现有的“达到 20 步后有框即拍照”分支，不单独证明构图达标。去重距离阈值保持原值；当前位置处理策略已更新为下述完整覆盖后统一选点。

## 1. 软件职能与执行流程

本项目面向无人机巡检，构建由视觉目标驱动的 VLN 任务闭环：**航线飞行中检测定位 → 目标筛选去重 → 当前位置悬停复检 → 就地拍摄或回访拍摄 → 恢复航线巡检**。我方负责视觉感知、目标管理和观测任务编排，交控方负责起飞、航线执行、导航控制及到点反馈。

本目录为 `TJK-UAV2`，运行代码位于 `src/`。若将本目录部署为无人机端 `~/TJK-UAV2`，运行目录为 `~/TJK-UAV2/src`；本文文件路径均相对于 `src/`。修改本地目录不会自动更新无人机上的部署。

### 系统框架

| 层级 | 职能 |
| --- | --- |
| 无人机任务端：`task/`、`util/` | 编排检测、目标筛选、观测、拍摄与控制权交还 |
| 硬件与导航适配：`hardware/`、`interface/` | 读取 K40T 图像、FAST-LIO 位姿和航点，控制云台与变焦，对接交控导航接口 |
| Windows 推理端：`win/` | 通过 HTTP 提供目标检测、深度定位及跟踪；各模型在独立 Python 环境中运行，异步保存图像 |

### VLN 任务闭环

1. **航线巡检与实时检测**：交控执行既定航线；任务端持续采集图像，配对当时的机体位姿和航点，发送至 Windows 推理端。
2. **目标位置估计**：检测模型输出目标框或掩码；深度模型估计深度及相机内参，在目标区域反投影得到三维点，剔除异常深度后取中位位置，再结合机体位姿转换至 SLAM 世界坐标系。
3. **筛选与去重**：保留同坐标系、距采样位置不超过 8 m 的有效目标；按历史位置匹配目标 ID，各坐标轴差值小于 1.8 m 时视为同一目标。已完成拍摄或累计尝试达到 3 次的目标不再触发观测。
4. **触发观测任务**：发现待观测目标后暂停巡检检测，保留原图 A、采样位姿 Pₐ 和航点快照。`--autofocus` 模式先接管当前位置 Pᵦ，确认悬停后重新检测定位；匹配到原目标则就地拍摄，否则回到 Pₐ，使用 A 和原框初始化 SAM2 后跟踪拍摄。非 autofocus 的观测点模式保持原流程。
5. **恢复巡检**：完成本批目标后，通过交控接口交还任务控制权，恢复原航线及实时检测，继续下一轮闭环。

本文“检测点”指与检测图像配对保存的无人机位姿。

### 两种执行模式

Windows 可继续使用已有 `TJK-UAV` 推理服务，HTTP 请求与响应格式没有改变；若运行本目录服务，则在 `D:\TJK\TJK-UAV2\src` 下启动。服务入口会自动切换到 `D:\TJK\envs\yolo11\python.exe`。无人机端命令在实际部署目录（例如 `~/TJK-UAV2/src`）执行。先启动 Windows 服务并等待 `Ready`，再启动无人机端交控任务及本项目任务端。

**模式 A：完整覆盖时留在当前位置，否则整批回原采样位置**

```powershell
# Windows 端：加载检测、深度模型及 SAM2 跟踪器
python win/server.py --det yolo --depth moge3 --autofocus
```

```bash
# 无人机端
cd ~/TJK-UAV2/src
python3 task/main.py --autofocus
```

发现待观测目标后执行以下步骤：

1. 读取当前位置 Pᵦ，将该位置和航向通过原 `/initialpose` 接口下发，接管任务控制权。等待交控到点反馈，并额外检查位姿连续稳定；仅暂停检测线程不会让飞机悬停。
2. 相机恢复 1×、云台零位后，绕过旧巡检队列，重新并行读取图像 B 和位姿，将 B 发送 Windows 检测和定位。等待期间不发布恢复航线指令，交控继续保持悬停目标。
3. 先将 B 的定位结果与 A 中本批待拍目标匹配，沿用 A 的编号，不能因为这些编号已存在而将它们过滤为已完成。匹配要求同一世界坐标系、三维欧氏距离不超过 `util/dedup.py` 的 `RECHECK_MATCH_DISTANCE_M=1.8` 米，且双方均只有唯一候选。此覆盖检查不分配新 ID、不修改历史；任何 A 目标缺失或匹配歧义都视为未完整覆盖。
4. **全部覆盖时选择 Pᵦ**：保留 A、B 共同待拍目标，对 B 剩余候选按历史记录去重，跳过已完成/尝试耗尽的目标，将 B 独有的新目标和仍可重试目标加入本批。新目标此时才分配编号；与已确认目标重叠、或同时匹配多个历史编号的候选暂缓处理，避免重复拍摄或误用编号。确认的 A 目标补充 B 世界坐标到历史，后续更容易去重。
5. 在 Pᵦ 依次拍完整个选定批次，SAM2 使用复核新图和新框。每个后续目标轮到前重新采图定位，与整个 B 批次关联（包括已拍目标，防止身份串用）。刷新后未确认的目标保持未完成、不使用旧框强行拍摄，也不再切回 A；交还巡检后等待重新检测。每批仅接纳首次 B 图的待拍目标，后续刷新用于更新已有目标，不无限追加新目标，避免长期占用巡检。
6. **未全部覆盖时选择 Pₐ**：不先拍任何 B 目标，不为 B 独有目标分配编号或修改历史。再次清空巡检队列，回 Pₐ 一次，拍摄 A 整批待拍目标。等待位姿稳定后，恢复相机 1×、云台零位，使用 A 和原检测框初始化 SAM2，随后用实时新帧跟踪；不在 Pₐ 额外做 YOLO＋深度确认。此路径依赖回访位姿和视角恢复到与 A 足够接近的状态。
7. 跟踪返回失败不标记完成，实际进入跟踪的目标计入一次尝试，最多三次；仅复核失败/刷新缺失不消耗跟踪尝试次数。处理完所选位置后释放控制权，等待 1 秒，从空巡检队列恢复采图传输，不要求从 A 先飞回 B。

例如：A 待拍 1、2，B 检出 1、2、3，则在 B 拍 1、2、3；A 待拍 1、2、3，B 检出 2、3、4，则直接回 A 拍 1、2、3，4 本批不处理。

Windows 继续使用另一目录 `D:/TJK/TJK-UAV/src` 下的 `python win/server.py --det yolo --depth moge3 --autofocus`，无需修改该服务。上述选择、去重和任务编排都在无人机侧完成；仍使用原 `/detect`、`/track/init`、`/track` 接口及原有请求/响应字段。

复检返回后若检查发现飞机已偏离悬停位置和航向的允许范围，或发现原采样与当前世界坐标系不同，则停止本次流程并记录异常，不继续使用位置已经不对应的确认图。该检查不是对 HTTP 等待期间运动轨迹的连续监测，等待期间的位置保持由交控负责。

悬停参数位于 `util/flight.py`：位置误差上限 0.3 m、航向误差上限 5°；连续 3 次读取需满足目标误差限制，相邻读取的位置变化不超过 0.05 m、航向变化不超过 2°；等待稳定超时为 10 s。参数需根据实机悬停精度验证。图像和位姿仍为采样时配对，不是严格曝光同步。

此处相机调焦（`autofocus`）指通过目标跟踪、云台居中和变焦完成自动取景，原理如下：

1. **连续跟踪**：SAM2 跟踪器逐帧更新同一目标的位置和目标框，作为相机调整的反馈。
2. **云台居中**：计算目标框中心相对画面中心的偏差，优先调整偏差较大的水平或垂直方向；转动量随偏差增大、随当前变焦倍率增大而减小，逐步将目标移至画面中心。
3. **变焦取景**：目标居中后，根据目标框宽、高相对画面的占比调整倍率。取两者较大值作为目标占比，以约 30% 为取景目标；目标偏小时放大，偏大时缩小，调整后继续跟踪并校正居中。
4. **拍摄与恢复**：居中及尺寸满足阈值后拍摄；最多调整 20 轮，到达上限且跟踪仍有效时也执行拍摄，跟踪丢失则跳过本次拍摄。结束后恢复云台零位和 1 倍变焦，处理完本批目标后交还控制权。

**模式 B：直接前往目标观测点拍摄**

```powershell
# Windows 端
python win/server.py --det yolo --depth moge3
```

```bash
# 无人机端
cd ~/TJK-UAV2/src
python3 task/main.py --skip-exposure-point
```

发现新目标后，依据目标估计位置和航线方向生成观测点，跳过初次返回检测点的步骤，直接前往各观测点并朝向目标拍摄。完成本批目标后，**仍返回检测点**，再交还控制权，使巡检从检测点继续，避免从观测点直接续飞而跳过检测点之后尚未完成检测的航段，造成漏检。

任务端不带参数时，会先返回检测点，再执行观测点访问；同时传入两个参数时，优先执行 `--autofocus` 流程。当前每个目标每次拍摄 1 张图像，并发送至推理端检测和存档，拍摄阶段不重复计算深度。

新增日志事件：`uav.hold_current_pose`、`uav.wait_for_stable_pose`、`uav.recheck_targets`、`uav.recheck_decision`、`uav.observe_current_or_return`。`held_recheck` JSON 行包含复检图的位姿、检测结果、匹配及缺失 ID。复检使用负数 `frame_id`，与原巡检正数编号区分；沿用现有 HTTP 请求 ID 和耗时日志。

离线回归测试（在项目根目录执行，不连接 ROS 或真实相机）：

```powershell
& D:/TJK/envs/yolo11/python.exe -m unittest discover -s src/tests -v
```

2026-11-08 完整覆盖选点版本验证：37 项单元测试通过；使用 `TJK-UAV` 原有真实 YOLO、MoGe3、SAM2，对 26 张配有采样位姿的历史原图推理，并执行 5 个回放场景，流程断言全部通过，包括 B 独有目标加入拍摄、部分覆盖整批回 A。结果见本项目 `logs/offline_model_replay_20261108_203157/summary.json`，两端日志也在该目录。该验证模拟 ROS、飞行和相机执行器，输入图片不会响应云台指令；部分场景触发 20 步上限拍摄，不能把流程通过视为真机构图或跟踪质量验证通过。

测试使用替身覆盖 ROS、飞控及模型接口；实际交控接管后的持续悬停效果和目标位置关联阈值仍需实机验证。

### 启动与接口约定

- 无人机端依赖 ROS 1、MAVROS、FAST-LIO 和 K40T，视频流地址与超时配置在 `hardware/video_stream.py` 顶部，云台及 Windows 服务地址配置在 `hardware/k40t.py` 顶部。

可先独立测试相机连接，无需启动 ROS 或 Windows 推理服务：

```bash
python3 hardware/test_camera.py
# 连续解码 30 帧，并保存最后一帧；总超时 20 秒
python3 hardware/test_camera.py --frames 30 --timeout 20 --save /tmp/k40t.jpg
```

测试默认连续解码 10 帧，总超时 15 秒，成功输出 `PASS`，失败返回非零退出码。测试不发送飞行或云台控制指令。取流同时兼容 OpenCV 4.2 和较新版：4.2 使用 FFmpeg RTSP socket 超时，较新版使用 OpenCV 的打开和读取超时参数。
- Windows 端 Conda 安装在 `D:\anaconda`，运行环境统一放在 `D:\TJK\envs`。`win/server.py` 自动从项目的同级 `envs/` 目录选择独立环境，MoGe 也使用 Conda，不再使用从其他机器复制的 `.venv`。
- 交控任务启动命令见 `start.txt`。任务端依赖 `OFFBOARD` 状态运行，离开该模式时退出。
- 推理服务默认端口为 `8790`，接口为 `/health`、`/detect`、`/track/init` 和 `/track`；跟踪接口需服务端启用 `--autofocus`。

| ROS 话题 | 用途 |
| --- | --- |
| `/laserMapping/odometry` | 获取 SLAM 位姿 |
| `/buaa_goal` | 获取航线航点 |
| `/initialpose` | 下发观测/回访位姿，触发交控接管该任务 |
| `/vln_point_reached` | 接收位置及航向到达反馈，单次发布 `True` |
| `/recover_mission` | 发布 `True`，交还控制权并恢复航线任务 |

`/initialpose` 使用交控自定义协议：`orientation.w` 承载航向角，单位为度。目标位姿与定位结果须使用同一 SLAM 坐标系。

### 本机 Windows 环境

三个环境均为 Python 3.10，模型源码及权重保持在原目录：

| 模型 | Conda 环境 | PyTorch / torchvision | 源码或权重目录 |
| --- | --- | --- | --- |
| YOLO26l | `D:\TJK\envs\yolo11` | `2.7.1+cu118` / `0.22.1+cu118` | `D:\TJK\yolo\runs\bolt_yolo26l_10-22\weights\best.pt` |
| SAM2.1 | `D:\TJK\envs\sam2` | `2.7.1+cu118` / `0.22.1+cu118` | `D:\TJK\sam2`，以 editable 方式安装 |
| MoGe-3 | `D:\TJK\envs\moge3` | `2.10.0+cu128` / `0.25.0+cu128` | `D:\TJK\MoGe`，以 editable 方式安装 |

YOLO 使用 `ultralytics==8.4.163`。SAM2 安装时设置 `SAM2_BUILD_CUDA=0`，不编译可选的连通区域 CUDA 扩展；模型主体仍使用 GPU。MoGe 使用 `triton-windows==3.6.0.post26`，FlexGEMM、utils3d-moge 和 pipeline 使用 `MoGe/pyproject.toml` 指定的 Git 提交。MoGe 的 Triton 内核需要支持 CUDA 12 的 NVIDIA 驱动；本机已从 497.17 更新至 NVIDIA Studio 616.92，安装退出码为 0，未执行重启。

无需修改系统 Python 或执行 Conda 激活即可启动：

```powershell
cd D:\TJK\TJK-UAV\src
python win/server.py --det yolo --depth moge3 --autofocus
```

`D:\TJK\envs` 已加入 Conda 的环境搜索目录，可通过 `D:\anaconda\Scripts\conda.exe env list` 查看 `yolo11`、`sam2`、`moge3`。SAM3 和 DA3 是可选方案，本次未创建对应环境或下载权重。

完整依赖快照保存在 `win/requirements-yolo11.txt`、`win/requirements-sam2.txt` 和 `win/requirements-moge3.txt`。在本机重建某个环境时，先用 Conda 创建 Python 3.10 环境，再用该环境的 Python 安装对应文件。例如：

```powershell
& D:\anaconda\Scripts\conda.exe create --prefix D:\TJK\envs\sam2 python=3.10 pip -y
& D:\TJK\envs\sam2\python.exe -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu118
$env:SAM2_BUILD_CUDA = "0"
& D:\TJK\envs\sam2\python.exe -m pip install --no-build-isolation -r D:\TJK\TJK-UAV\src\win\requirements-sam2.txt
```

SAM2 的 `--no-build-isolation` 使安装流程复用预先安装的 PyTorch，避免额外下载不同版本。快照中的 editable 路径指向本机源码，迁移目录时需同步修改。

2026-11-01 本机验证：三个环境的 `pip check` 均通过；三个模型均完成 1920×1080 预热。使用 `yolo/videos/source/1010.mp4` 的真实视频帧，通过 HTTP 完成目标检测、三维定位、跟踪初始化、连续跟踪和不计算深度的拍摄存档。直接执行上述 `python win/server.py ...` 命令已输出 `Ready`，`/health` 的检测、深度和跟踪状态均为 `true`。这些检查未连接无人机或控制真实硬件。

## 2. 模型与官方链接

| 模型 | 本项目用途 | 官方项目 |
| --- | --- | --- |
| Ultralytics YOLO26l | 使用合并后的 10-22 数据集训练，检测巡检目标并输出目标框及置信度；以框区域采样深度 | [Ultralytics](https://github.com/ultralytics/ultralytics) |
| SAM 3 | 可选检测方案，以参考图像及框提示匹配目标并输出分割掩码 | [SAM 3](https://github.com/facebookresearch/sam3) |
| SAM 2.1 | 自动取景时跟踪目标，为云台居中及变焦提供目标框 | [SAM 2](https://github.com/facebookresearch/sam2) |
| MoGe-3 | 单目几何估计，提供深度和相机内参，用于目标三维定位 | [MoGe](https://github.com/microsoft/MoGe) |
| Depth Anything 3 | 可选深度方案，当前加载 `DA3NESTED-GIANT-LARGE`，用于目标三维定位 | [Depth Anything 3](https://github.com/ByteDance-Seed/Depth-Anything-3) |

服务端通过 `--det yolo|sam3`、`--depth moge3|da3` 选择组合。使用 SAM 3 时，还需传入 `--ref-img` 和 `--ref-box`；框文件包含像素坐标 `x1 y1 x2 y2`。

**当前 YOLO26l 使用 `detection-luoshuan/datasets/10-22` 合并数据集训练，加载 `yolo/runs/bolt_yolo26l_10-22/weights/best.pt`，输入长边 1920，默认置信度阈值 0.5。模型在 Windows 推理端加载，不部署在无人机端。** 无人机端通过 HTTP 发送图像并获取检测结果；checkpoint 路径配置见 `win/detector_yolo.py`。

## 3. 文件说明

| 文件 | 作用 |
| --- | --- |
| `task/main.py` | 任务入口；选择模式，编排检测、回访、观测拍摄和控制权交还 |
| `task/detect_thread.py` | 并行采集图像与位姿，配对航点，调用检测并筛选新目标 |
| `task/autofocus.py` | 调用跟踪器，调整云台和变焦，触发拍摄并恢复相机状态 |
| `task/approach_target.py` | 根据目标位置与航线方向生成观测位姿，安排访问顺序 |
| `hardware/k40t.py` | K40T 视频读取、云台/变焦控制，以及推理服务客户端 |
| `hardware/pose.py` | 订阅 FAST-LIO 里程计，获取有效位姿 |
| `interface/to_target_pose.py` | 下发目标位置与航向，提供独立调试入口 |
| `interface/check_goal_reached.py` | 接收并等待交控到点反馈，提供独立调试入口 |
| `interface/next_waypoint.py` | 缓存上一航点与下一航点，供观测点规划使用 |
| `interface/release_control.py` | 发送控制权释放指令，恢复交控航线任务 |
| `util/flight.py` | 封装导航接口连接、目标下发、到点等待与控制权释放 |
| `util/dedup.py` | 维护目标 ID、历史位置、观测状态和尝试次数，实现空间去重 |
| `util/terminal_log.py` | 将任务端终端输出同步保存至 `TJK-UAV/logs/<运行时间>/log.txt` |
| `win/server.py` | HTTP 推理服务入口；管理独立模型进程、请求路由及图像存档 |
| `win/detector_base.py` | 检测模型公共预热与进程通信逻辑 |
| `win/detector_yolo.py` | 在我方 Windows 推理端加载 YOLO checkpoint，执行目标检测 |
| `win/detector_sam3.py` | 基于参考图像和框提示执行 SAM 3 检测与分割 |
| `win/tracker_sam2.py` | SAM 2.1 跟踪器，支持目标框初始化与逐帧跟踪 |
| `win/depth_base.py` | 深度采样、异常值过滤、三维反投影及世界坐标转换 |
| `win/depth_moge3.py` | MoGe-3 深度与相机内参估计适配 |
| `win/depth_da3.py` | Depth Anything 3 深度与相机内参估计适配 |
| `win/image_writer.py` | 异步保存原图及标注图至 `TJK-UAV/logs/<运行时间>/original/`、`annotation/` |
| `start.txt` | 当前部署启动命令及导航接口调试示例 |
| `code-style.md` | 项目代码风格约定 |

## 4. 最新进度与 TODO

**v1：VLN 接口调试完毕，已打通目标位姿下发、到点反馈及控制权交还链路。交控方已改进回访控制和雷达配合相机定位方案；两项改进效果及两种模式下的完整任务流程，均待双方联合真机验证。**

### 我方

- **检测漏检**：当前已切换到合并 10-22 数据集训练的 YOLO26l，漏检改善情况仍需现场验证；需交控方补充不同场景的采集数据。
- **跟踪失误**：当前跟踪存在失误。我方负责调研更优跟踪器，并验证云台调整和变焦过程中的跟踪稳定性。

### 交控方

- **返回检测点不稳定**：此前回访过程中存在大幅摇摆，现阶段采用锁机头措施缓和。交控方已改进回访控制，双方需通过真机测试确认摇摆是否减轻及到点是否稳定。
- **目标定位与重复复检**：交控方已改进斜装雷达配合相机的定位方案，目标是缩短定位时间、提高定位一致性并减少重复复检；这些效果尚待双方真机测试确认。

### TODO

- [ ] 交控方补充不同场景的采集数据；我方完成标注、训练与漏检评估。
- [ ] 我方调研更优跟踪器，验证跟踪丢失及变焦场景表现。
- [ ] 联合真机测试交控回访改进，重点检查大幅摇摆与到点稳定性。
- [ ] 联合真机测试斜装雷达与相机定位方案，评估定位速度、精度及重复复检情况。
- [ ] 构建面向端侧算力约束的任务智能体，解析自然语言指令，调度检测、定位、导航与拍摄接口，采用推理端决策、无人机端轻量执行的分层架构。
- [ ] 设计基于视觉的最佳拍摄位姿估计模型，根据目标可见性、遮挡及画面占比，估计适合拍摄的观测位置与朝向，引导无人机完成主动观测。
- [ ] 使用 SAM 3 模型的文本提示与视觉提示能力，实现语言描述、参考图像与场景目标之间的语义对应，完成语义驱动的目标识别与分割，并接入定位、去重与观测流程。
