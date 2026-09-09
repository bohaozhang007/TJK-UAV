# Agent 端状态

维护方：Windows Agent 端 Codex。更新日期：2026-09-09。
最新消息：[agent-002](messages/agent-002.md)，回复 [robot-001](messages/robot-001.md)。

## 当前安排

用户决定先由 Robot 端完善并验证“途中中止 → 返回曝光点 → 模拟 TRACK → 返回曝光点 →
继续原航点”的多轮完整控制链路。Agent 本轮仅整理审查反馈与交接，没有修改实现。
消息已本地写入，待用户同步；未 push、连接无人机或触发飞行。

## 已完成与验证

- v21 已有异步检测、曝光位置回退、复用 TRACK、返回后继续航点、DA3 空间去重与 owl_ego Client。
- 上一轮 Windows 审查：Agent 22 项、Robot 32 项离线测试通过；未运行模型或连接 Robot。
- 临时脚本复现规划启动宽限/健康判断冲突、缺少明确 ON_GROUND 的降落成功判定、
  RGB 先于后一个 odom 到达时报错。详见 agent-002；已有测试通过不等于这些问题解决。
- 本次仅修改文档，未重跑测试；尚无真实 Agent 与 Robot 完整任务联调验收。

## 当前配置假设

沿用 owl/v21.yaml、SAM3 + SAM2 + DA3、owl_ego；扫描跳过，去重半径初始 100 cm。
用户已接受固定水平 pitch=0°、零安装平移、中心主点、方形像素及初始 HFOV=90°。
这些为显式近似，未知畸变不得标记成已校正。Agent 的近似 profile opt-in 尚未实现。

## 待完成与阻塞项

- Robot：规划启动/执行失联/安全保持状态一致性，观测同步瞬态处理，明确落地状态判定，
  连续相对运动耗时与多轮中断恢复验证，回复 robot-002。
- Agent：显式接纳并记录近似 profile，同时检查 rectified 与 calibration_quality；
  依据双方确认的状态语义处理规划启动，针对观测暂时不可用做有界重试。
- 后续进行实际 Agent → Robot HTTP → EGO/mock FCU 联调；Robot 单端模拟不等于该验收。
- 实机控制互斥、世界坐标一致性、PX4 failsafe 与实际飞行验证仍未完成，默认禁飞。
