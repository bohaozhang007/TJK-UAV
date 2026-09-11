# Agent / Robot 协作索引

共同规则：[AGENTS.md](../../AGENTS.md)。接口唯一依据：
[owl_ego_contract.md](../owl_ego_contract.md)。

当前进度：

- [agent_status.md](agent_status.md)：仅 Agent 端维护。
- [robot_status.md](robot_status.md)：仅 Robot 端维护。

任务、回复与技术记录均放在 `messages/`：

- [agent-001](messages/agent-001.md)：原 Agent v21 开发 prompt，原文保留。
- [robot-001](messages/robot-001.md)：对 agent-001 的完整回复，包含实现、相机排查、
  最终水平近似配置、验证证据和待 Agent 配合事项。
- [agent-002](messages/agent-002.md)：审查问题与 Robot 多轮中止、模拟 TRACK、返回及续航任务。
- [robot-002](messages/robot-002.md)：本轮修复、三轮完整链路与七类故障验证、Agent 接口适配要求。
- [agent-003](messages/agent-003.md)：Agent 接入完成，本端回归与三轮 Agent/Robot HTTP 模拟。

消息按实际对话往返组织：`agent-001 → robot-001 → agent-002 → robot-002`。
agent-003 已在 Agent 本地写入，待用户同步；下一轮由 Robot 回复 robot-003。
同一轮本地讨论只补充当前己方消息，不另起编号。

不再设置 archive 或独立 camera-check、handoff、implementation 文件，也不保留
旧跳转页。消息记录历史，status 记录当前状态，contract 记录接口；职责不重复。
