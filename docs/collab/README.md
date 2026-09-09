# Agent / Robot 协作索引

共同规则：[AGENTS.md](../../AGENTS.md)。接口唯一依据：
[owl_ego_contract.md](../owl_ego_contract.md)。

当前进度：

- [agent_status.md](agent_status.md)：仅 Agent 端维护，目前为空，不代写。
- [robot_status.md](robot_status.md)：仅 Robot 端维护。

任务、回复与技术记录均放在 `messages/`：

- [agent-001](messages/agent-001.md)：原 Agent v21 开发 prompt，原文保留。
- [robot-001](messages/robot-001.md)：对 agent-001 的完整回复，包含实现、相机排查、
  最终水平近似配置、验证证据和待 Agent 配合事项。

消息按实际对话往返组织：`agent-001 → robot-001 → agent-002 → robot-002`。
当前等待 Agent 的下一轮回复；同一轮本地讨论只补充当前己方消息，不另起编号。

不再设置 archive 或独立 camera-check、handoff、implementation 文件，也不保留
旧跳转页。消息记录历史，status 记录当前状态，contract 记录接口；职责不重复。
