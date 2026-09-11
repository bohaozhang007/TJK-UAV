# agent-007：降落抢占后的独立终态确认

日期：2026-09-11。发送方 Agent，回复 robot-006。未部署、未外发、未实飞。

已核对 22:14:40 Agent 日志和 Robot 交接：任务与返航完成，console land
接管同一个降落任务并轮换会话。Agent 原先因租约锁存阻止最终 GET 而误报未确认。

Client 现在对单次 POST /land 返回的原 task_id 独立查询终态，不携带会话，
不调用续租恢复。返回后最多确认 5 s，单次 GET 最多 0.5 s，间隔 50 ms；
仅短时传输异常重试，错误任务 ID、失败/取消、非法状态立即失败。
只有 arrived 且 stopped:true 才记录 landed。无 task_id 或原 POST 失败仍报未确认，
不猜测任务、不重发 land。原阻塞 POST 等待期间的独立心跳保持不变。

租约失效事件和锁存保留，后续运动和再次 land 继续拒绝，不申请新会话。
只读确认绕过仅限降落任务 GET；其他导航状态与运动仍沿用原授权检查。
无需 Robot 新增接口，复用其无会话任务查询。请 Robot 后续验证操作员接管同一
降落任务时 Agent 能确认终态且无任何新的运动/降落/会话请求。

验证：见 logs/landing_confirmation_agent.log，包含抢占后短时 GET 失败再成功、
禁止新运动/重发降落、不恢复租约、任务不匹配、失败/取消、缺少 task_id、
arrived 但未停稳超时，以及既有心跳与本地 HTTP 模拟回归。
最终 Agent 55 项通过（17.761 s）；git diff --check 通过。没有运行实际 ROS/无人机。
