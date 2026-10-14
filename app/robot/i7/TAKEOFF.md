# 起降控制模式

配置位于 `app/robot/config/i7.yaml`：

```yaml
control:
  px4_takeoff_land: false
```

默认 `false` 保持现有流程：保持 POSITION，`init` → `takeoff`，等待
`WAIT_OFFBOARD` 后遥控器切 OFFBOARD，由上位机位置目标控制上升。
高度使用 `control.takeoff_height_m`。

设为 `true` 后，落地、未解锁时重启 bridge/server 和 console 使配置生效。
保持 POSITION，输入 `init` → `takeoff` → `wait`，无需手动切 OFFBOARD。
程序读取飞控 `MIS_TAKEOFF_ALT`，检查目标高度边界，请求 AUTO.TAKEOFF，
确认模式后解锁。PX4 负责上升；确认 AUTO.LOITER、已离地、到达高度并停稳后，
以实测悬停位置交接 OFFBOARD，供后续移动指令使用。

PX4 文档的 MIS_TAKEOFF_ALT 默认值为 2.5 米；实际使用当前飞控参数值，
不会写入或覆盖该参数，也不使用 `control.takeoff_height_m` 覆盖它。
参考：https://docs.px4.io/v1.16/en/flight_modes_mc/takeoff

两种模式的 `land` 均使用现有 AUTO.LAND 流程，等待新鲜的落地且未解锁反馈确认。
PX4 自主上升期间，`stop` 的 OFFBOARD 悬停前提尚未成立；中止请用 `land`
或遥控器接管。切回 POSITION 等非预期模式会终止任务，不会自动重新夺回控制。

飞控拒绝模式/解锁、遥测失效、控制权失效或交接超时不会自动重试。
起飞任务失败不等于落地：PX4 可能仍处于自主起飞或悬停模式，需检查反馈，
使用 land 或遥控器完成降落。旧固件若不支持未解锁时进入 AUTO.TAKEOFF，
本流程会报错，不会尝试绕过检查。

此功能通过无飞行单元测试；实机自主起飞及 OFFBOARD 交接仍需验证。
