# 地面站联调计划 (GCS Integration Plan)

目标：让 joc-app-rust 飞控与 `groundctrl` 真实地面站 (QGC 兼容 MAVLink v2) 完整联调，
补齐核心缺口，使地面站能正确解析下行遥测、并能上行下发指令。

## 链路现状
- 下行：telemetry 任务每 20ms 经 usb0 (USB CDC, COM12) 发 MAVLink v2 帧（HB/LP/SS/ATT/VFR/GPS 6 类）。
- 上行：uplink 任务每 10ms 轮询 usb0.read，增量解析 COMMAND_LONG / PARAM_REQUEST_LIST / PARAM_SET。
- 地面站：groundctrl-tools (headless CLI) 连 COM12 验证。

## 待修复项（编码器字段布局对齐标准 common.xml）

### 1. ATTITUDE (msg 30) — 当前错误
- 现状：`time_boot_ms i32` + 四元数 `q[4] f32`（offset 4..20）。
- 标准：28 字节 = `time_boot_ms i32` + `roll f32` + `pitch f32` + `yaw f32` + `rollspeed f32` + `pitchspeed f32` + `yawspeed f32`。
- 修复：从 Quaternion 提取 euler 角（需加 roll/pitch 提取公式），用 w/x/y/z 角速度填 rollspeed/pitchspeed/yawspeed。

### 2. VFR_HUD (msg 74) — 当前错误
- 现状：6×f32（airspeed, groundspeed, alt, climb, heading, throttle）= 24 字节，顺序错。
- 标准：20 字节 = `airspeed f32` + `groundspeed f32` + `heading i16 (cdeg)` + `throttle uint16 (%)` + `alt f32` + `climb f32`。
- 修复：按标准顺序/类型写入；heading 用 yaw_deg*100 取整，throttle 用 G_THROTTLE。

### 已确认正确（无需改）
- HEARTBEAT / SYS_STATUS / LOCAL_POSITION_NED / GLOBAL_POSITION_INT / COMMAND_ACK 字段布局与 CRC_EXTRA 均对齐。

## 上行指令（已就绪，联调验证）
- ARM/DISARM → G_CMD_ARMED → control 逻辑或。
- DO_SET_MODE (param2=custom_mode) / TAKEOFF→ALT_HOLD / LAND / RTL → G_CMD_MODE。
- PARAM_REQUEST_LIST → 逐条 PARAM_VALUE 流水（5 个参数）。
- PARAM_SET → 写入 G_PARAM_VALS 并回显。
- COMMAND_ACK 应答每个指令。

## 执行步骤
1. 修复 `flyctrl/core/src/comm/mavlink.rs`：encode_attitude (euler) + encode_vfr_hud (标准顺序/类型)。
2. 在 `Quaternion` 加 roll()/pitch() euler 提取辅助（vehicle.rs）。
3. flyctrl_core 加单元测试验证 ATTITUDE/VFR_HUD 字段布局与 decode 通过。
4. 重新构建 flyctrl_core + joc-app-rust，烧录 App 分区（flash_app.py）。
5. 运行 groundctrl-tools 联调：监听下行确认 6 类消息字段正确；下发 ARM/SET_MODE/PARAM 确认 ACK/PARAM_VALUE。
6. 更新 verify_downlink.py 统计（如需要）。

## 验证标准
- groundctrl-tools 解析 HEARTBEAT custom_mode 为合法 ArduCopter 码（非垃圾值）。
- ATTITUDE 角速度/姿态、VFR_HUD 高度/航向/油门数值合理。
- 下发 ARM 后 control 解锁、心跳 base_mode 置 ARM 位；SET_MODE 后 custom_mode 反映。
- PARAM_REQUEST_LIST 收到 5 条 PARAM_VALUE，PARAM_SET 回显新值。
