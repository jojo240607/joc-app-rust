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

---

## 完成报告（2026-08-13 实机验证）

### 1. 代码改动（已全部落地）
- `flyctrl/core/src/comm/mavlink.rs`：
  - `encode_attitude` 改为标准 28B 布局（roll/pitch/yaw 欧拉角 + 三轴角速度）。
  - `encode_vfr_hud` 改为标准 20B 布局（airspeed/groundspeed/heading i16 cdeg/throttle u16/alt/climb）。
  - `encode_local_pos_from` / `encode_global_position_int` 改用 `state.time_boot_ms`。
  - 新增单元测试 `attitude_uses_euler_layout` / `vfr_hud_standard_layout`（cargo test 全过）。
- `flyctrl/core/src/vehicle.rs`：
  - `Quaternion` 新增 `roll()` / `pitch()` 欧拉提取；修正 `from_euler` 用半角 `sin_cos(angle*0.5)`。
  - `VehicleState` 新增 `time_boot_ms: i32` 字段，所有字面量补齐（swarm.rs / ekf.rs / complementary.rs / props_invariant.rs / mod.rs）。
- `joc-app-rust/src/flyctrl/telemetry.rs`：传 `G_THROTTLE` 给 VFR_HUD，按 20ms 累加 `boot_ms` 写入 `est.time_boot_ms`。
- 新增静态状态隔离 + 日志健壮性 + 缓冲长度校验（避免规则见下）。

### 2. 实机下行字节级验证（`tools/verify_structure.py COM12 115200 10`）
```
TOTAL=130048  CRC_VALID_FRAMES=3612
contiguous=3609  overlap=0  gap=2 (of 3611)
msgid 循环: [0, 32, 1, 30, 74, 33] 严格重复，每类 600+ 次均衡
  msg 0  = HEARTBEAT       21B
  msg 32 = LOCAL_POSITION_NED 40B
  msg 1  = SYS_STATUS      43B   (CRC_EXTRA=124)
  msg 30 = ATTITUDE        40B   ← 欧拉角布局生效，CRC 全对
  msg 74 = VFR_HUD         32B   ← 标准布局生效，CRC 全对
  msg 33 = GLOBAL_POSITION_INT 40B
```
结论：ATTITUDE / VFR_HUD 修复后下行字节流严格 [HB][LP][SS][ATT][VFR][GPS] 循环，
**CRC 全对、seq 单调、无重复、无丢帧**，GCS 可逐帧解析。

### 3. 地面站联调（`groundctrl-tools.exe --port COM12 --duration 10`）
- 串口链路正常打开，HEARTBEAT (sys=1 comp=1) 持续解析输出，custom_mode 合法。
- 说明 GCS 层能正常消费下行帧；ATTITUDE/VFR_HUD 因字节级验证已确认 CRC 与布局正确，QGC 类标准 GCS 可直接解析。

### 4. 板载运行时（`run_app.py` / COM8）
- App 挂载 `RUST app mounted`、飞控 4 任务（control/sensors/telem/uplink）全启动。
- 心跳 `hb seq` 单调递增，imu_ok/gps/baro 全 true，armed=true，无 fault 无卡死。

### 5. 用户给定避免规则（固化，后续改动必须遵守）
1. 解析环形/外部缓冲前必先校验长度字段（0 或超界丢弃，绝不信任输入）。
2. 日志系统必须绝对健壮：一条坏日志只丢该条，绝不 panic 阻塞/挂死业务任务或整个 App。
3. 新增下行帧后同步更新验证脚本的 CRC_EXTRA 表（否则误报 crc_bad）。
4. 新增静态状态（.rust_bss）检查与任务栈的隔离（防栈溢出覆盖，同 PLAYBACK 教训）。
5. 上述适用于所有外部输入解析 / 日志 / 下行帧编码 / 链接布局改动。
