# 地面站（groundctrl）联调计划

> 联调对象：**自研地面站 `groundctrl`**（工程 `D:\project\mcu\oop\groundctrl`，与 joc-app-rust 平级），
> **不是 QGroundControl**。地面站用标准 MAVLink v2 协议经 USB CDC（ST VCP，VID_0483/PID_5740）
> 与板子通信。物理链路：板子 `usb0` ↔ PC COM 口 ↔ groundctrl `SerialLink`（dtr_on_open=false）。
>
> 本文档跟踪板子端 `joc-app-rust` 与地面站的**双向联调进度**，并列出待完善项与优先级。
> 历史已完成的下行帧修复 / 上行接收（uplink）见 `GCS_INTEGRATION_PLAN.md` 与 `uplink-plan.md`。

---

## 1. 能力矩阵（板子端 vs 地面站期望）

### 1.1 已打通（已端到端验证 ✅）

| 方向 | 消息 | 状态 |
|---|---|---|
| 下行 | HEARTBEAT(0) / LOCAL_POSITION_NED(32) / SYS_STATUS(1) / ATTITUDE(30) / VFR_HUD(74) / GLOBAL_POSITION_INT(33) | 字节级验证：CRC 全对、seq 单调、无重复/丢帧 |
| 上行 | COMMAND_LONG(76)：ARM/DISARM(400) / DO_SET_MODE(176) / NAV_TAKEOFF / NAV_LAND / NAV_RTL / REQUEST_AUTOPILOT_CAPABILITIES(520) | 持续发送实测：板子收到→解析→`G_CMD_ARMED`/`G_CMD_MODE` 透传 control |
| 上行 | PARAM_REQUEST_LIST(21) → PARAM_VALUE(22) 流水 | 5 个参数全回 |
| 上行 | PARAM_SET(23) / PARAM_REQUEST_READ(20) | 范围校验 + 回显 |
| 上行 | COMMAND_ACK(77) / AUTOPILOT_VERSION(300) | 正常 |

关键解锁点：**`RC_FORCE_ARM` 已改 `false`**（2026-08-16 提交），地面站 DISARM 不再被虚拟 RC 强制 arm 覆盖，**解锁/上锁真正由地面站 COMMAND_LONG 决定**。

### 1.2 待完善（地面站已支持，板子端缺失）

| # | 功能 | 地面站侧 | 板子端现状 | 优先级 |
|---|---|---|---|---|
| 1 | **MISSION 航点** | `upload_mission`/`download_mission`/MISSION_COUNT(44)/MISSION_ITEM_INT(73)/MISSION_REQUEST(40)/MISSION_ACK(47)/MISSION_REQUEST_LIST(43) | 完全未解析（uplink.rs 无对应分支） | **高** |
| 2 | **RC_CHANNELS_OVERRIDE** | `rc_channels_override` → msg 70 | 未解析；control 仍只读 sim 虚拟 RC | **高** |
| 3 | **FENCE 围栏** | `upload_fence`/`download_fence`/FENCE_POINT(160)/FENCE_FETCH_POINT(161)/FENCE_COUNT(170) | 未解析 | 中 |
| 4 | **MAVLink FTP 固件升级** | `FILE_TRANSFER_PROTOCOL`(110) + `upload_firmware`/`download_firmware` | 无 FTP 服务端 | 中（需经地面站刷机才要） |
| 5 | **REQUEST_DATA_STREAM / SET_MESSAGE_INTERVAL** | request_data_stream / set_message_interval | 遥测固定 20ms，不响应流率请求（不致命） | 低 |

---

## 2. 实现计划（按优先级）

### 2.1 阶段一：MISSION 航点双向（RC_OVERRIDE 一并做）

目标：地面站能上传/下载航点，并能用 RC 通道覆盖直接操控（替代 sim RC）。

板子端改动（`src/flyctrl/uplink.rs`）：
- 新增 `MISSION_*` 路由分支 + 板载航点存储（`.rust_bss` 静态数组 + 计数）。
- 握手协议对齐 groundctrl（见 §3 消息映射）：
  - 地面站 `upload_mission`：发 MISSION_COUNT → 板子回 MISSION_REQUEST(idx) → 地面站发 MISSION_ITEM_INT(idx) → 板子存 + 回 MISSION_ACK。
  - 地面站 `download_mission`：发 MISSION_REQUEST_LIST → 板子回 MISSION_COUNT + 逐个 MISSION_ITEM_INT + MISSION_ACK。
- 新增 `RC_CHANNELS_OVERRIDE`(70) 解析 → 写 `G_RC_OVERRIDE[8]`（`.rust_bss`），control 任务优先取 override（非 0 时覆盖 sim RC）。

### 2.2 阶段二：FENCE 围栏

- FENCE_COUNT / FENCE_POINT / FENCE_FETCH_POINT 路由 + 板载围栏存储。
- 注意：地面站 `mlink/mission.rs` 已**手写 FENCE_POINT 编解码**（绕过 mavlink crate 字段顺序 bug），板子端需与之字节级对齐（先用 `verify_*` 对拍）。

### 2.3 阶段三：MAVLink FTP 固件升级（可选）

- 板子实现最小 FTP 服务端：`FILE_TRANSFER_PROTOCOL` 解析 + 经 `flash` 驱动写 spare sector。
- 仅当需经地面站刷机时实现；否则保持 USB CDC 仅做遥测/指令。

### 2.4 阶段四（低优先）：SET_MESSAGE_INTERVAL

- 板子维护 per-msgid 间隔表，telemetry 按表发；或最简：固定 20ms 不变，ACK 一下忽略间隔。

---

## 3. 关键约束（来自历史踩坑）

1. **地面站不是 QGC**：涉及 custom_mode / 模式显示 / 传感器位等协议细节，以 groundctrl 实际解析约定为准（groundctrl 的 `mlink/mod.rs` 已手写标准顺序编码，绕开 mavlink crate 0.11.2 字段顺序 bug）。
2. **usb0 复用 `Device::get`，切勿 `Device::open`**（二次 open → USBD_Init 重置 TX 状态机 → write 卡死）。
3. **上行 read 非阻塞**（host 未发数据时返回 0，uplink 主循环 `msleep(10)` 让出）。
4. **uplink(prio10) 写全局、control(prio4)/telem(prio12) 读**：单写者 + control 优先级更高，无需 mutex（同 seqlock 理由）。
5. **静态状态（`.rust_bss`）+ `zeroed()`**：App XIP 无 .data，落 Flash 写即 BusFault；新增全局同理。
6. **日志健壮 + 缓冲长度校验**（用户给定规则，见 GCS_INTEGRATION_PLAN.md §5）。
7. **新增下行帧后同步 CRC_EXTRA 表**（tools 侧 `_verify_usb.py` / `verify_structure.py` 需一致，否则误报 crc_bad）。
8. **测试方式坑**：uplink 每 10ms poll read 返回 0 是常态（host 不发命令）；验证必须用持续发送 + 抓 COM8 看 `RX raw`/`ARM_DISARM`，不能靠 GDB halt 快照或单次发命令（详见 `verify_uplink.py` 头注释）。

---

## 4. 验证标准（Definition of Done）

- [x] 下行 6 帧字节级 CRC 全对、seq 单调
- [x] ARM/DISARM 由地面站命令决定（RC_FORCE_ARM=false）
- [x] 参数读写闭环
- [ ] **MISSION 上传/下载**：地面站 `upload_mission` 后板子存 N 个航点；`download_mission` 回同样 N 个
- [ ] **RC_OVERRIDE**：地面站发通道覆盖后 control 实际读到（替代 sim RC）
- [ ] **FENCE 上传/下载**：围栏点一致
- [ ] （可选）FTP 固件升级经地面站跑通

---

## 5. 进度记录

- **2026-08-13**：下行 ATTITUDE/VFR_HUD 修复，6 帧字节级验证全过；groundctrl-tools 实连解析 HEARTBEAT 正常。
- **2026-08-16**：清理调试文件；`RC_FORCE_ARM=false` 提交（解锁关键修复）；上行 ARM/DISARM 持续发送实测全通；明确联调对象为 groundctrl 非 QGC；建立能力矩阵（§1.2 缺口 1-5）。
- **2026-08-16 起**：按计划 §2.1 实现 MISSION 航点双向 + RC_OVERRIDE。
  - ✅ flyctrl-core mavlink.rs：新增 MISSION_COUNT/ITEM_INT/REQUEST/ACK/REQUEST_LIST + RC_CHANNELS_OVERRIDE 的 msg_id、CRC_EXTRA、decode/encode。
  - ✅ uplink.rs：MISSION 握手状态机（upload/download）+ RC_OVERRIDE 全局 + get_rc_override() 接口；路由分支已接。
  - ✅ control.rs：RC_OVERRIDE 优先于 sim RC（PWM 1000-2000 归一化），超时 2s 回退 sim RC。
  - ⏳ 待烧录实测：groundctrl 实连验证 MISSION 上传/下载 + RC_OVERRIDE 操控。
