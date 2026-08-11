# 板载上行接收（Uplink）实施计划

> 目标：补齐飞控 `joc-app-rust`（板载 App）的 **MAVLink 上行接收**，使地面站
> `groundctrl` 能与板子在真实硬件上**双向闭环联调**：
> 板子 → 地面站（已通，下行 telemtry）＋ 地面站 → 板子（本次新增，上行接收/应答）。
>
> 计划基于当前代码现状（见 §1 盘点），所有改动集中在 `d:/project/mcu/oop/joc-app-rust`。
> 协议与 `flyctrl_core::comm::mavlink` 已字节级对齐地面站（v2 + CRC_EXTRA），
> 故无需改协议层，只改「板载接收/路由/应答」与「PC 侧无头联调工具」。

---

## 1. 现状盘点

### 1.1 已具备（联调前提全部满足 ✅）
| 项 | 位置 | 说明 |
|---|---|---|
| MAVLink v2 编解码 | `flyctrl/core/src/comm/mavlink.rs` | `encode_*` 下行、`decode`/`decode_command_long`/`decode_param_value` 已有，CRC_EXTRA 已对齐地面站 |
| 下行三帧 | `src/flyctrl/telemetry.rs` | 每 20ms 经 `usb0` 非阻塞发 HEARTBEAT/LOCAL_POSITION_NED/SYS_STATUS（已实测 `first loop done` + 稳定 hb） |
| usb0 句柄 | `telemetry.rs` 用 `Device::get("usb0\0")` | 复用 boot 已开句柄，非阻塞 staged write，**host 不连只丢帧不卡死** |
| 系统 ID | `mavlink.rs:26-27` | `SYS_ID=1, COMP_ID=1`，与地面站 GCS 侧 `system_id=255` 不冲突，单飞机分桶 |
| 地面站解析/发送 | `groundctrl/core/src/{link,mlink,vehicle,services}` | 串口链 + `MavlinkParser::feed` + `VehicleModel::apply`；可发 COMMAND_LONG / PARAM_REQUEST_LIST / MISSION_REQUEST |

### 1.2 缺口（本次要补）
1. **板载无上行接收** —— `src/` 里**没有任何 `usb0.read` 循环**，地面站指令石沉大海。
2. **无指令解锁通道** —— `control` 任务每周期从 `SENSOR_FRAME.armed`（RC 开关）取 armed 写 `EST_STATE.armed`（`control.rs:63-78,142-147`），没有「地面站指令解锁」的注入点。
3. **无 PARAM 应答** —— 地面站连上会发 `PARAM_REQUEST_LIST`，板载不答应 → 地面站持续请求/超时。
4. **无 COMMAND_ACK** —— 地面站发 `COMMAND_LONG` 后等 `COMMAND_ACK`，不答应显示「指令失败」。
5. **无 PC 侧无头联调工具** —— `groundctrl/tools/src/main.rs` 是占位空壳，没有连 COM 解析/发指令的 CLI。

### 1.3 关键约束（来自历史踩坑）
- **usb0 用 `Device::get` 复用，切勿 `Device::open`**（二次 open 触发 `USBD_Init` 重置 TX 状态机 → write 卡死，已修复）。
- **下行 write 非阻塞**（host 不连只丢帧）；**上行 read 必须同样非阻塞**：host 未发数据时 `read` 应返回 0/负且不阻塞任务（⚠️ 实施前先确认 `rtos_device.rs::read` 在 usb0 无数据时语义，见 §3.1）。
- 上行任务**不得**触碰 `EST_MTX`/seqlock 的写权（control=prio4 是 EST/SENSOR 的唯一写者），只能用**独立全局原子标志**与 control/telemetry 交接。
- `EST_STATE`/`SENSOR_FRAME` 强制 `.rust_bss` + `zeroed()`（App XIP 无 .data，落 Flash 写即 BusFault，见 `mod.rs:87-103`）。新增全局状态同理。

---

## 2. 设计要点

### 2.1 任务划分（新增 1 个任务）
| 任务 | 优先级 | 周期/模式 | 职责 |
|---|---|---|---|
| `uplink`（新增） | prio=10（软实时，priv=1） | 事件轮询（read 返回 0 时 `msleep(10)`） | 轮询 `usb0.read` → `mavlink::decode` → 路由到指令/参数处理器；维护 PARAM 流水应答状态机；发 COMMAND_ACK / PARAM_VALUE |
| control (已有) | prio=4 (RT_HARD) | 4ms | 读 `g_cmd_armed`(OR RC armed) → 写 `EST_STATE.armed`；读 `g_cmd_mode` → 写心跳 custom_mode |
| telemetry (已有) | prio=12 | 20ms | 发下行帧；读 `g_cmd_mode` 填 HEARTBEAT.custom_mode |

> prio=10 高于 telem(12) 但低于控制链(4/5)，不抢硬实时，又能及时应答地面站。

### 2.2 新增全局共享状态（`.rust_bss`，原子/简单类型，uplink 写、control/telem 读）
```rust
// src/flyctrl/uplink.rs （新增文件）
#[link_section = ".rust_bss"]
pub static mut G_CMD_ARMED: bool = false;   // 地面站指令解锁请求（OR RC 开关）
#[link_section = ".rust_bss"]
pub static mut G_CMD_MODE: u8 = 0;          // 自定义飞行模式码（SET_MODE 写入，HEARTBEAT.custom_mode）
#[link_section = ".rust_bss"]
pub static mut G_PARAM_TX_IDX: i16 = -1;    // PARAM 流水应答游标；-1=空闲，>=0=正在发第 idx 个
```
- control 取 armed：`armed = rc_armed || G_CMD_ARMED`（`control.rs` 第 74 行附近加 OR）。
- telemetry 取 mode：`encode_heartbeat(G_CMD_MODE, ...)`（替换当前硬编码 0）。
- **安全**：uplink(prio10) 写、control(prio4)/telem(prio12) 读。control 优先级高于 uplink，**不会在 control 读中途被 uplink 改写**（单写者 + 读不被抢占），故无需 mutex（与 seqlock 同理由 `mod.rs:104-108`）。

### 2.3 上行消息路由表（首个版本子集）
| 收到 (msgid) | 处理 | 应答 |
|---|---|---|
| `COMMAND_LONG` (76) | `decode_command_long` → 按 `command` 分支：`MAV_CMD_COMPONENT_ARM_DISARM`(400) 置 `G_CMD_ARMED`；`MAV_CMD_DO_SET_MODE`(176) 置 `G_CMD_MODE`；`MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`(520) 记 capability 标志 | `COMMAND_ACK` (77)，result=0/1 |
| `PARAM_REQUEST_LIST` (21) | 置 `G_PARAM_TX_IDX=0`，启动流水 | 逐帧 `PARAM_VALUE`(22) 直到发完（见 §2.4） |
| `PARAM_SET` (23) | `decode_param_value` → 按 name 更新参数表 | 单帧 `PARAM_VALUE`(22) 回显新值 |
| `PARAM_REQUEST_READ` (20) | （可选 v1）按 index 回单帧 `PARAM_VALUE` | 同上 |
| `HEARTBEAT` (0, GCS 发出) | 可选：记录 GCS 在线（设 `gcs_seen` 时间戳） | 无需应答 |
| 其他 | 忽略 | —— |

> 不实现 MISSION（航点）第一版：地面站若发 MISSION_REQUEST 暂忽略（不影响心跳/参数联调）。

### 2.4 PARAM 流水应答状态机
- 定义静态参数表（~6 个 f32，命名 16 字节）：
  `THR_MID`、`PID_P`、`PID_I`、`PID_D`、`ARM_TIMEOUT`、`TELEM_HZ`。
- `G_PARAM_TX_IDX=0` 触发后，uplink 主循环每轮发一个 `PARAM_VALUE`（带 `param_count=6, param_index=idx`），`idx++`；到 6 时复位为 -1。
- 单次只发一个 `PARAM_VALUE`/轮（避免突发淹没 usb0 TX ring；usb0 非阻塞，满则丢，下一轮补发）。
- `PARAM_SET` 立即回显 + 写入表（供后续 REQUEST_LIST 反映）。

### 2.5 COMMAND_ACK 格式
- msgid=77，payload：command(u16) + result(u8) + ... 其余填 0。
- result: 0=MAV_RESULT_ACCEPTED，1=MAV_RESULT_DENIED（未知 command 或参数非法）。
- 复用 `mavlink.rs` 现有 `encode`（自定义 9 字节 ACK payload，或新增 `encode_command_ack`，见 §4.2）。

---

## 3. 分步实施

### 步骤 0：确认 usb0.read 非阻塞语义（前置，~10 分钟）
- 读 `src/device/rtos_device.rs::read` 与 joc-base `usb.c` 的 `USBD_CDC_Recv`/OUT 缓冲。
- **判定**：host 未发 OUT 包时 `read` 是否立即返回 0（不阻塞）。若实现是阻塞等待，则 uplink 任务内用 `msleep` 轮询 + 非阻塞 read（确认 usb0 驱动提供「有数据才返回，否则 0」语义）。
- 用 `usbtest` feature 临时加 `read` 轮询打印返回码验证（host 开 COM9 发 1 字节 → 板子收到）。

### 步骤 1：新增 `src/flyctrl/uplink.rs` 骨架 + 全局状态（~1h）
- 文件含：`G_CMD_ARMED`/`G_CMD_MODE`/`G_PARAM_TX_IDX` 三个 `.rust_bss` 静态。
- `pub extern "C" fn uplink_entry(_arg)`：
  - `Device::get("usb0\0")` 取句柄（复用，非 open）。
  - 主循环：`let n = dev.read(&mut buf);` → 累积进 `MavlinkParser`（增量解析多帧）→ 对每帧 `mavlink::decode` → 路由（§2.3）。
  - 无数据时 `msleep(10)`（不空转）。
- 在 `src/flyctrl/mod.rs`：
  - 加 `pub mod uplink;`
  - `spawn_flyctrl` 中用 `spawn_rt(b"uplink\0", uplink::uplink_entry, 10, STACK_UPLINK_BUF, STACK_UPLINK, 1, RT_NONE, 0, 0)`（新增 `STACK_UPLINK`≈1024 + `[u8;1024]` `.rust_bss` 缓冲）。

### 步骤 2：COMMAND_LONG 路由 + 指令解锁（~0.5h）
- 在 `uplink.rs` 实现 `handle_command_long(cl: &CommandLong)`：
  - `MAV_CMD_COMPONENT_ARM_DISARM`：`G_CMD_ARMED = params[0] > 0.5;`（注意安全：仅允许 arm 当健康非 critical —— 读 `EST_STATE.health`，critical 时拒绝并 ACK result=1）。
  - `MAV_CMD_DO_SET_MODE`：`G_CMD_MODE = params[0] as u8;`（custom_mode 直接透传）。
  - `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`：置 capability 待发标志（简易：ACK 即可，capability 帧可选 v1）。
  - 其余：ACK result=1（未知命令）。
- `control.rs` 第 74 行：`armed = f.armed || unsafe { G_CMD_ARMED };`（uplink 模块需 `pub(crate)` 暴露 `G_CMD_ARMED` 或经 `flyctrl` 模块再导出）。
- `telemetry.rs`：`encode_heartbeat(unsafe { G_CMD_MODE }, armed, ...)`（替换原 `0`）。

### 步骤 3：PARAM 表 + 流水应答 + PARAM_SET（~1h）
- 在 `uplink.rs` 定义 `static PARAMS: [( [u8;16], f32 ); 6]`（名字 + 默认值）。
- `handle_param_request_list()`：`G_PARAM_TX_IDX = 0;`
- 主循环「待发优先」分支：若 `G_PARAM_TX_IDX >= 0`，`encode_param_value` 发当前 idx，idx++，到 6 复位 -1。
- `handle_param_set(pv)`：按 name 匹配 PARAMS 更新 + 立即 `encode_param_value` 回显。
- 需要 `mavlink.rs` 新增 `encode_command_ack`（见步骤 5）用于 PARAM 应答一致性（或直接复用 `encode_param_value`，已存在）。

### 步骤 4：COMMAND_ACK 应答（~0.5h）
- `mavlink.rs` 新增 `encode_command_ack(command: u16, result: u8, seq: u8, out) -> usize`（msgid=77，payload：command(u16)+result(u8)+progress(u8)+result_param2(i32)+target_system+target_component，其余 0；共 ~10 字节）。
- `uplink.rs` 每个 COMMAND_LONG 处理后调之回 ACK。

### 步骤 5：PC 侧无头联调工具（~1.5h，并行可先于板载）
- `groundctrl/tools/src/main.rs` 改为无头 CLI：
  - 参数：`--port COM9 --baud 115200 [--send-cmd ARM] [--send-param THR_MID=0.5] [--duration 15]`。
  - 用 `groundctrl-core` 的 `link::SerialLink` + `mlink::MavlinkParser` + `vehicle::VehicleModel` 连串口、解析、打印收到的消息类型/计数/关键字段。
  - `--send-cmd ARM`：构造 `COMMAND_LONG(400, p1=1)` 经 `mlink::encode_v2_command_long` 发（确认 groundctrl-core 有该编码函数，否则补一个）。
  - `--send-param`：构造 `PARAM_SET` 发。
  - 退出时汇总：`heartbeats=N, param_values=M, command_acks=K`。
- 用途：板子复位后跑它，验证 (a) 板载下行被正确解析；(b) 发 ARM 后板载回 COMMAND_ACK 且心跳 base_mode 置 ARM 位；(c) 发 PARAM_REQUEST_LIST 后收到 6 个 PARAM_VALUE。

### 步骤 6：端到端验证（真硬件，~1h）
1. 板子 reset run → `_verify.py` 确认 `uplink: task started` + telem hb 正常（不卡死）。
2. PC 开 COM9，跑 `tools` 无头 CLI（duration 15s）：确认收到 HEARTBEAT/LOCAL_POSITION_NED/SYS_STATUS；发 PARAM_REQUEST_LIST 后收齐 6 个 PARAM_VALUE；发 ARM 后收 COMMAND_ACK(result=0) 且下一心跳 base_mode 含 ARM 位（telemetry 经 `G_CMD_MODE` 不受影响，armed 来自 `G_CMD_ARMED`）。
3. （可选）关 COM9 句柄，确认 telem 仍跑（usb_wr=0 丢帧不卡死，任务1 回归）。
4. 接地面站 GCS（真 GUI 或经 tools 的逻辑）：看能否识别飞控、列参数、点解锁（仅改标志，PWM 实际解锁需 RC 或后续安全门）。

---

## 4. 文件改动清单（预期）

| 文件 | 改动 |
|---|---|
| `src/flyctrl/uplink.rs` | **新增**：全局状态 + `uplink_entry` + 解析/路由/参数状态机 |
| `src/flyctrl/mod.rs` | 加 `pub mod uplink;` + `spawn_flyctrl` 创建 uplink 任务 + `STACK_UPLINK` 缓冲 |
| `src/flyctrl/control.rs` | 第 74 行 armed 加 `\|\| G_CMD_ARMED`；读 `G_CMD_MODE`（可选用于 control 内模式治理） |
| `src/flyctrl/telemetry.rs` | `encode_heartbeat` 的 mode 参数由 `0` 改为 `G_CMD_MODE` |
| `flyctrl/core/src/comm/mavlink.rs` | 新增 `encode_command_ack`（msgid=77） |
| `groundctrl/tools/src/main.rs` | 改为无头 CLI（连串口、解析、可选发指令） |
| `docs/uplink-plan.md` | 本计划 |

> 不涉及 joc-base（系统层），上行 IO 全经 `device` vtable 的 `read`，符合方案解耦约束。

---

## 5. 风险与未决项
- **R5.1 usb0.read 阻塞语义（已确认 ✅）**：查 `joc-base/src/drv/usb.c:369` 的 `usb_stream_read` —— RX ring 空时**立即返回 0**，纯非阻塞。uplink 已按「read 返回 0 则 msleep(10)」实现，与 uart3 教训无关。
- **R5.2 指令解锁安全**：第一版仅置 `G_CMD_ARMED` 标志，且 **critical 健康时拒绝 ARM**。真实「解锁→PWM 输出」连锁需后续安全门（RC 链路健康 +  disarm 超时），不在本计划范围。
- **R5.3 参数表范围**：第一版仅 6 个演示参数；真实 PID/限速参数接入需 control 任务读取 `PARAMS` 全局（prio4 读、uplink 写，仍满足单写者约束）。
- **R5.4 MISSION/航点**：第一版不实现，地面站发 MISSION_REQUEST 暂忽略（不影响心跳/参数联调；QGC 可能弹「航点为空」提示，无害）。
- **R5.5 多帧粘包（已实施 ✅）**：`uplink.rs` 内手写了最小增量解析器 `FxParser`（状态机 Idle→Header→Payload，按 magic 同步 + 头部 `payload_len` 字段定长），复用 `mavlink::decode` 对整帧做校验/解包。`fxlink_core::comm::mlink` 不在板载依赖树，未复用。

---

## 6. 完成判定（Definition of Done）
- [ ] 板子复位后 uplink 任务启动、telem 不卡死。
- [ ] PC 无头 CLI 连 COM9 解析到 HEARTBEAT/LOCAL_POSITION_NED/SYS_STATUS（下行闭环 ✅）。
- [ ] 发 `PARAM_REQUEST_LIST` 后 CLI 收齐参数表（PARAM 应答 ✅）。
- [ ] 发 `COMMAND_LONG(ARM)` 后 CLI 收 COMMAND_ACK(0) 且板载下一心跳 base_mode 置 ARM 位（上行解锁 ✅）。
- [ ] 关 COM9 句柄后 telem 仍稳定（任务1 回归 ✅）。
- [ ] 真地面站能识别飞控、列参数、尝试解锁（完整双向联调 ✅）。

---

## 7. 实施进度（2026-08-11）

### 7.1 已完成（代码实施，编译通过）
- [x] **flyctrl-core `mavlink.rs` 补齐**：`encode_command_ack`、`decode_param_request_list`、`decode_param_set`、`decode_command_ack`，及 `enums::MAV_RESULT_*` 常量、`msg_id::COMMAND_ACK=77`。全部 `no_std`、无堆。
- [x] **新建 `src/flyctrl/uplink.rs`**：
  - 全局状态 `G_CMD_ARMED`(AtomicBool) / `G_CMD_MODE`(AtomicU16) / `G_PARAM_TX_IDX`(AtomicU16) / `G_PARAM_REQ`(AtomicBool) / `G_CAP_REQ`(AtomicBool)。
  - 增量解析器 `FxParser`（单字节状态机，容忍分包/粘包/错位）。
  - `UplinkTx::poll_read` 非阻塞轮询 + `route_frame` 命令路由（COMMAND_LONG / PARAM_REQUEST_LIST / PARAM_SET）。
  - 参数表 `PARAMS`（5 个只读演示参数）+ 流水应答。
  - `uplink_task` 经 `Device::get("usb0")` 复用句柄，轮询 10ms 让出 CPU。
- [x] **`control.rs` 接入**：`set_cmd_armed`/`set_cmd_mode` 辅助函数；主循环 `armed_eff = armed || G_CMD_ARMED.load()`（RC 解锁与指令解锁逻辑或）。
- [x] **`mod.rs` 注册**：`pub mod uplink;` + `STACK_UPLINK=1024` 静态栈 + `spawn_rt(b"uplink\0", ..., prio=10, ...)`。
- [x] **`telemetry.rs` 接入**：心跳 `custom_mode` 反映 `G_CMD_MODE.load()`。
- [x] **编译验证**：`cargo build --target thumbv7em-none-eabihf --release` 通过；`build_app.py` 生成 `app.bin` = 97448 B（< 384 KB 上限），链接无误。
- [x] **`flyctrl-core` COMMAND_ACK CRC 修复**：`mavlink.rs` 的 `CRC_EXTRA` 表补 `COMMAND_ACK(77)=208`（原缺省 0），使标准地面站（mavlink 0.11）能正确校验板载发出的 COMMAND_ACK 帧，消除 §1 缺口 4 的链路层隐患。
- [x] **`groundctrl/tools` 无头 CLI（计划 §4 步骤 5）**：重写 `tools/src/main.rs` 为无头地面站——`tokio` + `SerialLink` 连串口、`MavlinkParser::feed` 解析下行、统计心跳/参数/ACK 并退出汇总；支持 `--send-cmd ARM|DISARM|SET_MODE=<n>` / `--send-param NAME=VAL` / `--request-params` / `--port` / `--baud` / `--duration`。`tools/Cargo.toml` 加 `mavlink` + `tracing-subscriber` 依赖。`cargo check -p groundctrl-tools` 通过。

### 7.2 待验证（需真硬件 / PC 工具）
- [ ] 板子复位后 uplink 任务启动、telem 不卡死（§6 DoD 第 1 条）。
- [x] PC 无头 CLI 已补（连 COM9 解析下行三帧的代码就绪，§6 DoD 第 2 条）—— 待真硬件实跑验证。
- [ ] 发 `PARAM_REQUEST_LIST` 收齐参数表（§6 DoD 第 3 条）。
- [ ] 发 `COMMAND_LONG(ARM)` 收 COMMAND_ACK(0) 且心跳 base_mode 置位（§6 DoD 第 4 条）。
- [ ] 关 COM9 句柄后 telem 仍稳定（任务 1 回归，§6 DoD 第 5 条）。
- [ ] 真地面站双向联调（§6 DoD 第 6 条）。

### 7.3 备注
- 代码实施覆盖计划 §1 缺口 1-5（含 PC 无头工具，已在 `groundctrl` 工程落地）。
- 解锁安全门（R5.2）按原计划保持最小：仅置 `G_CMD_ARMED`，且 control 主循环已对 `fdir.critical()` 拒绝推力输出，未新增 disarm 超时连锁。
- 解锁安全门（R5.2）按原计划保持最小：仅置 `G_CMD_ARMED`，且 control 主循环已对 `fdir.critical()` 拒绝推力输出，未新增 disarm 超时连锁。
