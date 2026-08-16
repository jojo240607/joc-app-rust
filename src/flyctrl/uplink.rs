//! 上行接收：地面站 -> 飞控的命令通道（经 usb0 / USB CDC）。
//!
//! 设计要点（与下行 telemetry 对称、无堆、非阻塞）：
//! - `usb_dev.read` 是**非阻塞**的：RX ring 空时立即返回 0，不阻塞任务；
//!   因此 uplink 任务用轮询循环 + `msleep(10)` 让出 CPU，绝不 busy-yield。
//! - 增量解析器 `FxParser` 单字节状态机，跨多次 read 拼出完整 MAVLink v2 帧，
//!   解决 USB-CDC 分包/粘包（一次 read 可能含半帧、多帧、或错位字节）。
//! - 路由层只处理三类上行消息：COMMAND_LONG（解锁/SET_MODE/请求能力）、
//!   PARAM_REQUEST_LIST（参数表流水）、PARAM_SET（写参数）。其余消息忽略。
//! - 应答（COMMAND_ACK / PARAM_VALUE / 能力心跳）经同一个 `usb_dev` 非阻塞写出，
//!   与下行 telemetry 共享 usb0；两者 seq 各自独立计数，互不干扰。
//! - 解锁/模式等"持久影响控制律"的状态，经 `core::G_*` 原子共享给 control 任务，
//!   避免 uplink 直接调用控制律（保持任务边界清晰、零竞争）。

use crate::info;
use crate::device::rtos_device::Device;
use crate::flyctrl::control;
use crate::rtos_sync::msleep;
use core::ffi::c_void;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8, Ordering};
use flyctrl_core::comm::link::{Frame, MAX_FRAME_LEN, MAX_FRAME_LEN as ML_MAX};
use flyctrl_core::comm::mavlink::{self, enums, CommandLong, MAVLINK_MAGIC};
use crate::abi::RTOS_PRIO_MAIN;

// ── 全局共享状态（uplink -> control / telemetry） ──────────────────────
/// 指令解锁位：地面站经 COMMAND_LONG(ARM/DISARM) 置位；control 任务用 `rc_armed || G_CMD_ARMED`。
pub static G_CMD_ARMED: AtomicBool = AtomicBool::new(false);
/// 指令模式：地面站经 COMMAND_LONG(DO_SET_MODE) 或 TAKEOFF/LAND/RTL 设置；存 ArduCopter 标准 custom_mode。
/// 映射：STABILIZE=0, ALT_HOLD=2, LOITER=5, RTL=6, LAND=9, GUIDED=4。telemetry 心跳 custom_mode 反映此值。
pub static G_CMD_MODE: AtomicU16 = AtomicU16::new(0);
/// 当前油门百分比(0..100)：control 任务每周期写入，telemetry 经 VFR_HUD 下发。
pub static G_THROTTLE: AtomicU8 = AtomicU8::new(0);
/// 参数流水游标：PARAM_REQUEST_LIST 触发后，uplink 逐条发 PARAM_VALUE。
pub static G_PARAM_TX_IDX: AtomicU16 = AtomicU16::new(0);
/// 参数请求进行中标志（避免与周期性流水冲突）。
pub static G_PARAM_REQ: AtomicBool = AtomicBool::new(false);
/// 地面站请求自驾仪能力（COMMAND_LONG REQUEST_AUTOPILOT_CAPABILITIES）。
pub static G_CAP_REQ: AtomicBool = AtomicBool::new(false);

// ── 增量帧解析器（单字节状态机，无堆） ────────────────────────────────
#[derive(Clone, Copy, PartialEq)]
enum FxState {
    Idle,    // 寻找 magic
    Header,  // 收齐 9 字节头部（含 len 域）
    Payload, // 收齐 payload + 2 CRC
}

/// MAVLink v2 增量解析器：跨多次 read 拼帧，容忍分包/粘包/错位。
#[derive(Clone, Copy)]
pub struct FxParser {
    state: FxState,
    frame: Frame, // data[..len] 为已收字节
    need: usize,  // 当前状态还需收多少字节才成帧
}

impl FxParser {
    pub const fn new() -> Self {
        FxParser {
            state: FxState::Idle,
            frame: Frame { data: [0u8; MAX_FRAME_LEN], len: 0 },
            need: 0,
        }
    }

    /// 喂一个字节；返回 Some(完整帧) 表示该字节使一帧闭合。
    fn feed_byte(&mut self, b: u8) -> Option<Frame> {
        match self.state {
            FxState::Idle => {
                if b == MAVLINK_MAGIC {
                    self.frame.data[0] = b;
                    self.frame.len = 1;
                    self.state = FxState::Header;
                    self.need = 9; // 还需 9 字节头部（incompat..msgid[3]）
                }
                // 非 magic 字节直接丢弃（容忍错位/噪声）
                None
            }
            FxState::Header => {
                self.frame.data[self.frame.len] = b;
                self.frame.len += 1;
                self.need -= 1;
                if self.need == 0 {
                    // 头部齐：payload_len = data[1]
                    let plen = self.frame.data[1] as usize;
                    let total = 10 + plen + 2; // magic(1)+9头部+payload+2crc
                    if total > MAX_FRAME_LEN {
                        // 非法长度：复位
                        self.reset();
                        return None;
                    }
                    self.need = total - self.frame.len;
                    self.state = FxState::Payload;
                }
                None
            }
            FxState::Payload => {
                self.frame.data[self.frame.len] = b;
                self.frame.len += 1;
                self.need -= 1;
                if self.need == 0 {
                    let f = self.frame;
                    self.reset();
                    return Some(f);
                }
                None
            }
        }
    }

    /// 喂一段字节，对每个闭合帧调用 `on_frame`（避免返回 Vec，零堆）。
    fn feed(&mut self, buf: &[u8], mut on_frame: impl FnMut(&Frame)) {
        for &b in buf {
            if let Some(f) = self.feed_byte(b) {
                on_frame(&f);
            }
        }
    }

    fn reset(&mut self) {
        self.state = FxState::Idle;
        self.frame.len = 0;
        self.need = 0;
    }
}

// ── 参数表（可读写，固定数组，无堆） ──────────────────────────────────
/// 参数名表（const，不可变）。名字映射到 PidController 的真实增益字段。
/// 顺序必须与 `G_PARAM_VALS` / `pid_gain_apply` 的索引约定一致（见 control.rs）。
const PARAM_NAMES: &[[u8; 16]] = &[
    *b"KpXY\0\0\0\0\0\0\0\0\0\0\0\0",
    *b"KpZ\0\0\0\0\0\0\0\0\0\0\0\0\0",
    *b"KvXY\0\0\0\0\0\0\0\0\0\0\0\0",
    *b"KvZ\0\0\0\0\0\0\0\0\0\0\0\0\0",
    *b"HoverThrust\0\0\0\0\0",
];

/// 每个参数的合法取值范围（含端点），用于 PARAM_SET 写前校验。
/// 越界写入直接拒绝（回 MAV_RESULT_FAILED），绝不写入非法增益导致控制律发散。
const PARAM_MIN: [f32; 5] = [0.0, 0.0, 0.0, 0.0, 0.1];
const PARAM_MAX: [f32; 5] = [5.0, 5.0, 5.0, 5.0, 1.0];

/// 参数值表（可读写，地面站 PARAM_SET 写入；初始值与 `PidController::default_quad` 对齐）。
/// control 任务每周期原子读此表并应用到 pid 增益，使参数设置真正生效。
///
/// 注意：此表被强制进 `.rust_bss`（见 link_section），而系统区加载器只清零 APP_RAM、
/// 不拷贝 `.app_data` 初值——Rust 运行时也不会为 `.bss` 重填非零初值。因此源码里的
/// `[0.5,0.5,0.8,0.8,0.5]` 初值会被丢弃、运行期全 0。必须在 `init_param_defaults()`
/// 里显式写入（与 mod.rs 里 EST_STATE 的运行时填充同款手法）。
#[link_section = ".rust_bss"]
static mut G_PARAM_VALS: [f32; 5] = [0.5, 0.5, 0.8, 0.8, 0.5];

/// 运行时填充 G_PARAM_VALS 初始值（`.rust_bss` 初值被加载器清零，必须显式写）。
/// 由 spawn_flyctrl 在任务创建前调用一次。
pub fn init_param_defaults() {
    unsafe {
        *core::ptr::addr_of_mut!(G_PARAM_VALS) = [0.5, 0.5, 0.8, 0.8, 0.5];
    }
}

/// 参数个数。
const PARAM_COUNT: usize = 5;

/// 把 16 字节参数名转成 &str（截断到首个 NUL），用于日志。
fn param_name_str(name: &[u8; 16]) -> &str {
    let mut end = 0;
    while end < 16 && name[end] != 0 { end += 1; }
    // 安全：name 是源文件常量或协议字段，均为 ASCII。
    unsafe { core::str::from_utf8_unchecked(&name[..end]) }
}

// ── 应答发送 + 读取解析（非阻塞，与 telemetry 共享 usb0） ──────────────
struct UplinkTx {
    dev: Device,
    seq: u8,
    parser: FxParser,
}

impl UplinkTx {
    fn new(dev: Device) -> Self {
        UplinkTx { dev, seq: 0, parser: FxParser::new() }
    }

    /// 非阻塞轮询一次 usb0.read，喂入增量解析器；每闭合一帧即路由。
    fn poll_read(&mut self, buf: &mut [u8]) {
        let n = self.dev.read(buf);
        // 诊断：只在真正读到字节时打印（含 n==0 的噪声会淹没串口，去掉）。
        // 防御：驱动 read 可能返回负数错误码或异常长度；只在 0 < n <= buf.len() 时解析，
        // 否则跳过（避免切片越界），不阻塞轮询。
        if n > 0 && (n as usize) <= buf.len() {
            // 诊断：确认板子确实收到 OUT 字节（临时日志，验证后删除）
            info!(tag: "uplink", "RX raw n={} first16={:02X?}",
                  n, &buf[..core::cmp::min(n as usize, 16)]);
            // 解耦：feed 闭包借用 &mut self.parser，故路由用自由函数，
            // 仅捕获 self.dev（共享）与 self.seq（可变）的独立字段引用。
            let dev = &self.dev;
            let seq = &mut self.seq;
            self.parser.feed(&buf[..n as usize], |f: &Frame| {
                match mavlink::decode(f) {
                    Some((msgid, payload)) => {
                        info!(tag: "uplink", "frame decoded msgid={} plen={}", msgid, f.len);
                        route_frame(dev, seq, msgid, payload);
                    }
                    None => {
                        info!(tag: "uplink", "frame decode FAILED magic={:02X} len={}",
                              f.data[0], f.len);
                    }
                }
            });
        } else if n != 0 {
            info!(tag: "uplink", "poll_read n={} (buf.len={})", n, buf.len());
        }
    }
}

/// 非阻塞写出一帧；host 不连/无 IN-token 时 usb write 返回 0（丢帧），不阻塞。
fn send_frame(dev: &Device, out: &[u8; ML_MAX], n: usize) -> i32 {
    let r = dev.write(&out[..n]);
    if r <= 0 {
        info!(tag: "uplink", "send_frame FAILED n={} ret={}", n, r);
    }
    r
}

fn next_seq(seq: &mut u8) -> u8 {
    let s = *seq;
    *seq = seq.wrapping_add(1);
    s
}

fn ack(dev: &Device, seq: &mut u8, command: u16, result: u8) {
    let s = next_seq(seq);
    let mut out = [0u8; ML_MAX];
    let n = mavlink::encode_command_ack(command, result, 0, 0, s, &mut out);
    let _ = send_frame(dev, &out, n);
}

fn param_value(dev: &Device, seq: &mut u8, idx: u16) {
    let idx = idx as usize;
    if idx >= PARAM_COUNT { return; }
    let name = PARAM_NAMES[idx];
    let val = unsafe { G_PARAM_VALS[idx] };
    let s = next_seq(seq);
    let mut out = [0u8; ML_MAX];
    let n = mavlink::encode_param_value(
        &name,
        val,
        enums::MAV_PARAM_TYPE_REAL32,
        PARAM_COUNT as u16,
        idx as u16,
        s,
        &mut out,
    );
    let _ = send_frame(dev, &out, n);
}

// ── 命令路由 ──────────────────────────────────────────────────────────
/// 处理一帧已 decode 的 MAVLink 消息（自由函数，避免与 parser 的可变借用冲突）。
fn route_frame(dev: &Device, seq: &mut u8, msgid: u32, payload: &[u8]) {
    match msgid {
        mavlink::msg_id::COMMAND_LONG => {
            if let Some(cmd) = mavlink::decode_command_long(payload) {
                handle_command_long(dev, seq, cmd);
            }
        }
        mavlink::msg_id::PARAM_REQUEST_LIST => {
            // 触发参数流水（下次循环逐条发）
            G_PARAM_TX_IDX.store(0, Ordering::Relaxed);
            G_PARAM_REQ.store(true, Ordering::Relaxed);
            info!(tag: "uplink", "PARAM_REQUEST_LIST rx; start streaming {} params", PARAM_COUNT);
        }
        mavlink::msg_id::PARAM_SET => {
            if let Some(ps) = mavlink::decode_param_set(payload) {
                handle_param_set(dev, seq, ps);
            }
        }
        mavlink::msg_id::PARAM_REQUEST_READ => {
            // 按参数名点读单个参数（地面站参数表单点刷新）。
            if let Some((id, _idx)) = mavlink::decode_param_request_read(payload) {
                let name = param_name_str(&id);
                if let Some(i) = find_param(&id) {
                    param_value(dev, seq, i as u16);
                    info!(tag: "uplink", "PARAM_REQUEST_READ '{}' -> idx={}", name, i);
                } else {
                    info!(tag: "uplink", "PARAM_REQUEST_READ '{}' not found", name);
                }
            }
        }
        _ => {
            // 其余消息（HEARTBEAT/ATTITUDE 等上行）第一版忽略
        }
    }
}

fn handle_command_long(dev: &Device, seq: &mut u8, cmd: CommandLong) {
    // 仅响应广播(0)或本机(sys=1)目标
    if cmd.target_system != 0 && cmd.target_system != mavlink::SYS_ID {
        return;
    }
    match cmd.command {
        enums::MAV_CMD_COMPONENT_ARM_DISARM => {
            let arm = cmd.params[0] > 0.5;
            G_CMD_ARMED.store(arm, Ordering::Relaxed);
            control::set_cmd_armed(arm);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "ARM_DISARM cmd={} -> armed={}", cmd.command, arm);
        }
        enums::MAV_CMD_DO_SET_MODE => {
            // 标准 MAVLink：param1 = base_mode（含 CUSTOM_MODE_ENABLED 位 0x80），param2 = custom_mode。
            let custom = cmd.params[1] as u16; // ArduCopter 自定义模式码（0/2/5/6/9...）
            G_CMD_MODE.store(custom, Ordering::Relaxed);
            control::set_cmd_mode(custom);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "DO_SET_MODE -> custom_mode={}", custom);
        }
        // 起飞/降落/返航：直接切到对应 ArduCopter 自定义模式，地面站据此显示模式名。
        enums::MAV_CMD_NAV_TAKEOFF => {
            // TAKEOFF 在悬停类模式中按 ALT_HOLD（带目标高度）处理。
            let mode = enums::COPTER_MODE_ALT_HOLD;
            G_CMD_MODE.store(mode, Ordering::Relaxed);
            control::set_cmd_mode(mode);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "TAKEOFF -> mode={}", mode);
        }
        enums::MAV_CMD_NAV_LAND => {
            let mode = enums::COPTER_MODE_LAND;
            G_CMD_MODE.store(mode, Ordering::Relaxed);
            control::set_cmd_mode(mode);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "LAND -> mode={}", mode);
        }
        enums::MAV_CMD_NAV_RETURN_TO_LAUNCH => {
            let mode = enums::COPTER_MODE_RTL;
            G_CMD_MODE.store(mode, Ordering::Relaxed);
            control::set_cmd_mode(mode);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "RTL -> mode={}", mode);
        }
        enums::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES => {
            G_CAP_REQ.store(true, Ordering::Relaxed);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "REQUEST_AUTOPILOT_CAPABILITIES rx");
        }
        _ => {
            // 第一版未实现的指令：明确拒绝（便于地面站诊断）
            ack(dev, seq, cmd.command, enums::MAV_RESULT_UNSUPPORTED);
            info!(tag: "uplink", "CMD {} unsupported -> ACK_UNSUPPORTED", cmd.command);
        }
    }
}

/// 按 16 字节名查找参数索引；找不到返回 None。
fn find_param(id: &[u8; 16]) -> Option<usize> {
    for i in 0..PARAM_COUNT {
        if PARAM_NAMES[i] == *id {
            return Some(i);
        }
    }
    None
}

fn handle_param_set(dev: &Device, seq: &mut u8, ps: mavlink::ParamSet) {
    // 参数表可读写：找到同名项 -> 范围校验 -> 写入并回显（带 MAV_RESULT）。
    let name = param_name_str(&ps.id);
    match find_param(&ps.id) {
        Some(i) => {
            // 写前范围校验：越界直接拒绝，绝不写入非法增益。
            let v = ps.value;
            if v < PARAM_MIN[i] || v > PARAM_MAX[i] {
                // 越界：回 COMMAND_ACK(FAILED)，command 填 PARAM_SET(msg_id=23) 作约定。
                ack(dev, seq, mavlink::msg_id::PARAM_SET as u16, enums::MAV_RESULT_FAILED);
                info!(tag: "uplink", "PARAM_SET '{}' = {:.4} OUT OF RANGE [{:.2},{:.2}] -> REJECT",
                      name, v, PARAM_MIN[i], PARAM_MAX[i]);
                return;
            }
            unsafe { G_PARAM_VALS[i] = v; }
            // 标准做法：仅回显新值（PARAM_VALUE），地面站据此确认写入成功。
            // 越界分支才回 COMMAND_ACK(FAILED)，合法分支靠回显帧确认。
            param_value(dev, seq, i as u16);
            info!(tag: "uplink", "PARAM_SET '{}' = {:.4} (written, applied next ctrl tick)", name, v);
        }
        None => {
            info!(tag: "uplink", "PARAM_SET '{}' not found -> ignored", name);
        }
    }
}

// ── 参数 -> 控制律桥接（control 任务每周期调用，零锁、原子读） ──────────
/// 把地面站参数表 `G_PARAM_VALS` 应用到给定 PID 控制器。
/// control 任务每周期调用一次，使 PARAM_SET 在下一控制拍立即生效。
/// 读取用 Relaxed 原子序（uplink 单写、control 单读，无需 Acquire/Release 同步语义）。
pub fn sync_gains_to_pid(pid: &mut flyctrl_core::controller::PidController) {
    let mut g = [0f32; 5];
    for i in 0..PARAM_COUNT {
        g[i] = unsafe { G_PARAM_VALS[i] };
    }
    pid.apply_gains(&g);
}

// ── uplink 任务入口 ───────────────────────────────────────────────────
pub extern "C" fn uplink_task(_arg: *mut c_void) {
    info!(tag: "uplink", "task started; poll usb0.read, prio={}", RTOS_PRIO_MAIN);

    // 复用 boot 已打开的 usb0（见 telemetry 的 Device::get 约定，避免二次 open 重置 USB 状态机）。
    let usb_dev = match Device::get("usb0\0") {
        Some(d) => d,
        None => {
            info!(tag: "uplink", "usb0 not found; uplink disabled");
            return;
        }
    };

    let mut tx = UplinkTx::new(usb_dev);
    let mut rx_buf = [0u8; 64];

    let mut loops: u32 = 0;
    loop {
        // 非阻塞轮询 usb0.read + 增量解析 + 路由（RX ring 空时返回 0，不阻塞）。
        tx.poll_read(&mut rx_buf);

        // 参数流水：每次循环最多发一条，避免单次 burst 占满 USB 下行缓冲。
        if G_PARAM_REQ.load(Ordering::Relaxed) {
            let idx = G_PARAM_TX_IDX.load(Ordering::Relaxed) as usize;
            if idx < PARAM_COUNT {
                param_value(&tx.dev, &mut tx.seq, idx as u16);
                G_PARAM_TX_IDX.store((idx + 1) as u16, Ordering::Relaxed);
            } else {
                // 流水完成
                G_PARAM_REQ.store(false, Ordering::Relaxed);
                info!(tag: "uplink", "param stream done ({} items)", PARAM_COUNT);
            }
        }

        // 能力请求：真发 AUTOPILOT_VERSION 帧（响应 REQUEST_AUTOPILOT_CAPABILITIES）。
        if G_CAP_REQ.load(Ordering::Relaxed) {
            G_CAP_REQ.store(false, Ordering::Relaxed);
            let caps = enums::MAV_PROTOCOL_CAPABILITY_MAVLINK2
                | enums::MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
            let s = next_seq(&mut tx.seq);
            let mut out = [0u8; ML_MAX];
            let n = mavlink::encode_autopilot_version(caps, s, &mut out);
            let _ = send_frame(&tx.dev, &out, n);
            info!(tag: "uplink", "AUTOPILOT_VERSION sent (cap=MAVLINK2|PARAM_FLOAT)");
        }

        // 低频存活日志（约每 10s 一次），用于联调确认 uplink 任务未卡死。
        loops += 1;
        if loops % 1000 == 0 {
            info!(tag: "uplink", "poll alive loop={}", loops);
        }

        // 让出 CPU 10ms（与下行 20ms 错开），保持非阻塞轮询。
        msleep(10);
    }
}
