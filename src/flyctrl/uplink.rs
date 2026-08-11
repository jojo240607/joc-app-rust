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
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use flyctrl_core::comm::link::{Frame, MAX_FRAME_LEN, MAX_FRAME_LEN as ML_MAX};
use flyctrl_core::comm::mavlink::{self, enums, CommandLong, MAVLINK_MAGIC};
use crate::abi::RTOS_PRIO_MAIN;

// ── 全局共享状态（uplink -> control / telemetry） ──────────────────────
/// 指令解锁位：地面站经 COMMAND_LONG(ARM/DISARM) 置位；control 任务用 `rc_armed || G_CMD_ARMED`。
pub static G_CMD_ARMED: AtomicBool = AtomicBool::new(false);
/// 指令模式：地面站经 COMMAND_LONG(DO_SET_MODE) 设置；telemetry 心跳 custom_mode 反映此值。
pub static G_CMD_MODE: AtomicU16 = AtomicU16::new(0);
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

// ── 参数表（只读演示参数，固定数组，无堆） ────────────────────────────
/// 单个参数项：16 字节 NUL 结尾名 + f32 值。
struct Param {
    name: [u8; 16],
    value: f32,
}

/// 第一版参数表（演示/校准常量）；后续可扩展为可读写映射。
const PARAMS: &[Param] = &[
    Param { name: *b"Thrust\0\0\0\0\0\0\0\0\0\0", value: 0.75 },
    Param { name: *b"YawP\0\0\0\0\0\0\0\0\0\0\0\0", value: 0.40 },
    Param { name: *b"RollP\0\0\0\0\0\0\0\0\0\0\0", value: 0.30 },
    Param { name: *b"PitchP\0\0\0\0\0\0\0\0\0\0", value: 0.30 },
    Param { name: *b"MaxThrust\0\0\0\0\0\0\0", value: 1.00 },
];

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
        if n > 0 {
            // 解耦：feed 闭包借用 &mut self.parser，故路由用自由函数，
            // 仅捕获 self.dev（共享）与 self.seq（可变）的独立字段引用。
            let dev = &self.dev;
            let seq = &mut self.seq;
            self.parser.feed(&buf[..n as usize], |f: &Frame| {
                if let Some((msgid, payload)) = mavlink::decode(f) {
                    route_frame(dev, seq, msgid, payload);
                }
            });
        }
    }
}

/// 非阻塞写出一帧；host 不连/无 IN-token 时 usb write 返回 0（丢帧），不阻塞。
fn send_frame(dev: &Device, out: &[u8; ML_MAX], n: usize) -> i32 {
    dev.write(&out[..n])
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
    if idx >= PARAMS.len() { return; }
    let p = &PARAMS[idx];
    let s = next_seq(seq);
    let mut out = [0u8; ML_MAX];
    let n = mavlink::encode_param_value(
        &p.name,
        p.value,
        enums::MAV_PARAM_TYPE_REAL32,
        PARAMS.len() as u16,
        idx as u16,
        s,
        &mut out,
    );
    let _ = send_frame(dev, &out, n);
}

// ── 命令路由 ──────────────────────────────────────────────────────────
/// 处理一帧已 decode 的 MAVLink 消息（自由函数，避免与 parser 的可变借用冲突）。
fn route_frame(dev: &Device, seq: &mut u8, msgid: u8, payload: &[u8]) {
    // [BISECT] 临时空置所有路由动作：只记录收到的 msgid，不调用任何 control 接口、
    // 不发 ACK/参数，用于区分"usb0.read 轮询+解析" vs "路由动作"导致的卡死。
    match msgid {
        mavlink::msg_id::COMMAND_LONG => {
            info!(tag: "uplink", "[BISECT] COMMAND_LONG rx (route disabled)");
        }
        mavlink::msg_id::PARAM_REQUEST_LIST => {
            info!(tag: "uplink", "[BISECT] PARAM_REQUEST_LIST rx (route disabled)");
        }
        mavlink::msg_id::PARAM_SET => {
            info!(tag: "uplink", "[BISECT] PARAM_SET rx (route disabled)");
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
            // param1 = 自定义模式码（与 flightmode::FlightMode 映射）
            let mode = cmd.params[0] as u16;
            G_CMD_MODE.store(mode, Ordering::Relaxed);
            control::set_cmd_mode(mode);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "DO_SET_MODE -> mode={}", mode);
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

fn handle_param_set(dev: &Device, seq: &mut u8, ps: mavlink::ParamSet) {
    // 第一版：参数表只读，PARAM_SET 仅回显已存值（拒绝写入）。
    let name = param_name_str(&ps.id);
    // 找到同名项回显，否则拒绝
    let mut found = false;
    for (i, p) in PARAMS.iter().enumerate() {
        if p.name == ps.id {
            param_value(dev, seq, i as u16);
            found = true;
            break;
        }
    }
    if !found {
        info!(tag: "uplink", "PARAM_SET '{}' not found -> read-only", name);
    }
}

// ── uplink 任务入口 ───────────────────────────────────────────────────
// [VERIFY] 验证版：真正轮询读 usb0（调用 poll_read）。用于验证 usb.c 的
// OUT 反压自愈修复（read 末尾自动 usbd_cdc_out_reenarm）是否消除"usb0.read
// 卡死"。验证完回退到 BISECT idle 版或正式实现。
pub extern "C" fn uplink_task(_arg: *mut c_void) {
    info!(tag: "uplink", "task started; prio={}", RTOS_PRIO_MAIN);

    let usb_dev = match Device::get("usb0\0") {
        Some(d) => d,
        None => {
            info!(tag: "uplink", "usb0 not found; uplink disabled");
            return;
        }
    };
    info!(tag: "uplink", "usb0 got; poll_read ENABLED (verify usb0.read deadlock fix)");
    let mut tx = UplinkTx::new(usb_dev);
    let mut buf = [0u8; 64];
    let mut loops: u32 = 0;
    let mut last: u32 = 0;
    loop {
        tx.poll_read(&mut buf);
        loops += 1;
        // 每 500 循环打印一次存活证据，证明任务没卡死在 read。
        if loops.wrapping_sub(last) >= 500 {
            last = loops;
            info!(tag: "uplink", "alive loop={} (poll_read non-blocking)",
                  loops);
        }
        crate::rtos_sync::msleep(10);
    }
}
