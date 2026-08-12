//! joc-app-rust 应用层日志系统（区别于 C 侧 `I/main:` 系统日志）。
//!
//! 设计：**生产者-消费者解耦**，彻底消除"任务在日志路径上阻塞"的根因。
//!   - 生产者：任何任务调 `info!/warn!/error!` → `emit` 只把格式化后的日志写入
//!     无锁环形缓冲 `LOG_RING`，**绝不直接写串口**（串口 uart0 是阻塞式 DMA 发送，
//!     曾导致 telem 在 `tx_idle` 信号量上死锁）。写 ring 用 `sem_trywait` 非阻塞互斥，
//!     拿不到锁就直接丢弃本次日志（日志可丢，绝不阻塞任务）。
//!   - 消费者：独立低优先级任务 `log_task`（prio 28），每 5ms 唤醒一次，从 `LOG_RING`
//!     批量读出积压日志，经 `dev_write(uart0)` 一次输出。即使它阻塞在 uart0 DMA TX，
//!     也只影响它自己（最低优先之一），**不会拖死任何业务任务**。
//!
//! 分级"关键不丢"：ring 满时，只允许覆盖 Info 级旧条目；Warn/Error 条目**不被 Info 覆盖**，
//! 保证已存入的关键日志完整（新的 Warn/Error 在满时丢弃本次，但存量关键日志保留）。
//!
//! 输出通道：复用 RTOS 调试控制台 `uart0`（USART1 / COM8），前缀统一 `R/...`。
//! App 仅 `get` 查找 + `write`，**绝不 open/close**：uart0 已由 C 侧 `g_console` 打开，
//! 若 App close 会 deinit 共享 UART 导致 C 侧 printf 在 TXE busy-wait 死循环冻结系统。

use core::ffi::{c_char, c_void};
use core::sync::atomic::{AtomicU32, Ordering};

use crate::abi::*;
use crate::device::Device;
use crate::rtos_sync::{msleep, spawn_rt, RT_NONE};

/// 日志级别（单字母输出）。
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Debug,
    Info,
    Warn,
    Error,
}

impl Level {
    const fn letter(self) -> u8 {
        match self {
            Level::Debug => b'D',
            Level::Info => b'I',
            Level::Warn => b'W',
            Level::Error => b'E',
        }
    }
    /// 该级别是否"关键"（关键条目不被 Info 覆盖）。
    const fn critical(self) -> bool {
        matches!(self, Level::Warn | Level::Error)
    }
}

/* ===========================================================================
 * 无锁环形缓冲（单生产者写 / 单消费者读，用原子头尾索引 + 非阻塞互斥）
 * =========================================================================== */

const LOG_RING_SIZE: usize = 2048; // 静态 ring，放 App RAM

#[link_section = ".rust_bss"]
static mut LOG_RING: [u8; LOG_RING_SIZE] = [0u8; LOG_RING_SIZE];
#[link_section = ".rust_bss"]
static LOG_HEAD: AtomicU32 = AtomicU32::new(0); // 消费者读位置
#[link_section = ".rust_bss"]
static LOG_TAIL: AtomicU32 = AtomicU32::new(0); // 生产者写位置

/// 日志写互斥（非阻塞 trywait；拿不到就丢本次日志）。
#[link_section = ".rust_bss"]
static LOG_MTX: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

fn try_lock() -> bool {
    LOG_MTX
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_ok()
}

fn unlock() {
    LOG_MTX.store(false, Ordering::Release);
}

/// 在 ring 中写入一条 `[len][level][payload]` 日志。head/tail 均为字节偏移。
/// 满时策略：只允许覆盖 Info 级旧条目（跳过 Warn/Error），Warn/Error 不覆盖。
/// 返回是否写入成功（成功 = 已入 ring，待 log_task 输出）。
fn ring_push(level: Level, payload: &[u8]) -> bool {
    if payload.is_empty() || payload.len() > 255 {
        return false;
    }
    let ring: &mut [u8; LOG_RING_SIZE] = unsafe { &mut LOG_RING };
    let head = LOG_HEAD.load(Ordering::Relaxed) as usize;
    let tail = LOG_TAIL.load(Ordering::Relaxed) as usize;
    // 条目长度 = 1(len) + 1(level) + payload
    let need = payload.len() + 2;
    if need >= LOG_RING_SIZE {
        return false;
    }
    // 空余空间
    let free = if tail >= head { LOG_RING_SIZE - (tail - head) } else { head - tail };
    if free >= need {
        // 有空间，直接写
        let n = payload.len();
        ring[tail] = (n + 1) as u8; // len（不含 len 字节，含 level）
        ring[(tail + 1) % LOG_RING_SIZE] = level.letter();
        for (i, &b) in payload.iter().enumerate() {
            ring[(tail + 2 + i) % LOG_RING_SIZE] = b;
        }
        let nt = (tail + need) % LOG_RING_SIZE;
        LOG_TAIL.store(nt as u32, Ordering::Release);
        return true;
    }
    // ring 满。若写入关键级别(Warn/Error)：跳过覆盖（丢本次），保存量关键日志。
    if level.critical() {
        return false;
    }
    // Info：向前覆盖最旧的非关键(Info)条目，直到腾出空间或只剩关键条目。
    let mut h = head;
    let mut scanned = 0usize;
    while free + scanned < need && scanned < LOG_RING_SIZE {
        let cur = h;
        let cur_len = ring[cur] as usize + 1; // 该条目总长（len+level+payload）
        let cur_level = ring[(cur + 1) % LOG_RING_SIZE];
        if cur_level == b'W' || cur_level == b'E' {
            // 关键条目，不覆盖；无法腾出 → 放弃本次 Info
            return false;
        }
        h = (h + cur_len) % LOG_RING_SIZE;
        scanned += cur_len;
    }
    if free + scanned < need {
        return false; // 全是关键条目，无法覆盖
    }
    // 覆盖 [head, h) 之间的 Info 条目，把 head 前移到 h，再写入
    LOG_HEAD.store(h as u32, Ordering::Release);
    let tail = LOG_TAIL.load(Ordering::Relaxed) as usize;
    let n = payload.len();
    ring[tail] = (n + 1) as u8;
    ring[(tail + 1) % LOG_RING_SIZE] = level.letter();
    for (i, &b) in payload.iter().enumerate() {
        ring[(tail + 2 + i) % LOG_RING_SIZE] = b;
    }
    LOG_TAIL.store(((tail + need) % LOG_RING_SIZE) as u32, Ordering::Release);
    true
}

/// 从 ring 读取一段可输出的连续字节（一次最多 `out.len()`），并前移 head。
/// 返回读取的字节数（可能为 0）。读取的是条目原始字节（含 len/level 头）。
fn ring_pop(out: &mut [u8]) -> usize {
    let ring: &mut [u8; LOG_RING_SIZE] = unsafe { &mut LOG_RING };
    let head = LOG_HEAD.load(Ordering::Acquire) as usize;
    let tail = LOG_TAIL.load(Ordering::Acquire) as usize;
    if head == tail {
        return 0; // 空
    }
    // 读一条完整条目
    let first = ring[head];
    let entry_len = (first as usize) + 1; // len 字段 + (level + payload)
    let n = entry_len.min(out.len());
    let mut read = 0usize;
    // 单条复制，支持跨越尾部回绕
    for i in 0..n {
        out[read] = ring[(head + i) % LOG_RING_SIZE];
        read += 1;
    }
    let nh = (head + entry_len) % LOG_RING_SIZE;
    LOG_HEAD.store(nh as u32, Ordering::Release);
    read
}

/* ===========================================================================
 * emit：生产者入口。只写 ring，绝不阻塞、绝不直接写串口。
 * =========================================================================== */
pub(crate) fn emit(level: Level, tag: &str, args: core::fmt::Arguments) {
    // 非阻塞互斥：拿不到锁（别的任务正在写）→ 丢弃本次日志（日志可丢）。
    if !try_lock() {
        return;
    }
    let mut buf = [0u8; 180];
    let mut len = 0usize;

    buf[len] = b'R'; len += 1;
    buf[len] = b'/'; len += 1;
    buf[len] = level.letter(); len += 1;
    buf[len] = b' '; len += 1;

    let ticks = unsafe {
        let slot = &*core::ptr::addr_of!(g_app_slot);
        match slot.tick_count {
            Some(f) => f(),
            None => 0,
        }
    };
    len += write_u32(&mut buf[len..], ticks);
    buf[len] = b' '; len += 1;

    let tb = tag.as_bytes();
    let n = tb.len().min(buf.len() - len - 4);
    buf[len..len + n].copy_from_slice(&tb[..n]);
    len += n;
    buf[len] = b':'; len += 1;
    buf[len] = b' '; len += 1;

    let mut w = BufWriter::new(&mut buf[len..]);
    let _ = core::fmt::write(&mut w, args);
    len += w.pos();

    buf[len] = b'\r'; len += 1;
    buf[len] = b'\n'; len += 1;

    ring_push(level, &buf[..len]);
    unlock();
}

/* ===========================================================================
 * log_task：消费者。低优先级，周期 drain ring → uart0 输出。
 * =========================================================================== */

/// 日志任务栈（放 App RAM）。
#[link_section = ".rust_bss"]
static mut LOG_TASK_STACK: [u8; 1024] = [0u8; 1024];

/// 日志任务优先级：低于所有业务任务（ctrl=4,sensor=5,uplink=10,telem=12,monitor≈14），
/// 靠近 idle(31)，即使阻塞在 uart0 也不影响业务。取 28。
const LOG_TASK_PRIO: u8 = 28;

pub extern "C" fn log_task_entry(_arg: *mut c_void) {
    // uart0 复用 C 侧已打开的句柄；只 get + write，绝不 open/close。
    let uart = Device::get("uart0\0");
    let mut buf = [0u8; 128];
    loop {
        // drain：把 ring 里积压的条目尽可能读出并一次写 uart0（非阻塞互斥）。
        // 可能输出多条，直到 ring 空。单条最长 128B（覆盖 1+1+~120 payload）。
        while let Some(dev) = uart.as_ref() {
            let n = {
                // 与生产者互斥地取走一条
                if !try_lock() {
                    break;
                }
                let got = ring_pop(&mut buf);
                unlock();
                got
            };
            if n == 0 {
                break; // ring 空
            }
            // 把 `[len][level][payload]` 的 level+payload 交给 uart0（去掉 len 头，
            // 保留我们拼好的 "R/L ticks tag: msg\r\n" 完整行）。
            let _ = dev.write(&buf[1..n]);
        }
        msleep(5);
    }
}

/// 创建日志任务（在 spawn_flyctrl 前调用一次）。
pub fn spawn_log_task() {
    unsafe {
        spawn_rt(
            b"log_task\0",
            log_task_entry,
            LOG_TASK_PRIO,
            LOG_TASK_STACK.as_mut_ptr(),
            LOG_TASK_STACK.len(),
            1,
            RT_NONE,
            0,
            0,
        );
    }
}

/// 把 u32 写成十进制 ASCII，返回写入字节数。
fn write_u32(dst: &mut [u8], mut v: u32) -> usize {
    if v == 0 {
        if !dst.is_empty() {
            dst[0] = b'0';
            return 1;
        }
        return 0;
    }
    let mut tmp = [0u8; 10];
    let mut i = 0;
    while v > 0 && i < tmp.len() {
        tmp[i] = b'0' + (v % 10) as u8;
        v /= 10;
        i += 1;
    }
    let mut n = 0;
    while i > 0 {
        i -= 1;
        if n < dst.len() {
            dst[n] = tmp[i];
            n += 1;
        }
    }
    n
}

/// 把 core::fmt 输出限制写进固定切片，超出截断（不 panic）。
struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BufWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    fn pos(&self) -> usize {
        self.pos
    }
}

impl<'a> core::fmt::Write for BufWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let b = s.as_bytes();
        let n = b.len().min(self.buf.len() - self.pos);
        self.buf[self.pos..self.pos + n].copy_from_slice(&b[..n]);
        self.pos += n;
        Ok(())
    }
}

/* ----------------------------- 宏接口 ----------------------------------- */

/// `info!(tag: "flyctrl", "msg {} {}", a, b)`
#[macro_export]
macro_rules! info {
    (tag: $tag:expr, $($arg:tt)*) => {
        $crate::log::emit($crate::log::Level::Info, $tag, format_args!($($arg)*))
    };
}

/// `warn!(tag: "flyctrl", "msg")`
#[macro_export]
macro_rules! warn {
    (tag: $tag:expr, $($arg:tt)*) => {
        $crate::log::emit($crate::log::Level::Warn, $tag, format_args!($($arg)*))
    };
}

/// `error!(tag: "flyctrl", "msg")`
#[macro_export]
macro_rules! error {
    (tag: $tag:expr, $($arg:tt)*) => {
        $crate::log::emit($crate::log::Level::Error, $tag, format_args!($($arg)*))
    };
}

/// `debug!(tag: "flyctrl", "msg")` —— 仅 debug build 编入（release 剔除）。
#[macro_export]
macro_rules! debug {
    (tag: $tag:expr, $($arg:tt)*) => {
        if cfg!(debug_assertions) {
            $crate::log::emit($crate::log::Level::Debug, $tag, format_args!($($arg)*))
        }
    };
}
