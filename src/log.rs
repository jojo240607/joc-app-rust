//! joc-app-rust 应用层日志系统（区别于 C 侧 `I/main:` 系统日志）。
//!
//! 输出通道：复用 RTOS 调试控制台 `uart0`（USART1 / COM8），前缀统一 `R/...`，
//! 与系统日志 `I/...` 明显区分。App 仅 `get` 查找 + `write`，**绝不 open/close**：
//! uart0 已由 C 侧 `g_console` 打开为控制台，App 复用同一设备实例；若 App 每次
//! close 会把 C 侧控制台 UART 整个 deinit（清 UE/释放 PA9），导致 C 侧 printf 卡死
//! 在 `uart_hal_putc` 的 TXE busy-wait，进而冻结整个系统。故只写不关。
//!
//! 特性：
//!  - 三级：`info!` / `warn!` / `error!` + `debug!`（debug build 才编入）。
//!  - 每条带 tick 时间戳（`g_app_slot.tick_count()`）与调用点标签（tag）。
//!  - 宏风格，用法接近 `log` crate：`rlog::info!(tag: "flyctrl", "heartbeat sent")`。
//!  - 全部 `no_std`，仅经 ABI 契约 `g_app_slot` 调用，不碰裸 RTOS 符号。

use core::ffi::{c_char, c_void};

use crate::abi::*;

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
}

/// 把一条日志经 uart0 写出。格式：`R/<L> <ticks> <tag>: <msg>\r\n`。
///
/// 复用 C 侧已打开的控制台 uart0：**只 get + write，绝不 open/close**。
/// uart0 由 RTOS `g_console` 在启动早期 open 为控制台，App 共享同一设备实例；
/// 若 App 重复 open/close 会把 C 侧控制台 UART 整个 deinit，导致 C 侧 printf
/// 在 `uart_hal_putc` 的 TXE busy-wait 中死循环、冻结系统。日志频率低（飞控仅
/// 周期/状态变更时打），该开销可接受。
pub(crate) fn emit(level: Level, tag: &str, args: core::fmt::Arguments) {
    // 栈上缓冲：前缀 + 时间戳 + tag + msg。飞控单条日志不会超 160 字节。
    let mut buf = [0u8; 200];
    let mut len = 0usize;

    // 前缀 R/<L> 空格
    buf[len] = b'R'; len += 1;
    buf[len] = b'/'; len += 1;
    buf[len] = level.letter(); len += 1;
    buf[len] = b' '; len += 1;

    // tick 时间戳（十进制，ASCII）
    let ticks = unsafe {
        let slot = &*core::ptr::addr_of!(g_app_slot);
        match slot.tick_count {
            Some(f) => f(),
            None => 0,
        }
    };
    len += write_u32(&mut buf[len..], ticks);
    buf[len] = b' '; len += 1;

    // tag
    let tb = tag.as_bytes();
    let n = tb.len().min(buf.len() - len - 4);
    buf[len..len + n].copy_from_slice(&tb[..n]);
    len += n;
    buf[len] = b':'; len += 1;
    buf[len] = b' '; len += 1;

    // msg（core::fmt 写进剩余空间）
    let mut w = BufWriter::new(&mut buf[len..]);
    let _ = core::fmt::write(&mut w, args);
    len += w.pos();

    // 行尾
    buf[len] = b'\r'; len += 1;
    buf[len] = b'\n'; len += 1;

    unsafe {
        let slot = &*core::ptr::addr_of!(g_app_slot);
        let (Some(get), Some(write)) = (slot.dev_get, slot.dev_write) else {
            return;
        };
        let name = b"uart0\0".as_ptr() as *const c_char;
        let dev = get(name);
        if dev.is_null() {
            return;
        }
        // 仅写：uart0 已由 C 侧 g_console open 为控制台，App 不复用 open/close，
        // 避免 deinit 共享 UART 导致 C 侧 printf 在 TXE busy-wait 死循环冻结系统。
        let _ = write(dev, buf.as_ptr() as *const c_void, len);
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
    // 倒序写
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
