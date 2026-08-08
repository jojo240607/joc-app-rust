//! 对 RTOS 设备接口的安全封装。Rust 应用经此操作驱动，不碰裸寄存器。
//!
//! 方案 Y 轻量版：所有系统调用经 `g_app_slot` 服务表（含 `dev_get` / `dev_open` /
//! `dev_read` / `dev_write` / `dev_ioctl` / `dev_close`），不引用裸 `device_manager_get`
//! 符号，故 App 可独立链接成应用分区镜像。零成本：方法即直接转发 vtable / 服务表调用。
//!
//! 关键约定（与 joc-base RTOS 的契约一致）：
//! - App 绝不碰裸寄存器（规避 CCM/MPU/DMA 风险）；所有 IO 走 `g_app_slot.dev_*`。
//! - `Device` 同时支持两套取设备方式：
//!   * `Device::get(name)`：仅按名查找句柄（自动补 `\0`），不立即 open；
//!   * `Device::open(name)`：查找 + 自动 open，返回 `Option`（任务编排常用）。
//! - `Drop` 自动 `close`，无遗漏/重复关闭风险。

use core::ffi::{c_char, c_void};

use crate::abi::*;
use crate::ioctl;

/// 设备句柄：从 `g_app_slot.dev_get` 拿到 `*mut device_t`，封装成类型安全 API。
///
/// 内部记录 `opened` 状态，`Drop` 时自动 `close`。
pub struct Device {
    dev: *mut device_t,
    opened: bool,
}

impl Device {
    /// 按名查找设备；返回 None 表示未注册。
    ///
    /// 经服务表 `dev_get`（不引用裸 `device_manager_get` 符号，支持独立分区镜像）。
    /// 仅查找，不 open；需用 `open()` 实例方法显式打开，或改用 `Device::open()` 一步到位。
    pub fn get(name: &str) -> Option<Device> {
        let mut buf = [0u8; 32];
        if name.len() >= buf.len() {
            return None;
        }
        buf[..name.len()].copy_from_slice(name.as_bytes());
        buf[name.len()] = 0;
        let slot = unsafe { &*core::ptr::addr_of!(g_app_slot) };
        let get = slot.dev_get?; // None → 服务表未初始化
        let p = get(buf.as_ptr() as *const c_char);
        if p.is_null() {
            None
        } else {
            Some(Device { dev: p, opened: false })
        }
    }

    /// 查找 + 自动 open；返回 None 表示未注册或 open 失败。
    ///
    /// 任务编排常用：总线/外设名对齐 RTOS 已注册节点
    /// （`src/board/stm32f4_discovery.c`）：`uart0`(USART1) / `pwm0`(TIM3) /
    /// `i2c0`(I2C1) / `spi0`(SPI1) / `uart1`(USART2, GPS)。
    pub fn open(name: &[u8]) -> Option<Self> {
        let dev = unsafe {
            match g_app_slot.dev_get {
                Some(get) => get(name.as_ptr() as *const c_char),
                None => return None,
            }
        };
        if dev.is_null() {
            return None;
        }
        let rc = unsafe {
            match g_app_slot.dev_open {
                Some(open) => open(dev),
                None => return None,
            }
        };
        if rc != 0 {
            return None;
        }
        Some(Self { dev, opened: true })
    }

    /// 对已查找到的句柄显式 open；返回 0 成功，<0 失败。
    pub fn open_dev(&mut self) -> i32 {
        if self.opened {
            return 0;
        }
        let rc = unsafe {
            match g_app_slot.dev_open {
                Some(open) => open(self.dev),
                None => return -1,
            }
        };
        if rc == 0 {
            self.opened = true;
        }
        rc
    }

    /// 读设备；返回读到的字节数，<0 为错误。
    pub fn read(&self, buf: &mut [u8]) -> i32 {
        unsafe {
            match g_app_slot.dev_read {
                Some(read) => read(self.dev, buf.as_mut_ptr() as *mut c_void, buf.len()),
                None => -1,
            }
        }
    }

    /// 写设备；返回写入的字节数，<0 为错误。
    pub fn write(&self, buf: &[u8]) -> i32 {
        unsafe {
            match g_app_slot.dev_write {
                Some(write) => write(self.dev, buf.as_ptr() as *const c_void, buf.len()),
                None => -1,
            }
        }
    }

    /// 设备私有控制；cmd 见 `ioctl` 模块常量。arg 为 RTOS 侧结构体指针，由调用方保证布局匹配。
    pub fn ioctl(&self, cmd: i32, arg: *mut c_void) -> i32 {
        unsafe {
            match g_app_slot.dev_ioctl {
                Some(ioctl_fn) => ioctl_fn(self.dev, cmd, arg),
                None => -1,
            }
        }
    }

    /// 取设备的中断号（经 vtable.irq_id）；<0 表示该设备不占用独立 IRQ 线。
    pub fn irq_id(&self) -> i32 {
        unsafe {
            if (*self.dev).vtable.is_null() {
                return -1;
            }
            match (*(*self.dev).vtable).irq_id {
                Some(f) => f(self.dev as *mut c_void),
                None => -1,
            }
        }
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        if self.opened {
            unsafe {
                if let Some(close) = g_app_slot.dev_close {
                    close(self.dev);
                }
            }
            self.opened = false;
        }
    }
}

/// I2C 传输描述符：布局须与 RTOS `drv/i2c.h` 一致。
#[repr(C)]
pub struct I2cXfer {
    pub addr: u16,    // 7-bit 从机地址
    pub buf: *mut u8, // 数据缓冲
    pub len: u16,
    pub result: i32,  // OUT: 0=ACK, -1=NACK/timeout
}

/// SPI 传输描述符：布局须与 RTOS `drv/spi.h` 一致。
#[repr(C)]
pub struct SpiXfer {
    pub tx_buf: *const u8, // NULL = 发 0xFF
    pub rx_buf: *mut u8,   // NULL = 丢弃
    pub len: u16,
}

/// I2C 写单个寄存器后读 N 字节（标准 sensor 事务）。
///
/// 调用方需保证 `buf` 生命周期覆盖两次 ioctl，且 `dev` 已 open。
pub fn i2c_write_read(dev: &Device, addr: u16, reg: u8, buf: &mut [u8]) -> i32 {
    // 1) 写寄存器地址
    let mut tx = [reg];
    let mut w = I2cXfer { addr, buf: tx.as_mut_ptr(), len: 1, result: 0 };
    let r = dev.ioctl(ioctl::I2C_IOCTL_MASTER_WRITE, &mut w as *mut I2cXfer as *mut c_void);
    if r != 0 || w.result != 0 {
        return -1;
    }
    // 2) 读数据
    let mut rd = I2cXfer { addr, buf: buf.as_mut_ptr(), len: buf.len() as u16, result: 0 };
    let r = dev.ioctl(ioctl::I2C_IOCTL_MASTER_READ, &mut rd as *mut I2cXfer as *mut c_void);
    if r != 0 || rd.result != 0 {
        return -1;
    }
    0
}
