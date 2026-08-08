//! 对 C 侧 device 接口的安全封装。Rust 应用经此操作驱动，不碰裸寄存器。
//! 零成本：方法即直接转发 vtable 调用。

use core::ffi::{c_char, c_void};

#[repr(C)]
#[derive(Clone, Copy)]
pub struct deviceVtable {
    pub open: Option<extern "C" fn(*mut device) -> i32>,
    pub close: Option<extern "C" fn(*mut device) -> i32>,
    pub read: Option<extern "C" fn(*mut device, *mut c_void, usize) -> i32>,
    pub write: Option<extern "C" fn(*mut device, *const c_void, usize) -> i32>,
    pub ioctl: Option<extern "C" fn(*mut device, i32, *mut c_void) -> i32>,
    pub irq_id: Option<extern "C" fn(*mut device) -> i32>,
}

#[repr(C)]
pub struct device {
    pub vtable: *const deviceVtable,
    pub type_: u32,
    pub name: *const c_char,
    pub class_: u32,
}

/// 设备句柄：从 `device_manager_get` 拿到 `*mut device`，封装成类型安全 API。
/// Copy：仅含裸指针，复制即复制句柄，无所有权负担。
#[derive(Clone, Copy)]
pub struct Device(*mut device);

impl Device {
    /// 按名查找设备；返回 None 表示未注册。
    pub fn get(name: &str) -> Option<Device> {
        let mut buf = [0u8; 32];
        if name.len() >= buf.len() {
            return None;
        }
        buf[..name.len()].copy_from_slice(name.as_bytes());
        buf[name.len()] = 0;
        let p = unsafe { device_manager_get(buf.as_ptr() as *const c_char) };
        if p.is_null() {
            None
        } else {
            Some(Device(p))
        }
    }

    fn vt(&self) -> &'static deviceVtable {
        unsafe { &*(*self.0).vtable }
    }

    pub fn open(&self) -> i32 {
        match self.vt().open {
            Some(f) => f(self.0),
            None => -1,
        }
    }

    pub fn close(&self) -> i32 {
        match self.vt().close {
            Some(f) => f(self.0),
            None => -1,
        }
    }

    /// 读设备；返回读到的字节数，<0 为错误。
    pub fn read(&self, buf: &mut [u8]) -> i32 {
        match self.vt().read {
            Some(f) => f(self.0, buf.as_mut_ptr() as *mut c_void, buf.len()),
            None => -1,
        }
    }

    /// 写设备；返回写入的字节数，<0 为错误。
    pub fn write(&self, buf: &[u8]) -> i32 {
        match self.vt().write {
            Some(f) => f(self.0, buf.as_ptr() as *const c_void, buf.len()),
            None => -1,
        }
    }

    /// 设备私有控制；cmd 见 ioctl 模块常量。
    pub fn ioctl(&self, cmd: i32, arg: *mut c_void) -> i32 {
        match self.vt().ioctl {
            Some(f) => f(self.0, cmd, arg),
            None => -1,
        }
    }

    pub fn irq_id(&self) -> i32 {
        match self.vt().irq_id {
            Some(f) => f(self.0),
            None => -1,
        }
    }
}

extern "C" {
    fn device_manager_get(name: *const c_char) -> *mut device;
}
