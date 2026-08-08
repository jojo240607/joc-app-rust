//! QMC5883L（I2C 从机 0x0D）：三轴磁力计。
//!
//! 经 RTOS `i2c0` 总线（I2C1）组合；缺失/无响应时构造返回 `None` 由上层降级。

use crate::device::{Device, i2c_write_read};

pub struct MagQmc5883 {
    bus: Device,
    addr: u16,
}

impl MagQmc5883 {
    const DATA_X_L: u8 = 0x00;

    pub fn new(bus_name: &[u8], addr: u16) -> Option<Self> {
        Device::open(bus_name).map(|bus| Self { bus, addr })
    }

    pub fn read(&self) -> Option<[f32; 3]> {
        let mut raw = [0u8; 6];
        if i2c_write_read(&self.bus, self.addr, Self::DATA_X_L, &mut raw) != 0 {
            return None;
        }
        let v = |o: usize| i16::from_le_bytes([raw[o], raw[o + 1]]) as f32;
        Some([v(0), v(2), v(4)])
    }
}
