//! RTOS 设备抽象层：安全封装 `g_app_slot.dev_*` vtable + 总线传输描述符。
//!
//! 具体传感器驱动在 `crate::sensors` 用这里的 `Device` / `I2cXfer` / `SpiXfer` 组合构建。

pub mod rtos_device;

pub use rtos_device::{Device, I2cXfer, SpiXfer, i2c_write_read};
