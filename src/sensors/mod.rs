//! Rust 传感器驱动层：用 RTOS 总线（i2c0/spi0）组合具体传感器。
//!
//! RTOS 层只提供总线/通用外设驱动；具体传感器设备由本层经总线构建：
//! MPU6050 / BMP280 / QMC5883L via I2C，GPS via UART（降级），缺失则 SimImu 模拟源。

pub mod baro_bmp280;
pub mod gps_ublox;
pub mod imu_mpu6050;
pub mod mag_qmc5883;
pub mod rc_sbus;
pub mod sim_imu;

pub use baro_bmp280::BaroBmp280;
pub use gps_ublox::GpsUblox;
pub use imu_mpu6050::ImuMpu6050;
pub use mag_qmc5883::MagQmc5883;
pub use rc_sbus::RcSbus;
pub use sim_imu::SimImu;
