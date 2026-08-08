//! 传感器采样任务（2ms 周期）：采集所有总线传感器 → 写共享帧。
//!
//! 写 SENSOR_FRAME（经 SENSOR_MTX）。mag(QMC5883L@0x0D) 待 joc-base I2C 驱动
//! 修复前走「缺失降级」路径（见 crate 级文档与 memory）。

use core::ffi::c_void;

use flyctrl_core::hal::sensor::{GpsSensor, RcReceiver};
use flyctrl_core::units::Second;
use flyctrl_core::vehicle::RcInput;

use crate::{info, warn};
use crate::rtos_sync::msleep;
use crate::sensors::{BaroBmp280, GpsUblox, ImuMpu6050, RcSbus, SimImu};
use crate::flyctrl::{SENSOR_FRAME, SENSOR_MTX};

/// 传感器采样任务入口。
pub extern "C" fn sensors_entry(_arg: *mut c_void) {
    info!(tag: "sensors", "task started; period=2ms prio=5");

    let mut sim = SimImu::new();
    // IMU（MPU6050@0x68）
    let imu = ImuMpu6050::new(b"i2c0\0", 0x68);
    if imu.is_none() { warn!(tag: "sensors", "imu(mpu6050) not available -> sim source"); }
    // Baro（BMP280@0x76）
    let baro = BaroBmp280::new(b"i2c0\0", 0x76);
    if baro.is_none() { warn!(tag: "sensors", "baro(bmp280) not available"); }
    // Mag（QMC5883L@0x0D）—— 见 memory：读取卡死，待 I2C 修复前不实例化
    let use_mag = false; // TODO: 修复 joc-base I2C 后改为 MagQmc5883::new(b"i2c0\0", 0x0D)
    // GPS（uart1 / USART2，NMEA）
    let mut gps = GpsUblox::new(b"uart1\0");
    if gps.is_none() { warn!(tag: "sensors", "gps(ublox) not available -> no pos fix"); }
    // RC（uart2 / USART3，SBUS）
    let mut rc = RcSbus::new(b"uart2\0");
    if rc.is_none() { warn!(tag: "sensors", "rc(sbus) not available -> neutral"); }

    let mut seq: u32 = 0;
    loop {
        let dt = Second(2.0 / 1000.0);

        // IMU
        let imu_sample = match imu.as_ref() {
            Some(s) => s.read(),
            None => Some(sim.next(dt.0)),
        };

        // RC
        let rc_in = match rc.as_mut() {
            Some(r) => r.read(),
            None => RcInput::neutral(),
        };
        let armed = rc_in.armed && rc_in.fresh;

        // GPS（可降级）
        let gps_sample = gps.as_mut().and_then(|g| g.read());
        // Baro（可降级）
        let baro_alt = baro.as_ref().and_then(|b| b.read_altitude());

        // 写共享帧（短临界区）
        {
            let _g = unsafe { SENSOR_MTX.guard() };
            let f = unsafe { &mut *core::ptr::addr_of_mut!(SENSOR_FRAME) };
            f.imu = imu_sample;
            f.rc = rc_in;
            f.gps = gps_sample;
            f.baro_alt = baro_alt.map(|m| m.0);
            f.imu_ok = imu_sample.is_some();
            f.gps_ok = gps_sample.is_some();
            f.baro_ok = baro_alt.is_some();
            f.mag_ok = false; // TODO: 接 mag（待 I2C 修复）
            f.armed = armed;
        }

        seq = seq.wrapping_add(1);
        if seq == 1 {
            info!(tag: "sensors",
                  "first loop done; imu={} rc={} gps={} baro={}",
                  imu_sample.is_some(), rc_in.fresh, gps_sample.is_some(), baro_alt.is_some());
        }
        if seq % 500 == 0 {
            info!(tag: "sensors", "hb seq={} imu={} gps={} baro={}",
                  seq, imu_sample.is_some(), gps_sample.is_some(), baro_alt.is_some());
        }
        let _ = use_mag;

        msleep(2);
    }
}
