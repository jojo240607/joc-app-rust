//! 传感器采集任务：周期性读取 IMU / 气压 / GPS / RC，写入 `SENSOR_FRAME`（由 `SENSOR_MTX` 保护）。
//!
//! Discovery 开发板上**没有任何 I2C 从设备**（板载仅引出 I2C1 的 PB6/PB7，无 MPU6050/BMP280 等
//! 物理传感器），且 joc-base 的 I2C 驱动在从机无应答时会**卡死**（见 memory 74117454）。因此默认
//! `ENABLE_I2C_SENSORS = false`，IMU/Baro 直接走 `None` 降级（由 FDIR/EKF 用零测量 + 降级健康度
//! 处理）。待 joc-base I2C 驱动修复并外接真实传感器后，再将其置为 `true`。
//!
//! GPS/RC 经 UART，其 `new()` 只做配置/超时轮询（不会卡死），故保持可用；无物理连接时
//! `healthy()==false`，飞控照常降级运行。

use core::ffi::c_void;

use flyctrl_core::hal::sensor::{GpsSensor, RcReceiver};
use flyctrl_core::vehicle::{ImuSample, PosSample, RcInput};

use crate::flyctrl::{SENSOR_FRAME, SENSOR_MTX};
use crate::rtos_sync::Mutex;
use crate::info;
use crate::abi::g_app_slot;

/// 是否实例化需要真实 I2C 从设备的传感器（IMU/Baro）。
/// Discovery 板无 I2C 从设备 + joc-base I2C 驱动无从机时卡死 → 默认关闭。
const ENABLE_I2C_SENSORS: bool = false;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Sensors;

impl Sensors {
    /// 创建任务实例（静态单例，使用固定的全局名字）。
    pub fn new(_name: &[u8]) -> Option<Self> {
        Some(Self)
    }

    /// 任务主体：周期性采集并写共享帧。
    pub extern "C" fn entry(_arg: *mut c_void) {
        info!(tag: "sensor", "task started");

        // 模拟 IMU（无硬件依赖，始终可用）。
        let mut sim = crate::sensors::sim_imu::SimImu::new();

        // I2C 传感器：受 ENABLE_I2C_SENSORS 门控，避免无 I2C 从机时卡死整个飞控。
        let mut imu: Option<crate::sensors::imu_mpu6050::ImuMpu6050> =
            if ENABLE_I2C_SENSORS {
                match crate::sensors::imu_mpu6050::ImuMpu6050::new(b"i2c0\0", 0x68) {
                    Some(d) => {
                        info!(tag: "sensor", "imu(mpu6050) available");
                        Some(d)
                    }
                    None => {
                        info!(tag: "sensor", "imu(mpu6050) not available");
                        None
                    }
                }
            } else {
                None
            };

        let mut baro: Option<crate::sensors::baro_bmp280::BaroBmp280> =
            if ENABLE_I2C_SENSORS {
                match crate::sensors::baro_bmp280::BaroBmp280::new(b"i2c0\0", 0x76) {
                    Some(d) => {
                        info!(tag: "sensor", "baro(bmp280) available");
                        Some(d)
                    }
                    None => {
                        info!(tag: "sensor", "baro(bmp280) not available");
                        None
                    }
                }
            } else {
                None
            };

        // GPS 经 UART，new() 仅做波特探测超时（不卡死）；无物理模块时 healthy=false。
        let mut gps: Option<crate::sensors::gps_ublox::GpsUblox> =
            match crate::sensors::gps_ublox::GpsUblox::new(b"uart1\0") {
                Some(d) => {
                    info!(tag: "sensor", "gps(uart1) opened");
                    Some(d)
                }
                None => {
                    info!(tag: "sensor", "gps(uart1) not available");
                    None
                }
            };

        // RC 经 UART，new() 仅下发线路配置 ioctl（不卡死）。
        let mut rc: Option<crate::sensors::rc_sbus::RcSbus> =
            match crate::sensors::rc_sbus::RcSbus::new(b"uart2\0") {
                Some(d) => {
                    info!(tag: "sensor", "rc(sbus uart2) opened");
                    Some(d)
                }
                None => {
                    info!(tag: "sensor", "rc(sbus uart2) not available");
                    None
                }
            };

        let mut first = true;
        let mut loop_cnt: u32 = 0;
        loop {
            info!(tag: "sensor", "loop enter");

            let imu_sample: Option<ImuSample> = if let Some(d) = imu.as_mut() {
                d.read()
            } else {
                Some(sim.next(0.002))
            };

            let baro_sample = if let Some(d) = baro.as_mut() {
                d.read_altitude().map(|m| m.0)
            } else {
                None
            };

            let gps_sample: Option<PosSample> = if let Some(d) = gps.as_mut() {
                d.read()
            } else {
                None
            };

            let rc_input: RcInput = if let Some(d) = rc.as_mut() {
                d.read()
            } else {
                RcInput::neutral()
            };

            // 写共享帧（受 SENSOR_MTX 保护）
            info!(tag: "sensor", "before mtx");
            {
                let _g = unsafe { Mutex::guard(&*core::ptr::addr_of!(SENSOR_MTX)) };
                info!(tag: "sensor", "in mtx");
                let f = unsafe { &mut *core::ptr::addr_of_mut!(SENSOR_FRAME) };
                f.imu = imu_sample;
                f.baro_alt = baro_sample;
                f.gps = gps_sample;
                f.rc = rc_input;
                f.imu_ok = imu_sample.is_some();
                f.baro_ok = baro_sample.is_some();
                f.gps_ok = gps_sample.is_some();
                info!(tag: "sensor", "wrote frame");
            }

            if first {
                info!(tag: "sensor", "first loop done");
                first = false;
            }
            loop_cnt += 1;
            if loop_cnt % 100 == 0 {
                info!(tag: "sensor", "loop {}", loop_cnt);
            }

            // 周期 ~2ms（500Hz）
            unsafe {
                if let Some(f) = g_app_slot.msleep {
                    f(2);
                }
            }
        }
    }
}

/// 模块级入口（供 `mod.rs::spawn_flyctrl` 经 `spawn_rt` 注册）。
pub extern "C" fn sensors_entry(arg: *mut c_void) {
    Sensors::entry(arg);
}
