//! 传感器采集任务：周期性读取 IMU / 气压 / GPS / RC，写入 `SENSOR_FRAME`（经 seqlock，见 `SENSOR_SEQ`）。
//!
//! 三种数据源（编译期开关互斥）：
//!   1. `USE_PLAYBACK`（默认 true）：从 `sensors::dataset` 循环回放真实形态数据，
//!      经 `sensors::virtual_sensors` 的虚拟驱动喂给飞控。板载无外设时让闭环真正跑起来，
//!      数据由 `tools/gen_dataset.py` 生成（默认合成真实形态片段；可注入真实飞行日志）。
//!   2. `ENABLE_I2C_SENSORS`（默认 false）：实例化真实 I2C 传感器（IMU/Baro）。
//!      Discovery 板无 I2C 从设备 + joc-base I2C 驱动无从机时卡死 → 默认关闭。
//!   3. 以上皆否：IMU 走 `SimImu` 模拟源，其余降级。
//!
//! GPS/RC 经 UART，其 `new()` 仅做配置/超时轮询（不会卡死），故保持可用；
//! 无物理连接时 `healthy()==false`，飞控照常降级运行。

use core::ffi::c_void;

use flyctrl_core::hal::sensor::{BaroSensor, GpsSensor, ImuSensor, RcReceiver};
use flyctrl_core::units::Meter;
use flyctrl_core::vehicle::{ImuSample, PosSample, RcInput};

use crate::flyctrl::{SENSOR_FRAME, SENSOR_SEQ};
use crate::info;
use crate::abi::g_app_slot;

/// 默认开启虚拟回放（板载无传感器时让飞控闭环真实形态运行）。
const USE_PLAYBACK: bool = true;
/// 诊断：是否把回放数据写入共享 SENSOR_FRAME。false 时控制环回退到 SimImu。
const WRITE_FRAME: bool = true;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Sensors;

impl Sensors {
    pub fn new(_name: &[u8]) -> Option<Self> {
        Some(Self)
    }

    pub extern "C" fn entry(_arg: *mut c_void) {
        info!(tag: "sensor", "task started");

        // 采样周期（秒），与回放推进一致。
        let sample_dt: f32 = 0.002; // 500Hz 采样

        // ---- 数据源初始化 ----
        let mut imu: Option<crate::sensors::virtual_sensors::VirtualImu> = None;
        let mut baro: Option<crate::sensors::virtual_sensors::VirtualBaro> = None;
        let mut gps: Option<crate::sensors::virtual_sensors::VirtualGps> = None;
        let mut rc: Option<crate::sensors::virtual_sensors::VirtualRc> = None;

        // 非 playback 路径所需的备选数据源（SimImu / 真实 I2C / UART GPS-RC）。
        // 注意：UART GPS/RC 的 `new()` 会做波特率自适应探测（阻塞最多 ~1s），
        // 仅在不回放时才需要，避免拖慢/干扰回放闭环。
        let mut sim = crate::sensors::sim_imu::SimImu::new();
        let mut imu_hw: Option<crate::sensors::imu_mpu6050::ImuMpu6050> = None;
        let mut baro_hw: Option<crate::sensors::baro_bmp280::BaroBmp280> = None;
        let mut gps_uart: Option<crate::sensors::gps_ublox::GpsUblox> = None;
        let mut rc_uart: Option<crate::sensors::rc_sbus::RcSbus> = None;

        if USE_PLAYBACK {
            info!(tag: "sensor", "mode=playback (virtual drivers over dataset)");
            imu = crate::sensors::virtual_sensors::VirtualImu::new();
            baro = crate::sensors::virtual_sensors::VirtualBaro::new();
            gps = crate::sensors::virtual_sensors::VirtualGps::new();
            rc = crate::sensors::virtual_sensors::VirtualRc::new();
            info!(tag: "sensor", "init imu={} baro={} gps={} rc={}",
                  imu.is_some(), baro.is_some(), gps.is_some(), rc.is_some());
        } else {
            imu_hw = None; // 真实 I2C IMU 在此初始化（当前硬件不可用，留 None）
            baro_hw = None;
            gps_uart = match crate::sensors::gps_ublox::GpsUblox::new(b"uart1\0") {
                Some(d) => {
                    info!(tag: "sensor", "gps(uart1) opened");
                    Some(d)
                }
                None => {
                    info!(tag: "sensor", "gps(uart1) not available");
                    None
                }
            };
            rc_uart = match crate::sensors::rc_sbus::RcSbus::new(b"uart2\0") {
                Some(d) => {
                    info!(tag: "sensor", "rc(sbus uart2) opened");
                    Some(d)
                }
                None => {
                    info!(tag: "sensor", "rc(sbus uart2) not available");
                    None
                }
            };
        }

        let mut first = true;
        let mut loop_cnt: u32 = 0;
        loop {
            // 回放模式：每个采样周期推进一次全局读指针（四类数据同步对齐）。
            if USE_PLAYBACK {
                unsafe {
                    crate::sensors::dataset::PLAYBACK.advance(sample_dt);
                }
            }

            // ---- 读取各类传感器 ----
            let imu_sample: Option<ImuSample> = if USE_PLAYBACK {
                imu.as_mut().map(|d| d.read())
            } else if let Some(d) = imu_hw.as_mut() {
                d.read()
            } else {
                Some(sim.next(sample_dt))
            };

            let baro_sample: Option<f32> = if USE_PLAYBACK {
                baro.as_mut().map(|d| d.read_altitude().0)
            } else if let Some(d) = baro_hw.as_mut() {
                d.read_altitude().map(|m| m.0)
            } else {
                None
            };

            let gps_sample: Option<PosSample> = if USE_PLAYBACK {
                gps.as_mut().and_then(|d| d.read())
            } else if let Some(d) = gps_uart.as_mut() {
                d.read()
            } else {
                None
            };

            let rc_input: RcInput = if USE_PLAYBACK {
                rc.as_mut().map(|d| d.read()).unwrap_or_else(RcInput::neutral)
            } else if let Some(d) = rc_uart.as_mut() {
                d.read()
            } else {
                RcInput::neutral()
            };

            // ---- 写共享帧（seqlock：sensors 单写、control 单读，control 优先级更高）----
            // 诊断开关 WRITE_FRAME：false 时传感器照常回放/读取但不写共享帧，
            // 用于区分“写入共享帧后被控制环处理”与“传感器任务自身”两类崩溃。
            if WRITE_FRAME {
                unsafe {
                    SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 奇：写入中
                    let f = &mut *core::ptr::addr_of_mut!(SENSOR_FRAME);
                    f.imu = imu_sample;
                    f.baro_alt = baro_sample;
                    f.gps = gps_sample;
                    f.rc = rc_input;
                    f.imu_ok = imu_sample.is_some();
                    f.baro_ok = baro_sample.is_some();
                    f.gps_ok = gps_sample.is_some();
                    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
                    SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 偶：写入完成
                }
            }

            if first {
                info!(tag: "sensor", "first loop done");
                first = false;
            }
            loop_cnt += 1;
            if loop_cnt % 50 == 0 {
                info!(tag: "sensor", "loop {} gps_w={} imu_w={} baro_w={}",
                      loop_cnt, gps_sample.is_some(), imu_sample.is_some(), baro_sample.is_some());
            }

            unsafe {
                if let Some(f) = g_app_slot.msleep {
                    f((sample_dt * 1000.0) as u32);
                }
            }
        }
    }
}

/// 模块级入口（供 `mod.rs::spawn_flyctrl` 经 `spawn_rt` 注册）。
pub extern "C" fn sensors_entry(arg: *mut c_void) {
    Sensors::entry(arg);
}
