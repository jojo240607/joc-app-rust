//! 传感器采集任务：周期性读取 IMU / 气压 / GPS / RC，写入 `SENSOR_FRAME`（经 seqlock，见 `SENSOR_SEQ`）。
//!
//! 数据源由 `sensors::stack::SensorStack` 统一封装，底层是虚拟回放（`VirtualXxx`）
//! 还是真实驱动（`ImuMpu6050` / `BaroBmp280` / `GpsUblox` / `RcSbus`）由编译期
//! `cfg(feature = "real-sensors")` 决定，本任务**不感知**差异，只调用 trait 方法。
//! 调试用虚拟源（默认），接真实硬件时只需开启 feature，算法/控制/遥测层零改动。

use core::ffi::c_void;

use flyctrl_core::vehicle::{ImuSample, PosSample, RcInput};

use crate::flyctrl::{SENSOR_FRAME, SENSOR_SEQ};
use crate::info;
use crate::abi::g_app_slot;
use crate::sensors::stack::SensorStack;

/// 诊断：是否把采集数据写入共享 SENSOR_FRAME。false 时控制环回退到内部虚拟源，
/// 用于区分“写入共享帧后被控制环处理”与“传感器任务自身”两类问题。
const WRITE_FRAME: bool = true;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Sensors;

impl Sensors {
    pub fn new(_name: &[u8]) -> Option<Self> {
        Some(Self)
    }

    pub extern "C" fn entry(_arg: *mut c_void) {
        info!(tag: "sensor", "task started (WRITE_FRAME={}, real={})",
              WRITE_FRAME as u32, cfg!(feature = "real-sensors") as u32);

        // 采样周期（秒），与回放推进一致。
        let sample_dt: f32 = 0.002; // 500Hz 采样

        // ---- 统一数据源：虚拟或真实由编译期 feature 决定 ----
        let mut stack = SensorStack::new();
        let h = stack.health();
        info!(tag: "sensor", "stack init imu={} baro={} gps={} rc={} mag={}",
              h.imu, h.baro, h.gps, h.rc, h.mag);

        let mut first = true;
        let mut loop_cnt: u32 = 0;
        loop {
            // 虚拟回放模式下，每个采样周期推进一次全局读指针（四类数据同步对齐）。
            #[cfg(not(feature = "real-sensors"))]
            unsafe {
                crate::sensors::sim::dataset::PLAYBACK.advance(sample_dt);
            }

            // ---- 读取各类传感器（统一 trait 接口，不区分虚拟/真实）----
            let imu_sample: Option<ImuSample> = Some(stack.read_imu());
            let baro_sample: Option<f32> = Some(stack.read_altitude());
            let gps_sample: Option<PosSample> = stack.read_gps();
            let rc_input: RcInput = stack.read_rc();

            // ---- 写共享帧（seqlock：sensors 单写、control 单读，control 优先级更高）----
            if WRITE_FRAME {
                unsafe {
                    SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 奇：写入中
                    let f = &mut *core::ptr::addr_of_mut!(SENSOR_FRAME);
                    f.imu = imu_sample;
                    f.baro_alt = baro_sample;
                    f.gps = gps_sample;
                    f.rc = rc_input;
                    f.armed = rc_input.armed;
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
            // 低频心跳（每 500 loop 一次）：高频连续 info! 会令 uart0 dev_write 在 TX ring
            // 满时阻塞，生产代码不应周期密集打印。
            if loop_cnt % 500 == 0 {
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
