//! 监控/心跳任务（1000ms 周期）：系统级日志 + 看门狗喂狗。
//!
//! 读 SENSOR_FRAME（经 SENSOR_MTX）与 EST_STATE（经 EST_MTX）汇总健康。

use core::ffi::c_void;

use crate::abi::RTOS_PRIO_BLINK;
use crate::rtos_sync::{msleep, tick_count};
use crate::flyctrl::{EST_MTX, EST_STATE, SENSOR_FRAME, SENSOR_MTX};
use crate::{info};

/// 监控/心跳任务入口。
pub extern "C" fn monitor_entry(_arg: *mut c_void) {
    info!(tag: "monitor", "task started; period=1000ms prio={}", RTOS_PRIO_BLINK);
    let mut seq: u32 = 0;
    loop {
        // 汇总各源健康（从共享帧 + 估计）
        let (imu_ok, gps_ok, baro_ok, armed, crit);
        {
            let _g = unsafe { SENSOR_MTX.guard() };
            let f = unsafe { &*core::ptr::addr_of!(SENSOR_FRAME) };
            imu_ok = f.imu_ok;
            gps_ok = f.gps_ok;
            baro_ok = f.baro_ok;
            armed = f.armed;
        }
        {
            let _g = unsafe { EST_MTX.guard() };
            let s = unsafe { &*core::ptr::addr_of!(EST_STATE) };
            crit = s.health == flyctrl_core::fdir::Health::Critical;
        }

        info!(tag: "monitor",
              "sys imu={} gps={} baro={} armed={} crit={} uptime={}ms",
              imu_ok, gps_ok, baro_ok, armed, crit, tick_count());

        seq = seq.wrapping_add(1);
        msleep(1000);
    }
}
