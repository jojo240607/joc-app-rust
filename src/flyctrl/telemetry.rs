//! 遥测下行任务（20ms 周期）：从最新估计发标准 MAVLink。
//!
//! 读 EST_STATE（经 EST_MTX），经 uart3 下行 heartbeat / local_pos / sys_status。

use core::ffi::c_void;

use flyctrl_core::comm::mavlink;
use flyctrl_core::fdir::Health;

use crate::abi::RTOS_PRIO_MAIN;
use crate::device::Device;
use crate::{info, warn};
use crate::rtos_sync::{msleep, tick_count};
use crate::flyctrl::{EST_MTX, EST_STATE};

/// 帧缓冲放在静态区（不占任务栈）。
/// 遥测任务独占该缓冲，循环内串行复用，无需互斥。
#[link_section = ".bss.telem_frame"]
static mut FRAME_BUF: [u8; flyctrl_core::comm::link::MAX_FRAME_LEN] =
    [0u8; flyctrl_core::comm::link::MAX_FRAME_LEN];

/// 遥测下行任务入口。
pub extern "C" fn telemetry_entry(_arg: *mut c_void) {
    info!(tag: "telem", "task started; period=20ms prio={}", RTOS_PRIO_MAIN);

    let uart_dev = Device::open(b"uart3\0");
    if uart_dev.is_none() {
        warn!(tag: "telem", "uart3 (telemetry) not available -> no MAVLink downlink");
    }

    let mut frame_buf = unsafe { &mut FRAME_BUF };
    let mut seq: u8 = 0;
    loop {
        // 读最新估计（短临界区）
        let (est, health, armed);
        {
            let _g = unsafe { EST_MTX.guard() };
            let s = unsafe { &*core::ptr::addr_of!(EST_STATE) };
            est = s.est;
            health = s.health;
            armed = s.armed;
        }

        if let Some(d) = &uart_dev {
            let n = mavlink::encode_heartbeat(0, armed, seq, &mut frame_buf);
            let _ = d.write(&frame_buf[..n]);
            let n = mavlink::encode_local_pos_from(mavlink::SYS_ID, &est, seq, &mut frame_buf);
            let _ = d.write(&frame_buf[..n]);
            let n = mavlink::encode_sys_status(health != Health::Critical, seq, &mut frame_buf);
            let _ = d.write(&frame_buf[..n]);
        }

        seq = seq.wrapping_add(1);
        if seq == 1 {
            info!(tag: "telem", "first loop done; armed={}", armed);
        }
        if seq % 50 == 0 {
            // 系统健康快照（原为 monitor 任务，现已并入 telem，每 1s 一次）。
            let (mut imu_ok, mut gps_ok, mut baro_ok, mut sens_armed): (bool, bool, bool, bool);
            {
                unsafe {
                    let mut s1;
                    loop {
                        s1 = crate::flyctrl::SENSOR_SEQ;
                        if s1 & 1 != 0 { continue; }
                        let f = &*core::ptr::addr_of!(crate::flyctrl::SENSOR_FRAME);
                        imu_ok = f.imu_ok;
                        gps_ok = f.gps_ok;
                        baro_ok = f.baro_ok;
                        sens_armed = f.armed;
                        let s2 = crate::flyctrl::SENSOR_SEQ;
                        if s1 == s2 { break; }
                    }
                    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
                }
            }
            info!(tag: "telem",
                  "hb seq={} imu/gps/baro={}/{}/{} sens_armed={} crit={} uptime={}ms",
                  seq, imu_ok, gps_ok, baro_ok, sens_armed, health == Health::Critical, crate::rtos_sync::tick_count());
        }

        msleep(20);
    }
}
