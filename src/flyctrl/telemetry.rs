//! 遥测下行任务（20ms 周期）：从最新估计发标准 MAVLink。
//!
//! 读 EST_STATE（经 EST_MTX），经 uart3 下行 heartbeat / local_pos / sys_status。

use core::ffi::c_void;

use flyctrl_core::comm::link::Frame;
use flyctrl_core::comm::mavlink;
use flyctrl_core::fdir::Health;

use crate::abi::RTOS_PRIO_MAIN;
use crate::device::Device;
use crate::{info, warn};
use crate::rtos_sync::msleep;
use crate::flyctrl::{EST_MTX, EST_STATE};

/// 遥测下行任务入口。
pub extern "C" fn telemetry_entry(_arg: *mut c_void) {
    info!(tag: "telem", "task started; period=20ms prio={}", RTOS_PRIO_MAIN);

    let uart_dev = Device::open(b"uart3\0");
    if uart_dev.is_none() {
        warn!(tag: "telem", "uart3 (telemetry) not available -> no MAVLink downlink");
    }

    // 复用帧缓冲（避免每次分配）
    let mut frame_buf = [0u8; flyctrl_core::comm::link::MAX_FRAME_LEN];
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
            let _ = d.write(Frame::from_bytes(&frame_buf[..n]).as_slice());
            let n = mavlink::encode_local_pos_from(mavlink::SYS_ID, &est, seq, &mut frame_buf);
            let _ = d.write(Frame::from_bytes(&frame_buf[..n]).as_slice());
            let n = mavlink::encode_sys_status(health != Health::Critical, seq, &mut frame_buf);
            let _ = d.write(Frame::from_bytes(&frame_buf[..n]).as_slice());
        }

        seq = seq.wrapping_add(1);
        if seq == 1 {
            info!(tag: "telem", "first loop done; armed={}", armed);
        }
        if seq % 50 == 0 {
            info!(tag: "telem", "hb seq={} armed={} crit={}",
                  seq, armed, health == Health::Critical);
        }

        msleep(20);
    }
}
