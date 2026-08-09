//! 遥测下行任务（20ms 周期）：从最新估计发标准 MAVLink。
//!
//! 读 EST_STATE（经 EST_MTX），经 uart3 下行 heartbeat / local_pos / sys_status。

use core::ffi::c_void;

use flyctrl_core::comm::mavlink;
use flyctrl_core::fdir::Health;

use crate::abi::RTOS_PRIO_MAIN;
use crate::device::Device;
use crate::ioctl;
use crate::{info, warn};
use crate::rtos_sync::{msleep, tick_count};
use crate::flyctrl::{EST_MTX, EST_STATE};

/// 帧缓冲放在静态区（不占任务栈）。
/// 遥测任务独占该缓冲，循环内串行复用，无需互斥。
#[link_section = ".bss.telem_frame"]
static mut FRAME_BUF: [u8; flyctrl_core::comm::link::MAX_FRAME_LEN] =
    [0u8; flyctrl_core::comm::link::MAX_FRAME_LEN];

/// 遥测下行任务入口。
///
/// 下行通道优先级：USB CDC(`usb0`) > 串口(`uart3`)。USB CDC 是 CDC-ACM 虚拟串口，
/// 电脑端免 USB-TTL 转接即可直接收到 MAVLink 流做仿真分析；仅当 USB 未枚举时回退
/// 到 uart3（USART6）。usb0 与 C 侧 g_console(uart0) 相互独立，Rust 复用写数据不会
/// 破坏控制台。写前用 `USB_IOCTL_CONNECTED` 探测枚举状态，未连接则跳过 usb、走 uart3。
pub extern "C" fn telemetry_entry(_arg: *mut c_void) {
    info!(tag: "telem", "task started; period=20ms prio={} downlink=usb0(uart3 fallback)", RTOS_PRIO_MAIN);

    let usb_dev = Device::open(b"usb0\0");
    if usb_dev.is_none() {
        warn!(tag: "telem", "usb0 (USB CDC) not available -> MAVLink only via uart3");
    }
    let uart_dev = Device::open(b"uart3\0");
    if uart_dev.is_none() {
        warn!(tag: "telem", "uart3 (telemetry) not available -> no serial downlink");
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

        // 下行通道选择：USB CDC 已枚举则优先 usb0，否则回退 uart3。
        // 避免往未枚举的 USB 端点堆积数据（被驱动静默丢弃或占 TX 缓冲）。
        let use_usb = usb_dev.as_ref().map_or(false, |d| {
            let mut conn: i32 = 0;
            d.ioctl(ioctl::USB_IOCTL_CONNECTED, &mut conn as *mut i32 as *mut c_void) == 0
                && conn != 0
        });
        let downlink = if use_usb {
            usb_dev.as_ref()
        } else {
            uart_dev.as_ref()
        };

        if let Some(d) = downlink {
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
                  "hb seq={} ch={} imu/gps/baro={}/{}/{} sens_armed={} crit={} uptime={}ms",
                  seq, if use_usb { "usb0" } else { "uart3" },
                  imu_ok, gps_ok, baro_ok, sens_armed, health == Health::Critical, crate::rtos_sync::tick_count());
        }

        msleep(20);
    }
}
