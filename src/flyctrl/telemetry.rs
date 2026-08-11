//! 遥测下行任务（20ms 周期）：从最新估计发标准 MAVLink。
//!
//! 读 EST_STATE（经 EST_MTX），经 usb0(USB CDC) 单通道下行
//! heartbeat / local_pos / sys_status。
//!
//! 注：uart3(USART6) 未接到 PC，无法闭环验证，故下行只用 usb0。

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
#[link_section = ".rust_bss"]
static mut FRAME_BUF: [u8; flyctrl_core::comm::link::MAX_FRAME_LEN] =
    [0u8; flyctrl_core::comm::link::MAX_FRAME_LEN];

/// 遥测下行任务入口。
///
/// 下行通道：USB CDC(`usb0`) 单写。
/// USB CDC 是 CDC-ACM 虚拟串口，电脑端免 USB-TTL 转接即可直接收 MAVLink 流做仿真分析。
/// `usb_stream_write` 是非阻塞 staged 写：host 未连 / 未 IN-token 时 TX ring 填满后
/// 仅返回 0（丢帧），绝不阻塞任务。因此**不**用 `USB_IOCTL_CONNECTED`(DTR 控制线) 来决定
/// 是否写 usb0 —— 上位机开 COM9 但 DTR=False 时 conn 仍为 0，据此跳过会致上位机收不到
/// 数据。正确做法是无条件写，host 一连即收。usb0 与 C 侧 g_console(uart0) 相互独立。
///
/// uart3(USART6) 未接到 PC，无法闭环验证，下行暂不挂 uart3（其 IRQ 引擎 write 为阻塞式，
/// 一旦唤醒中断异常会永久卡死 telem；而 usb0 的非阻塞 staged 写天然满足"host 不连不卡死"）。
pub extern "C" fn telemetry_entry(_arg: *mut c_void) {
    info!(tag: "telem", "task started; period=20ms prio={} downlink=usb0", RTOS_PRIO_MAIN);

    // usb0 复用系统层已在 boot 阶段 open 的句柄（Device::get），
    // 切勿二次 Device::open —— 二次 open 会再次 USBD_Init + 重绑 ISR，
    // 重置 USB TX 状态机导致后续 write 阻塞/卡死（已实测复现）。
    let usb_dev = Device::get("usb0\0");
    if usb_dev.is_none() {
        warn!(tag: "telem", "usb0 (USB CDC) not available -> no downlink");
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

        // 下行：USB CDC(usb0) 单通道写。
        //
        // 注意：USB CDC 的 usb_stream_write 是【非阻塞 staged】——host 未连 / 未 IN-token
        // 时 TX ring 填满后 write 仅返回 0（丢帧），绝不阻塞任务（已实测验证）。
        // 因此**不要**用 USB_IOCTL_CONNECTED(DTR 控制线) 决定是否写 usb0：上位机打开
        // COM9 但 DTR=False（避免 CH340 复位）时 conn 仍为 0，若据此跳过 usb0 会导致
        // 上位机连着 USB 却收不到 MAVLink。正确做法是无条件写，host 连上即收到。
        let mut wrote_usb = 0i32;
        let send = |d: &Device, fb: &mut [u8; flyctrl_core::comm::link::MAX_FRAME_LEN],
                    seq: u8, est: &_, armed: bool, health_ok: bool| -> i32 {
            let mut total = 0i32;
            for n in [
                mavlink::encode_heartbeat(0, armed, seq, fb),
                mavlink::encode_local_pos_from(mavlink::SYS_ID, est, seq, fb),
                mavlink::encode_sys_status(health_ok, seq, fb),
            ] {
                let k = d.write(&fb[..n]);
                if k > 0 { total += k; }
            }
            total
        };
        if let Some(d) = usb_dev.as_ref() {
            wrote_usb = send(d, frame_buf, seq, &est, armed, health != Health::Critical);
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
                  "hb seq={} usb_wr={} imu/gps/baro={}/{}/{} sens_armed={} crit={} uptime={}ms",
                  seq, wrote_usb,
                  imu_ok, gps_ok, baro_ok, sens_armed, health == Health::Critical, crate::rtos_sync::tick_count());
        }

        msleep(20);
    }
}
