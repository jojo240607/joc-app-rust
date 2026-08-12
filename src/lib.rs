//! joc-app-rust：独立 Rust 应用层（飞控）。
//! 通过 abi/device/ioctl 模块调用 RTOS ABI 契约；不依赖 rtos.h 内部。
//!
//! 挂载方式：RTOS 侧 app_main 在控制台循环前调用 `rust_app_start()`，
//! 本层在内部用 ABI 契约 `rtos_task_create_rt` 自行创建飞控硬实时任务。
//! 任务本身（EKF + PID + FDIR + MAVLink 遥测）见 `flyctrl_task` 模块。

#![no_std]
#![allow(static_mut_refs)]

// 链接脚本（app.ld）提供的 .data 段边界符号（LMA=Flash 初值地址，VMA=RAM 运行地址）。
extern "C" {
    static _appdata_lma: u8;
    static mut _sappdata: u8;
    static mut _eappdata: u8;
}

pub mod abi;
pub mod device;
pub mod ioctl;
pub mod sensors;
pub mod flyctrl;
pub mod rtos_sync;
pub mod log;

#[cfg(feature = "usbtest")]
mod usbtest;

use abi::*;

/* ===========================================================================
 * 统一挂载点：由 RTOS app_main 经 g_app_slot.app_start 调用（rust_app_start）
 * 本层只引用 g_app_slot 服务表，不碰任何裸 RTOS 符号——体现方案 Y 解耦：
 *  - 任务经 g_app_slot.task_create_rt 创建；
 *  - 驱动 IO 经 device vtable（dev_get/open/read/write/ioctl/close）。
 * 在此仅做版本校验并拉起飞控任务，RTOS C 侧不再包含任何 app 逻辑。
 * =========================================================================== */
/// 调试自报：往调试控制台 uart0（USART1，RTOS 已持有，App 不占用）打一行，
/// 证明 App 入口已执行、App 分区已成功挂载。仅查找 + open + write + close 本层临时句柄，
/// 不影响 C 侧已打开的 g_console。
fn report_mounted() {
    info!(tag: "app_slot", "RUST app mounted (rust_app_start)");
}

#[no_mangle]
pub extern "C" fn rust_app_start() -> i32 {
    unsafe {
        // 初始化 .data：把 Flash LMA 处的初值拷贝到 RAM VMA。
        // 系统加载器仅清零 App .bss，不拷贝 .data；App 独立镜像必须自拷贝，
        // 否则所有非零 const 初值（如 make_name 的 NAMES 数组）在 RAM 中全为 0。
        let src = _appdata_lma as *const u8;
        let dst = _sappdata as *mut u8;
        let n = (_eappdata as usize) - (_sappdata as usize);
        for i in 0..n {
            *dst.add(i) = *src.add(i);
        }

        let slot = &mut *core::ptr::addr_of_mut!(g_app_slot);

        // 双重防御：版本不符直接拒绝挂载（build.rs 已做链接期校验）
        if slot.magic != APP_SLOT_MAGIC || slot.version != RTOS_ABI_VERSION as u32 {
            error!(tag: "app_slot", "ABI mismatch magic={:#x} ver={} (exp {})",
                   slot.magic, slot.version, RTOS_ABI_VERSION);
            return -1;
        }

        // 调试自报：往调试控制台 uart0（USART1，RTOS 已持有）打一行，证明 App 入口已执行。
        // 不创建 Device 实例，避免 Drop 自动 close 干扰 C 侧已打开的控制台。
        report_mounted();

        // 拉起应用层多任务。
        //  - demo feature：极简打日志任务，不碰任何外设，仅验证拉起链路；
        //  - usbtest feature：隔离验证 usb0 写通道是否通畅（不碰 EST_MTX/传感器）；
        //  - 默认：正式飞控多任务（采样/控制/遥测/监控，经 RTOS 设备 vtable）。
        #[cfg(feature = "demo")]
        crate::demo::spawn_demo();
        #[cfg(feature = "usbtest")]
        crate::usbtest::start();
        #[cfg(not(any(feature = "demo", feature = "usbtest")))]
        crate::flyctrl::spawn_flyctrl();

        0
    }
}

/* ===========================================================================
 * Demo 入口（feature = "demo"）：极简任务，只经 ABI 打周期日志，不碰任何外设。
 * 目的：先验证「RTOS 异步 app_host 任务 → rust_app_start → 创建 RTOS 任务」
 * 的拉起链路是否跑通，并能从 App 经 g_app_slot 打日志到控制台，console 不阻塞。
 * 跑通后再切回正式 flyctrl（去掉 --features demo）。
 * =========================================================================== */
#[cfg(feature = "demo")]
mod demo {
    use crate::abi::*;
    use crate::device::Device;
    use crate::info;
    use crate::rtos_sync::{msleep, spawn_rt, RT_NONE};
    use core::ffi::c_void;

    // demo 任务独立栈（放 App RAM，4KB：含 usb0 读写调试路径）。
    #[link_section = ".rust_bss"]
    static mut DEMO_STACK: [u8; 4096] = [0u8; 4096];

    extern "C" fn demo_task_entry(_arg: *mut c_void) {
        let mut n: u32 = 0;
        // [BISECT] usb0 上下行调试：
        //  - downlink：每 20ms 写一个 ~28B 大帧（模拟 telem 高频 MAVLink 心跳），
        //    验证高频大帧是否导致 usb0 TX ring 满（usb_wr 变 0）。
        //  - uplink：非阻塞读 usb0，收到的字节 echo 回 host（验证 host→板→host 闭环）。
        let usb = Device::get("usb0\0");
        info!(tag: "demo", "usb0 handle={}", if usb.is_some() { 1u32 } else { 0u32 });
        // 模拟一个 MAVLink v2 心跳帧（28B，长度随 seq 低位变化以模拟多消息）。
        let mut big = [0u8; 32];
        big[0] = 0xFD;
        big[1] = 0x09;
        big[2] = 0x00;
        loop {
            n = n.wrapping_add(1);
            // downlink 大帧（长度字段随 seq 取模，模拟 payload 变化）
            let paylen = 8 + (n % 9) as u8;
            big[1] = paylen;
            let frame_len = (10 + paylen as usize) as i32;
            let mut wr: i32 = -1;
            if let Some(d) = &usb {
                wr = d.write(&big[..frame_len as usize]);
            }
            // uplink 读取并 echo
            let mut rbuf = [0u8; 64];
            let mut rd: i32 = 0;
            let mut ech: i32 = 0;
            if let Some(d) = &usb {
                let r = d.read(&mut rbuf);
                if r > 0 {
                    rd = r;
                    ech = d.write(&rbuf[..r as usize]);
                }
            }
            if n % 25 == 0 {
                info!(tag: "demo", "alive seq={} usb_wr={} rd={} echo={}", n, wr, rd, ech);
            }
            msleep(20);
        }
    }

    /// 把 u32 十进制写入 buf，返回写入字节数（无分配）。
    fn write_u32(buf: &mut [u8], mut v: u32) -> usize {
        if v == 0 {
            if !buf.is_empty() { buf[0] = b'0'; }
            return 1;
        }
        let mut tmp = [0u8; 10];
        let mut i = 0;
        while v > 0 {
            tmp[i] = b'0' + (v % 10) as u8;
            v /= 10;
            i += 1;
        }
        let mut n = 0;
        while i > 0 {
            i -= 1;
            if n < buf.len() {
                buf[n] = tmp[i];
                n += 1;
            }
        }
        n
    }

    pub fn spawn_demo() {
        // 经 g_app_slot 间接创建任务（与 flyctrl 一致，不直接引用裸 RTOS 符号，
        // 否则 App 独立链接时 rtos_msleep/rtos_task_create_rt 找不到定义）。
        unsafe {
            spawn_rt(
                b"demo_app\0",
                demo_task_entry,
                RTOS_PRIO_BH_MED, // 中优先，不抢硬实时
                DEMO_STACK.as_mut_ptr(),
                DEMO_STACK.len(),
                1, // priv=1：App 任务保持特权，与正式 flyctrl 一致
                RT_NONE,
                0,
                0,
            );
        }
        info!(tag: "demo", "demo task spawned (link verified)");
    }
}

/* ===========================================================================
 * panic 处理：abort -> UDF，触发内核 fault handler（飞控可据此联动 WDT）
 * =========================================================================== */
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // 触发 UDF 未定义指令异常，由 RTOS fault handler 捕获/恢复。
    unsafe { core::arch::asm!("udf #0", options(noreturn)) };
}
