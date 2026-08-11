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
        //  - 默认：正式飞控多任务（采样/控制/遥测/监控，经 RTOS 设备 vtable）。
        #[cfg(feature = "demo")]
        crate::demo::spawn_demo();
        #[cfg(not(feature = "demo"))]
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
    use crate::info;
    use crate::rtos_sync::{spawn_rt, RT_NONE};
    use core::ffi::c_void;

    // demo 任务独立栈（放 App RAM，1KB 足够周期日志）。
    #[link_section = ".rust_bss"]
    static mut DEMO_STACK: [u8; 1024] = [0u8; 1024];

    extern "C" fn demo_task_entry(_arg: *mut c_void) {
        let mut n: u32 = 0;
        loop {
            n = n.wrapping_add(1);
            info!(tag: "demo", "demo task alive seq={} ticks={}", n, crate::rtos_sync::tick_count());
            crate::rtos_sync::msleep(500);
        }
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
