//! joc-app-rust：独立 Rust 应用层（飞控）。
//! 通过 abi/device/ioctl 模块调用 RTOS ABI 契约；不依赖 rtos.h 内部。
//!
//! 挂载方式：RTOS 侧 app_main 在控制台循环前调用 `rust_app_start()`，
//! 本层在内部用 ABI 契约 `rtos_task_create_rt` 自行创建飞控硬实时任务。
//! 任务本身（EKF + PID + FDIR + MAVLink 遥测）见 `flyctrl_task` 模块。

#![no_std]
#![allow(static_mut_refs)]

pub mod abi;
pub mod device;
pub mod ioctl;
pub mod sensors;
pub mod flyctrl_task;

use abi::*;

/* ===========================================================================
 * 统一挂载点：由 RTOS app_main 经 g_app_slot.app_start 调用（rust_app_start）
 * 本层只引用 g_app_slot 服务表，不碰任何裸 RTOS 符号——体现方案 Y 解耦：
 *  - 任务经 g_app_slot.task_create_rt 创建；
 *  - 驱动 IO 经 device vtable（dev_get/open/read/write/ioctl/close）。
 * 在此仅做版本校验并拉起飞控任务，RTOS C 侧不再包含任何 app 逻辑。
 * =========================================================================== */
#[no_mangle]
pub extern "C" fn rust_app_start() -> i32 {
    unsafe {
        let slot = &mut *core::ptr::addr_of_mut!(g_app_slot);

        // 双重防御：版本不符直接拒绝挂载（build.rs 已做链接期校验）
        if slot.magic != APP_SLOT_MAGIC || slot.version != RTOS_ABI_VERSION as u32 {
            return -1;
        }

        // 拉起飞控硬实时任务：EKF + PID + FDIR + MAVLink 遥测（经 RTOS 设备 vtable）。
        // 若 RTOS 尚未提供 imu/pwm/uart 设备节点，任务自动降级为模拟源，链路仍可验证。
        crate::flyctrl_task::spawn_flyctrl_task();

        0
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
