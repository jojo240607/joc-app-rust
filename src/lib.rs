//! joc-app-rust：独立 Rust 应用层（飞控示例 + 驱动操作）。
//! 通过 abi/device/ioctl 模块调用 RTOS ABI 契约；不依赖 rtos.h 内部。
//!
//! 挂载方式：RTOS 侧 app_main 在控制台循环前调用 `rust_app_start(ctx)`，
//! 本层在内部用 ABI 契约 `rtos_task_create` / `rtos_task_create_rt` 自行创建
//! demo 与飞控任务。任务栈由本层静态数组提供（位于主 SRAM，避免占用 CCM）。

#![no_std]
#![allow(static_mut_refs)]

pub mod abi;
pub mod device;
pub mod ioctl;

use core::ffi::c_void;

use abi::*;
use device::Device;

/* ===========================================================================
 * 任务栈（主 SRAM 静态数组，repr(align) 满足 RTOS 栈对齐要求；不进 CCM）
 * =========================================================================== */
#[repr(align(8))]
struct Stack1024 {
    #[allow(dead_code)]
    buf: [u8; 1024],
}
#[repr(align(8))]
struct Stack512 {
    #[allow(dead_code)]
    buf: [u8; 512],
}
static mut RUST_DEMO_STACK: Stack1024 = Stack1024 { buf: [0u8; 1024] };
static mut RUST_ATT_STACK: Stack512 = Stack512 { buf: [0u8; 512] };

/* ===========================================================================
 * 全局状态（裸指针，避免 static_mut 警告）
 * =========================================================================== */
static mut RUST_SEM: rtos_sem_t = rtos_sem_t {
    count: 0,
    limit: 64,
    waitq: core::ptr::null_mut(),
};
static mut RUST_TICKS: u32 = 0;

/* ===========================================================================
 * 统一挂载点：由 RTOS app_main 调用（rust_app_start）
 * 在此自行创建所有 demo / 飞控任务，RTOS C 侧不再包含任何 demo 逻辑。
 * =========================================================================== */
#[no_mangle]
pub extern "C" fn rust_app_start(_ctx: *mut c_void) {
    unsafe {
        rtos_sem_init(&raw mut RUST_SEM, 0, 64);

        // demo 心跳任务（普通任务，prio=14）
        rtos_task_create(
            b"rust_demo\0".as_ptr() as *const _,
            rust_task_entry,
            core::ptr::null_mut(),
            RTOS_PRIO_BLINK,
            &raw mut RUST_DEMO_STACK as *mut _ as *mut c_void,
            core::mem::size_of::<Stack1024>(),
        );

        // 飞控 1kHz 硬实时姿态环（prio=3 <= RTOS_PRIO_BH_HIGH，priv=1）
        let attr = rtos_task_attr_t {
            rt_class: RTOS_RT_HARD,
            deadline_ticks: 1,
            wcet_ticks: 0,
        };
        rtos_task_create_rt(
            b"att_rust\0".as_ptr() as *const _,
            rust_attitude_loop,
            core::ptr::null_mut(),
            RTOS_PRIO_BH_HIGH - 1,
            &raw mut RUST_ATT_STACK as *mut _ as *mut c_void,
            core::mem::size_of::<Stack512>(),
            1,
            &attr as *const rtos_task_attr_t,
        );
    }
}

/* ===========================================================================
 * demo 心跳任务入口
 * =========================================================================== */
#[no_mangle]
pub extern "C" fn rust_task_entry(_arg: *mut c_void) {
    unsafe {
        loop {
            // 周期心跳：每 500ms 自增一次，供 RUST 命令读取验证任务存活。
            RUST_TICKS = RUST_TICKS.wrapping_add(1);
            rtos_msleep(500);
        }
    }
}

/// 调试用：手动触发一次心跳脉冲（give 信号量）。保留给需要事件驱动的场景。
#[no_mangle]
pub extern "C" fn rust_wait_once() {
    unsafe { rtos_sem_give(&raw mut RUST_SEM) };
}

/// 供调试读取的心跳计数。
#[no_mangle]
pub extern "C" fn rust_ticks() -> u32 {
    unsafe { RUST_TICKS }
}

/* ===========================================================================
 * 飞控示例：1kHz 硬实时姿态环
 *  - TIM IRQ 上半部仅 sem_give（ISR 安全）；本任务经 sem 同步到周期。
 *  - 控制律纯 Rust 计算，无堆分配、固定栈，WCET 可静态分析。
 *  - 驱动 IO 经 device vtable（ADC 采样输入、PWM 输出），Rust 不碰寄存器。
 * =========================================================================== */

static mut ATT_SEM: rtos_sem_t = rtos_sem_t {
    count: 0,
    limit: 64,
    waitq: core::ptr::null_mut(),
};

/// 由 C 侧 TIM ISR 在 1kHz 溢出时调用（ISR 安全）。
#[no_mangle]
pub extern "C" fn rust_att_isr_give() {
    unsafe { rtos_sem_give(&raw mut ATT_SEM) };
}

/// 飞控姿态环任务入口（硬实时，prio=3 <= RTOS_PRIO_BH_HIGH）。
#[no_mangle]
pub extern "C" fn rust_attitude_loop(_arg: *mut c_void) {
    unsafe { rtos_sem_init(&raw mut ATT_SEM, 0, 64) };

    // 取设备句柄（板级必注册 adc0 / pwm0）
    let adc = Device::get("adc0");
    let pwm = Device::get("pwm0");

    let mut samples: [u8; 8] = [0; 8];
    let mut duty: i32 = 50;

    loop {
        unsafe { rtos_sem_wait(&raw mut ATT_SEM) };

        // 1) 采样（经 device vtable，DMA 缓冲位置由 C 驱动保证）
        if let Some(dev) = adc {
            let _ = dev.read(&mut samples);
        }

        // 2) 控制律（示例：纯算术，无分配；真实飞控在此做姿态解算 + PID）
        let raw = (samples[0] as u32) | ((samples[1] as u32) << 8);
        let _ = raw; // 占位：实际应转物理量、跑 PID
        duty = if duty < 90 { duty + 1 } else { 10 };

        // 3) 输出（经 device vtable）
        if let Some(dev) = pwm {
            let mut d = duty;
            let _ = dev.ioctl(ioctl::PWM_IOCTL_SET_DUTY_PERCENT, &mut d as *mut i32 as *mut c_void);
        }
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
