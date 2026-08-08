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
pub mod flyctrl_task;

use core::ffi::{c_char, c_void};

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

/* 硬实时姿态环所用的 TIM 中断线（示例：TIM6_DAC_IRQn=54）。
 * 后续阶段应由设备 vtable 的 irq_id() 取得，而非硬编码；此处先落服务表
 * 注册机制，真实 TIM 周期触发配置经 device "tim0" ioctl 接入。 */
const TIMX_IRQN: u8 = 54;
static mut RUST_TICKS: u32 = 0;

/* ===========================================================================
 * 统一挂载点：由 RTOS app_main 经 g_app_slot.app_start 调用（rust_app_start）
 * 本层只引用 g_app_slot 服务表，不碰任何裸 RTOS 符号——体现方案 Y 解耦：
 *  - 任务经 g_app_slot.task_create / task_create_rt 创建；
 *  - 中断回调经 g_app_slot.irq_reg[0] 注册位 + irq_attach 落真实路由
 *    （irq_manager 统一分发，App 不碰 NVIC/VTOR）。
 * 在此自行创建所有 demo / 飞控任务，RTOS C 侧不再包含任何 demo 逻辑。
 * =========================================================================== */
#[no_mangle]
pub extern "C" fn rust_app_start() -> i32 {
    unsafe {
        let slot = &mut *core::ptr::addr_of_mut!(g_app_slot);

        // 双重防御：版本不符直接拒绝挂载（build.rs 已做链接期校验）
        if slot.magic != APP_SLOT_MAGIC || slot.version != RTOS_ABI_VERSION as u32 {
            return -1;
        }

        // 1) 注册硬实时中断回调：TIMx -> att_isr_give -> ATT_SEM
        //    App 只填「注册位」，物理接线由系统侧 app_slot_irq_attach 完成。
        let reg = &mut slot.irq_reg[0];
        reg.used = 1;
        reg.irq_id = TIMX_IRQN;            // App 只知道逻辑 id，不碰 NVIC
        reg.prio_class = IRQ_CLASS_KERNEL as u8;
        reg.rt_class = 1;                  // 硬实时，与现有 att_rust 一致
        reg.isr_cb = Some(att_isr_give);
        reg.ctx = &raw mut ATT_SEM as *mut c_void;
        if let Some(attach) = slot.irq_attach {
            attach(reg as *const app_irq_reg_t);
        }

        // 2) demo 心跳任务（普通任务，prio=14）经服务表创建
        if let Some(tc) = slot.task_create {
            tc(
                b"rust_demo\0".as_ptr() as *const _,
                rust_task_entry,
                core::ptr::null_mut(),
                RTOS_PRIO_BLINK,
                &raw mut RUST_DEMO_STACK as *mut _ as *mut c_void,
                core::mem::size_of::<Stack1024>(),
            );
        }

        // 3) 飞控 1kHz 硬实时姿态环（prio=3 <= RTOS_PRIO_BH_HIGH，priv=1）
        let attr = rtos_task_attr_t {
            rt_class: RTOS_RT_HARD,
            deadline_ticks: 1,
            wcet_ticks: 0,
        };
        if let Some(tc) = slot.task_create_rt {
            tc(
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

        // 4) 启动硬实时时钟源：打开 timer2(TIM6, IRQ54) 经服务表 ioctl ENABLE。
        //    这一步是飞控真活的关键——TIM6 默认不计数、不产生溢出 IRQ，
        //    att_isr_give 就永远不被调用、ATT_SEM 永不释放、att_rust 会永久
        //    阻塞在 sem_wait。ENABLE 让 TIM6 以板级配置的 20Hz 周期溢出，
        //    驱动 ISR 清 UIF、同 IRQ 线上的 att_isr_give 仅 give 信号量。
        if let (Some(dg), Some(do_), Some(dio)) =
            (slot.dev_get, slot.dev_open, slot.dev_ioctl)
        {
            let tim = dg(b"timer2\0".as_ptr() as *const c_char);
            if !tim.is_null() {
                if do_(tim) == 0 {
                    let _ = dio(tim, ioctl::TIMER_IOCTL_ENABLE, core::ptr::null_mut());
                }
            }
        }

        // 5) 接入真实飞控任务：EKF + PID + FDIR + MAVLink 遥测（经 RTOS 设备 vtable）。
        //    若 RTOS 尚未提供 imu/pwm 设备节点，任务自动降级为模拟源，链路仍可验证。
        crate::flyctrl_task::spawn_flyctrl_task();

        0
    }
}

/* ===========================================================================
 * demo 心跳任务入口：每 500ms 自增 RUST_TICKS 并经 uart0 打印一行，
 * 作为「App 层真在跑」的可观测证据（主机/USB CDC 控制台可见）。
 * =========================================================================== */
#[no_mangle]
pub extern "C" fn rust_task_entry(_arg: *mut c_void) {
    unsafe {
        // 经服务表打开 uart0（板级调试串口），失败则仍静默自增。
        let mut uart: *mut c_void = core::ptr::null_mut();
        let mut have_uart = false;
        {
            let slot = &*core::ptr::addr_of!(g_app_slot);
            if let (Some(dg), Some(do_)) = (slot.dev_get, slot.dev_open) {
                let d = dg(b"uart0\0".as_ptr() as *const c_char);
                if !d.is_null() && do_(d) == 0 {
                    uart = d as *mut c_void;
                    have_uart = true;
                }
            }
        }

        loop {
            RUST_TICKS = RUST_TICKS.wrapping_add(1);
            if have_uart {
                let slot = &*core::ptr::addr_of!(g_app_slot);
                if let Some(dw) = slot.dev_write {
                    let mut line = [0u8; 48];
                    let n = fmt_ticks(RUST_TICKS, &mut line);
                    let _ = dw(uart as *mut device_t, line.as_ptr() as *const c_void, n as usize);
                }
            }
            if let Some(ms) = (*core::ptr::addr_of!(g_app_slot)).msleep {
                ms(500);
            }
        }
    }
}

/// 最小格式化：把 u32 拼成 "RUST ticks=N\r\n"（不依赖核心格式化，零分配）。
fn fmt_ticks(v: u32, out: &mut [u8]) -> usize {
    let mut tmp = [0u8; 10];
    let mut i = 0;
    let mut n = v;
    if n == 0 {
        tmp[i] = b'0';
        i += 1;
    } else {
        while n > 0 {
            tmp[i] = b'0' + (n % 10) as u8;
            n /= 10;
            i += 1;
        }
    }
    // 倒序拼到 out
    let mut p = 0;
    let pre = b"RUST ticks=";
    for &c in pre {
        if p < out.len() { out[p] = c; p += 1; }
    }
    while i > 0 {
        i -= 1;
        if p < out.len() { out[p] = tmp[i]; p += 1; }
    }
    for &c in b"\r\n" {
        if p < out.len() { out[p] = c; p += 1; }
    }
    p
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

/// 由系统侧 irq_manager 经 g_app_slot.irq_reg[0] 注册位分发（TIM 1kHz 溢出时）。
/// 上半部仅 sem_give（ISR 安全），ctx 指向 ATT_SEM。走服务表 sem_give。
#[no_mangle]
pub extern "C" fn att_isr_give(ctx: *mut c_void) {
    unsafe {
        let sem = ctx as *mut rtos_sem_t;
        if let Some(g) = (*core::ptr::addr_of!(g_app_slot)).sem_give {
            g(sem);
        }
    }
}

/// 飞控姿态环任务入口（硬实时，prio=3 <= RTOS_PRIO_BH_HIGH）。
#[no_mangle]
pub extern "C" fn rust_attitude_loop(_arg: *mut c_void) {
    unsafe {
        // 经服务表初始化 ATT_SEM（方案 Y：不碰裸 rtos_sem_init 符号）
        if let Some(si) = (*core::ptr::addr_of!(g_app_slot)).sem_init {
            si(&raw mut ATT_SEM as *mut rtos_sem_t, 0, 64);
        }

        // 取设备句柄（经 device vtable，符合方案 Y 的「驱动只经 device」约定）
        let adc = Device::get("adc0");
        let pwm = Device::get("pwm0");

        let mut samples: [u8; 8] = [0; 8];
        let mut duty: i32 = 50;

        loop {
            // 经服务表等待周期信号量（TIM6 溢出 -> att_isr_give 释放）
            if let Some(sw) = (*core::ptr::addr_of!(g_app_slot)).sem_wait {
                sw(&raw mut ATT_SEM as *mut rtos_sem_t);
            }

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
