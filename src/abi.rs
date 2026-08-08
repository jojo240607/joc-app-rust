//! 手写镜像 rtos_abi.h（RTOS_ABI_VERSION 必须与此处一致，由 build.rs 校验）。
//! 字段顺序、调用约定须与 C 侧严格一致；C 侧字段变更须同步本文件并 +RTOS_ABI_VERSION。

#![allow(non_snake_case)]
#![allow(non_camel_case_types)]

use core::ffi::{c_char, c_void};

/* ---- 优先级常量 ---- */
pub const RTOS_PRIO_BH_HIGH: u8 = 4;
pub const RTOS_PRIO_BH_MED: u8 = 6;
pub const RTOS_PRIO_MAIN: u8 = 12;
pub const RTOS_PRIO_BLINK: u8 = 14;
pub const RTOS_PRIO_BIST: u8 = 24;
pub const RTOS_PRIO_IDLE: u8 = 31;

/* ---- 硬实时类别 ---- */
pub const RTOS_RT_NONE: u8 = 0;
pub const RTOS_RT_HARD: u8 = 1;
pub const RTOS_RT_SOFT: u8 = 2;

/* ---- 任务 ---- */
pub type rtos_task_entry_t = extern "C" fn(*mut c_void);

#[repr(C)]
pub struct rtos_task_attr_t {
    pub rt_class: u8,
    pub deadline_ticks: u32,
    pub wcet_ticks: u32,
}

extern "C" {
    pub fn rtos_task_create(
        name: *const c_char,
        entry: rtos_task_entry_t,
        arg: *mut c_void,
        prio: u8,
        stack: *mut c_void,
        stack_size: usize,
    );
    pub fn rtos_task_create_rt(
        name: *const c_char,
        entry: rtos_task_entry_t,
        arg: *mut c_void,
        prio: u8,
        stack: *mut c_void,
        stack_size: usize,
        priv_: u8,
        attr: *const rtos_task_attr_t,
    );
    pub fn rtos_msleep(ms: u32);
    pub fn rtos_tick_count() -> u32;
    pub fn rtos_cycle_now() -> u32;
}

/* ---- IPC: 信号量 ---- */
#[repr(C)]
pub struct rtos_sem_t {
    pub count: u32,
    pub limit: u32,
    pub waitq: *mut c_void,
}

/* ---- IPC: 互斥量 ---- */
#[repr(C)]
pub struct rtos_mutex_t {
    pub owner: *mut c_void, // task_t* opaque
    pub ceil_prio: u8,
    pub recursive: u8,
    pub rec_count: u8,
    pub waitq: *mut c_void,
}

/* ---- IPC: 消息队列 ---- */
#[repr(C)]
pub struct rtos_mq_t {
    pub buf: *mut u8,
    pub item_size: usize,
    pub cap: usize,
    pub count: usize,
    pub head: usize,
    pub recv_waitq: *mut c_void,
    pub send_waitq: *mut c_void,
}

/* ---- IPC: 事件标志 ---- */
#[repr(C)]
pub struct rtos_event_t {
    pub flags: u32,
    pub waitq: *mut c_void,
}

extern "C" {
    pub fn rtos_sem_init(s: *mut rtos_sem_t, initial: u32, limit: u32);
    pub fn rtos_sem_wait(s: *mut rtos_sem_t) -> i32;
    pub fn rtos_sem_trywait(s: *mut rtos_sem_t) -> i32;
    pub fn rtos_sem_give(s: *mut rtos_sem_t);

    pub fn rtos_mutex_init(m: *mut rtos_mutex_t, ceil_prio: u8);
    pub fn rtos_mutex_init_rec(m: *mut rtos_mutex_t, ceil_prio: u8);
    pub fn rtos_mutex_lock(m: *mut rtos_mutex_t) -> i32;
    pub fn rtos_mutex_trylock(m: *mut rtos_mutex_t) -> i32;
    pub fn rtos_mutex_unlock(m: *mut rtos_mutex_t) -> i32;
    pub fn rtos_mutex_timedlock(m: *mut rtos_mutex_t, timeout_ms: u32) -> i32;

    pub fn rtos_mq_init(q: *mut rtos_mq_t, buf: *mut c_void, item_size: usize, cap: usize);
    pub fn rtos_mq_send(q: *mut rtos_mq_t, item: *const c_void) -> i32;
    pub fn rtos_mq_trysend(q: *mut rtos_mq_t, item: *const c_void) -> i32;
    pub fn rtos_mq_recv(q: *mut rtos_mq_t, item: *mut c_void) -> i32;
    pub fn rtos_mq_tryrecv(q: *mut rtos_mq_t, item: *mut c_void) -> i32;
    pub fn rtos_mq_send_fromisr(q: *mut rtos_mq_t, item: *const c_void) -> i32;

    pub fn rtos_event_init(e: *mut rtos_event_t);
    pub fn rtos_event_set(e: *mut rtos_event_t, bits: u32);
    pub fn rtos_event_clear(e: *mut rtos_event_t, bits: u32);
    pub fn rtos_event_wait(e: *mut rtos_event_t, mask: u32, wait_all: i32, block: i32) -> u32;

    pub fn rtos_rt_violation() -> u32;
    pub static mut g_rtos_deadline_violation: u32;

    pub fn rtos_lock_scheduler();
    pub fn rtos_unlock_scheduler();
}

/* ===========================================================================
 * app_slot_t 镜像（方案 Y 轻量版）：RTOS 系统暴露给 App 的「函数指针表 +
 * 中断回调注册位」契约。字段顺序、调用约定须与 C 侧 src/app_slot/app_slot.h
 * 严格一致。C 侧字段变更须同步本文件并 +RTOS_ABI_VERSION。
 * =========================================================================== */
pub const APP_SLOT_MAGIC: u32 = 0x4150_5053;   // "APPS"
pub const APP_IRQ_REG_MAX: usize = 8;

/* 中断类别（镜像 irq.h irq_class_t） */
pub const IRQ_CLASS_NORMAL: u8 = 0;
pub const IRQ_CLASS_KERNEL: u8 = 1;
pub const IRQ_CLASS_ZERO_LATENCY: u8 = 2;

#[repr(C)]
pub struct app_irq_reg_t {
    pub used: u8,
    pub irq_id: u8,
    pub prio_class: u8,   // 0=NORMAL,1=KERNEL,2=ZERO_LATENCY
    pub rt_class: u8,     // 0=普通,1=硬实时
    pub isr_cb: Option<extern "C" fn(*mut c_void)>,
    pub ctx: *mut c_void,
}

/* 设备 vtable 镜像（与 rtos_abi.h deviceVtable 同构；device 指针 opaque） */
#[repr(C)]
pub struct deviceVtable {
    pub open: Option<extern "C" fn(*mut c_void) -> i32>,
    pub close: Option<extern "C" fn(*mut c_void) -> i32>,
    pub read: Option<extern "C" fn(*mut c_void, *mut c_void, usize) -> i32>,
    pub write: Option<extern "C" fn(*mut c_void, *const c_void, usize) -> i32>,
    pub ioctl: Option<extern "C" fn(*mut c_void, i32, *mut c_void) -> i32>,
    pub irq_id: Option<extern "C" fn(*mut c_void) -> i32>,
}

#[repr(C)]
pub struct device_t {
    pub vtable: *const deviceVtable,
    pub type_: u32,
    pub name: *const c_char,
    pub class: u32,
}

/* ABI 版本：与 C 侧 tools/abi/rtos_abi.h 的 RTOS_ABI_VERSION 对齐。
 * build.rs 在链接期比对两者，不一致则编译失败。 */
pub const RTOS_ABI_VERSION: u32 = 1;

pub type app_slot_irq_attach_t = extern "C" fn(*const app_irq_reg_t) -> i32;

#[repr(C)]
pub struct app_slot_t {
    pub magic: u32,
    pub version: u32,
    pub reserved: u32,

    /* 内核服务 */
    pub task_create: Option<
        extern "C" fn(
            *const c_char,
            rtos_task_entry_t,
            *mut c_void,
            u8,
            *mut c_void,
            usize,
        ),
    >,
    pub task_create_rt: Option<
        extern "C" fn(
            *const c_char,
            rtos_task_entry_t,
            *mut c_void,
            u8,
            *mut c_void,
            usize,
            u8,
            *const rtos_task_attr_t,
        ),
    >,
    pub msleep: Option<extern "C" fn(u32)>,
    pub tick_count: Option<extern "C" fn() -> u32>,
    pub cycle_now: Option<extern "C" fn() -> u32>,

    /* IPC 服务 */
    pub sem_init: Option<extern "C" fn(*mut rtos_sem_t, u32, u32)>,
    pub sem_wait: Option<extern "C" fn(*mut rtos_sem_t) -> i32>,
    pub sem_trywait: Option<extern "C" fn(*mut rtos_sem_t) -> i32>,
    pub sem_give: Option<extern "C" fn(*mut rtos_sem_t)>,

    /* 设备服务 */
    pub dev_get: Option<extern "C" fn(*const c_char) -> *mut device_t>,
    pub dev_open: Option<extern "C" fn(*mut device_t) -> i32>,
    pub dev_read: Option<extern "C" fn(*mut device_t, *mut c_void, usize) -> i32>,
    pub dev_write: Option<extern "C" fn(*mut device_t, *const c_void, usize) -> i32>,
    pub dev_ioctl: Option<extern "C" fn(*mut device_t, i32, *mut c_void) -> i32>,
    pub dev_close: Option<extern "C" fn(*mut device_t) -> i32>,

    /* 中断回调注册位 */
    pub irq_reg: [app_irq_reg_t; APP_IRQ_REG_MAX],

    /* 系统注册入口 */
    pub irq_attach: Option<app_slot_irq_attach_t>,
    pub irq_enable: Option<extern "C" fn(u8) -> i32>,
    pub irq_disable: Option<extern "C" fn(u8) -> i32>,

    /* 生命周期 */
    pub app_start: Option<extern "C" fn() -> i32>,
    pub app_stop: Option<extern "C" fn()>,
}

/* 系统在固定链接地址定义的实例；App 经 extern 引用，不可自行定义。 */
extern "C" {
    pub static mut g_app_slot: app_slot_t;
}
