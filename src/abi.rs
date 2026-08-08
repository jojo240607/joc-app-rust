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
