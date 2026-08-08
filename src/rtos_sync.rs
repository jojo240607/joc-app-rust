//! RTOS 同步原语 + 任务创建 helper 的 Rust 薄封装。
//!
//! 关键：所有 RTOS 内核服务都经 `g_app_slot` vtable 的函数指针间接调用
//! （系统区固件在 0x2001DC00 填好），**绝不**直接链接 extern "C" 符号——
//! 应用分区独立链接时没有 RTOS 实现，直接 extern 调用会链接失败。
//!
//! 互斥量：ABI 契约未提供 mutex 服务，这里用「二值信号量」(sem_init 初值 1)
//! 充当互斥量，语义等价（临界区串行化）。注意这没有优先级天花板，飞控任务
//! 优先级档位已拉开（control=4 < sensors=5 < telem=12 < monitor=14），
//! 持锁路径极短，可接受。若需严格天花板，待 joc-base ABI 扩展 mutex 服务。

use core::ffi::{c_char, c_void};
use core::ptr::{addr_of, null_mut};

use crate::abi::{
    g_app_slot, rtos_sem_t, rtos_task_attr_t, rtos_task_entry_t, RTOS_RT_HARD, RTOS_RT_NONE,
    RTOS_RT_SOFT,
};

/// 互斥量（基于 RTOS 二值信号量）。
///
/// 用法：在 `rust_app_start` 里对每个静态 Mutex 调一次 `init()`，
/// 之后各任务 `lock()`/`unlock()`。提供 RAII `guard()`。
pub struct Mutex {
    sem: rtos_sem_t,
}

// rtos_sem_t 含裸指针，Rust 视为 !Sync/!Send；但 RTOS 单核多任务下我们手工保证
// 只通过地址访问，且锁语义保证互斥，故显式标注 Sync/Send。
unsafe impl Sync for Mutex {}
unsafe impl Send for Mutex {}

impl Mutex {
    /// 编译期零初始化的未初始化互斥量；必须先 `init()` 才能使用。
    pub const fn uninit() -> Self {
        Mutex {
            sem: rtos_sem_t {
                count: 0,
                limit: 0,
                waitq: null_mut(),
            },
        }
    }

    /// 运行时初始化（只调一次）：二值信号量初值 1、上限 1。
    ///
    /// 注意：必须接收 `&self` 并经指针转换把 `self.sem` 地址传给 `sem_init`，
    /// 不能取 `&mut self` 后写 `self.sem` —— 因为 `Mutex` 实例是 `static mut`，
    /// 对 `static mut` 取 `&mut` 违反别名规则（Rust UB），会导致编译器把
    /// 第二次 `init` 的写入优化掉（实测：SENSOR_MTX init 生效、EST_MTX 失效，
    /// 表现为后建的互斥量 sem 仍为全 0 → sem_wait 永久阻塞 → 高优先任务饿死其余任务）。
    /// 这里只做 `*const → *mut` 的指针转换（不创建 Rust 的 `&mut`），与 `lock`/`unlock` 一致。
    pub fn init(&self, _ceil_prio: u8) {
        unsafe {
            if let Some(f) = (*addr_of!(g_app_slot)).sem_init {
                f(&self.sem as *const rtos_sem_t as *mut rtos_sem_t, 1, 1);
            }
        }
    }

    /// 加锁（阻塞直到获得）。
    #[inline]
    pub fn lock(&self) {
        unsafe {
            if let Some(f) = (*addr_of!(g_app_slot)).sem_wait {
                f(&self.sem as *const rtos_sem_t as *mut rtos_sem_t);
            }
        }
    }

    /// 解锁。
    #[inline]
    pub fn unlock(&self) {
        unsafe {
            if let Some(f) = (*addr_of!(g_app_slot)).sem_give {
                f(&self.sem as *const rtos_sem_t as *mut rtos_sem_t);
            }
        }
    }

    /// RAII 守卫：离开作用域自动解锁。
    pub fn guard(&self) -> MutexGuard<'_> {
        self.lock();
        MutexGuard { m: self }
    }

    /// 诊断：返回当前 sem 计数（确认 init 是否生效）。
    pub fn debug_count(&self) -> u32 {
        unsafe { (*addr_of!(self.sem)).count }
    }
}

/// MutexGuard：drop 时自动 unlock。
pub struct MutexGuard<'a> {
    m: &'a Mutex,
}

impl<'a> Drop for MutexGuard<'a> {
    fn drop(&mut self) {
        self.m.unlock();
    }
}

/// 经 g_app_slot 的 RTOS msleep（SysTick 1000Hz 驱动调度）。
#[inline]
pub fn msleep(ms: u32) {
    unsafe {
        if let Some(f) = (*addr_of!(g_app_slot)).msleep {
            f(ms);
        }
    }
}

/// 经 g_app_slot 的 RTOS tick_count（系统启动后 tick 数）。
#[inline]
pub fn tick_count() -> u32 {
    unsafe {
        if let Some(f) = (*addr_of!(g_app_slot)).tick_count {
            f()
        } else {
            0
        }
    }
}

/// 创建并启动一个 RTOS 任务。
///
/// - `name`：以 `\0` 结尾的 ASCII 名。
/// - `entry`：`extern "C" fn(*mut c_void)` 入口。
/// - `prio`：优先级档位。
/// - `stack`/`stack_size`：调用方提供的静态栈缓冲。
/// - `priv_`：1=特权。
/// - `rt_class`：RTOS_RT_NONE / RTOS_RT_HARD / RTOS_RT_SOFT（HARD 时 prio ≤ RTOS_PRIO_BH_HIGH）。
/// - `deadline_ticks`/`wcet_ticks`：硬实时最坏期限/预算。
pub fn spawn_rt(
    name: &[u8],
    entry: rtos_task_entry_t,
    prio: u8,
    stack: *mut u8,
    stack_size: usize,
    priv_: u8,
    rt_class: u8,
    deadline_ticks: u32,
    wcet_ticks: u32,
) {
    unsafe {
        if let Some(f) = (*addr_of!(g_app_slot)).task_create_rt {
            let attr = rtos_task_attr_t {
                rt_class,
                deadline_ticks,
                wcet_ticks,
            };
            f(
                name.as_ptr() as *const c_char,
                entry,
                null_mut(),
                prio,
                stack as *mut c_void,
                stack_size,
                priv_,
                &attr as *const rtos_task_attr_t,
            );
        }
    }
}

/// 硬实时任务档位（prio ≤ RTOS_PRIO_BH_HIGH）。
pub const RT_HARD: u8 = RTOS_RT_HARD;
/// 软实时任务档位（prio > RTOS_PRIO_BH_HIGH）。
pub const RT_SOFT: u8 = RTOS_RT_SOFT;
/// 普通任务档位。
pub const RT_NONE: u8 = RTOS_RT_NONE;
