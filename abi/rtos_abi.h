#ifndef RTOS_ABI_H
#define RTOS_ABI_H

/* ===========================================================================
 * jOS RTOS <-> Rust 应用层 ABI 契约（稳定子集）
 *
 * 这是 RTOS 暴露给独立 Rust 工程（joc-app-rust）的唯一接口。Rust 侧把本文件
 * 镜像成 rtos_abi.rs（手写或 cbindgen 生成），不依赖 rtos.h / device.h 内部。
 *
 * 约定：
 *  - 所有函数 extern "C"，调用约定与 C 侧一致（thumbv7em-none-eabihf + hard-float）。
 *  - 结构体均 #[repr(C)]，字段顺序与本文件严格一致；字段变更必须 +RTOS_ABI_VERSION。
 *  - 驱动操作只经 device vtable（见 device 接口段），Rust 不碰裸寄存器。
 *  - 硬实时任务经 rtos_task_create_rt 创建，prio 必须 <= RTOS_PRIO_BH_HIGH。
 * ========================================================================= */

#include <stdint.h>
#include <stddef.h>

/* 契约版本：任何结构体字段/签名变更都必须 +1；Rust 侧 build.rs 比对，不符则失败。 */
#define RTOS_ABI_VERSION 1

/* ---- 优先级常量（来自 rtos.h / rtos_config.h 的公开档位） ---- */
#define RTOS_PRIO_BH_HIGH 4    /* 硬实时任务上限：prio <= 此值才允许 rt_class=RTOS_RT_HARD */
#define RTOS_PRIO_BH_MED  6
#define RTOS_PRIO_MAIN    12
#define RTOS_PRIO_BLINK   14
#define RTOS_PRIO_BIST    24
#define RTOS_PRIO_IDLE    31

/* ---- 硬实时类别（rtos_task_attr_t.rt_class） ---- */
#define RTOS_RT_NONE  0
#define RTOS_RT_HARD  1
#define RTOS_RT_SOFT  2

/* ===========================================================================
 * 任务管理
 * ========================================================================= */
typedef void (*rtos_task_entry_t)(void *arg);

/* 非实时/通用任务创建（等价 rtos.h rtos_task_create） */
void rtos_task_create(const char *name, rtos_task_entry_t entry, void *arg,
                       uint8_t prio, void *stack, size_t stack_size);

/* 硬实时任务创建（等价 rtos.h rtos_task_create_rt）。
 * priv: 1=特权（飞控关键任务推荐，直接经 device vtable 操作外设、最低延迟）。
 * attr: 指向 rtos_task_attr_t；rt_class=RTOS_RT_HARD 时 prio 必须 <= RTOS_PRIO_BH_HIGH。 */
typedef struct {
    uint8_t  rt_class;        /* RTOS_RT_NONE / RTOS_RT_HARD / RTOS_RT_SOFT */
    uint32_t deadline_ticks;  /* 相对释放的最坏完成期限(0=无) */
    uint32_t wcet_ticks;      /* 单次运行最坏预算(0=不限)，超出即 WCET 违约 */
} rtos_task_attr_t;
void rtos_task_create_rt(const char *name, rtos_task_entry_t entry, void *arg,
                          uint8_t prio, void *stack, size_t stack_size,
                          uint8_t priv, const rtos_task_attr_t *attr);

void rtos_msleep(uint32_t ms);
uint32_t rtos_tick_count(void);

/* 高精度周期计数（DWT CYCCNT @HCLK，不受 BASEPRI 影响），供飞控测延迟/相位补偿。 */
uint32_t rtos_cycle_now(void);

/* ===========================================================================
 * IPC 原语（信号量 / 互斥量 / 消息队列 / 事件标志）
 * 结构体精简声明：隐藏 task_t* 等内核内部类型，用 void* 代替，避免 Rust 侧依赖。
 * 字段顺序与 rtos.h 一致，字段变更须 +RTOS_ABI_VERSION。
 * ========================================================================= */
typedef struct {
    uint32_t count;
    uint32_t limit;
    void    *waitq;
} rtos_sem_t;
void rtos_sem_init(rtos_sem_t *s, uint32_t initial, uint32_t limit);
int  rtos_sem_wait(rtos_sem_t *s);     /* 阻塞直到有许可 */
int  rtos_sem_trywait(rtos_sem_t *s);  /* 非阻塞，无许可返回 -1 */
void rtos_sem_give(rtos_sem_t *s);     /* 释放许可（ISR 安全） */

typedef struct {
    void    *owner;       /* task_t*（内核内部），ABI 侧视为 opaque */
    uint8_t  ceil_prio;
    uint8_t  recursive;
    uint8_t  rec_count;
    void    *waitq;
} rtos_mutex_t;
void rtos_mutex_init(rtos_mutex_t *m, uint8_t ceil_prio);
void rtos_mutex_init_rec(rtos_mutex_t *m, uint8_t ceil_prio);
int  rtos_mutex_lock(rtos_mutex_t *m);     /* 阻塞直到获得；自锁返回 -1 */
int  rtos_mutex_trylock(rtos_mutex_t *m);  /* 非阻塞 */
int  rtos_mutex_unlock(rtos_mutex_t *m);   /* 释放；非持有者返回 -1 */
int  rtos_mutex_timedlock(rtos_mutex_t *m, uint32_t timeout_ms);

typedef struct {
    uint8_t *buf;
    size_t   item_size;
    size_t   cap;
    size_t   count;
    size_t   head;
    void    *recv_waitq;
    void    *send_waitq;
} rtos_mq_t;
void rtos_mq_init(rtos_mq_t *q, void *buf, size_t item_size, size_t cap);
int  rtos_mq_send(rtos_mq_t *q, const void *item);       /* 满则阻塞 */
int  rtos_mq_trysend(rtos_mq_t *q, const void *item);    /* 满返回 -1 */
int  rtos_mq_recv(rtos_mq_t *q, void *item);             /* 空则阻塞 */
int  rtos_mq_tryrecv(rtos_mq_t *q, void *item);          /* 空返回 -1 */
int  rtos_mq_send_fromisr(rtos_mq_t *q, const void *item); /* ISR 安全发送 */

typedef struct {
    uint32_t flags;
    void    *waitq;
} rtos_event_t;
void rtos_event_init(rtos_event_t *e);
void rtos_event_set(rtos_event_t *e, uint32_t bits);
void rtos_event_clear(rtos_event_t *e, uint32_t bits);
/* 阻塞等待：wait_all=1 要求 mask 所有位，否则任意一位；block=0 非阻塞。
 * 成功返回当前 flags；非阻塞且未满足返回 (uint32_t)-1。 */
uint32_t rtos_event_wait(rtos_event_t *e, uint32_t mask, int wait_all, int block);

/* ===========================================================================
 * 硬实时观测 / 锁调度
 * ========================================================================= */
/* 硬实时违约汇总（OR 所有任务）：g_rtos_deadline_violation | g_rtos_wcet_violation。
 * 飞控健康监控任务可周期读取；非 0 即存在违约，应联动看门狗。 */
uint32_t rtos_rt_violation(void);
extern volatile uint32_t g_rtos_deadline_violation;  /* OR 所有硬实时任务违约 */

/* 锁调度（非抢占临界区）：只屏蔽 PendSV，保护原子外设序列，不被时间片打断。
 * 仅在任务上下文调用；持锁时长会被审计。 */
void rtos_lock_scheduler(void);
void rtos_unlock_scheduler(void);

/* ===========================================================================
 * 设备接口（来自 iface/device.h + devmgr）—— Rust 操作驱动的唯一入口
 * 精简镜像：device 内部类型 opaque 化；驱动私有 ioctl 命令见 rtos_abi_ioctl.h。
 * ========================================================================= */
typedef struct device device;
typedef struct deviceVtable {
    int (*open)(device *self);                        /* bring up / (re)configure */
    int (*close)(device *self);                       /* tear down */
    int (*read)(device *self, void *buf, size_t len); /* bytes read, <0 on error */
    int (*write)(device *self, const void *buf, size_t len); /* bytes written, <0 */
    int (*ioctl)(device *self, int cmd, void *arg);   /* device-specific control */
    int (*irq_id)(device *self);                      /* chip IRQ line, or -1 */
} deviceVtable;

/* 设备类枚举（用于应用侧分类，值与原 device.h 一致） */
typedef enum {
    DEVICE_TYPE_CLOCK = 0,  DEVICE_TYPE_ADC,  DEVICE_TYPE_UART, DEVICE_TYPE_GPIO,
    DEVICE_TYPE_TEMP_SENSOR, DEVICE_TYPE_PINMUX, DEVICE_TYPE_SYSTICK, DEVICE_TYPE_TIMER,
    DEVICE_TYPE_PWM, DEVICE_TYPE_EXTI, DEVICE_TYPE_I2C, DEVICE_TYPE_SPI,
    DEVICE_TYPE_SDIO, DEVICE_TYPE_SD_CARD, DEVICE_TYPE_DAC, DEVICE_TYPE_RTC,
    DEVICE_TYPE_RNG, DEVICE_TYPE_CRC, DEVICE_TYPE_IWDG, DEVICE_TYPE_WWDG,
    DEVICE_TYPE_FLASH, DEVICE_TYPE_I2S, DEVICE_TYPE_CAN, DEVICE_TYPE_USB,
    DEVICE_TYPE_DMA, DEVICE_TYPE_COUNT
} driver_type_t;

typedef enum {
    DEVICE_CLASS_STREAM = 0, DEVICE_CLASS_BLOCK, DEVICE_CLASS_EVENT, DEVICE_CLASS_CONTROL,
    DEVICE_CLASS_COUNT
} device_class_t;

struct device {
    const deviceVtable *vtable;
    driver_type_t       type;
    const char         *name;
    device_class_t      class;
};

/* 按名查找设备；返回 NULL 表示未注册。等价 device_manager_get。 */
device *device_manager_get(const char *name);

#endif /* RTOS_ABI_H */
