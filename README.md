# joc-app-rust

STM32F407（jOS RTOS）上的 **Rust 应用层**。与 C 固件（`joc-base`）解耦为独立工程，
通过 **干净 ABI 契约** 调用内核，不依赖 `rtos.h` / `device.h` 内部头文件。

```
┌──────────────────────────────────────────────────────────┐
│  C 固件 (joc-base)  → RTOS 内核 + 驱动 + 设备注册          │
│    app_main_task: app_slot_init() 填充函数指针表           │
│                  → g_app_slot.app_start = rust_app_start   │
│                  → app_start() 调用 App 入口              │
└───────────────┬──────────────────────────────────────────┘
                │  extern g_app_slot（app_slot_t 服务表）
                │  App 只经这张函数指针表拿能力，不碰裸 RTOS 符号
┌───────────────▼──────────────────────────────────────────┐
│  Rust 应用层 (joc-app-rust) → 飞控 demo / 业务任务         │
│    经 g_app_slot 服务表调内核/IPC/设备；                  │
│    经 irq_reg[] 注册中断回调（irq_manager 兜路由）；       │
│    不碰裸寄存器 / NVIC / VTOR                              │
│    产出 libapp.a，由 C 固件链接器统一链接                  │
└──────────────────────────────────────────────────────────┘
```

> **方案 Y 轻量版（当前落地）**：App 不直接 `extern "C"` 引用 `rtos_*` 裸符号，
> 而是通过一个固定的 **`app_slot_t` 函数指针表 + 中断注册位**（`g_app_slot`）
> 调用系统。系统侧 `app_slot_init()` 在运行时把函数指针填进表；App 只 `extern`
> 引用同一地址、填 `irq_reg[]` 并调 `app_start`。这样 App 镜像与系统 ABI 完全解耦，
> 系统升级只要 `RTOS_ABI_VERSION` 不变，App 镜像可直接复用。
> 完整设计见 `joc-base/docs/app-slot-design.md`。

---

## 1. 架构

### 1.1 工程组织

`joc-app-rust` 与 `joc-base` 是**平级独立 git 仓库**，互不内联编译：

| 路径 | 作用 |
|------|------|
| `Cargo.toml` | crate-type=`staticlib`，产出 `libapp.a`；release profile 为 `opt-level=z` + `lto` + `panic=abort` |
| `rust-toolchain.toml` | stable + `thumbv7em-none-eabihf` + `rust-src`/`llvm-tools-preview` |
| `.cargo/config.toml` | `target-cpu=cortex-m4`、`relocation-model=static`；**不配置 linker**（链接由 C 侧完成） |
| `build.rs` | 构建期校验 `RTOS_ABI_VERSION`（见 §3.4） |
| `abi/rtos_abi.h` | **契约头**，从 joc-base 同步而来，是唯一权威定义 |
| `src/abi.rs` | 手写镜像 `rtos_abi.h` + `app_slot_t` 的 Rust 声明（`#[repr(C)]` 结构体 + `extern "C"` 函数） |
| `src/device.rs` | device vtable 的安全封装（`Device::get/open/read/write/ioctl`） |
| `src/ioctl.rs` | 镜像 `rtos_abi_ioctl.h` 的驱动私有 ioctl 命令常量 |
| `src/lib.rs` | 挂载点 `rust_app_start`（经 `g_app_slot` 服务表）+ 任务栈 + demo/飞控任务 |

### 1.2 挂载流程（方案 Y 轻量版，含真·双分区）

App 有两种集成方式，编译期均与系统解耦：

- **轨 A（同编进一个 ELF，开发期方便）**：`joc-base` 用 `-DRUST_APP_LIB` 注入
  `libapp.a`，`task_app_main.c` 走 `#ifdef RUST_APP_LIB` 分支：`app_slot_init()`
  填充服务表 → `g_app_slot.app_start = rust_app_start` → 调用入口（见 §2.3）。
- **轨 B（真分区，目标形态，已硬件验证 PASS）**：App 独立链接成 `app.bin`
  （含 16B `app_header_t` 头部），单独烧到 `APP_FLASH` 分区；系统启动后
  `app_slot_load_app()` 从固定地址 `0x08060000` 读头部、校验、清零 App `.bss`、
  把入口钉到 `g_app_slot.app_start` 并调用。App 镜像与系统镜像**互不重编**。
  两种轨共用同一份 `rust_app_start` 与服务表契约，差异只在"谁把入口送进 `g_app_slot`"。

`rust_app_start(void)` 无参（**旧版经裸 `app_ctx_t*` 注入的签名已废弃**）。
内部：
- 校验 `magic` / `version` 双重防御 ABI 错配（错配经 Rust 日志系统打 `error!` 并返回）；
- 经 Rust 日志系统（`src/log.rs`）打 `info!` 自报 `RUST app mounted`（见 §4.1）；
- `spawn_flyctrl()` 经服务表 `task_create_rt` **自行创建**飞控多任务（见 `src/flyctrl/`）：
  `control`(prio=`RTOS_PRIO_BH_HIGH`(4)，rt_class=HARD，priv=1) / `sensors`(5) /
  `telem`(12) / `monitor`(14)，在控制任务内跑 EKF+PID+FDIR+MAVLink 遥测（见 §7）。
C 固件对 Rust 任务内容**一无所知**——运行时 Rust 任务与 C 任务在内核眼里无差别。
轨 B 下系统对 App 内容完全不可见，只识别固定地址的头部 + 固定地址的 `g_app_slot`。

> 早期示例用的 `rust_demo` / `att_rust` 占位任务已移除，当前挂载任务见 §6 / §7.1。

### 1.3 内存约束（必须遵守）

- **任务栈放在主 SRAM 静态数组**（如飞控的 `FLYCTRL_STACK: Stack2048`），**绝不进 CCM**。
  CCM 是 CPU-only（DMA 访问不到），且已被 RTOS 的 TCB 池/任务栈占满（~95%）。
- 栈数组用 `#[repr(align(8))]` 包裹，满足 RTOS 栈对齐要求。
- 所有 ABI 对象（sem/mutex/mq/event）的存储由 Rust 提供，但**不要放进 CCM 可达的 DMA 缓冲区**。

### 1.4 硬实时规则

- 硬实时任务经 `rtos_task_create_rt` 创建，`prio` 必须 `<= RTOS_PRIO_BH_HIGH`(4)。
- `priv=1`：飞控关键任务推荐特权模式，直接经 device vtable 操作外设、最低延迟。
- 中断上半部只做 `rtos_sem_give` 等 ISR 安全操作；控制律在 Rust 任务内跑；
  周期靠 TIM IRQ 同步信号量，而非 `rtos_msleep`。
- 实时违约汇总读 `rtos_rt_violation()` / `g_rtos_deadline_violation`（非 0 即违约，应联动看门狗）。

---

## 2. 编译

### 2.1 前置

- Rust stable + `thumbv7em-none-eabihf` target：
  ```sh
  rustup target add thumbv7em-none-eabihf
  rustup component add rust-src llvm-tools-preview
  ```
- 与 C 固件同一工具链的 `arm-none-eabi-gcc`（仅用于 C 侧链接，Rust 编译不调用）。

### 2.2 构建 libapp.a（轨 A，同编进一个 ELF）

```sh
cd joc-app-rust
cargo build --release --target thumbv7em-none-eabihf
# 产出：target/thumbv7em-none-eabihf/release/libapp.a
```

构建期 `build.rs` 会比对 `abi/rtos_abi.h` 的 `RTOS_ABI_VERSION` 与 `RUST_ABI_VERSION`，
**不一致立即 panic**，阻断契约漂移。

### 2.2b 构建独立 App 分区镜像（轨 B，真分区，推荐日常用）

`build_app.py` 把 `libapp.a` 经独立链接脚本 `app.ld` 链接成 `app.elf`，再 `objcopy`
成 `app.bin`（**自动带 16B `app_header_t` 头部**：magic=0x41504800 / abi=1 /
`ABSOLUTE(rust_app_start)` / 0）。`app.ld` 用 `PROVIDE(g_app_slot=0x2001DC00)`
把服务表地址钉死，App 不引用任何系统符号：

```sh
cd joc-app-rust
cargo build --release --target thumbv7em-none-eabihf   # 先出 libapp.a
python build_app.py                                    # -> app.elf / app.bin
# app.bin 可直接烧到 APP_FLASH (0x08060000)
```

> 头部 entry 是**裸地址**（bit0=0）；系统侧 `app_slot_load_app()` 挂载时会 `OR 1`
> 补 Thumb 位再经函数指针调用（Cortex-M 间接跳转目标必须带 Thumb 位，否则 INVSTATE 崩）。

### 2.3 链接进 C 固件（轨 A）

在 `joc-base` 侧用 `-DRUST_APP_LIB` 注入（CMakeLists.txt 已是注入式，无内联 cargo build）：

```sh
cd joc-base
cmake -S . -B build \
      -DRUST_APP_LIB=$(cygpath -w ../joc-app-rust/target/thumbv7em-none-eabihf/release/libapp.a) \
      -DRTOS_SELFTEST=OFF
cmake --build build
# 产出：build/stm32f407_minimal.elf / .bin
```

链接器用 `--gc-sections` 回收未引用符号；只有 `rust_app_start`（被 C 侧 `#ifdef` 分支引用）
及其下游符号会被保留。

### 2.3b 烧录工作流（轨 B：烧一次 RTOS，专注 App）

**系统区与 App 区编译期解耦、可独立烧录**，满足"RTOS 烧一次后只烧 App 分区"的开发模式：

```sh
# 首次 / RTOS 升级时（偶尔）：只烧系统区
cd joc-base && flash_sys.bat          # stm32f407_minimal.bin -> 0x08000000

# 日常应用层迭代（只动 App 分区）：
cd joc-app-rust && python build_app.py
python flash.py --no-build          # 仅烧 App 分区 (0x08060000)
# 或一条龙双分区烧录：python flash.py （自动 build_app.py + 烧系统区 + App 分区）
```

`RTOS_ABI_VERSION` 变 → 运行期 `app_slot_load_app` 拒绝挂载并打印 `app ABI mismatch`，
**不会**总线故障。验证：用 `python debug.py` 进 GDB，断 `rust_app_start` 确认到达且无 fault、
断 `console_run` 确认 App 返回后系统恢复命令循环；或烧录后 `python listen.py` 看 COM8
是否出现 `RUST app mounted` + `flyctrl: task started`（见 §4.1）。
完整设计见 `joc-base/docs/app-slot-design.md` §7。

---

## 3. 系统 API（经 `app_slot_t` 服务表）

所有能力经全局 `g_app_slot: app_slot_t`（见 `src/abi.rs`，镜像 C 侧
`src/app_slot/app_slot.h`）。App **只经这张表的方法调用系统**，不再 `extern "C"`
引用 `rtos_*` 裸符号（这是方案 Y 与早期「裸 ABI 直调」版本的根本区别）。

### 3.1 app_slot_t 服务表布局

```rust
// src/abi.rs（#[repr(C)]，字段顺序须与 C 侧严格一致）
pub struct app_slot_t {
    pub magic: u32;          // APP_SLOT_MAGIC = 0x41505053 ("APPS")
    pub version: u32;       // 须 == RTOS_ABI_VERSION
    pub reserved: u32,
    // 内核服务
    pub task_create:  Option<...>,     // (name, entry, arg, prio, stack, stack_size)
    pub task_create_rt: Option<...>,   // (+ priv, *const rtos_task_attr_t)
    pub msleep: Option<extern "C" fn(u32)>,
    pub tick_count: Option<extern "C" fn() -> u32>,
    pub cycle_now:  Option<extern "C" fn() -> u32>,
    // IPC 服务
    pub sem_init:   Option<extern "C" fn(*mut rtos_sem_t, u32, u32)>,
    pub sem_wait:   Option<extern "C" fn(*mut rtos_sem_t) -> i32>,
    pub sem_trywait:Option<extern "C" fn(*mut rtos_sem_t) -> i32>,
    pub sem_give:   Option<extern "C" fn(*mut rtos_sem_t)>,
    // 设备服务（统一 device vtable 镜像）
    pub dev_get:  Option<extern "C" fn(*const c_char) -> *mut device_t>,
    pub dev_open: Option<extern "C" fn(*mut device_t) -> i32>,
    pub dev_read: Option<extern "C" fn(*mut device_t, *mut c_void, usize) -> i32>,
    pub dev_write:Option<extern "C" fn(*mut device_t, *const c_void, usize) -> i32>,
    pub dev_ioctl:Option<extern "C" fn(*mut device_t, i32, *mut c_void) -> i32>,
    pub dev_close:Option<extern "C" fn(*mut device_t) -> i32>,
    // 中断回调注册位（方案 Y 轻量版关键）
    pub irq_reg:   [app_irq_reg_t; APP_IRQ_REG_MAX],   // APP_IRQ_REG_MAX = 8
    pub irq_attach:  Option<extern "C" fn(*const app_irq_reg_t) -> i32>,
    pub irq_enable:  Option<extern "C" fn(u8) -> i32>,
    pub irq_disable: Option<extern "C" fn(u8) -> i32>,
    // 生命周期（系统侧填入 App 入口）
    pub app_start: Option<extern "C" fn() -> i32>,
    pub app_stop:  Option<extern "C" fn()>,
}
extern "C" { pub static mut g_app_slot: app_slot_t; }  // 系统预留符号，App 不可自定
```

### 3.2 任务（经服务表）

```rust
// 取表并通过 Option 调用（None 防御）
let slot = &*core::ptr::addr_of!(g_app_slot);
if let Some(tc) = slot.task_create {
    tc("rust_task\0".as_ptr(), rust_task_entry, 0 as *mut c_void,
       RTOS_PRIO_BLINK, stack.as_mut_ptr() as *mut c_void, stack.len());
}
// 硬实时：task_create_rt(name, entry, arg, prio, stack, stack_size, priv, *attr)
```

优先级常量：`RTOS_PRIO_BH_HIGH=4`、`RTOS_PRIO_BH_MED=6`、`RTOS_PRIO_MAIN=12`、
`RTOS_PRIO_BLINK=14`、`RTOS_PRIO_BIST=24`、`RTOS_PRIO_IDLE=31`。
硬实时类别：`RTOS_RT_NONE=0`、`RTOS_RT_HARD=1`、`RTOS_RT_SOFT=2`。

### 3.3 IPC（经服务表）

```rust
let slot = &*core::ptr::addr_of!(g_app_slot);
if let Some(si) = slot.sem_init { si(&raw mut ATT_SEM as *mut rtos_sem_t, 0, 64); }
if let Some(sw) = slot.sem_wait { sw(&raw mut ATT_SEM as *mut rtos_sem_t); }  // 阻塞
if let Some(g)  = slot.sem_give { g(sem); }                                   // ISR 安全
```

互斥量 / 消息队列 / 事件标志：见 `src/abi.rs` 同名 `Option<extern "C" fn>` 字段。

### 3.4 设备（唯一外设入口）

```rust
let slot = &*core::ptr::addr_of!(g_app_slot);
let dev = if let Some(dg) = slot.dev_get { dg("uart0\0".as_ptr()) } else { null_mut() };
let _ = (slot.dev_open)(dev);
let n = (slot.dev_write)(dev, buf.as_ptr() as *const c_void, buf.len());
// 也提供安全封装 src/device.rs：Device::get/open/read/write/ioctl
```

- **Rust 不碰裸寄存器**，所有驱动操作经 `device` vtable（服务表的 `dev_*` 字段）。
- ioctl 命令常量见 `src/ioctl.rs`（镜像 `rtos_abi_ioctl.h`，值与 C 侧严格一致）。
- 定时器启动新增 `TIMER_IOCTL_ENABLE`(0x05) / `TIMER_IOCTL_DISABLE`(0x06)：
  App 经 `dev_open("timer2")` + `dev_ioctl(TIMER_IOCTL_ENABLE, null)` 即可启动
  TIM6 并 arm IRQ，无需裸 `event_device` vtable。

### 3.5 中断回调注册（irq_reg[] 位）

App 想挂 ISR 时**只填 `irq_reg[]` 注册位**，真实 NVIC 编程由系统 `irq_manager` 兜住：

```rust
let slot = &mut *core::ptr::addr_of_mut!(g_app_slot);
let reg = &mut slot.irq_reg[0];
reg.used       = 1;
reg.irq_id     = 54;                 // TIM6 = IRQ54（板级 timer2）
reg.prio_class = IRQ_CLASS_KERNEL;   // 1
reg.rt_class   = 1;                  // 硬实时
reg.isr_cb     = Some(att_isr_give); // App 回调：上半部只 sem_give
reg.ctx        = &raw mut ATT_SEM as *mut c_void;
if let Some(a) = slot.irq_attach { a(reg as *const app_irq_reg_t); }
```

`isr_cb` 约束（ISR 安全）：只做 `sem_give` / 写内存 / 置 flag；
**不调** `task_create` / `mq_init` 等调度器变更 API（见 §1.4）。

### 3.6 ABI 版本校验

`RTOS_ABI_VERSION`（`g_app_slot.version`，源自 C 侧 `rtos_abi.h`）与
`RUST_ABI_VERSION`（`build.rs` 常量）必须相等。
**任何 `app_slot_t` 字段 / 函数签名变更都必须 +RTOS_ABI_VERSION，并同步 `src/abi.rs`**——
否则 `cargo build` 直接失败，不会把漂移带进运行时。`rust_app_start` 还会在
运行期复校 `magic` 与 `version`，错配立即返回错误。

---

## 4. 调试与运行

### 4.1 板载运行时验证 + Rust 应用层日志系统（无需调试器）

Rust 应用层自带一套**独立于 C 侧系统日志**的日志系统（`src/log.rs`），专门给 App
（飞控等 Rust 任务）使用，与系统 `I/main:` 日志靠前缀区分、物理同串口：

- 系统日志：`I/main: jOS RTOS ready ...`（C 侧 `g_console`，前缀 `I/`）
- Rust 日志：`R/<L> <ticks> <tag>: <msg>`（App 侧，前缀 `R/`）

每条 Rust 日志含：**级别单字母**（`D`/`I`/`W`/`E`）+ **tick 时间戳**（`g_app_slot.tick_count()`）
+ **标签**（调用点模块/任务名，如 `app_slot` / `flyctrl`）+ 消息。

#### 接口（宏风格，接近 `log` crate，手写 `no_std`）

```rust
use crate::{info, warn, error, debug};   // 宏由 #[macro_export] 导出到 crate 根

info!(tag: "flyctrl", "task started; loop={}ms", 4);
warn!(tag: "flyctrl", "pwm{} not available", i);
error!(tag: "app_slot", "ABI mismatch magic={:#x}", magic);
debug!(tag: "flyctrl", "verbose trace {}", x);  // 仅 debug build 编入，release 剔除
```

特性：
- **三级 + debug**：`info!` / `warn!` / `error!` 始终编入；`debug!` 经 `cfg!(debug_assertions)`
  **编译期剔除**（release build 不占体积）。
- **时间戳 + 标签**：每条自动带 `tick_count()` 与调用点 `tag`，便于联调定位。
- **通道**：复用 RTOS 调试控制台 `uart0`（USART1 / COM8）。日志为无状态写
  （`dev_get("uart0")` + `dev_open` + `dev_write` + `dev_close`），**不持有设备句柄**，
  不干扰 C 侧已打开的 `g_console`。`uart0` 是调试控制台，飞控业务下行仍走 USART6 遥测口。
- **全部 `no_std`**，仅经 ABI 契约 `g_app_slot` 调用，不碰裸 RTOS 符号。

#### 实机输出示例

板子复位后 COM8（115200）可见（注意系统 `I/` 与 App `R/` 混在同一串口、靠前缀区分）：

```
I/main: jOS RTOS ready (STM32F407 Discovery, OOC)          ← C 侧系统日志
I/main: READY. Commands: ...
I/app_slot: [boot] app partition found: entry=0x080620AC -> mounting
R/I 30 app_slot: RUST app mounted (rust_app_start)        ← Rust 日志（App 挂载自报）
R/I 35 flyctrl: task started; loop=4ms prio=4             ← Rust 日志（飞控任务启动）
R/I 500 flyctrl: hb seq=250 armed=0 crit=0 gps=0 baro=0 mag=0 alt=0.00  ← 节流心跳(≈1s/条)
```

飞控任务每隔 250 个周期（≈1s）打一条节流心跳 `hb`，避免 4ms 周期刷爆串口。

> 注意：串口打开时 CH340 的 DTR 脉冲会复位板子，所以连接后应等 BIST 跑完（~2s）再发命令。

### 4.1b 运行 / 监听脚本（本工程正式脚本）

| 脚本 | 作用 |
|------|------|
| `run_app.py` | 启动 OpenOCD + GDB `monitor reset run` 让板子从 Flash 运行，同时监听 COM8 输出（默认 12s） |
| `listen.py`  | 纯监听 COM8（默认 18s），**不碰 OpenOCD**，最可靠——已 open 端口后手动按板子复位键触发启动打印 |

```sh
cd joc-app-rust
python run_app.py                 # reset run + 监听（COM8 115200 12s）
python run_app.py COM9 115200 20  # 指定端口/波特/秒数
python listen.py                  # 纯监听，open 后手动按复位键
```

- 两个脚本打开端口时都强制 `dtr=False; rts=False`，避免 CH340 的 DTR 脉冲复位板子。
- `run_app.py` 的 ST-Link `monitor reset run` 在部分板子上与 CH340 缓冲不同步，可能抓不到
  启动打印（但板子确实在跑）；若输出为 0 字节，**改用 `listen.py` 并在其 open 端口后手动按
  复位键**即可稳定捕获（与 §4.1 示例打印同一来源）。
- 依赖：`run_app.py` 需 OpenOCD（路径硬编码在脚本顶部 `OCD_DIR`）+ `arm-none-eabi-gdb`（PATH）；
  `listen.py` 仅需 `pyserial`。

### 4.2 烧录一条龙（本工程脚本）

`flash.py` 把 OpenOCD 路径、系统镜像路径都硬编码在脚本顶部，在本工程目录即可一条龙烧录
（系统区 `0x08000000` + App 分区 `0x08060000` 一起烧），**无需切到 joc-base**：

```sh
cd joc-app-rust
python flash.py            # 自动 build_app.py 生成 app.bin + 自启 OpenOCD 烧录双分区
python flash.py --no-build # 仅烧录（已生成过 app.bin 时）
```

- 脚本自启 OpenOCD（GDB server :3333）、用 GDB `monitor flash write_image erase` 烧两个分区、
  烧完自动 SIGTERM 关闭 OpenOCD，不留后台进程。
- 系统镜像默认取 `../joc-base/build_rel/stm32f407_minimal.bin`（joc-base 用 `build_rel.bat`
  或 `-DRTOS_SELFTEST=OFF` 构建；其分区 APP_RAM=0x20004000/0x1BC00、APP_SLOT=0x2001FC00
  与本工程 `app.ld` 一致）；OpenOCD 路径默认 `D:/soft/openocd/...`。若环境不同，改 `flash.py`
  顶部的 `SYS_BIN` / `OCD_DIR` 即可。

### 4.3 GDB + OpenOCD 调试（本工程 `debug.py`）

`debug.py` 自启 OpenOCD 并进入交互式 GDB，在 App 挂载入口 `rust_app_start` 断住，
可直接单步 / 看变量 / 查服务表：

```sh
cd joc-app-rust
python debug.py          # 进交互式 GDB，停在 rust_app_start
```

断住后常用 GDB 命令：`c`（继续跑）、`si`/`stepi`（单步）、`bt`（栈）、
`p g_app_slot`（看服务表）、`info registers`、`monitor reset halt`（重新 halt）。

> **Cortex-M 断点注意**：Flash 上**不能下软件断点**，`break`/`thbreak` 会报
> `No hardware breakpoint support` 导致断点没设上、板子直接跑飞。必须改用
> **`hbreak`**（硬件断点，Cortex-M 仅 6 个）。`debug.py` 已内置 `hbreak rust_app_start`。
> 也可手动：`hbreak flyctrl_entry`（证飞控任务被调度）、
> `hbreak att_isr_give`（TIM6 IRQ54 命中，证 `irq_reg[0]` 经 `irq_manager` 路由）。

断点验证结果（已实机验证 PASS）：
```
Hardware assisted breakpoint 1 at 0x8060094
Breakpoint 1, 0x08060094 in rust_app_start ()
pc  0x8060094  <rust_app_start+20>
```
说明：加载双符号时 `add-symbol-file app.elf 0x08060000` 会把 `.text_addr` 解析到
`0x8060000`，GDB 报 `section .text not found` 是 app.elf 用 `app.ld` 链接、section 名
不同的无害警告，不影响符号与断点。

也可用绝对地址断点（轨 B 系统 ELF 不含 Rust 符号时）：`hbreak *0x08060080`
（App 入口，见 `app.bin` 头部 entry）确认挂载到达且无 fault。

也可读 `g_task_pool`（TCB 在 CCM，`task_t.name` 在 +4 偏移，`state` 在 +11 单字节）：
应能看到 `flyctrl`(prio4, rt_class=1, priv=1) 条目。

### 4.3 panic / fault

`panic=abort`：Rust `panic!` 编译为 `UD` 指令（默认）或触发内核 fault handler。
板载 fault handler 会捕获并报告（见 joc-base 的 RTOSROBUST/UDF_Recover 自测）。

### 4.4 常见坑

- **串口看不到 `RUST app mounted` / `flyctrl: task started`**：先确认 `rust_app_start`
  被调用（GDB 断 `rust_app_start`）。命中但无 App 输出，查 `g_app_slot.dev_write` /
  `dev_open` / `dev_get` 是否被正确填充（`app_slot_init` 在 `task_app_main.c` 里先于
  `app_start()` 调用），以及 `uart0` 设备节点是否存在。若看到 `error! ... ABI mismatch`
  则是 `RTOS_ABI_VERSION` 错配，App 拒绝挂载（属预期防御）。
- **`run_app.py` 输出 0 字节**：ST-Link `monitor reset run` 在部分板子上与 CH340 缓冲不同步，
  抓不到启动打印（但板子在跑）。改用 `listen.py` 并在其 open 端口后**手动按板子复位键**，
  即可稳定捕获（与 §4.1 示例同一来源）。
- **链接报 undefined reference to `rust_app_start`**：CMake 没注入 `RUST_APP_LIB=1` 宏，
  导致 C 侧 `#ifdef RUST_APP_LIB` 分支为假、`g_app_slot.app_start` 未赋值 → libapp.a 被 gc 裁掉。
- **`rust_app_start` 返回前断言 version 不符**：`RTOS_ABI_VERSION`（`g_app_slot.version`）与
  `build.rs` 的 `RUST_ABI_VERSION` 不一致，`cargo build` 应在链接期就失败；若漏检进运行时，
  `rust_app_start` 会复校 `magic`/`version` 并返回错误，App 不挂载。
- **栈溢出**：飞控任务栈 2048B（opt-level=z 下 Rust 栈帧偏厚），若用大局部数组需加大。
- **串口 DTR 复位**：CH340 打开时 DTR 脉冲会复位板子；脚本已强制 `dtr=False`，但若用其他
  串口助手，连接后请等 BIST 跑完（~2s）再发命令。

---

## 5. 修改契约的步骤（方案 Y：改 `app_slot_t`）

1. 改 `joc-base` 的 `src/app_slot/app_slot.h`（`app_slot_t` 字段 / 函数签名）或
   新增 ioctl 命令（如 `src/drv/timer.h`）→ **+RTOS_ABI_VERSION**（结构体/签名变更）。
2. 同步本工程 `src/abi.rs` 的 `app_slot_t` / `app_irq_reg_t` `#[repr(C)]` 声明与常量
   （`RTOS_ABI_VERSION`、`APP_SLOT_MAGIC`、`APP_IRQ_REG_MAX`、ioctl 命令等）。
3. 同步 `src/ioctl.rs` 的驱动私有 ioctl 命令常量。
4. 若版本号变了，改 `build.rs` 的 `RUST_ABI_VERSION` 保持一致。
5. `cargo build` 校验通过 → 走 §2.3 重新链接烧录。
   （字段顺序必须与 C 侧逐字节一致；`#[repr(C)]` 保证布局，但仍需人工核对。）

---

## 6. 当前挂载的任务（作为示例参考）

| 任务 | 类型 | prio | 栈 | 行为 |
|------|------|------|-----|------|
| `flyctrl` | 硬实时(HARD) | 4 | 2048B | `flyctrl_entry`：EKF+PID+FDIR+MAVLink 遥测，经 imu/pwm/uart 设备 vtable（priv=1）；每 250 周期打节流心跳 `hb` |

中断注册：`irq_reg[0]` = TIM6(IRQ54) → `att_isr_give(ATT_SEM)`，`irq_class=KERNEL`、`rt_class=HARD`；
TIM6 经 `dev_open("timer2")` + `TIMER_IOCTL_ENABLE` 启动并 arm IRQ（飞控任务当前用 `msleep(4ms)`
节拍，TIM IRQ 精确同步为 TODO，见 §7.2）。

扩展业务时，在 `rust_app_start` 内填更多 `irq_reg[]` 槽并调 `task_create[_rt]` 增任务即可，
RTOS C 侧无需改动（只要 `app_slot_t` 服务表已暴露所需能力）。

## 7. 飞控接入 `flyctrl-core`（真实控制算法示例）

把独立的 `flyctrl` 飞控项目（EKF 估计 + PID/LQR/MPC 控制 + FDIR + MAVLink 遥测）作为 App 层依赖接入，
在硬实时任务里跑真实控制律，**不依赖 RTOS 后端的具体寄存器实现**：

- 依赖：`Cargo.toml` 加 `flyctrl-core = { path = "../flyctrl/core" }`（**不启用 `stm32f407`**，
  App 经 RTOS 设备 vtable 做 IO，绝不碰裸寄存器，规避 CCM/MPU/DMA 风险）。
  `flyctrl-core` 为 `no_std` + 仅依赖 `libm`，可干净交叉编译到 `thumbv7em-none-eabihf`。
- 任务：`src/flyctrl/`（每任务一个文件：`control.rs` / `sensors_task.rs` / `telemetry.rs` /
  `monitor.rs`，共享数据/栈/启动在 `mod.rs`）经 `spawn_flyctrl()` 在 `rust_app_start` 中创建，
  `rtos_task_create_rt`，`control` prio=`RTOS_PRIO_BH_HIGH`(4)，priv=1。
- 算法链路（每周期）：
  1. 经 `g_app_slot.dev_*` 读 `"imu"`（约定 24B=6×f32 LE：accel3+gyro3）→ `ImuSample`；
  2. `EkfEstimator::step` 估计 `VehicleState`（位置测量当前 `None`，待 GPS/baro 设备接入）；
  3. `Fdir::update`（四源监控）+ `RtlHome::try_lock`（首次定位锁 home）；
  4. `PidController::control`（armed 才出推力，否则零油门）；
  5. 经 `"pwm"` 写 16B=4×f32 归一化推力（X 型混控）；
  6. 经 `"uart"` 下行标准 MAVLink（HEARTBEAT + LOCAL_POSITION_NED + SYS_STATUS，QGC 可解析）。
- **降级策略**：若 RTOS 尚未提供 `imu`/`pwm` 设备节点（当前阶段），`Device::open` 返回 `None`，
  任务自动切换为内置 `SimImu` 模拟源 + 丢弃 PWM 写，保证 EKF+PID+FDIR+MAVLink 整链路仍可编译/运行/验证；
  RTOS 侧补齐 `imu`/`pwm`/`uart` 设备节点（产出约定二进制格式）后，**无需改此文件即自动切换**。

### 7.1 挂载任务表（更新）

| 任务 | 类型 | prio | 栈 | 行为 |
|------|------|------|-----|------|
| `flyctrl` | 硬实时(HARD) | 4 | 2048B | `flyctrl_entry`：EKF+PID+FDIR+MAVLink，经 imu/pwm/uart 设备 vtable（priv=1）；节流心跳经 Rust 日志 `info!` 打 `hb` |

> 早期示例任务 `rust_demo` / `att_rust` 已移除，控制律全部并入 `flyctrl` 单硬实时任务。
> 所有 App 侧打印统一走 `src/log.rs` 日志系统（前缀 `R/`），不再直接 `dev_write` 裸字符串。

### 7.2 RTOS 侧待补齐的设备约定（接入时由驱动实现）

- `"imu"`  `read` → 24B 小端 6×f32（accel.x,y,z, gyro.x,y,z）
- `"pwm"`  `write` → 16B 小端 4×f32 归一化推力 [0,1]
- `"uart"` `write` → MAVLink v1 帧字节流（遥测下行；可复用 joc-base 的 USB CDC / UART 驱动）
- （可选）`"gps"`/`"baro"`/`"mag"` `read` → 位置/高度/航向测量，喂入 `EkfEstimator` 与 `Fdir`
- 周期同步：当前 `flyctrl_entry` 用 `rtos_msleep(4ms)`；RTOS 接入 TIM IRQ 后可经 `sem_wait` 精确同步（同 TIM6 IRQ 模式，留 TODO）
