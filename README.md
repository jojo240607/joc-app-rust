# joc-app-rust

STM32F407（jOS RTOS）上的 **Rust 应用层**。与 C 固件（`joc-base`）解耦为独立工程，
通过 **干净 ABI 契约** 调用内核，不依赖 `rtos.h` / `device.h` 内部头文件。

```
┌──────────────────────────────────────────────────────────┐
│  C 固件 (joc-base)  → RTOS 内核 + 驱动 + 设备注册          │
│    app_main_task 在控制台循环前调用 rust_app_start(ctx)    │
└───────────────┬──────────────────────────────────────────┘
                │  extern "C"  ABI 契约（rtos_abi.h）
┌───────────────▼──────────────────────────────────────────┐
│  Rust 应用层 (joc-app-rust) → 飞控 demo / 业务任务         │
│    仅经 device vtable 操作外设，不碰裸寄存器               │
│    产出 libapp.a，由 C 固件链接器统一链接                  │
└──────────────────────────────────────────────────────────┘
```

---

## 1. 架构

### 1.1 工程组织

`joc-app-rust` 与 `joc-base` 是**平级独立 git 仓库**，互不内联编译：

| 路径 | 作用 |
|------|------|
| `Cargo.toml` | crate-type=`staticlib`，产出 `libapp.a`；release profile 为 `opt-level=z` + `lto` + `panic=abort` |
| `rust-toolchain.toml` | stable + `thumbv7em-none-eabihf` + `rust-src`/`llvm-tools-preview` |
| `.cargo/config.toml` | `target-cpu=cortex-m4`、`relocation-model=static`；**不配置 linker**（链接由 C 侧完成） |
| `build.rs` | 构建期校验 `RTOS_ABI_VERSION`（见 §3.3） |
| `abi/rtos_abi.h` | **契约头**，从 joc-rtos 同步而来，是唯一权威定义 |
| `src/abi.rs` | 手写镜像 `rtos_abi.h` 的 Rust 声明（`#[repr(C)]` 结构体 + `extern "C"` 函数） |
| `src/device.rs` | device vtable 的安全封装（`Device::get/open/read/write/ioctl`） |
| `src/ioctl.rs` | 镜像 `rtos_abi_ioctl.h` 的驱动私有 ioctl 命令常量 |
| `src/lib.rs` | 挂载点 `rust_app_start` + 任务栈 + demo/飞控任务 |

### 1.2 挂载流程

1. C 固件 `task_app_main.c` 在控制台循环前调用 `rust_app_start(app_ctx)`，
   该符号在 `#ifdef RUST_APP_LIB` 分支声明/调用（宏由 CMake 注入）。
2. `rust_app_start` 内部用 ABI 契约**自行创建**所有 Rust 任务：
   - `rust_demo`：普通任务（`rtos_task_create`，prio=14，每 500ms 心跳自增 `RUST_TICKS`）
   - `att_rust`：硬实时姿态环（`rtos_task_create_rt`，prio=3，rt_class=RTOS_RT_HARD，priv=1）
3. C 固件对 Rust 任务内容**一无所知**——运行时 Rust 任务与 C 任务在内核眼里无差别。

### 1.3 内存约束（必须遵守）

- **任务栈放在主 SRAM 静态数组**（如 `RUST_DEMO_STACK: Stack1024`），**绝不进 CCM**。
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

### 2.2 构建 libapp.a

```sh
cd joc-app-rust
cargo build --release --target thumbv7em-none-eabihf
# 产出：target/thumbv7em-none-eabihf/release/libapp.a
```

构建期 `build.rs` 会比对 `abi/rtos_abi.h` 的 `RTOS_ABI_VERSION` 与 `RUST_ABI_VERSION`，
**不一致立即 panic**，阻断契约漂移。

### 2.3 链接进 C 固件

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

---

## 3. 系统 API（ABI 契约）

所有 API 声明见 `src/abi.rs`（镜像 `abi/rtos_abi.h`）。以下为常用子集。

### 3.1 任务

```rust
rtos_task_create(name: *const c_char, entry, arg, prio: u8, stack, stack_size);
rtos_task_create_rt(name, entry, arg, prio, stack, stack_size, priv: u8, attr: *const rtos_task_attr_t);
rtos_msleep(ms: u32);
rtos_tick_count() -> u32;          // 系统 tick 计数
rtos_cycle_now() -> u32;           // DWT CYCCNT @HCLK，高精度相位补偿
```

优先级常量：`RTOS_PRIO_BH_HIGH=4`、`RTOS_PRIO_BH_MED=6`、`RTOS_PRIO_MAIN=12`、
`RTOS_PRIO_BLINK=14`、`RTOS_PRIO_BIST=24`、`RTOS_PRIO_IDLE=31`。
硬实时类别：`RTOS_RT_NONE=0`、`RTOS_RT_HARD=1`、`RTOS_RT_SOFT=2`。

### 3.2 IPC

```rust
// 信号量
rtos_sem_init(&sem, initial, limit);
rtos_sem_wait(&sem) -> i32;        // 阻塞
rtos_sem_trywait(&sem) -> i32;     // 非阻塞，-1 无许可
rtos_sem_give(&sem);               // ISR 安全

// 互斥量 / 消息队列 / 事件标志：见 abi.rs 同名 extern "C"
```

### 3.3 设备（唯一外设入口）

```rust
let dev = Device::get("uart0").expect("no uart0");
dev.open();
let n = dev.write(b"hello");
let m = dev.read(&mut buf);
dev.ioctl(UART_IOCTL_SET_BAUDRATE, &mut arg as *mut _ as *mut c_void);
```

- **Rust 不碰裸寄存器**，所有驱动操作经 `device` vtable。
- ioctl 命令常量见 `src/ioctl.rs`（镜像 `rtos_abi_ioctl.h`，值与 C 侧严格一致）。

### 3.4 ABI 版本校验

`RTOS_ABI_VERSION`（C 侧 `rtos_abi.h`）与 `RUST_ABI_VERSION`（`build.rs` 常量）必须相等。
**任何契约结构体字段 / 函数签名变更都必须 +RTOS_ABI_VERSION，并同步 `src/abi.rs`**——
否则 `cargo build` 直接失败，不会把漂移带进运行时。

---

## 4. 调试与运行

### 4.1 板载运行时验证（无需调试器）

固件烧录后，串口（CH340 COM8，115200）控制台：

```
PING          → PONG                    （system 任务 + 控制台循环正常）
RUST          → RUST mounted: rust_demo alive, ticks=N
```

`RUST` 命令调用 Rust 暴露的 `rust_ticks()` 并回显心跳计数；`ticks` 持续增长即证明
Rust 任务在运行、C↔Rust 调用链打通。

> 注意：串口打开时 CH340 的 DTR 脉冲会复位板子，所以连接后应等 BIST 跑完（~2s）再发命令。

### 4.2 GDB 读任务池（确认 Rust TCB 挂载）

1. 启动常驻 OpenOCD（ST-Link + stm32f4x）：
   ```sh
   openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init"
   ```
2. 用 GDB 连 3333，reset run 跑几秒后读 `g_task_pool`（TCB 在 CCM，`task_t.name` 在 +4 偏移，
   `state` 在 +11 单字节）：应能看到 `rust_demo`(prio14) 与 `att_rust`(prio3, rt_class=1) 条目。

### 4.3 panic / fault

`panic=abort`：Rust `panic!` 编译为 `UD` 指令（默认）或触发内核 fault handler。
板载 fault handler 会捕获并报告（见 joc-base 的 RTOSROBUST/UDF_Recover 自测）。

### 4.4 常见坑

- **`RUST` 命令无输出 / `rust_ticks` 恒 0**：Rust 任务没挂载。检查
  (a) 固件构建是否传了 `-DRUST_APP_LIB`；
  (b) `task_app_main.c` 是否走到 `rust_app_start`（nm 查 ELF 应有 `rust_app_start` 符号）；
  (c) `rust_task_entry` 是否真的在自增（当前实现每 500ms +1）。
- **链接报 undefined reference to `rust_app_start`**：CMake 没注入 `RUST_APP_LIB=1` 宏，
  导致 C 侧 `#ifdef RUST_APP_LIB` 分支为假、`rust_app_start` 未声明调用 → libapp.a 被 gc 裁掉。
- **链接报 undefined reference to `rust_ticks` 等**：`rust_app.h`（C 侧）声明了符号但 Rust 没实现，
  或符号名/调用约定（extern "C"）不一致。
- **栈溢出**：任务栈默认 1024/512 字节（opt-level=z 下 Rust 栈帧偏厚），飞控任务若用大局部数组需加大。

---

## 5. 修改契约的步骤

1. 改 `joc-base` 的 `src/rtos/.../rtos_abi.h` 或新增 ioctl 命令 → **+RTOS_ABI_VERSION**（结构体/签名变更）。
2. 把 `abi/rtos_abi.h` 同步到本工程的 `abi/rtos_abi.h`。
3. 同步 `src/abi.rs` / `src/ioctl.rs` 的 `#[repr(C)]` 声明与常量。
4. 若版本号变了，改 `build.rs` 的 `RUST_ABI_VERSION` 保持一致。
5. `cargo build` 校验通过 → 走 §2.3 重新链接烧录。

---

## 6. 当前挂载的任务（作为示例参考）

| 任务 | 类型 | prio | 栈 | 行为 |
|------|------|------|-----|------|
| `rust_demo` | 普通 | 14 | 1024B | 每 500ms `RUST_TICKS += 1`，供 `RUST` 命令观测 |
| `att_rust`  | 硬实时(HARD) | 3 | 512B | 飞控姿态环占位（priv=1），扩展点 |

扩展业务时，在 `rust_app_start` 内用 `rtos_task_create[_rt]` 增任务即可，
RTOS C 侧无需改动。
