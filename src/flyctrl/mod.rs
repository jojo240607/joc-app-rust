//! 飞控应用：多任务实时架构（采样 / 控制 / 遥测 / 监控 分离）。
//!
//! 模块划分：
//!   - `control`   硬实时控制律任务（见 `control.rs`）
//!   - `sensors`   传感器采样任务（见 `sensors_task.rs`）
//!   - `telemetry` 遥测下行任务（见 `telemetry.rs`）
//!   - `monitor`   系统监控/心跳任务（见 `monitor.rs`）
//!
//! 任务优先级档位（见 `crate::abi`）：
//!   - `control`   prio=4  硬实时(priv=1, RTOS_RT_HARD) 周期 4ms：取最新样本 → EKF → FDIR → PID → PWM
//!   - `sensors`   prio=5  软实时(priv=1)              周期 2ms：采 IMU/RC/Baro/Mag/GPS → 写共享帧
//!   - `telemetry` prio=12 (priv=1)                    周期 20ms：从最新估计发 MAVLink(标准)
//!   - `monitor`   prio=14 (priv=1)                    周期 1000ms：心跳日志 + 看门狗
//!
//! 跨任务共享数据经 RTOS 互斥量（`crate::rtos_sync::Mutex`）保护：
//!   - SENSOR_FRAME / SENSOR_MTX：最新传感器样本（sensors 写、control 读）
//!   - EST_STATE   / EST_MTX  ：最新估计状态 + 健康（control 写、telemetry/monitor 读）
//!
//! 注意：mag(QMC5883L@0x0D) 当前读取会在单总线 I2C 事务中卡死（见 memory：joc-app-rust
//! mag.read 卡死 bug），待 joc-base I2C 驱动修复前，sensors 任务对 mag 走「缺失降级」路径，
//! 不实际发起读事务，避免拖垮采样线程。

pub mod control;
pub mod monitor;
pub mod sensors_task;
pub mod telemetry;

use flyctrl_core::vehicle::{ImuSample, PosSample, Quaternion, RcInput, VehicleState};
use flyctrl_core::fdir::Health;
use flyctrl_core::units::{Meter, MeterPerSecond, RadianPerSecond};

use crate::abi::RTOS_PRIO_BH_HIGH;
use crate::rtos_sync::{spawn_rt, Mutex, RT_HARD, RT_NONE};

/* ===================== 共享数据 ===================== */

/// 最新传感器样本帧（sensors 写、control 读）。mag 缺失时为 None。
pub struct SensorFrame {
    pub imu: Option<ImuSample>,
    pub rc: RcInput,
    pub gps: Option<PosSample>,
    pub baro_alt: Option<f32>,
    pub imu_ok: bool,
    pub gps_ok: bool,
    pub baro_ok: bool,
    pub mag_ok: bool,
    pub armed: bool,
}

impl SensorFrame {
    const fn empty() -> Self {
        SensorFrame {
            imu: None,
            rc: RcInput { roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.0, armed: false, mode: 0, fresh: false },
            gps: None,
            baro_alt: None,
            imu_ok: false,
            gps_ok: false,
            baro_ok: false,
            mag_ok: false,
            armed: false,
        }
    }
}

/// 最新估计状态 + 健康（control 写、telemetry/monitor 读）。
pub struct EstState {
    pub est: VehicleState,
    pub health: Health,
    pub armed: bool,
}

impl EstState {
    const fn empty() -> Self {
        EstState {
            est: VehicleState {
                pos: [Meter(0.0), Meter(0.0), Meter(0.0)],
                vel: [MeterPerSecond(0.0), MeterPerSecond(0.0), MeterPerSecond(0.0)],
                att: Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
                omega: [RadianPerSecond(0.0), RadianPerSecond(0.0), RadianPerSecond(0.0)],
            },
            health: Health::Degraded,
            armed: false,
        }
    }
}

/// 最新估计状态 + 健康（control 写、telemetry/monitor 读）。
/// 注意：含 `health: Health` enum（非零判别式），若用 `EstState::empty()` 作初始化器会带
/// 非零字节、被 Rust 放进 `.data` 段；而 App 链接契约（XIP + 仅 .bss，见 app.ld）下 `.data`
/// 会落到 Flash，运行时写它即写 Flash → BusFault。故用 `#[link_section=".bss.est_state"]`
/// 强制进 .bss，并用 `zeroed()` 作全零初始化器（Health::Healthy=0 为合法变体，zeroed 安全）；
/// 真正的初值在 `spawn_flyctrl` 里运行时 `EST_STATE = EstState::empty()` 填充——写落 RAM 安全。
#[link_section = ".bss.est_state"]
pub static mut EST_STATE: EstState = unsafe { core::mem::zeroed() };

/// 全局共享帧 + 互斥量（静态存储，启动时 init）。
pub static mut SENSOR_FRAME: SensorFrame = SensorFrame::empty();
pub static mut SENSOR_MTX: Mutex = Mutex::uninit();
pub static mut EST_MTX: Mutex = Mutex::uninit();

/* ===================== 任务栈 ===================== */

const STACK_CTRL: usize = 3072; // 控制律含 EKF+PID，栈需求大
const STACK_SENS: usize = 2048; // 采样含 I2C/UART 缓冲
const STACK_TELEM: usize = 1536;
const STACK_MON: usize = 1024;

static mut STACK_CTRL_BUF: [u8; STACK_CTRL] = [0u8; STACK_CTRL];
static mut STACK_SENS_BUF: [u8; STACK_SENS] = [0u8; STACK_SENS];
static mut STACK_TELEM_BUF: [u8; STACK_TELEM] = [0u8; STACK_TELEM];
static mut STACK_MON_BUF: [u8; STACK_MON] = [0u8; STACK_MON];

/* ===================== 启动 ===================== */

/// 构建 "pwmN"（N=0..3）设备名（含结尾 \0）。
pub(crate) fn make_name(n: u8) -> &'static [u8] {
    static mut NAMES: [[u8; 5]; 4] = [*b"pwm0\0", *b"pwm1\0", *b"pwm2\0", *b"pwm3\0"];
    unsafe { &NAMES[n as usize] }
}

/// 初始化共享互斥量并创建四任务。由 lib.rs::rust_app_start 调用。
pub fn spawn_flyctrl() {
    // 运行时填充 EST_STATE（其 static 被强制进 .bss，初始化器已丢弃，必须此处填充）。
    unsafe { (*core::ptr::addr_of_mut!(EST_STATE)) = EstState::empty(); }

    // 初始化互斥量（天花板优先级取可能锁定者的最高 prio）
    unsafe {
        SENSOR_MTX.init(RTOS_PRIO_BH_HIGH); // sensors(5)/control(4) 都可能锁
        EST_MTX.init(RTOS_PRIO_BH_HIGH);    // control(4)/telem(12)/monitor(14)
    }

    // control：硬实时 prio=4, priv=1, RTOS_RT_HARD
    spawn_rt(
        b"control\0",
        control::control_entry,
        RTOS_PRIO_BH_HIGH,
        unsafe { STACK_CTRL_BUF.as_mut_ptr() },
        STACK_CTRL,
        1,
        RT_HARD,
        0,
        0,
    );
    // sensors：软实时 prio=5, priv=1
    spawn_rt(
        b"sensors\0",
        sensors_task::sensors_entry,
        5,
        unsafe { STACK_SENS_BUF.as_mut_ptr() },
        STACK_SENS,
        1,
        RT_NONE,
        0,
        0,
    );
    // telemetry：prio=12, priv=1
    spawn_rt(
        b"telem\0",
        telemetry::telemetry_entry,
        crate::abi::RTOS_PRIO_MAIN,
        unsafe { STACK_TELEM_BUF.as_mut_ptr() },
        STACK_TELEM,
        1,
        RT_NONE,
        0,
        0,
    );
    // monitor：prio=14, priv=1
    spawn_rt(
        b"monitor\0",
        monitor::monitor_entry,
        crate::abi::RTOS_PRIO_BLINK,
        unsafe { STACK_MON_BUF.as_mut_ptr() },
        STACK_MON,
        1,
        RT_NONE,
        0,
        0,
    );

    crate::info!(tag: "flyctrl", "spawned 4 tasks: control/sensors/telem/monitor");
}
