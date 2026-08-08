//! 飞控任务编排：把 `flyctrl-core` 的真实估计算法 + 控制律 + FDIR + MAVLink 遥测接入 RTOS。
//!
//! 设计要点（与 joc-base RTOS 的契约一致）：
//! - 走 RTOS 的**设备 vtable**（`g_app_slot.dev_*`）做 IO，App 绝不碰裸寄存器（规避 CCM/MPU/DMA 风险）。
//! - **RTOS 层只提供总线/通用外设驱动**（spi0/i2c0/uart0/pwm0/timerX/adc0…）；**具体传感器设备由 Rust 应用层
//!   用总线组合构建**（见 `crate::sensors`）。这正是方案 Y 的边界。
//! - 硬实时任务（`rtos_task_create_rt`，prio ≤ RTOS_PRIO_BH_HIGH=4，priv=1），控制律在 Rust 任务里跑。
//! - **节拍**：经 RTOS 既有 `msleep`（SysTick 1000Hz 已驱动 RTOS 调度）。Rust 侧**不抢 TIM 中断**（timer 已被
//!   C 侧 event_device 占用其 IRQ 线）；若要亚毫秒精确节拍，由 C 侧 timer 驱动补「溢出→sem」桥。
//! - **降级**：若总线未注册或传感器无响应，IMU→SimImu 模拟源，Baro/Mag/Gps→标记源不可用，链路仍可验证。
//!
//! 设备/总线二进制约定：
//!   * `"uart0"` write → MAVLink v1 帧字节流（遥测下行，RTOS 已注册 USART1）
//!   * `"pwm0"`  write → 16B = 4×f32 LE 归一化推力 [0,1]（X 型混控，RTOS 已注册）
//!   * `"i2c0"`  ioctl(I2C_IOCTL_MASTER_*) → 挂载 I2C 传感器的总线（RTOS 已注册 I2C1）
//!   * `"uart1"` read  → GPS NMEA/UBX 字节流（RTOS 已注册 USART2；当前降级）

use core::ffi::c_char;

use flyctrl_core::comm::link::{Frame, MAX_FRAME_LEN};
use flyctrl_core::comm::mavlink;
use flyctrl_core::controller::{Controller, PidController, Setpoint};
use flyctrl_core::estimator::{Estimator, EkfEstimator};
use flyctrl_core::fdir::{Fdir, Health, RtlHome};
use flyctrl_core::units::{Meter, Radian, Second};
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, Ned, PosSample, VehicleState};

use crate::abi::*;
use crate::device::Device;
use crate::sensors::{BaroBmp280, ImuMpu6050, MagQmc5883, SimImu};

/// 控制环周期（ms）。当前经 RTOS `msleep` 节拍；与典型 PWM 频率同量级。
const FC_LOOP_MS: u32 = 4;

/// 把 4 路归一化推力打包为 16B 小端。
fn pack_pwm(cmd: &ActuatorCmd) -> [u8; 16] {
    let mut b = [0u8; 16];
    for i in 0..4 {
        b[i * 4..i * 4 + 4].copy_from_slice(&cmd.motor[i].to_le_bytes());
    }
    b
}

/// 飞控主循环（硬实时任务入口）。
pub extern "C" fn flyctrl_entry(_arg: *mut core::ffi::c_void) {
    // --- 总线/外设接入（RTOS 已注册：uart0/pwm0/i2c0/spi0） ---
    let pwm_dev = Device::open(b"pwm0\0");    // 电机输出
    let uart_dev = Device::open(b"uart0\0");  // 遥测下行（USART1）
    let gps_dev = Device::open(b"uart1\0");   // GPS 串口（USART2；当前降级）

    // --- 传感器：由 Rust 经总线（i2c0）组合构建；缺失则降级（见 crate::sensors） ---
    let imu = ImuMpu6050::new(b"i2c0\0", 0x68); // MPU6050 @ I2C1
    let baro = BaroBmp280::new(b"i2c0\0", 0x76); // BMP280 @ I2C1
    let mag = MagQmc5883::new(b"i2c0\0", 0x0D); // QMC5883L @ I2C1

    // --- 飞控核心对象 ---
    let mut ekf = EkfEstimator::new(0.98, 0.01, 0.001, 0.1);
    let mut pid = PidController::default_quad();
    let mut fdir = Fdir::new();
    let mut rtl_home = RtlHome::new();

    // 期望状态：原点悬停（真实应来自 MAVLink SETPOINT / 任务规划）。
    let setpoint = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-1.0)], Radian(0.0));

    let mut sim_imu = SimImu::new();
    let mut frame_buf = [0u8; MAX_FRAME_LEN];
    let mut gps_buf = [0u8; 128];
    let mut seq: u8 = 0;
    let armed = false; // TODO: 经 MAVLink COMMAND_LONG 解锁后由命令链路置位（当前未接收入站命令）
    let last_gps: Option<PosSample> = None;

    loop {
        let dt = Second((FC_LOOP_MS as f32) / 1000.0);

        // --- 1) 采样 IMU（MPU6050 或模拟源） ---
        let imu_sample: ImuSample = match imu.as_ref() {
            Some(s) => match s.read() {
                Some(smpl) => smpl,
                None => sim_imu.next(dt.0), // 总线异常降级
            },
            None => sim_imu.next(dt.0),
        };

        // --- 2) 采样 GPS（uart1 NMEA/UBX，可降级） ---
        let gps = match &gps_dev {
            Some(d) => {
                let n = d.read(&mut gps_buf);
                // TODO: 解析 NMEA/UBX → NED。当前仅占位：标记有串口但解析待实现。
                if n > 0 {
                    last_gps // 占位：未解析，沿用上一拍
                } else {
                    last_gps
                }
            }
            None => None,
        };
        let gps_available = gps.is_some();

        // --- 3) 采样 Baro（高度观测，可降级） ---
        let baro_available = match baro.as_ref() {
            Some(b) => b.read_altitude().is_some(),
            None => false,
        };

        // --- 4) 状态估计（EKF；GPS 位置测量可选） ---
        let est: VehicleState = ekf.step(dt, imu_sample, gps);

        // --- 5) FDIR 监控（四源可用性） ---
        let mag_available = mag.as_ref().map_or(false, |m| m.read().is_some());
        let health: Health = fdir.update(&imu_sample, gps_available, baro_available, mag_available);
        if !rtl_home.locked {
            let _ = rtl_home.try_lock(Ned::new(est.pos[0].0, est.pos[1].0, est.pos[2].0));
        }

        // --- 6) 控制律（armed 才输出推力，否则零油门） ---
        let cmd: ActuatorCmd = if armed && !fdir.critical() {
            pid.control(dt, &setpoint, &est)
        } else {
            ActuatorCmd::zero()
        };

        // --- 7) 输出 PWM ---
        if let Some(d) = &pwm_dev {
            let _ = d.write(&pack_pwm(&cmd));
        }

        // --- 8) 遥测下行（标准 MAVLink） ---
        if let Some(d) = &uart_dev {
            let mode: u8 = 0; // TODO: 与 flightmode::FlightMode 对齐
            let n = mavlink::encode_heartbeat(mode, armed, seq, &mut frame_buf);
            let _ = d.write(Frame::from_bytes(&frame_buf[..n]).as_slice());
            let n = mavlink::encode_local_pos_from(mavlink::SYS_ID, &est, seq, &mut frame_buf);
            let _ = d.write(Frame::from_bytes(&frame_buf[..n]).as_slice());
            let n = mavlink::encode_sys_status(health != Health::Critical, seq, &mut frame_buf);
            let _ = d.write(Frame::from_bytes(&frame_buf[..n]).as_slice());
        }

        seq = seq.wrapping_add(1);

        // --- 9) 节拍：经 RTOS 既有 msleep（SysTick 1000Hz 已驱动调度） ---
        unsafe {
            if let Some(msleep) = g_app_slot.msleep {
                msleep(FC_LOOP_MS);
            }
        }
    }
}

/// 在 app_main 中调用：以硬实时任务创建飞控任务。
pub fn spawn_flyctrl_task() {
    // 硬实时属性：周期 4ms、截止 4ms、WCET 留余量（单位 tick；RTOS tick=1ms）。
    static ATTR: rtos_task_attr_t = rtos_task_attr_t {
        rt_class: RTOS_RT_HARD,
        deadline_ticks: FC_LOOP_MS,
        wcet_ticks: (FC_LOOP_MS * 3) / 4,
    };
    // 静态栈（CCM 不可用，放主 SRAM；RTOS 侧调度器管理）。
    static mut STACK: [u8; 2048] = [0u8; 2048];

    unsafe {
        if let Some(create_rt) = g_app_slot.task_create_rt {
            let stack = STACK.as_mut_ptr() as *mut core::ffi::c_void;
            create_rt(
                b"flyctrl\0".as_ptr() as *const c_char,
                flyctrl_entry,
                core::ptr::null_mut(),
                RTOS_PRIO_BH_HIGH, // prio ≤ 4，硬实时带
                stack,
                STACK.len(),
                1, // priv=1：直接经 vtable 操作外设，最低延迟
                &ATTR as *const rtos_task_attr_t,
            );
        }
    }
}
