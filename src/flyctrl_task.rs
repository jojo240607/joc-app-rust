//! 飞控任务：把 `flyctrl-core` 的真实估计算法 + 控制律 + FDIR + MAVLink 遥测接入 RTOS。
//!
//! 设计要点（与 joc-base RTOS 的契约一致）：
//! - 走 RTOS 的**设备 vtable**（`g_app_slot.dev_*`）做 IO，App 绝不碰裸寄存器（规避 CCM/MPU/DMA 风险）。
//! - 硬实时任务（`rtos_task_create_rt`，prio ≤ RTOS_PRIO_BH_HIGH=4，priv=1），控制律在 Rust 任务里跑，
//!   周期靠 `rtos_msleep` 同步（RTOS 接入 TIM IRQ 后可经 sem 精确同步，本文件留 TODO）。
//! - 设备二进制约定（与 RTOS 侧驱动约定）：
//!     * `"imu"`  read  → 24B = 6×f32 LE (accel.x,y,z, gyro.x,y,z)
//!     * `"pwm"`  write → 16B = 4×f32 LE 归一化推力 [0,1]（X 型混控）
//!     * `"uart"` write → MAVLink v1 帧字节流（遥测下行）
//! - **降级策略**：若 RTOS 尚未提供 `imu`/`pwm` 设备节点（当前阶段），自动降级为内置模拟源，
//!   保证飞控算法链路 + MAVLink 下行仍可运行/验证；RTOS 侧补齐设备后无需改此文件即自动切换。

use core::ffi::{c_char, c_void};
use flyctrl_core::comm::link::{Frame, MAX_FRAME_LEN};
use flyctrl_core::comm::mavlink;
use flyctrl_core::controller::{Controller, PidController, Setpoint};
use flyctrl_core::estimator::{Estimator, EkfEstimator};
use flyctrl_core::fdir::{Fdir, RtlHome};
use flyctrl_core::units::{Meter, Radian, Second};
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, MeterPerSecondSquared, RadianPerSecond, VehicleState};

use crate::abi::*;

/// 控制环周期（ms）。400Hz 与典型 PWM 频率同量级（RTOS 接入后可经 TIM IRQ 精确同步）。
const FC_LOOP_MS: u32 = 4;

/// 安全封装 RTOS 设备 vtable：open/read/write/close。
/// name 为 RTOS 设备节点名（如 "imu"/"pwm"/"uart"）。
struct Device {
    dev: *mut device_t,
    opened: bool,
}

impl Device {
    /// 经 `dev_get` 取得设备指针并 `open`。返回 None 表示 RTOS 未提供该节点（降级用）。
    fn open(name: &[u8]) -> Option<Self> {
        let dev = unsafe {
            match g_app_slot.dev_get {
                Some(get) => get(name.as_ptr() as *const c_char),
                None => return None, // RTOS 未填充服务表
            }
        };
        if dev.is_null() {
            return None;
        }
        let rc = unsafe {
            match g_app_slot.dev_open {
                Some(open) => open(dev),
                None => return None,
            }
        };
        if rc != 0 {
            return None;
        }
        Some(Self { dev, opened: true })
    }

    /// 读 `buf.len()` 字节到 `buf`。返回实际读取字节数（<0 视为错误）。
    fn read(&self, buf: &mut [u8]) -> i32 {
        unsafe {
            match g_app_slot.dev_read {
                Some(read) => read(self.dev, buf.as_mut_ptr() as *mut c_void, buf.len()),
                None => -1,
            }
        }
    }

    /// 写 `buf` 全部字节。返回实际写入字节数（<0 视为错误）。
    fn write(&self, buf: &[u8]) -> i32 {
        unsafe {
            match g_app_slot.dev_write {
                Some(write) => write(self.dev, buf.as_ptr() as *const c_void, buf.len()),
                None => -1,
            }
        }
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        if self.opened {
            unsafe {
                if let Some(close) = g_app_slot.dev_close {
                    close(self.dev);
                }
            }
            self.opened = false;
        }
    }
}

/// 从 24B 小端缓冲区解析 IMU 样本（accel3 + gyro3）。
fn parse_imu(buf: &[u8]) -> Option<ImuSample> {
    if buf.len() < 24 {
        return None;
    }
    let f = |o: usize| f32::from_le_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]);
    Some(ImuSample {
        accel: [
            MeterPerSecondSquared(f(0)),
            MeterPerSecondSquared(f(4)),
            MeterPerSecondSquared(f(8)),
        ],
        gyro: [
            RadianPerSecond(f(12)),
            RadianPerSecond(f(16)),
            RadianPerSecond(f(20)),
        ],
    })
}

/// 占位 IMU 源（RTOS 未提供 imu 设备时启用）：平滑正弦激励，避免 FDIR 误判冻结。
struct SimImu {
    t: f32,
}

impl SimImu {
    fn new() -> Self { Self { t: 0.0 } }
    fn next(&mut self, dt: f32) -> ImuSample {
        self.t += dt;
        ImuSample {
            accel: [
                MeterPerSecondSquared(0.05 * libm::sinf(self.t)),
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(9.81 + 0.05 * libm::cosf(self.t)),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        }
    }
}

/// 把 4 路归一化推力打包为 16B 小端。
fn pack_pwm(cmd: &ActuatorCmd) -> [u8; 16] {
    let mut b = [0u8; 16];
    for i in 0..4 {
        b[i * 4..i * 4 + 4].copy_from_slice(&cmd.motor[i].to_le_bytes());
    }
    b
}

/// 飞控主循环（硬实时任务入口）。
///
/// 通过 RTOS 设备 vtable 接入 IMU/PWM/UART；若设备不存在则降级为模拟源，
/// 仍驱动 EKF + PID + FDIR + MAVLink 遥测链路，供整链路验证。
pub extern "C" fn flyctrl_entry(_arg: *mut core::ffi::c_void) {
    // --- 设备接入（可降级） ---
    let imu_dev = Device::open(b"imu\0");
    let pwm_dev = Device::open(b"pwm\0");
    let uart_dev = Device::open(b"uart\0");

    // --- 飞控核心对象 ---
    let mut ekf = EkfEstimator::new(0.98, 0.01, 0.001, 0.1);
    let mut pid = PidController::default_quad();
    let mut fdir = Fdir::new();
    let mut rtl_home = RtlHome::new();

    // 期望状态：原点悬停（真实应来自 MAVLink SETPOINT / 任务规划）。
    let setpoint = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-1.0)], Radian(0.0));

    let mut sim_imu = SimImu::new();
    let mut imu_buf = [0u8; 32];
    let mut frame_buf = [0u8; MAX_FRAME_LEN];
    let mut seq: u8 = 0;
    let armed = false; // TODO: 经 MAVLink COMMAND_LONG 解锁后由命令链路置位（当前未接收入站命令）

    loop {
        let dt = Second((FC_LOOP_MS as f32) / 1000.0);

        // --- 1) 采样 IMU（真实设备或模拟源） ---
        let imu = match &imu_dev {
            Some(d) => {
                let n = d.read(&mut imu_buf);
                let n = if n < 0 { 0 } else { (n as usize).min(imu_buf.len()) };
                match parse_imu(&imu_buf[..n]) {
                    Some(s) => s,
                    None => sim_imu.next(dt.0), // 帧异常降级
                }
            }
            None => sim_imu.next(dt.0),
        };

        // --- 2) 状态估计（EKF；位置测量当前用 None，待 GPS/baro 设备接入） ---
        let est: VehicleState = ekf.step(dt, imu, None);

        // --- 3) FDIR 监控（四源；当前仅 IMU 真实，其余占位可用） ---
        // TODO: gps/baro/mag 经各自设备 read 后传入；现以 true 占位（RTOS 接入后替换）。
        let health = fdir.update(&imu, true, true, true);
        if !rtl_home.locked {
            let _ = rtl_home.try_lock(flyctrl_core::vehicle::Ned::new(
                est.pos[0].0, est.pos[1].0, est.pos[2].0,
            ));
        }

        // --- 4) 控制律（armed 才输出推力，否则零油门） ---
        let cmd: ActuatorCmd = if armed {
            pid.control(dt, &setpoint, &est)
        } else {
            ActuatorCmd::zero()
        };

        // --- 5) 输出 PWM（真实设备或丢弃） ---
        if let Some(d) = &pwm_dev {
            let _ = d.write(&pack_pwm(&cmd));
        }

        // --- 6) 遥测下行（标准 MAVLink；uart 不存在则跳过） ---
        if let Some(d) = &uart_dev {
            let mode: u8 = 0; // TODO: 与 flightmode::FlightMode 对齐
            let n = mavlink::encode_heartbeat(mode, armed, seq, &mut frame_buf);
            let _ = d.write(Frame::from_bytes(&frame_buf[..n]).as_slice());
            let n = mavlink::encode_local_pos_from(mavlink::SYS_ID, &est, seq, &mut frame_buf);
            let _ = d.write(Frame::from_bytes(&frame_buf[..n]).as_slice());
            let n = mavlink::encode_sys_status(health != flyctrl_core::fdir::Health::Critical, seq, &mut frame_buf);
            let _ = d.write(Frame::from_bytes(&frame_buf[..n]).as_slice());
        }

        seq = seq.wrapping_add(1);

        // --- 7) 节拍（RTOS 接入 TIM IRQ 后可经 sem 同步，留 TODO） ---
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
