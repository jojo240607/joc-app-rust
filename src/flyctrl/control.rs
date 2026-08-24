//! 控制律硬实时任务（核心，4ms 周期）。
//!
//! 流程：取最新传感器帧 → EKF → FDIR → PID → PWM。
//! 读 SENSOR_FRAME（经 seqlock，见 `SENSOR_SEQ`）、写 EST_STATE（经 EST_MTX）。

use core::ffi::c_void;

use flyctrl_core::controller::{Controller, PidController, Setpoint};
use flyctrl_core::estimator::{Estimator, EkfEstimator};
use flyctrl_core::fdir::{Fdir, Health};
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, Second};
#[cfg(not(feature = "hil"))]
use flyctrl_core::units::Radian;
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, RcInput, VehicleState};

use crate::abi::RTOS_PRIO_BH_HIGH;
use crate::device::Device;
use crate::ioctl;
use crate::{info, warn};
use crate::rtos_sync::msleep;
use core::sync::atomic::Ordering;
use crate::sensors::SimImu;

/// 诊断开关：开启后会在启动前几圈打印大量 dbg 行，极易压垮开机瞬间的
/// 设备串口 TX 缓冲、导致同期的传感器任务日志被丢弃（误判传感器任务“死亡”）。
/// 正常验证时关闭。
const VERBOSE: bool = false;
use crate::flyctrl::{make_name, EST_MTX, EST_STATE, SENSOR_FRAME, SENSOR_SEQ};

/// 上行指令解锁：地面站经 COMMAND_LONG(ARM/DISARM) 设置。
/// 与控制律内部 RC 解锁做逻辑或（任一为真即解锁）。
pub fn set_cmd_armed(arm: bool) {
    crate::flyctrl::uplink::G_CMD_ARMED.store(arm, Ordering::Relaxed);
}

/// 上行指令模式：地面站经 COMMAND_LONG(DO_SET_MODE) 设置。
/// telemetry 心跳 custom_mode 会读取此值反映当前模式。
pub fn set_cmd_mode(mode: u16) {
    crate::flyctrl::uplink::G_CMD_MODE.store(mode, Ordering::Relaxed);
}

/// 控制律硬实时任务入口。
pub extern "C" fn control_entry(_arg: *mut c_void) {
    info!(tag: "ctrl", "task started; period=4ms prio={}", RTOS_PRIO_BH_HIGH);

    // 控制律对象（每周期复用，避免重复分配）。
    let mut ekf = EkfEstimator::default_quad();
    let mut fdir = Fdir::new();
    let mut pid = PidController::default_quad();
    let mut hold_alt = Meter(0.0);
    let mut alt_locked = false;
    let mut seq: u32 = 0;

    // PWM 设备（4 路，control 专用）
    let mut pwm_dev: [Option<Device>; 4] = [None, None, None, None];
    let mut pwm_period: [u32; 4] = [0; 4];
    for i in 0..4 {
        let name = make_name(i as u8);
        if let Some(d) = Device::open(name) {
            // 设 400Hz（2500us 周期），取回 period_ticks 供占空比换算
            let mut freq = 400u32;
            let _ = d.ioctl(ioctl::PWM_IOCTL_SET_FREQ, &mut freq as *mut u32 as *mut c_void);
            let mut ticks = 0u32;
            let _ = d.ioctl(ioctl::PWM_IOCTL_GET_PERIOD_TICKS, &mut ticks as *mut u32 as *mut c_void);
            pwm_period[i] = ticks;
            pwm_dev[i] = Some(d);
        } else {
            warn!(tag: "ctrl", "pwm{} not available -> actuator disabled", i);
        }
    }

    let mut first = true;
    loop {
        let dt = Second(4.0 / 1000.0);
        // 应用地面站参数（每周期原子读 G_PARAM_VALS -> pid 增益；PARAM_SET 即时生效）。
        crate::flyctrl::uplink::sync_gains_to_pid(&mut pid);
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: loop enter"); }

        // --- 取最新传感器帧（seqlock：control 优先级高于所有写者，读不被打断） ---
        // 写者：sensors(prio=5，非 HIL) / uplink(prio=10，HIL)。两者优先级均低于本任务
        // (control, prio=4)，因此读过程不可能被写者抢占 → 单次读即原子一致，无需重试。
        // 【关键】绝不能 `continue` 忙等重试：若赶上写者正处于写入中（SENSOR_SEQ 为奇，
        // 写者被本任务抢占在置奇与置偶之间），忙等会让低优先级写者永远得不到调度，
        // control 无限自旋 → 整机卡死（HIL 注入期间已实测复现：运行数秒后日志/下行全停）。
        // 正确处理：直接采用本拍快照（可能新老混合/略旧），下一 4ms 拍自然取得一致新帧。
        let (mut imu, mut rc, mut gps, mut baro_alt, mut armed);
        unsafe {
            let f = &*core::ptr::addr_of!(SENSOR_FRAME);
            imu = f.imu;
            rc = f.rc;
            gps = f.gps;
            baro_alt = f.baro_alt;
            armed = f.armed;
        }
        // 指令解锁：与地面站上行命令做逻辑或（RC 解锁 或 指令解锁 任一为真）。
        let cmd_armed = crate::flyctrl::uplink::G_CMD_ARMED.load(Ordering::Relaxed);
        let armed_eff = armed || cmd_armed;
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: sen-mtx got"); }

        // 地面站 RC 通道覆盖（RC_CHANNELS_OVERRIDE）：生效时以地面站通道优先于 sim RC。
        // 通道映射：ch1=roll, ch2=pitch, ch3=throttle, ch4=yaw（标准 MAVLink 约定）。
        // PWM 1000-2000us 归一化到 0.0-1.0。
        let (rc_ov, rc_ov_valid) = crate::flyctrl::uplink::get_rc_override();
        let rc = if rc_ov_valid {
            let norm = |pwm: u16| -> f32 {
                let v = (pwm as f32 - 1000.0) / 1000.0;
                v.clamp(0.0, 1.0)
            };
            RcInput {
                throttle: norm(rc_ov[2]),
                roll: norm(rc_ov[0]),
                pitch: norm(rc_ov[1]),
                yaw: norm(rc_ov[3]),
                armed: rc_ov[0] > 1500, // 暂以 ch1 高位作为地面站解锁指示（占位，主解锁仍靠 COMMAND_LONG）
                mode: rc.mode,
                fresh: true,
            }
        } else {
            rc
        };

        // IMU 缺失 → 模拟源（总线异常降级）
        let imu_sample: ImuSample = match imu {
            Some(s) => s,
            None => SimImu::new().next(dt.0),
        };

        // --- 状态估计（EKF；GPS 位置测量可选） ---
        let est: VehicleState = ekf.step(dt, imu_sample, gps, None);

        // --- FDIR 监控（四源可用性；mag 暂用 false，待 I2C 修复后接 sensors 帧） ---
        let mag_ok = false; // TODO: 接 SENSOR_FRAME.mag_ok（待 joc-base I2C 修复）
        let health: Health = fdir.update(&imu_sample, gps.is_some(), baro_alt.is_some(), mag_ok);

        // 解锁瞬间锁定高度基准
        // [BISECT] armed_eff 已退化为 armed
        if armed_eff && !alt_locked {
            hold_alt = est.pos[2];
            alt_locked = true;
        } else if !armed_eff {
            alt_locked = false;
        }

        // --- 期望状态：模式决定目标（原点定高 / RTL 回原点 / LAND 缓降） ---
        // custom_mode 用 ArduCopter 标准码（G_CMD_MODE 由上行 DO_SET_MODE/TAKEOFF/LAND/RTL 写入）。
        // HIL：设定点直接来自 PC 仿真器（SET_POSITION_TARGET_LOCAL_NED），RC 路径编译期关闭。
        #[cfg(feature = "hil")]
        let setpoint = {
            use flyctrl_core::units::Radian;
            let sp = crate::flyctrl::uplink::hil_setpoint();
            if crate::flyctrl::uplink::hil_setpoint_valid() {
                Setpoint {
                    pos: [Meter(sp.x), Meter(sp.y), Meter(sp.z)],
                    yaw: Radian(sp.yaw),
                    vel: [MeterPerSecond(sp.vx), MeterPerSecond(sp.vy), MeterPerSecond(sp.vz)],
                    acc: [MeterPerSecondSquared(sp.afx), MeterPerSecondSquared(sp.afy), MeterPerSecondSquared(sp.afz)],
                }
            } else {
                // sim 尚未连接：保持当前位置定高，避免悬停指令冲击。
                Setpoint {
                    pos: [Meter(0.0), Meter(0.0), est.pos[2]],
                    yaw: Radian(0.0),
                    vel: [MeterPerSecond(0.0); 3],
                    acc: [MeterPerSecondSquared(0.0); 3],
                }
            }
        };
        #[cfg(not(feature = "hil"))]
        let setpoint = {
            let cmd_mode = crate::flyctrl::uplink::G_CMD_MODE.load(Ordering::Relaxed);
            use flyctrl_core::comm::mavlink::enums::COPTER_MODE_LAND;
            let thr_off = (rc.throttle - 0.5) * 2.0;
            // 默认目标：锁定高度基准（原点）。LAND 模式触发持续缓降。
            let mut target_alt = hold_alt.0 - thr_off * 2.0;
            if cmd_mode == COPTER_MODE_LAND {
                // LAND：在基准高度上每周期降 0.02m，趋向地面（D 向下，地面=0）。
                target_alt = (est.pos[2].0 - 0.02).max(0.0);
            }
            // RTL/LOITER 水平目标已为原点（N=0,E=0）；STABILIZE 保持同样基准，确保联调可观测。
            Setpoint {
                pos: [Meter(0.0), Meter(0.0), Meter(target_alt)],
                yaw: Radian(rc.yaw * 0.5),
                vel: [MeterPerSecond(0.0); 3],
                acc: [MeterPerSecondSquared(0.0); 3],
            }
        };

        // --- 控制律（armed 且链路健康才输出推力） ---
        // [BISECT] armed_eff 退化为 armed
        let cmd = if armed_eff && rc.fresh && !fdir.critical() {
            pid.control(dt, &setpoint, &est)
        } else {
            ActuatorCmd::zero()
        };

        // HIL：回传执行器指令供 telemetry 组 HIL_ACTUATOR_CONTROLS（PC 端注入 plant）。
        #[cfg(feature = "hil")]
        crate::flyctrl::uplink::set_actuator_cmd(&cmd.motor);

        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: pid ok"); }

        // --- 输出 PWM（4 路 ioctl 设占空比 ticks） ---
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: before pwm"); }
        for i in 0..4 {
            if let Some(d) = &pwm_dev[i] {
                let m = cmd.motor[i].clamp(0.0, 1.0);
                let us = 1000.0 + 1000.0 * m;
                let ticks = (us * pwm_period[i] as f32 / 2500.0) as u32;
                let mut t = ticks;
                let rc = d.ioctl(ioctl::PWM_IOCTL_SET_DUTY_TICKS, &mut t as *mut u32 as *mut c_void);
                if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: pwm{} rc={} ticks={}", i, rc, ticks); }
            }
        }
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: after pwm"); }
        if VERBOSE && seq == 0 {
            let ec = unsafe { EST_MTX.debug_count() };
            info!(tag: "ctrl", "dbg: est-mtx count={} sensor-seq={}", ec, unsafe { SENSOR_SEQ });
        }

        // --- 发布估计状态（telemetry/monitor 读） ---
        {
            let _g = unsafe { EST_MTX.guard() };
            if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: in est-guard"); }
            let s = unsafe { &mut *core::ptr::addr_of_mut!(EST_STATE) };
            if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: est-addr got"); }
            s.armed = armed_eff; // [BISECT] 退化为 armed
            if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: est-armed written"); }
            s.est = est;
            s.health = health;
            if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: est-written"); }
            // 油门百分比(0..100) 供 telemetry 经 VFR_HUD 下发。
            let throttle_avg = (cmd.motor[0] + cmd.motor[1] + cmd.motor[2] + cmd.motor[3]) / 4.0;
            crate::flyctrl::uplink::G_THROTTLE.store((throttle_avg.clamp(0.0, 1.0) * 100.0) as u8, Ordering::Relaxed);
        }
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: est-mtx got"); }

        seq = seq.wrapping_add(1);
        if first {
            first = false;
            info!(tag: "ctrl",
                  "first loop done; imu_ok={} armed={} crit={} alt={:.2}",
                  imu.is_some(), armed_eff, fdir.critical(), est.pos[2].0);
        }
        if seq % 250 == 0 {
            info!(tag: "ctrl", "hb seq={} armed={} crit={} alt={:.2} imu_ok={} gps={} baro={} gz={:.2} m=[{:.3},{:.3},{:.3},{:.3}]",
                  seq, armed_eff, fdir.critical(), est.pos[2].0,
                  imu.is_some(), gps.is_some(), baro_alt.is_some(), est.vel[2].0,
                  cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3]);
        }

        msleep(4);
        if VERBOSE && seq < 5 {
            info!(tag: "ctrl", "dbg: after sleep seq={}", seq);
        }
    }
}
