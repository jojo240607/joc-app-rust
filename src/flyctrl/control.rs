//! 控制律硬实时任务（核心，4ms 周期）。
//!
//! 流程：取最新传感器帧 → EKF → FDIR → PID → PWM。
//! 读 SENSOR_FRAME（经 SENSOR_MTX）、写 EST_STATE（经 EST_MTX）。

use core::ffi::c_void;

use flyctrl_core::controller::{Controller, PidController, Setpoint};
use flyctrl_core::estimator::{Estimator, EkfEstimator};
use flyctrl_core::fdir::{Fdir, Health};
use flyctrl_core::units::{Meter, MeterPerSecond, Radian, Second};
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, VehicleState};

use crate::abi::RTOS_PRIO_BH_HIGH;
use crate::device::Device;
use crate::ioctl;
use crate::{info, warn};
use crate::rtos_sync::msleep;
use crate::sensors::SimImu;
use crate::flyctrl::{make_name, EST_MTX, EST_STATE, SENSOR_FRAME, SENSOR_MTX};

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
        if seq == 0 { info!(tag: "ctrl", "dbg: loop enter"); }

        // --- 取最新传感器帧（互斥保护，短临界区） ---
        let (imu, rc, gps, baro_alt, armed);
        {
            let _g = unsafe { SENSOR_MTX.guard() };
            let f = unsafe { &*core::ptr::addr_of!(SENSOR_FRAME) };
            imu = f.imu;
            rc = f.rc;
            gps = f.gps;
            baro_alt = f.baro_alt;
            armed = f.armed;
        }
        if seq == 0 { info!(tag: "ctrl", "dbg: sen-mtx got"); }

        // IMU 缺失 → 模拟源（总线异常降级）
        let imu_sample: ImuSample = match imu {
            Some(s) => s,
            None => SimImu::new().next(dt.0),
        };

        // --- 状态估计（EKF；GPS 位置测量可选） ---
        let est: VehicleState = ekf.step(dt, imu_sample, gps);
        if seq == 0 { info!(tag: "ctrl", "dbg: ekf ok"); }

        // --- FDIR 监控（四源可用性；mag 暂用 false，待 I2C 修复后接 sensors 帧） ---
        let mag_ok = false; // TODO: 接 SENSOR_FRAME.mag_ok（待 joc-base I2C 修复）
        let health: Health = fdir.update(&imu_sample, gps.is_some(), baro_alt.is_some(), mag_ok);

        // 解锁瞬间锁定高度基准
        if armed && !alt_locked {
            hold_alt = est.pos[2];
            alt_locked = true;
        } else if !armed {
            alt_locked = false;
        }

        // --- 期望状态：原点定高 + 偏航缓动 ---
        let thr_off = (rc.throttle - 0.5) * 2.0;
        let target_alt = hold_alt.0 - thr_off * 2.0;
        let setpoint = Setpoint {
            pos: [Meter(0.0), Meter(0.0), Meter(target_alt)],
            yaw: Radian(rc.yaw * 0.5),
            vel: [MeterPerSecond(0.0); 3],
        };

        // --- 控制律（armed 且链路健康才输出推力） ---
        let cmd = if armed && rc.fresh && !fdir.critical() {
            pid.control(dt, &setpoint, &est)
        } else {
            ActuatorCmd::zero()
        };
        if seq == 0 { info!(tag: "ctrl", "dbg: pid ok"); }

        // --- 输出 PWM（4 路 ioctl 设占空比 ticks） ---
        if seq == 0 { info!(tag: "ctrl", "dbg: before pwm"); }
        for i in 0..4 {
            if let Some(d) = &pwm_dev[i] {
                let m = cmd.motor[i].clamp(0.0, 1.0);
                let us = 1000.0 + 1000.0 * m;
                let ticks = (us * pwm_period[i] as f32 / 2500.0) as u32;
                let mut t = ticks;
                let rc = d.ioctl(ioctl::PWM_IOCTL_SET_DUTY_TICKS, &mut t as *mut u32 as *mut c_void);
                if seq == 0 { info!(tag: "ctrl", "dbg: pwm{} rc={} ticks={}", i, rc, ticks); }
            }
        }
        if seq == 0 { info!(tag: "ctrl", "dbg: after pwm"); }
        if seq == 0 {
            let (sc, ec) = unsafe { (SENSOR_MTX.debug_count(), EST_MTX.debug_count()) };
            info!(tag: "ctrl", "dbg: sensor-mtx count={} est-mtx count={}", sc, ec);
        }

        // --- 发布估计状态（telemetry/monitor 读） ---
        {
            let _g = unsafe { EST_MTX.guard() };
            if seq == 0 { info!(tag: "ctrl", "dbg: in est-guard"); }
            let s = unsafe { &mut *core::ptr::addr_of_mut!(EST_STATE) };
            if seq == 0 { info!(tag: "ctrl", "dbg: est-addr got"); }
            s.armed = armed;
            if seq == 0 { info!(tag: "ctrl", "dbg: est-armed written"); }
            s.est = est;
            s.health = health;
            if seq == 0 { info!(tag: "ctrl", "dbg: est-written"); }
        }
        if seq == 0 { info!(tag: "ctrl", "dbg: est-mtx got"); }

        seq = seq.wrapping_add(1);
        if first {
            first = false;
            info!(tag: "ctrl",
                  "first loop done; imu_ok={} armed={} crit={} alt={:.2}",
                  imu.is_some(), armed, fdir.critical(), est.pos[2].0);
        }
        if seq % 250 == 0 {
            info!(tag: "ctrl", "hb seq={} armed={} crit={} alt={:.2}",
                  seq, armed, fdir.critical(), est.pos[2].0);
        }

        msleep(4);
    }
}
