//! 虚拟传感器回放驱动：从全局 `PLAYBACK` 读取真实（形态）数据集，伪装成真实硬件传感器。
//!
//! 四个虚拟驱动分别实现 `ImuSensor` / `GpsSensor` / `BaroSensor` / `RcReceiver` trait，
//! 接口与 `ImuMpu6050` / `GpsUblox` / `BaroBmp280` / `RcSbus` 完全一致，因此 `sensors_task`
//! 只需切换数据源即可，无需改动采集逻辑。
//!
//! 时间同步：所有驱动只读 `PLAYBACK.current()`（不各自推进）；索引推进由 `sensors_task`
//! 每采集周期调用一次 `PLAYBACK.advance(dt)`，保证四类数据在同一帧上对齐。

use flyctrl_core::hal::sensor::{BaroSensor, GpsSensor, ImuSensor, RcReceiver};
use flyctrl_core::units::{Meter, MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::{ImuSample, PosSample, RcInput};

use crate::sensors::dataset::{Frame, PLAYBACK};

/// 虚拟数据闭环演示开关：虚拟 RC 强制 `armed=true`，使控制律 PID→PWM 闭环真正执行。
/// 数据集 `rc` 通道无 armed 位，正常回放语义应为 false（飞控不输出推力）。
/// 当前保留为 `true`：以纯虚拟数据集驱动姿态解算→PID→PWM 整链路闭环演示，
/// 用于板上验证飞控算法在无需真实硬件传感器时也能正常运行。
const RC_FORCE_ARM: bool = true;

/// IMU 加速度计测量的是"比力"（含重力），而 `ImuSample.accel` 语义为机体加速度（不含重力）。
/// 数据集里的 `imu_accel` 为含重力值，这里减去近水平的重力分量得到比力。
const GRAVITY: f32 = 9.81;

pub struct VirtualImu;

impl VirtualImu {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl ImuSensor for VirtualImu {
    fn read(&mut self) -> ImuSample {
        let f: Frame = unsafe { PLAYBACK.current() };
        let ax = f.imu_accel[0];
        let ay = f.imu_accel[1];
        let az = f.imu_accel[2] - GRAVITY; // 去除重力 -> 比力（机体加速度）
        ImuSample {
            accel: [
                MeterPerSecondSquared(ax),
                MeterPerSecondSquared(ay),
                MeterPerSecondSquared(az),
            ],
            gyro: [
                RadianPerSecond(f.imu_gyro[0]),
                RadianPerSecond(f.imu_gyro[1]),
                RadianPerSecond(f.imu_gyro[2]),
            ],
        }
    }

    fn healthy(&self) -> bool {
        true
    }
}

pub struct VirtualBaro;

impl VirtualBaro {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl BaroSensor for VirtualBaro {
    fn read_altitude(&mut self) -> Meter {
        let f: Frame = unsafe { PLAYBACK.current() };
        Meter(f.baro_alt)
    }

    fn healthy(&self) -> bool {
        true
    }
}

pub struct VirtualGps;

impl VirtualGps {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl GpsSensor for VirtualGps {
    fn read(&mut self) -> Option<PosSample> {
        use crate::sensors::dataset::{
            GPS_ORIGIN, METERS_PER_DEG_LAT, METERS_PER_DEG_LON,
        };
        let f: Frame = unsafe { PLAYBACK.current() };
        // 数据集 gps = [lat, lon, alt(m)]；PosSample.pos 为 NED [x,y,z]（z 向下为正）。
        let lat = f.gps[0];
        let lon = f.gps[1];
        let alt = f.gps[2];
        // 相对起点的局部 NED（米）：经纬度差 × 米/度近似。
        let north = (lat - GPS_ORIGIN.0) * METERS_PER_DEG_LAT;
        let east = (lon - GPS_ORIGIN.1) * METERS_PER_DEG_LON;
        Some(PosSample {
            pos: [
                Meter(north),
                Meter(east),
                Meter(-alt), // NED：高度向上为负 z
            ],
        })
    }

    fn healthy(&self) -> bool {
        true
    }
}

pub struct VirtualRc;

impl VirtualRc {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl RcReceiver for VirtualRc {
    fn read(&mut self) -> RcInput {
        let f: Frame = unsafe { PLAYBACK.current() };
        let r = f.rc;
        RcInput {
            throttle: r[0],
            roll: r[1],
            pitch: r[2],
            yaw: r[3],
            armed: RC_FORCE_ARM,
            mode: (r[4].clamp(0.0, 1.0) * 255.0) as u8,
            fresh: true,
        }
    }

    fn healthy(&self) -> bool {
        true
    }
}
