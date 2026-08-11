//! 虚拟 RC 驱动：从全局 `PLAYBACK` 读取回放数据集，伪装成真实遥控接收机。
//!
//! 与 `rc::sbus::RcSbus` 实现同一 `RcReceiver` trait，使 `sensors_task`
//! 只需切换数据源即可，无需改动采集逻辑。

use flyctrl_core::hal::sensor::RcReceiver;
use flyctrl_core::vehicle::RcInput;

use crate::sensors::dataset::{Frame, PLAYBACK};

/// 虚拟数据闭环演示开关：虚拟 RC 强制 `armed=true`，使控制律 PID→PWM 闭环真正执行。
/// 数据集 `rc` 通道无 armed 位，正常回放语义应为 false（飞控不输出推力）。
/// 当前保留为 `true`：以纯虚拟数据集驱动姿态解算→PID→PWM 整链路闭环演示，
/// 用于板上验证飞控算法在无需真实硬件传感器时也能正常运行。
const RC_FORCE_ARM: bool = true;

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
