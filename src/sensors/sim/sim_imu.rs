//! 占位 IMU 源：总线/传感器不可用时启用，平滑正弦激励，避免 FDIR 误判冻结。

use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::ImuSample;

pub struct SimImu {
    t: f32,
}

impl SimImu {
    pub fn new() -> Self {
        Self { t: 0.0 }
    }

    pub fn next(&mut self, dt: f32) -> ImuSample {
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
