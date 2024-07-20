use coordinate_frame::EastNorthUp;
use num_traits::Float;
use serde::Deserialize;

use marg_orientation::types::GyroscopeReading;

use crate::simulation_utils::{Time, Timed};

/// L3GD20 gyroscope readings.
#[derive(Debug, Deserialize)]
pub struct L3GD20Gyro {
    /// The sample time, in seconds, relative to the Unix epoch.
    #[serde(rename = "host_time")]
    pub time: f64,
    /// Gyroscope reading on the x-axis.
    #[serde(rename = "x")]
    pub gyro_x: f32,
    /// Gyroscope reading on the y-axis.
    #[serde(rename = "y")]
    pub gyro_y: f32,
    /// Gyroscope reading on the z-axis.
    #[serde(rename = "z")]
    pub gyro_z: f32,
}

impl L3GD20Gyro {
    pub fn vec_into_timed(vec: Vec<Self>, time_offset: f64) -> Vec<Timed<GyroscopeReading<f32>>> {
        vec.into_iter()
            .map(Timed::from)
            .map(|t| t.with_time_offset(time_offset))
            .collect()
    }
}

impl Time for L3GD20Gyro {
    fn time(&self) -> f64 {
        self.time
    }
}

impl From<&L3GD20Gyro> for GyroscopeReading<f32> {
    fn from(value: &L3GD20Gyro) -> Self {
        // The L3GD20 gyroscope on the STM32F3 Discovery board measures East, North, Up.
        let frame = EastNorthUp::new(value.gyro_x, value.gyro_y, value.gyro_z);
        // Normalize by the sensor value range.
        let frame = frame / 5.714285;
        let frame = frame.map(|x| x.to_radians());
        GyroscopeReading::north_east_down(frame)
    }
}

impl From<L3GD20Gyro> for GyroscopeReading<f32> {
    fn from(value: L3GD20Gyro) -> Self {
        Self::from(&value)
    }
}

impl From<L3GD20Gyro> for Timed<GyroscopeReading<f32>> {
    fn from(value: L3GD20Gyro) -> Self {
        Self {
            time: value.time,
            reading: value.into(),
        }
    }
}
