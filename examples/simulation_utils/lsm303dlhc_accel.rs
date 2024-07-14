use crate::simulation_utils::{Time, Timed};
use coordinate_frame::NorthWestDown;
use marg_orientation::types::AccelerometerReading;
use serde::Deserialize;

/// LSM303DLHC accelerometer readings.
#[derive(Debug, Deserialize)]
pub struct LSM303DLHCAccelerometer {
    /// The sample time, in seconds, relative to the Unix epoch.
    #[serde(rename = "host_time")]
    pub time: f64,
    /// Accelerometer reading on the x-axis.
    #[serde(rename = "x")]
    pub acc_x: f32,
    /// Accelerometer reading on the y-axis.
    #[serde(rename = "y")]
    pub acc_y: f32,
    /// Accelerometer reading on the z-axis.
    #[serde(rename = "z")]
    pub acc_z: f32,
}

impl LSM303DLHCAccelerometer {
    pub fn vec_into_timed(
        vec: Vec<Self>,
        time_offset: f64,
    ) -> Vec<Timed<AccelerometerReading<f32>>> {
        vec.into_iter()
            .map(Timed::from)
            .map(|t| t.with_time_offset(time_offset))
            .collect()
    }
}

impl Time for LSM303DLHCAccelerometer {
    fn time(&self) -> f64 {
        self.time
    }
}

impl From<&LSM303DLHCAccelerometer> for AccelerometerReading<f32> {
    fn from(value: &LSM303DLHCAccelerometer) -> Self {
        // The HMC303DLHC's accelerometer on the STM32F3 Discovery board measures North, West, Down.
        let frame = NorthWestDown::new(value.acc_x, value.acc_y, value.acc_z);
        // Normalize by the sensor value range.
        let frame = frame / 16384.0;
        AccelerometerReading::north_east_down(frame)
    }
}

impl From<LSM303DLHCAccelerometer> for AccelerometerReading<f32> {
    fn from(value: LSM303DLHCAccelerometer) -> Self {
        Self::from(&value)
    }
}

impl From<LSM303DLHCAccelerometer> for Timed<AccelerometerReading<f32>> {
    fn from(value: LSM303DLHCAccelerometer) -> Self {
        Self {
            time: value.time,
            reading: value.into(),
        }
    }
}
