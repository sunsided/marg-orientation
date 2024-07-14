use coordinate_frame::SouthEastUp;
use serde::Deserialize;

use marg_orientation::types::MagnetometerReading;

use crate::simulation_utils::{Time, Timed};

/// LSM303DLHC magnetometer readings.
#[derive(Debug, Deserialize)]
pub struct LSM303DLHCMagnetometer {
    /// The sample time, in seconds, relative to the Unix epoch.
    #[serde(rename = "host_time")]
    pub time: f64,
    /// Magnetometer reading on the x-axis.
    #[serde(rename = "x")]
    pub compass_x: f32,
    /// Magnetometer reading on the y-axis.
    #[serde(rename = "y")]
    pub compass_y: f32,
    /// Magnetometer reading on the z-axis.
    #[serde(rename = "z")]
    pub compass_z: f32,
}

impl LSM303DLHCMagnetometer {
    pub fn vec_into_timed(
        vec: Vec<Self>,
        time_offset: f64,
    ) -> Vec<Timed<MagnetometerReading<f32>>> {
        vec.into_iter()
            .map(Timed::from)
            .map(|t| t.with_time_offset(time_offset))
            .collect()
    }
}

impl Time for LSM303DLHCMagnetometer {
    fn time(&self) -> f64 {
        self.time
    }
}

impl From<&LSM303DLHCMagnetometer> for MagnetometerReading<f32> {
    fn from(value: &LSM303DLHCMagnetometer) -> Self {
        // The HMC303DLHC's magnetometer on the STM32F3 Discovery board measures South, East, Up.
        let frame = SouthEastUp::new(value.compass_x, value.compass_y, value.compass_z);
        // Normalize by the sensor value range.
        let frame = frame / 1100.0;
        MagnetometerReading::north_east_down(frame)
    }
}

impl From<LSM303DLHCMagnetometer> for MagnetometerReading<f32> {
    fn from(value: LSM303DLHCMagnetometer) -> Self {
        Self::from(&value)
    }
}

impl From<LSM303DLHCMagnetometer> for Timed<MagnetometerReading<f32>> {
    fn from(value: LSM303DLHCMagnetometer) -> Self {
        Self {
            time: value.time,
            reading: value.into(),
        }
    }
}
