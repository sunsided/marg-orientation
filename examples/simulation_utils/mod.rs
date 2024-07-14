mod l3gd20_gyro;
mod lsm303dlhc_accel;
mod lsm303ldhc_magnetometer;

use coordinate_frame::WestUpNorth;
use std::ops::Deref;

pub use l3gd20_gyro::*;
pub use lsm303dlhc_accel::*;
pub use lsm303ldhc_magnetometer::*;

/// Kiss3d uses a West, Up, North system by default.
pub type Kiss3DCoordinates<T> = WestUpNorth<T>;

pub trait Time {
    /// Gets the sample time.
    fn time(&self) -> f64;
}

/// A timed reading.
pub struct Timed<R> {
    pub time: f64,
    pub reading: R,
}

impl<T> Timed<T> {
    pub fn with_time_offset(mut self, value: f64) -> Self {
        self.time -= value;
        self
    }
}

impl<R> Time for Timed<R> {
    fn time(&self) -> f64 {
        self.time
    }
}

impl<R> Deref for Timed<R> {
    type Target = R;

    fn deref(&self) -> &Self::Target {
        &self.reading
    }
}

/// Determines the sampling rate of a dataset.
pub fn determine_sampling_rate<T: Time>(data: &[T]) -> (f64, f64) {
    let (total_diff, count) = data.windows(2).fold((0.0, 0), |(sum, cnt), window| {
        let diff = window[1].time() - window[0].time();
        (sum + diff, cnt + 1)
    });
    let sample_time = total_diff / (count as f64);
    let sample_rate = 1.0 / sample_time;
    (sample_rate, sample_time)
}
