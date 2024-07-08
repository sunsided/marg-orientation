// Enable no_std mode.
#![cfg_attr(not(feature = "std"), no_std)]
// Ensure allow(unsafe_code) / forbid(unsafe_code) markers.
#![cfg_attr(feature = "unsafe", allow(unsafe_code))]
#![cfg_attr(not(feature = "unsafe"), forbid(unsafe_code))]
// Only enables the `doc_cfg` feature when the `docsrs` configuration attribute is defined.
#![cfg_attr(docsrs, feature(doc_cfg))]

mod accelerometer_noise;
mod accelerometer_reading;
mod euler_angles;
pub mod gyro_free;
mod gyroscope_bias;
mod gyroscope_noise;
mod gyroscope_reading;
mod macros;
mod magnetometer_noise;
mod magnetometer_reading;
mod num_traits;
pub mod test_estimator;
mod vector3;

pub use crate::accelerometer_noise::AccelerometerNoise;
pub use crate::accelerometer_reading::AccelerometerReading;
pub use crate::euler_angles::EulerAngles;
pub use crate::gyroscope_bias::GyroscopeBias;
pub use crate::gyroscope_noise::GyroscopeNoise;
pub use crate::gyroscope_reading::GyroscopeReading;
pub use crate::magnetometer_noise::MagnetometerNoise;
pub use crate::magnetometer_reading::MagnetometerReading;

pub use crate::num_traits::*;
