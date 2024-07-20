use crate::gyro_drift::GyroscopeAxisEstimator;
use crate::types::{GyroscopeBias, GyroscopeNoise, GyroscopeReading};
use minikalman::matrix::MatrixDataType;

/// An estimator that separates gyroscope angular velocities from axis-dependent bias terms.
pub struct GyroRateAndDriftEstimator<T> {
    x_filter: GyroscopeAxisEstimator<T>,
    y_filter: GyroscopeAxisEstimator<T>,
    z_filter: GyroscopeAxisEstimator<T>,
}

impl<T> GyroRateAndDriftEstimator<T> {
    pub fn new(
        drift_estimate: GyroscopeBias<T>,
        axis_noise: GyroscopeNoise<T>,
        process_noise: T,
    ) -> Self
    where
        T: MatrixDataType + Default,
    {
        Self {
            x_filter: GyroscopeAxisEstimator::new(
                drift_estimate.omega_x,
                axis_noise.x,
                process_noise,
            ),
            y_filter: GyroscopeAxisEstimator::new(
                drift_estimate.omega_y,
                axis_noise.y,
                process_noise,
            ),
            z_filter: GyroscopeAxisEstimator::new(
                drift_estimate.omega_z,
                axis_noise.z,
                process_noise,
            ),
        }
    }

    pub fn rate_estimate(&self) -> GyroscopeReading<T>
    where
        T: MatrixDataType,
    {
        GyroscopeReading::new(
            self.x_filter.angular_velocity(),
            self.y_filter.angular_velocity(),
            self.z_filter.angular_velocity(),
        )
    }

    pub fn bias_estimate(&self) -> GyroscopeBias<T>
    where
        T: MatrixDataType,
    {
        GyroscopeBias::new(
            self.x_filter.bias(),
            self.y_filter.bias(),
            self.z_filter.bias(),
        )
    }

    pub fn update(&mut self, delta_t: T)
    where
        T: MatrixDataType,
    {
        self.x_filter.predict(delta_t);
        self.y_filter.predict(delta_t);
        self.z_filter.predict(delta_t);
    }

    pub fn correct(&mut self, raw_reading: GyroscopeReading<T>)
    where
        T: MatrixDataType,
    {
        self.x_filter.correct(raw_reading.omega_x);
        self.y_filter.correct(raw_reading.omega_y);
        self.z_filter.correct(raw_reading.omega_z);
    }
}
