use minikalman::buffers::types::*;
use minikalman::matrix::MatrixDataType;
use minikalman::prelude::*;
use minikalman::regular::{RegularKalmanBuilder, RegularObservationBuilder};

use crate::gyro_drift::types::*;

/// An estimator for gyroscope axis-specific angular velocity and bias.
pub struct GyroscopeAxisEstimator<T> {
    filter: OwnedKalmanFilter<T>,
    measurement: OwnedObservation<T>,
}

impl<T> GyroscopeAxisEstimator<T> {
    /// Initializes a new instance of the [`GyroscopeAxisEstimator`] struct.
    ///
    /// ## Arguments
    /// * `drift_estimate` - The initial estimate for the axis drift value.
    /// * `gyro_noise` - The gyroscope noise values (sigma-squared) for the axis.
    /// * `process_noise` - A process noise value.
    pub fn new(drift_estimate: T, gyro_noise: T, process_noise: T) -> Self
    where
        T: MatrixDataType + Default,
    {
        let filter = Self::build_filter(drift_estimate, gyro_noise, process_noise);
        let measurement = Self::build_measurement(gyro_noise);

        Self {
            filter,
            measurement,
        }
    }
}

impl<T> GyroscopeAxisEstimator<T> {
    /// Performs a prediction step to obtain new gyroscope estimates.
    pub fn predict(&mut self, delta_t: T)
    where
        T: MatrixDataType,
    {
        self.filter.state_transition_mut().set(0, 1, delta_t);
        self.filter.predict();
    }

    /// Performs a correction step using accelerometer and magnetometer readings.
    ///
    /// ## Arguments
    /// * `accelerometer` - The accelerometer reading.
    pub fn correct(&mut self, angular_velocity: T)
    where
        T: MatrixDataType,
    {
        self.measurement.measurement_vector_mut().apply(|vec| {
            vec.set_row(0, -angular_velocity);
        });

        // Perform the update step.
        self.filter.correct(&mut self.measurement);
    }

    /// Gets the estimated angular velocity.
    pub fn angular_velocity(&self) -> T
    where
        T: Copy,
    {
        self.filter.state_vector().get_row(0)
    }

    /// Gets the estimated bias term.
    pub fn bias(&self) -> T
    where
        T: Copy,
    {
        self.filter.state_vector().get_row(1)
    }
}

impl<T> GyroscopeAxisEstimator<T> {
    /// Builds the Kalman filter used for prediction.
    fn build_filter(
        drift_estimate: T,
        sensor_noise: T,
        process_noise_value: T,
    ) -> OwnedKalmanFilter<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // State vector.
        let mut state_vec =
            StateVectorBuffer::<STATES, T, _>::new(MatrixData::new_array::<STATES, 1, STATES, T>(
                [zero; STATES],
            ));
        state_vec.apply(|vec| {
            vec.set_row(0, T::zero());
            vec.set_row(1, drift_estimate);
        });

        // State transition matrix.
        let mut state_transition =
            StateTransitionMatrixMutBuffer::<STATES, T, _>::new(MatrixData::new_array::<
                STATES,
                STATES,
                { STATES * STATES },
                T,
            >(
                [zero; { STATES * STATES }]
            ));
        state_transition.make_identity();

        // Estimate covariance matrix.
        let mut estimate_covariance =
            EstimateCovarianceMatrixBuffer::<STATES, T, _>::new(MatrixData::new_array::<
                STATES,
                STATES,
                { STATES * STATES },
                T,
            >(
                [zero; { STATES * STATES }]
            ));
        estimate_covariance.make_scalar(sensor_noise);

        // Process noise matrix.
        let mut process_noise = DirectProcessNoiseCovarianceMatrixMutBuffer::<STATES, T, _>::new(
            MatrixData::new_array::<STATES, STATES, { STATES * STATES }, T>(
                [zero; { STATES * STATES }],
            ),
        );
        process_noise.make_scalar(process_noise_value);

        // Predicted state vector.
        let predicted_state =
            PredictedStateEstimateVectorBuffer::<STATES, T, _>::new(MatrixData::new_array::<
                STATES,
                1,
                STATES,
                T,
            >([zero; STATES]));

        // Temporary estimate covariance matrix.
        let temp_state_matrix =
            TemporaryStateMatrixBuffer::<STATES, T, _>::new(MatrixData::new_array::<
                STATES,
                STATES,
                { STATES * STATES },
                T,
            >(
                [zero; { STATES * STATES }]
            ));

        RegularKalmanBuilder::new::<STATES, T>(
            state_transition,
            state_vec,
            estimate_covariance,
            process_noise,
            predicted_state,
            temp_state_matrix,
        )
    }

    /// Builds the Kalman filter observation used for the prediction.
    fn build_measurement(axis_noise: T) -> OwnedObservation<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // Measurement vector
        let measurement =
            MeasurementVectorBuffer::<OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                OBSERVATIONS,
                1,
                OBSERVATIONS,
                T,
            >([zero; OBSERVATIONS]));

        // Observation matrix
        let mut observation_matrix =
            ObservationMatrixMutBuffer::<OBSERVATIONS, STATES, T, _>::new(MatrixData::new_array::<
                OBSERVATIONS,
                STATES,
                { OBSERVATIONS * STATES },
                T,
            >(
                [zero; { OBSERVATIONS * STATES }],
            ));
        observation_matrix.apply(|mat| {
            mat.set_col(0, T::one());
            mat.set_col(1, T::zero());
        });

        // Measurement noise covariance
        let mut noise_covariance =
            MeasurementNoiseCovarianceMatrixBuffer::<OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    OBSERVATIONS,
                    OBSERVATIONS,
                    { OBSERVATIONS * OBSERVATIONS },
                    T,
                >([zero; { OBSERVATIONS * OBSERVATIONS }]),
            );
        noise_covariance.apply(|mat| {
            mat.set_at(0, 0, axis_noise);
        });

        // Innovation vector
        let innovation_vector =
            InnovationVectorBuffer::<OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                OBSERVATIONS,
                1,
                OBSERVATIONS,
                T,
            >([zero; OBSERVATIONS]));

        // Innovation covariance matrix
        let innovation_covariance =
            InnovationCovarianceMatrixBuffer::<OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                OBSERVATIONS,
                OBSERVATIONS,
                { OBSERVATIONS * OBSERVATIONS },
                T,
            >(
                [zero; { OBSERVATIONS * OBSERVATIONS }],
            ));

        // Kalman Gain matrix
        let kalman_gain =
            KalmanGainMatrixBuffer::<STATES, OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                STATES,
                OBSERVATIONS,
                { STATES * OBSERVATIONS },
                T,
            >(
                [zero; { STATES * OBSERVATIONS }],
            ));

        // Temporary residual covariance inverted matrix
        let temp_sinv = TemporaryResidualCovarianceInvertedMatrixBuffer::<OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<OBSERVATIONS, OBSERVATIONS, { OBSERVATIONS * OBSERVATIONS }, T>(
                [zero; { OBSERVATIONS * OBSERVATIONS }],
            ),
        );

        // Temporary H×P matrix
        let temp_hp =
            TemporaryHPMatrixBuffer::<OBSERVATIONS, STATES, T, _>::new(MatrixData::new_array::<
                OBSERVATIONS,
                STATES,
                { OBSERVATIONS * STATES },
                T,
            >(
                [zero; { OBSERVATIONS * STATES }],
            ));

        // Temporary P×Hᵀ matrix
        let temp_pht =
            TemporaryPHTMatrixBuffer::<STATES, OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                STATES,
                OBSERVATIONS,
                { STATES * OBSERVATIONS },
                T,
            >(
                [zero; { STATES * OBSERVATIONS }],
            ));

        // Temporary K×(H×P) matrix
        let temp_khp = TemporaryKHPMatrixBuffer::<STATES, T, _>::new(MatrixData::new_array::<
            STATES,
            STATES,
            { STATES * STATES },
            T,
        >(
            [zero; { STATES * STATES }]
        ));

        RegularObservationBuilder::new::<STATES, OBSERVATIONS, T>(
            observation_matrix,
            measurement,
            noise_covariance,
            innovation_vector,
            innovation_covariance,
            kalman_gain,
            temp_sinv,
            temp_hp,
            temp_pht,
            temp_khp,
        )
    }
}
