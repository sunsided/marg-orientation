use crate::dcm::types::{OwnedKalmanFilter, OwnedVector6Observation, OBSERVATIONS, STATES};
use crate::prelude::*;
use crate::types::{
    AccelerometerNoise, AccelerometerReading, EulerAngles, GyroscopeNoise, GyroscopeReading,
    MagnetometerNoise, MagnetometerReading, Vector3,
};
use crate::IsNaN;
use coordinate_frame::NorthEastDown;
use minikalman::buffers::types::*;
use minikalman::extended::{ExtendedKalmanBuilder, ExtendedObservationBuilder};
use minikalman::matrix::MatrixDataType;
use minikalman::prelude::*;
use minikalman::regular::{RegularKalmanBuilder, RegularObservationBuilder};

pub struct OwnedOrientationEstimator<T> {
    x_filter: OwnedKalmanFilter<T>,
    z_filter: OwnedKalmanFilter<T>,
    x_measurement: OwnedVector6Observation<T>,
    z_measurement: OwnedVector6Observation<T>,
    x_axis: Vector3<T>,
    y_axis: Vector3<T>,
    z_axis: Vector3<T>,
}

impl<T> OwnedOrientationEstimator<T> {
    /// Initializes a new instance of the [`OwnedOrientationEstimator`] struct.
    ///
    /// ## Arguments
    /// * `accelerometer_noise` - The accelerometer noise values (sigma-squared) for each axis.
    /// * `magnetometer_noise` - The magnetometer noise values (sigma-squared) for each axis.
    /// * `gyroscope_noise` - The gyroscope noise values (sigma-squared) for each axis.
    /// * `process_noise` - A process noise value.
    pub fn new(
        accelerometer_noise: AccelerometerNoise<T>,
        magnetometer_noise: MagnetometerNoise<T>,
        gyroscope_noise: GyroscopeNoise<T>,
        process_noise: T,
        alpha_accel: T,
        alpha_mag: T,
        beta_accel: T,
        beta_mag: T,
    ) -> Self
    where
        T: MatrixDataType + Default,
    {
        let x_axis = Vector3::new(T::one(), T::zero(), T::zero());
        let y_axis = Vector3::new(T::zero(), T::one(), T::zero());
        let z_axis = Vector3::new(T::zero(), T::zero(), T::one());

        let x_filter = Self::build_filter(x_axis, process_noise);
        let z_filter = Self::build_filter(z_axis, process_noise);
        let x_measurement = Self::build_measurement(
            accelerometer_noise,
            alpha_accel,
            gyroscope_noise,
            beta_accel,
        );
        let z_measurement =
            Self::build_measurement(magnetometer_noise, alpha_mag, gyroscope_noise, beta_mag);
        Self {
            x_filter,
            z_filter,
            x_measurement,
            z_measurement,
            x_axis,
            y_axis,
            z_axis,
        }
    }
}

impl<T> OwnedOrientationEstimator<T> {
    pub fn rotate_vector_world(&self, vector: Vector3<T>) -> Vector3<T>
    where
        T: MatrixDataType,
    {
        Vector3::new(
            vector.x * self.x_axis.x + vector.y * self.x_axis.y + vector.z * self.x_axis.z,
            vector.x * self.y_axis.x + vector.y * self.y_axis.y + vector.z * self.y_axis.z,
            vector.x * self.z_axis.x + vector.y * self.z_axis.y + vector.z * self.z_axis.z,
        )
    }

    pub fn estimated_angles(&self) -> EulerAngles<T>
    where
        T: Default,
    {
        // TODO: implement angle extraction
        EulerAngles::default()
    }

    pub fn gyro_rates(&self) -> GyroscopeReading<T>
    where
        T: MatrixDataType,
    {
        let state = self.z_filter.state_vector();
        GyroscopeReading::new(state.get_row(3), state.get_row(4), state.get_row(5))
    }

    pub fn north(&self) -> NorthEastDown<T>
    where
        T: MatrixDataType,
    {
        let state = self.x_filter.state_vector();
        NorthEastDown::new(state.get_row(0), state.get_row(1), state.get_row(2))
    }

    pub fn down(&self) -> NorthEastDown<T>
    where
        T: MatrixDataType,
    {
        let state = self.z_filter.state_vector();
        NorthEastDown::new(state.get_row(0), state.get_row(1), state.get_row(2))
    }

    /// Performs a prediction step to obtain new orientation estimates.
    pub fn update(&mut self, delta_t: T)
    where
        T: MatrixDataType + IsNaN,
    {
        // Update the down vector.
        Self::update_state_transition_matrix(&mut self.z_filter, self.z_axis, delta_t);
        self.z_filter.predict();
        self.update_axes_from_accelerometer_filter();
        self.re_orthogonalize_with_updated_z();

        // Update the north vector.
        Self::update_state_transition_matrix(&mut self.x_filter, self.x_axis, delta_t);
        self.x_filter.predict();
        self.update_axes_from_magnetometer_filter();
        self.re_orthogonalize_with_updated_x();

        self.panic_if_nan();
    }

    fn apply_normalized_vector<V, M>(vec: V, measurement: &mut M)
    where
        M: RowVectorMut<6, T>,
        V: Into<Vector3<T>>,
        T: MatrixDataType,
    {
        let vec: Vector3<T> = vec.into().normalized();
        measurement.set_row(0, vec.x);
        measurement.set_row(1, vec.y);
        measurement.set_row(2, vec.z);
    }

    fn apply_gyro<V, M>(gyro: V, measurement: &mut M)
    where
        M: RowVectorMut<6, T>,
        V: Into<Vector3<T>>,
        T: MatrixDataType,
    {
        let vec: Vector3<T> = gyro.into();
        measurement.set_row(3, vec.x);
        measurement.set_row(4, vec.y);
        measurement.set_row(5, vec.z);
    }

    /// Performs a correction step using accelerometer readings.
    ///
    /// ## Arguments
    /// * `accelerometer` - The accelerometer reading.
    /// * `delta_t` - The time delta between accelerometer measurement steps.
    pub fn correct_accelerometer(
        &mut self,
        accelerometer: AccelerometerReading<T>,
        gyroscope_reading: GyroscopeReading<T>,
        delta_t: T,
    ) where
        T: MatrixDataType + IsNaN,
    {
        // See the state according to the measurement.
        Self::update_state_transition_matrix(&mut self.z_filter, self.z_axis, delta_t);

        // Apply the normalized measurement.
        // For the Z direction, we measure the down vector exactly, so no projection is necessary.
        self.z_measurement.measurement_vector_mut().apply(|vec| {
            Self::apply_normalized_vector(accelerometer, vec);
            Self::apply_gyro(gyroscope_reading, vec);
        });

        // Perform the update step.
        self.z_filter.correct(&mut self.z_measurement);
        self.panic_if_nan();

        // After the correction, we can reuse the updated gyro values in the magnetometer filter.
        self.x_filter.state_vector_mut().apply(|vec| {
            let state = self.z_filter.state_vector();
            vec.set_row(3, state.get_row(3));
            vec.set_row(4, state.get_row(4));
            vec.set_row(5, state.get_row(5));
        });

        // The axes now need to be re-orthogonalized.
        self.update_axes_from_accelerometer_filter();
        self.re_orthogonalize_with_updated_z();
    }

    fn update_axes_from_accelerometer_filter(&mut self)
    where
        T: MatrixDataType,
    {
        let vec = self.z_filter.state_vector();
        let x = vec.get_row(0);
        let y = vec.get_row(1);
        let z = vec.get_row(2);
        self.z_axis = Vector3::new(x, y, z).normalized();
    }

    /// Performs a correction step using accelerometer readings.
    ///
    /// ## Arguments
    /// * `magnetometer` - The magnetometer reading.
    /// * `delta_t` - The time delta between magnetometer measurement steps.
    pub fn correct_magnetometer(
        &mut self,
        magnetometer: MagnetometerReading<T>,
        gyroscope_reading: GyroscopeReading<T>,
        delta_t: T,
    ) where
        T: MatrixDataType + IsNaN,
    {
        // The north vector is not exactly the magnetometer, so we need to perform a TRIAD update.
        let east = self
            .z_axis
            .cross(Vector3::from(magnetometer).normalized())
            .normalized();
        let north = east.cross(self.z_axis).normalized();

        // See the state according to the measurement.
        Self::update_state_transition_matrix(&mut self.x_filter, self.x_axis, delta_t);

        // Apply the normalized measurement.
        self.x_measurement.measurement_vector_mut().apply(|vec| {
            Self::apply_normalized_vector(north, vec);
            Self::apply_gyro(gyroscope_reading, vec);
        });

        // Perform the update step.
        self.x_filter.correct(&mut self.x_measurement);

        self.panic_if_nan();

        // After the correction, we can reuse the updated gyro values in the accelerometer filter.
        self.z_filter.state_vector_mut().apply(|vec| {
            let state = self.x_filter.state_vector();
            vec.set_row(3, state.get_row(3));
            vec.set_row(4, state.get_row(4));
            vec.set_row(5, state.get_row(5));
        });

        // The axes now need to be re-orthogonalized.
        self.update_axes_from_magnetometer_filter();
        self.re_orthogonalize_with_updated_x();
    }

    fn update_axes_from_magnetometer_filter(&mut self)
    where
        T: MatrixDataType,
    {
        let vec = self.x_filter.state_vector();
        let x = vec.get_row(0);
        let y = vec.get_row(1);
        let z = vec.get_row(2);
        self.x_axis = Vector3::new(x, y, z).normalized();
    }

    /// This function assumes that the `z_axis` value is corrected
    /// and does not need to be changed, however the remaining values need re-orthogonalization.
    fn re_orthogonalize_with_updated_z(&mut self)
    where
        T: MatrixDataType,
    {
        self.y_axis = self.z_axis.cross(self.x_axis).normalized();
        self.x_axis = self.y_axis.cross(self.z_axis).normalized();
        self.update_state_from_axes();
    }

    /// This function assumes that the `x_axis` value is corrected
    /// and does not need to be changed, however the remaining values need re-orthogonalization.
    fn re_orthogonalize_with_updated_x(&mut self)
    where
        T: MatrixDataType,
    {
        self.y_axis = self.z_axis.cross(self.x_axis).normalized();
        self.z_axis = self.x_axis.cross(self.y_axis).normalized();
        self.update_state_from_axes();
    }

    /// Applies the orthogonalized and normalized axes back to the state vectors.
    fn update_state_from_axes(&mut self)
    where
        T: MatrixDataType,
    {
        self.x_filter.state_vector_mut().apply(|vec| {
            vec.set_row(0, self.x_axis.x);
            vec.set_row(1, self.x_axis.y);
            vec.set_row(2, self.x_axis.z);
        });
        self.z_filter.state_vector_mut().apply(|vec| {
            vec.set_row(0, self.z_axis.x);
            vec.set_row(1, self.z_axis.y);
            vec.set_row(2, self.z_axis.z);
        });
    }

    /// Updates the state transition matrix to use either the down or north axis,
    /// depending on the kind of update performed.
    fn update_state_transition_matrix(
        filter: &mut OwnedKalmanFilter<T>,
        axis: Vector3<T>,
        delta_t: T,
    ) where
        T: MatrixDataType,
    {
        filter.state_transition_mut().apply(|mat| {
            mat.set_at(0, 3, T::zero());
            mat.set_at(0, 4, axis.z * delta_t);
            mat.set_at(0, 5, -axis.y * delta_t);

            mat.set_at(1, 3, -axis.z * delta_t);
            mat.set_at(1, 4, T::zero());
            mat.set_at(1, 5, axis.x * delta_t);

            mat.set_at(2, 3, axis.y * delta_t);
            mat.set_at(2, 4, -axis.x * delta_t);
            mat.set_at(2, 5, T::zero());
        });
    }

    #[allow(unused)]
    fn panic_if_nan(&self)
    where
        T: Copy + IsNaN,
    {
        #[cfg(debug_assertions)]
        self.x_filter.state_vector().inspect(|vec| {
            if vec.get_row(0).is_nan() || vec.get_row(1).is_nan() || vec.get_row(2).is_nan() {
                panic!("NaN angle detected in state estimate")
            }
        });
        #[cfg(debug_assertions)]
        self.z_filter.state_vector().inspect(|vec| {
            if vec.get_row(0).is_nan() || vec.get_row(1).is_nan() || vec.get_row(2).is_nan() {
                panic!("NaN angle detected in state estimate")
            }
        });
    }
}

impl<T> OwnedOrientationEstimator<T> {
    /// Builds the Kalman filter used for prediction.
    fn build_filter(axis: Vector3<T>, process_noise_value: T) -> OwnedKalmanFilter<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // State vector.
        let mut state_vec =
            StateVectorBuffer::<STATES, T, _>::new(MatrixData::new_array::<STATES, 1, STATES, T>(
                [zero; STATES],
            ));

        // default axis
        state_vec.set_row(0, axis.x);
        state_vec.set_row(1, axis.y);
        state_vec.set_row(2, axis.z);
        // no angular rotation
        state_vec.set_row(3, T::zero());
        state_vec.set_row(4, T::zero());
        state_vec.set_row(5, T::zero());

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

        // The main layout is an identity matrix; the top-right 3x3 submatrix
        // is set in each update and correction step as it is specific to the current state.
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
        estimate_covariance.apply(|mat| {
            let five = T::one() + T::one() + T::one() + T::one() + T::one();
            mat.set_at(0, 0, five);
            mat.set_at(1, 1, five);
            mat.set_at(2, 2, five);

            mat.set_at(3, 3, T::one());
            mat.set_at(4, 4, T::one());
            mat.set_at(5, 5, T::one());
        });

        // Process noise matrix.
        let mut process_noise = DirectProcessNoiseCovarianceMatrixMutBuffer::<STATES, T, _>::new(
            MatrixData::new_array::<STATES, STATES, { STATES * STATES }, T>(
                [zero; { STATES * STATES }],
            ),
        );
        process_noise.make_scalar(process_noise_value);
        process_noise.apply(|mat| {
            mat.set_at(3, 3, T::one());
            mat.set_at(4, 4, T::one());
            mat.set_at(5, 5, T::one());
        });

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
    fn build_measurement<N>(
        sensor_noise: N,
        alpha: T,
        gyroscope_noise: GyroscopeNoise<T>,
        beta: T,
    ) -> OwnedVector6Observation<T>
    where
        N: Into<Vector3<T>>,
        T: MatrixDataType + Default,
    {
        let sensor_noise = sensor_noise.into();
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

        // We're observing all states at once.
        observation_matrix.make_identity();

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
            mat.set_at(0, 0, alpha * sensor_noise.x);
            mat.set_at(1, 1, alpha * sensor_noise.y);
            mat.set_at(2, 2, alpha * sensor_noise.z);
            mat.set_at(3, 3, beta * gyroscope_noise.x);
            mat.set_at(4, 4, beta * gyroscope_noise.y);
            mat.set_at(5, 5, beta * gyroscope_noise.z);
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
