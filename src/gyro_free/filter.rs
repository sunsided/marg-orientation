use crate::gyro_free::types::*;
use crate::vector3::Vector3;
use crate::{
    Abs, AccelerometerNoise, AccelerometerReading, ArcSin, ArcTan, EulerAngles, IsNaN,
    MagnetometerNoise, MagnetometerReading, NormalizeAngle,
};
use core::ops::{Add, Mul, Sub};
use minikalman::buffers::types::*;
use minikalman::extended::{ExtendedKalmanBuilder, ExtendedObservationBuilder};
use minikalman::matrix::MatrixDataType;
use minikalman::prelude::*;
use num_traits::{One, Zero};

/// A magnetic field reference vector.
pub type MagneticReference<T> = MagnetometerReading<T>;

/// A MARG (Magnetic, Angular Rate, and Gravity) orientation estimator with generic type `T`.
pub struct OwnedOrientationEstimator<T> {
    filter: OwnedKalmanFilter<T>,
    /// Magnetometer measurements.
    mag_measurement: OwnedVector3Observation<T>,
    /// Accelerometer measurements.
    acc_measurement: OwnedVector3Observation<T>,
    /// The autocovariances of the noise terms.
    accelerometer_noise: AccelerometerNoise<T>,
    /// The autocovariances of the noise terms.
    magnetometer_noise: MagnetometerNoise<T>,
    /// Magnetic field reference vector for the current location.
    magnetic_field_ref: Vector3<T>,
    /// A bias term to avoid divisions by zero.
    epsilon: T,
}

impl<T> OwnedOrientationEstimator<T> {
    /// Initializes a new instance of the [`OwnedOrientationEstimator`] struct.
    ///
    /// ## Arguments
    /// * `accelerometer_noise` - The accelerometer noise values (sigma-squared) for each axis.
    /// * `magnetometer_noise` - The magnetometer noise values (sigma-squared) for each axis.
    /// * `magnetic_field_ref` - The magnetic field reference vector for the current location.
    /// * `process_noise` - A process noise value.
    /// * `epsilon` - A small bias term to avoid divisions by zero. Set to e.g. `1e-6`.
    pub fn new(
        accelerometer_noise: AccelerometerNoise<T>,
        magnetometer_noise: MagnetometerNoise<T>,
        magnetic_field_ref: MagneticReference<T>,
        process_noise: T,
        epsilon: T,
    ) -> Self
    where
        T: MatrixDataType + Default,
    {
        let filter = Self::build_filter(process_noise);
        let mag_measurement = Self::build_mag_measurement(&magnetometer_noise);
        let acc_measurement = Self::build_accel_measurement(&accelerometer_noise);

        let magnetic_field_ref = Vector3::new(
            magnetic_field_ref.x,
            magnetic_field_ref.y,
            magnetic_field_ref.z,
        )
        .normalized();

        Self {
            filter,
            mag_measurement,
            acc_measurement,
            accelerometer_noise,
            magnetometer_noise,
            magnetic_field_ref,
            epsilon,
        }
    }
}

impl<T> OwnedOrientationEstimator<T> {
    /// Performs a prediction step to obtain new orientation estimates.
    pub fn predict(&mut self)
    where
        T: MatrixDataType + IsNaN,
    {
        // Perform a regular Kalman Filter prediction step.
        self.filter
            .predict_nonlinear(|current, next| current.copy(next));
        self.panic_if_nan();
    }

    /// Performs a correction step using accelerometer and magnetometer readings.
    ///
    /// ## Arguments
    /// * `accelerometer` - The accelerometer reading.
    pub fn correct_accelerometer(&mut self, accelerometer: &AccelerometerReading<T>)
    where
        T: MatrixDataType + IsNaN,
    {
        // Normalize the vectors.
        let a = Vector3::from(accelerometer).normalized();
        self.acc_measurement.measurement_vector_mut().apply(|vec| {
            vec.set_row(0, a.x);
            vec.set_row(1, a.y);
            vec.set_row(2, a.z);
        });

        // Update the Jacobian.
        let two = T::one() + T::one();
        let (q0, q1, q2, q3) = self.estimated_quaternion();

        self.acc_measurement
            .observation_jacobian_matrix_mut()
            .apply(|mat| {
                let q0 = q0 * two;
                let q1 = q1 * two;
                let q2 = q2 * two;
                let q3 = q3 * two;

                mat.set_at(0, 0, q2);
                mat.set_at(0, 1, -q3);
                mat.set_at(0, 2, q0);
                mat.set_at(0, 3, -q1);

                mat.set_at(1, 0, -q1);
                mat.set_at(1, 1, -q0);
                mat.set_at(1, 2, q3);
                mat.set_at(1, 3, q2);

                mat.set_at(2, 0, q0);
                mat.set_at(2, 1, -q1);
                mat.set_at(2, 2, -q2);
                mat.set_at(2, 3, q3);
            });

        // Perform the update step.
        self.filter
            .correct_nonlinear(&mut self.acc_measurement, |state, measurement| {
                // let down = Vector3::new(T::zero(), T::zero(), T::one());
                // let rotated = Self::rotate_vector(state, &down);
                // measurement.set_row(0, rotated.x);
                // measurement.set_row(1, rotated.y);
                // measurement.set_row(2, rotated.z);

                measurement.set_row(0, two * (q1 * q3 - q0 * q2));
                measurement.set_row(1, two * (q2 * q3 + q0 * q1));
                measurement.set_row(2, T::one() - two * (q1 * q1 + q2 * q2));
            });

        self.normalize_state_quaternion();
        self.panic_if_nan();
    }

    /// Performs a correction step using accelerometer readings.
    ///
    /// ## Arguments
    /// * `magnetometer` - The magnetometer reading.
    pub fn correct_magnetometer(&mut self, magnetometer: &MagnetometerReading<T>)
    where
        T: MatrixDataType + IsNaN,
    {
        // Normalize the vector.
        self.mag_measurement.measurement_vector_mut().apply(|vec| {
            let m = Vector3::from(magnetometer).normalized();
            vec.set_row(0, m.x);
            vec.set_row(1, m.y);
            vec.set_row(2, m.z);
        });

        // Update the Jacobian.
        let one = T::one();
        let two = one + one;
        let (q0, q1, q2, q3) = self.estimated_quaternion();
        let (mx, my, mz) = (
            self.magnetic_field_ref.x,
            self.magnetic_field_ref.y,
            self.magnetic_field_ref.z,
        );
        self.mag_measurement
            .observation_jacobian_matrix_mut()
            .apply(|mat| {
                mat.set_at(0, 0, two * (mz * q2 - my * q3));
                mat.set_at(0, 1, two * (mz * q3 + my * q2));
                mat.set_at(0, 2, two * (mz * q0 + my * q1));
                mat.set_at(0, 3, two * (mz * q1 - my * q0));

                mat.set_at(1, 0, two * (mx * q3 - mz * q1));
                mat.set_at(1, 1, two * (mx * q2 + mz * q0));
                mat.set_at(1, 2, two * (mx * q1 - mz * q3));
                mat.set_at(1, 3, two * (mx * q0 + mz * q2));

                mat.set_at(2, 0, two * (my * q1 - mx * q2));
                mat.set_at(2, 1, two * (my * q0 - mx * q3));
                mat.set_at(2, 2, two * (my * q3 + mx * q0));
                mat.set_at(2, 3, two * (my * q2 + mx * q1));
            });

        // Perform the update step.
        self.filter
            .correct_nonlinear(&mut self.mag_measurement, |state, measurement| {
                let rotated = Self::rotate_vector_internal(state, &self.magnetic_field_ref);
                measurement.set_row(0, rotated.x);
                measurement.set_row(1, rotated.y);
                measurement.set_row(2, rotated.z);
            });

        self.normalize_state_quaternion();
        self.panic_if_nan();
    }

    pub fn rotate_vector(&self, vector: Vector3<T>) -> Vector3<T>
    where
        T: MatrixDataType,
    {
        Self::rotate_vector_internal(self.filter.state_vector(), &vector)
    }

    fn rotate_vector_internal(
        state: &StateVectorBufferOwnedType<STATES, T>,
        vec: &Vector3<T>,
    ) -> Vector3<T>
    where
        T: MatrixDataType,
    {
        let q0 = state.get_row(0);
        let q1 = state.get_row(1);
        let q2 = state.get_row(2);
        let q3 = state.get_row(3);

        let one = T::one();
        let two = one + one;
        let x = vec.x * (one - two * (q2 * q2 + q3 * q3))
            + vec.y * two * (q1 * q2 - q0 * q3)
            + vec.z * two * (q1 * q3 + q0 * q2);

        let y = vec.x * (q1 * q2 + q0 * q3)
            + vec.y * (one - two * (q1 * q1 + q3 * q3))
            + vec.z * two * (q2 * q3 - q0 * q1);

        let z = vec.x * (q1 * q3 - q0 * q2)
            + vec.y * two * (q2 * q3 + q0 * q1)
            + vec.z * (one - two * (q1 * q1 + q2 * q2));

        Vector3::new(x, y, z)
    }

    fn normalize_state_quaternion(&mut self)
    where
        T: MatrixDataType,
    {
        self.filter.state_vector_mut().apply(|vec| {
            let a = vec.get_row(0);
            let b = vec.get_row(1);
            let c = vec.get_row(2);
            let d = vec.get_row(3);
            let norm_sq = a * a + b * b + c * c + d * d;
            let norm = norm_sq.square_root();
            let a = a / norm;
            let b = b / norm;
            let c = c / norm;
            let d = d / norm;
            vec.set_row(0, a);
            vec.set_row(1, b);
            vec.set_row(2, c);
            vec.set_row(3, d);
        });
    }

    #[allow(unused)]
    fn panic_if_nan(&self)
    where
        T: Copy + IsNaN,
    {
        #[cfg(debug_assertions)]
        self.filter.state_vector().inspect(|vec| {
            if vec.get_row(0).is_nan() || vec.get_row(1).is_nan() || vec.get_row(2).is_nan() {
                panic!("NaN angle detected in state estimate")
            }
        });
    }

    /// Gets the estimated quaternion in (w, x, y, z) order.
    fn estimated_quaternion(&self) -> (T, T, T, T)
    where
        T: Copy,
    {
        self.filter.state_vector().inspect(|vec| {
            (
                vec.get_row(0),
                vec.get_row(1),
                vec.get_row(2),
                vec.get_row(3),
            )
        })
    }

    /// Obtains the current estimates of the roll, pitch and yaw angles.
    pub fn estimated_angles(&self) -> EulerAngles<T>
    where
        T: MatrixDataType
            + Abs<T, Output = T>
            + ArcSin<T, Output = T>
            + ArcTan<T, Output = T>
            + NormalizeAngle<Output = T>,
    {
        EulerAngles::new(self.roll(), self.pitch(), self.yaw())
    }

    /// Obtains the current estimate of the roll angle φ (phi), in radians.
    ///
    /// The roll angle is defined as the amount rotation around the y-axis (forward).
    pub fn roll(&self) -> T
    where
        T: MatrixDataType + ArcTan<T, Output = T> + NormalizeAngle<Output = T>,
    {
        let (w, x, y, z) = self.estimated_quaternion();
        let one = T::one();
        let two = one + one;
        let sinr_cosp = two * (w * x + y * z);
        let cosr_cosp = one - two * (x * x + y * y);
        let roll = sinr_cosp.atan2(cosr_cosp);
        roll.normalize_angle()
    }

    /// Obtains the current estimation variance (uncertainty) of the roll angle φ (phi), in radians².
    ///
    /// ## Interpretation
    /// - Low Variance: Indicates high certainty in the estimate. The state estimate is
    ///   considered to be precise, as it doesn't vary much from the mean.
    /// - High Variance: Indicates high uncertainty in the estimate. The state estimate is
    ///   considered to be less precise, as it has a wide spread around the mean.
    pub fn roll_variance(&self) -> T
    where
        T: Zero,
    {
        T::zero()
    }

    /// Obtains the current estimate of the pitch angle θ (theta), in radians.
    ///
    /// The pitch angle is defined as the amount rotation around the x-axis (right).
    pub fn pitch(&self) -> T
    where
        T: MatrixDataType + Abs<T, Output = T> + ArcSin<T, Output = T> + NormalizeAngle<Output = T>,
    {
        let (w, x, y, z) = self.estimated_quaternion();
        let one = T::one();
        let two = one + one;
        let sinp = two * (w * y - z * x);
        // TODO: If sin >= 1.0 || sin <= -1.0, clamp to +/- pi/2 instead
        let pitch = sinp.arcsin();
        pitch.normalize_angle()
    }

    /// Obtains the current estimation variance (uncertainty) of the pitch angle θ (theta), in radians².
    ///
    /// ## Interpretation
    /// - Low Variance: Indicates high certainty in the estimate. The state estimate is
    ///   considered to be precise, as it doesn't vary much from the mean.
    /// - High Variance: Indicates high uncertainty in the estimate. The state estimate is
    ///   considered to be less precise, as it has a wide spread around the mean.
    pub fn pitch_variance(&self) -> T
    where
        T: Zero,
    {
        T::zero()
    }

    /// Obtains the current estimate of the yaw angle ψ (psi), in radians.
    ///
    /// The yaw angle is defined as the amount rotation around the z-axis (up).
    pub fn yaw(&self) -> T
    where
        T: MatrixDataType + ArcTan<T, Output = T> + NormalizeAngle<Output = T>,
    {
        let (w, x, y, z) = self.estimated_quaternion();
        let one = T::one();
        let two = one + one;
        let siny_cosp = two * (w * z + x * y);
        let cosy_cosp = one - two * (y * y + z * z);
        let yaw = siny_cosp.atan2(cosy_cosp);
        yaw.normalize_angle()
    }

    /// Obtains the current estimation variance (uncertainty) of the yaw angle ψ (psi), in radians².
    ///
    /// ## Interpretation
    /// - Low Variance: Indicates high certainty in the estimate. The state estimate is
    ///   considered to be precise, as it doesn't vary much from the mean.
    /// - High Variance: Indicates high uncertainty in the estimate. The state estimate is
    ///   considered to be less precise, as it has a wide spread around the mean.
    pub fn yaw_variance(&self) -> T
    where
        T: Zero,
    {
        T::zero()
    }
}

impl<T> OwnedOrientationEstimator<T> {
    /// Builds the Kalman filter used for prediction.
    fn build_filter(process_noise_value: T) -> OwnedKalmanFilter<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // State vector.
        let mut state_vec =
            StateVectorBuffer::<STATES, T, _>::new(MatrixData::new_array::<STATES, 1, STATES, T>(
                [zero; STATES],
            ));

        // TODO: Seed with roll/pitch from accelerometer, yaw (heading) from magnetometer.
        state_vec.set_row(0, T::one());
        state_vec.set_row(1, T::zero());
        state_vec.set_row(2, T::zero());
        state_vec.set_row(3, T::zero());

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
        estimate_covariance.make_scalar(process_noise_value);

        // Process noise matrix.
        let mut process_noise = DirectProcessNoiseCovarianceMatrixMutBuffer::<STATES, T, _>::new(
            MatrixData::new_array::<STATES, STATES, { STATES * STATES }, T>(
                [zero; { STATES * STATES }],
            ),
        );
        process_noise.make_scalar(process_noise_value);

        // Predicted state vector.
        let mut predicted_state =
            PredictedStateEstimateVectorBuffer::<STATES, T, _>::new(MatrixData::new_array::<
                STATES,
                1,
                STATES,
                T,
            >([zero; STATES]));
        predicted_state.set_all(T::zero());

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

        ExtendedKalmanBuilder::new::<STATES, T>(
            state_transition,
            state_vec,
            estimate_covariance,
            process_noise,
            predicted_state,
            temp_state_matrix,
        )
    }

    /// Builds the Kalman filter observation used for the prediction.
    fn build_mag_measurement(
        magnetometer_noise: &MagnetometerNoise<T>,
    ) -> OwnedVector3Observation<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // Measurement vector
        let measurement =
            MeasurementVectorBuffer::<VEC3_OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                VEC3_OBSERVATIONS,
                1,
                VEC3_OBSERVATIONS,
                T,
            >(
                [zero; VEC3_OBSERVATIONS]
            ));

        // Observation matrix
        let mut observation_matrix =
            ObservationMatrixMutBuffer::<VEC3_OBSERVATIONS, STATES, T, _>::new(
                MatrixData::new_array::<VEC3_OBSERVATIONS, STATES, { VEC3_OBSERVATIONS * STATES }, T>(
                    [zero; { VEC3_OBSERVATIONS * STATES }],
                ),
            );
        observation_matrix.set_all(T::zero());

        // Measurement noise covariance
        let mut noise_covariance =
            MeasurementNoiseCovarianceMatrixBuffer::<VEC3_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    VEC3_OBSERVATIONS,
                    VEC3_OBSERVATIONS,
                    { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS },
                    T,
                >([zero; { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS }]),
            );
        noise_covariance.apply(|mat| {
            mat.set_at(0, 0, magnetometer_noise.x);
            mat.set_at(1, 1, magnetometer_noise.y);
            mat.set_at(2, 2, magnetometer_noise.z);
        });

        // Innovation vector
        let innovation_vector =
            InnovationVectorBuffer::<VEC3_OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                VEC3_OBSERVATIONS,
                1,
                VEC3_OBSERVATIONS,
                T,
            >(
                [zero; VEC3_OBSERVATIONS]
            ));

        // Innovation covariance matrix
        let innovation_covariance =
            InnovationCovarianceMatrixBuffer::<VEC3_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    VEC3_OBSERVATIONS,
                    VEC3_OBSERVATIONS,
                    { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS },
                    T,
                >([zero; { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS }]),
            );

        // Kalman Gain matrix
        let kalman_gain = KalmanGainMatrixBuffer::<STATES, VEC3_OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<STATES, VEC3_OBSERVATIONS, { STATES * VEC3_OBSERVATIONS }, T>(
                [zero; { STATES * VEC3_OBSERVATIONS }],
            ),
        );

        // Temporary residual covariance inverted matrix
        let temp_sinv =
            TemporaryResidualCovarianceInvertedMatrixBuffer::<VEC3_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    VEC3_OBSERVATIONS,
                    VEC3_OBSERVATIONS,
                    { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS },
                    T,
                >([zero; { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS }]),
            );

        // Temporary H×P matrix
        let temp_hp = TemporaryHPMatrixBuffer::<VEC3_OBSERVATIONS, STATES, T, _>::new(
            MatrixData::new_array::<VEC3_OBSERVATIONS, STATES, { VEC3_OBSERVATIONS * STATES }, T>(
                [zero; { VEC3_OBSERVATIONS * STATES }],
            ),
        );

        // Temporary P×Hᵀ matrix
        let temp_pht = TemporaryPHTMatrixBuffer::<STATES, VEC3_OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<STATES, VEC3_OBSERVATIONS, { STATES * VEC3_OBSERVATIONS }, T>(
                [zero; { STATES * VEC3_OBSERVATIONS }],
            ),
        );

        // Temporary K×(H×P) matrix
        let temp_khp = TemporaryKHPMatrixBuffer::<STATES, T, _>::new(MatrixData::new_array::<
            STATES,
            STATES,
            { STATES * STATES },
            T,
        >(
            [zero; { STATES * STATES }]
        ));

        ExtendedObservationBuilder::new::<STATES, VEC3_OBSERVATIONS, T>(
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

    /// Builds the Kalman filter observation used for the prediction.
    fn build_accel_measurement(
        accelerometer_noise: &AccelerometerNoise<T>,
    ) -> OwnedVector3Observation<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // Measurement vector
        let measurement =
            MeasurementVectorBuffer::<VEC3_OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                VEC3_OBSERVATIONS,
                1,
                VEC3_OBSERVATIONS,
                T,
            >(
                [zero; VEC3_OBSERVATIONS]
            ));

        // Observation matrix
        let mut observation_matrix =
            ObservationMatrixMutBuffer::<VEC3_OBSERVATIONS, STATES, T, _>::new(
                MatrixData::new_array::<VEC3_OBSERVATIONS, STATES, { VEC3_OBSERVATIONS * STATES }, T>(
                    [zero; { VEC3_OBSERVATIONS * STATES }],
                ),
            );
        observation_matrix.set_all(T::zero());

        // Measurement noise covariance
        let mut noise_covariance =
            MeasurementNoiseCovarianceMatrixBuffer::<VEC3_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    VEC3_OBSERVATIONS,
                    VEC3_OBSERVATIONS,
                    { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS },
                    T,
                >([zero; { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS }]),
            );
        noise_covariance.apply(|mat| {
            mat.set_at(0, 0, accelerometer_noise.x);
            mat.set_at(1, 1, accelerometer_noise.y);
            mat.set_at(2, 2, accelerometer_noise.z);
        });

        // Innovation vector
        let innovation_vector =
            InnovationVectorBuffer::<VEC3_OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                VEC3_OBSERVATIONS,
                1,
                VEC3_OBSERVATIONS,
                T,
            >(
                [zero; VEC3_OBSERVATIONS]
            ));

        // Innovation covariance matrix
        let innovation_covariance =
            InnovationCovarianceMatrixBuffer::<VEC3_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    VEC3_OBSERVATIONS,
                    VEC3_OBSERVATIONS,
                    { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS },
                    T,
                >([zero; { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS }]),
            );

        // Kalman Gain matrix
        let kalman_gain = KalmanGainMatrixBuffer::<STATES, VEC3_OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<STATES, VEC3_OBSERVATIONS, { STATES * VEC3_OBSERVATIONS }, T>(
                [zero; { STATES * VEC3_OBSERVATIONS }],
            ),
        );

        // Temporary residual covariance inverted matrix
        let temp_sinv =
            TemporaryResidualCovarianceInvertedMatrixBuffer::<VEC3_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    VEC3_OBSERVATIONS,
                    VEC3_OBSERVATIONS,
                    { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS },
                    T,
                >([zero; { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS }]),
            );

        // Temporary H×P matrix
        let temp_hp = TemporaryHPMatrixBuffer::<VEC3_OBSERVATIONS, STATES, T, _>::new(
            MatrixData::new_array::<VEC3_OBSERVATIONS, STATES, { VEC3_OBSERVATIONS * STATES }, T>(
                [zero; { VEC3_OBSERVATIONS * STATES }],
            ),
        );

        // Temporary P×Hᵀ matrix
        let temp_pht = TemporaryPHTMatrixBuffer::<STATES, VEC3_OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<STATES, VEC3_OBSERVATIONS, { STATES * VEC3_OBSERVATIONS }, T>(
                [zero; { STATES * VEC3_OBSERVATIONS }],
            ),
        );

        // Temporary K×(H×P) matrix
        let temp_khp = TemporaryKHPMatrixBuffer::<STATES, T, _>::new(MatrixData::new_array::<
            STATES,
            STATES,
            { STATES * STATES },
            T,
        >(
            [zero; { STATES * STATES }]
        ));

        ExtendedObservationBuilder::new::<STATES, VEC3_OBSERVATIONS, T>(
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
