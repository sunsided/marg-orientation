use crate::gyro_free::types::*;
use crate::prelude::*;
use crate::types::{
    AccelerometerNoise, AccelerometerReading, EulerAngles, MagnetometerNoise, MagnetometerReading,
    Vector3,
};
use crate::IsNaN;
use minikalman::buffers::types::*;
use minikalman::extended::{ExtendedKalmanBuilder, ExtendedObservationBuilder};
use minikalman::matrix::MatrixDataType;
use minikalman::prelude::*;

/// A magnetic field reference vector.
pub type MagneticReference<T> = MagnetometerReading<T>;

/// A Gyro-free Magnetic Field- and Gravity-based orientation estimator over a generic type `T`.
///
/// This estimator is "gyro-free", i.e. it can work with only accelerometer and magnetometer
/// readings; to ensure correct orientation, a magnetic field reference vector needs to be
/// provided (in the body frame), indicating the earth's magnetic field direction at the
/// location of the device.
///
/// The estimator operates in a North, East, Down coordinate frame and requires all readings
/// in that frame to operate correctly.
pub struct OwnedOrientationEstimator<T> {
    filter: OwnedKalmanFilter<T>,
    /// Accelerometer and magnetometer measurements.
    /// We share the same observation struct here because the dimensionality of the
    /// measurements is identical and both need to construct a fully filled Jacobian.
    measurement: OwnedVector3Observation<T>,
    /// Magnetic field reference vector for the current location.
    magnetic_field_ref: Vector3<T>,
    /// Accelerometer noise term for reconstructing the observation noise matrix.
    accelerometer_noise: AccelerometerNoise<T>,
    /// Magnetometer noise term for reconstructing the observation noise matrix.
    magnetometer_noise: MagnetometerNoise<T>,
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
    ) -> Self
    where
        T: MatrixDataType + Default,
    {
        let filter = Self::build_filter(process_noise);
        let measurement = Self::build_measurement();

        let magnetic_field_ref = Vector3::new(
            magnetic_field_ref.x,
            magnetic_field_ref.y,
            magnetic_field_ref.z,
        )
        .normalized();

        Self {
            filter,
            measurement,
            magnetic_field_ref,
            accelerometer_noise,
            magnetometer_noise,
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

    fn apply_normalized_vector<V, M>(vec: V, measurement: &mut M)
    where
        M: RowVectorMut<3, T>,
        V: Into<Vector3<T>>,
        T: MatrixDataType,
    {
        let vec: Vector3<T> = vec.into().normalized();
        measurement.set_row(0, vec.x);
        measurement.set_row(1, vec.y);
        measurement.set_row(2, vec.z);
    }

    /// Performs a correction step using accelerometer and magnetometer readings.
    ///
    /// ## How it works
    /// This function uses the down vector as a reference and rotates it using the
    /// estimated orientation transformation; it is then compared with the provided accelerometer
    /// reading.
    ///
    /// ## Arguments
    /// * `accelerometer` - The accelerometer reading.
    pub fn correct_accelerometer(&mut self, accelerometer: &AccelerometerReading<T>)
    where
        T: MatrixDataType + IsNaN,
    {
        // Apply the normalized measurement.
        self.measurement.measurement_vector_mut().apply(|vec| {
            Self::apply_normalized_vector(accelerometer, vec);
        });

        // Reconstruct the noise matrix.
        self.measurement
            .measurement_noise_covariance_mut()
            .apply(|mat| {
                mat.set_at(0, 0, self.accelerometer_noise.x);
                mat.set_at(1, 1, self.accelerometer_noise.y);
                mat.set_at(2, 2, self.accelerometer_noise.z);
            });

        // Update the Jacobian.
        let two = T::one() + T::one();
        let (q0, q1, q2, q3) = self.estimated_quaternion_internal();

        // This applies a simplified version of the Jacobian of the rotated vector with
        // respect to the rotation quaternion. In general, all vector components influence
        // the Jacobian; the down vector however has zeroes on both x and y axes and is known
        // to be exactly one on the x (down) axis. As such, a lot of terms simplify to zero
        // and remaining terms simplify.
        //
        // See the Jacobian on the magnetometer for a generalised version.
        self.measurement
            .observation_jacobian_matrix_mut()
            .apply(|mat| {
                let two_q0 = q0 * two;
                let two_q1 = q1 * two;
                let two_q2 = q2 * two;
                let two_q3 = q3 * two;

                mat.set_at(0, 0, two_q2);
                mat.set_at(0, 1, two_q3);
                mat.set_at(0, 2, two_q0);
                mat.set_at(0, 3, two_q1);

                mat.set_at(1, 0, -two_q1);
                mat.set_at(1, 1, -two_q0);
                mat.set_at(1, 2, two_q3);
                mat.set_at(1, 3, two_q2);

                mat.set_at(2, 0, two_q0);
                mat.set_at(2, 1, -two_q1);
                mat.set_at(2, 2, -two_q2);
                mat.set_at(2, 3, two_q3);
            });

        // Perform the update step.
        self.filter
            .correct_nonlinear(&mut self.measurement, |state, measurement| {
                let down = Vector3::new(T::zero(), T::zero(), T::one());
                let rotated = Self::rotate_vector_internal(state, down);
                measurement.set_row(0, rotated.x);
                measurement.set_row(1, rotated.y);
                measurement.set_row(2, rotated.z);

                measurement.set_row(0, two * (q1 * q3 + q0 * q2));
                measurement.set_row(1, two * (q2 * q3 - q0 * q1));
                measurement.set_row(2, T::one() - two * (q1 * q1 + q2 * q2));
            });

        self.normalize_state_quaternion();
        self.panic_if_nan();
    }

    /// Performs a correction step using accelerometer readings.
    ///
    /// ## How it works
    /// This function uses the magnetic field reference vector and rotates it using the
    /// estimated orientation transformation; it is then compared with the provided magnetometer
    /// reading.
    ///
    /// ## Arguments
    /// * `magnetometer` - The magnetometer reading.
    pub fn correct_magnetometer(&mut self, magnetometer: &MagnetometerReading<T>)
    where
        T: MatrixDataType + IsNaN,
    {
        // Apply the normalized measurement.
        self.measurement.measurement_vector_mut().apply(|vec| {
            Self::apply_normalized_vector(magnetometer, vec);
        });

        // Reconstruct the noise matrix.
        self.measurement
            .measurement_noise_covariance_mut()
            .apply(|mat| {
                mat.set_at(0, 0, self.magnetometer_noise.x);
                mat.set_at(1, 1, self.magnetometer_noise.y);
                mat.set_at(2, 2, self.magnetometer_noise.z);
            });

        // Update the Jacobian.
        let one = T::one();
        let two = one + one;
        let (q0, q1, q2, q3) = self.estimated_quaternion_internal();
        let (mx, my, mz) = self.magnetic_field_ref.into();

        // This applies a simplified version of the Jacobian of the rotated vector with
        // respect to the rotation quaternion. Unlike the accelerometer update, here, all
        // vector components affect the matrix.
        self.measurement
            .observation_jacobian_matrix_mut()
            .apply(|mat| {
                mat.set_at(0, 0, two * (q0 * mx - q3 * my + q2 * mz));
                mat.set_at(0, 1, two * (q1 * mx + q2 * my + q3 * mz));
                mat.set_at(0, 2, two * (q1 * my - q2 * mx + q0 * mz));
                mat.set_at(0, 3, two * (q1 * mz - q0 * my - q3 * mx));

                mat.set_at(1, 0, two * (q3 * mx + q0 * my - q1 * mz));
                mat.set_at(1, 1, two * (q2 * mx - q1 * my - q0 * mz));
                mat.set_at(1, 2, two * (q1 * mx + q2 * my + q3 * mz));
                mat.set_at(1, 3, two * (q0 * mx - q3 * my + q2 * mz));

                mat.set_at(2, 0, two * (q1 * my - q2 * mx + q0 * mz));
                mat.set_at(2, 1, two * (q3 * mx + q0 * my - q1 * mz));
                mat.set_at(2, 2, two * (q3 * my - q0 * mx - q2 * mz));
                mat.set_at(2, 3, two * (q1 * mx + q2 * my + q3 * mz));
            });

        // Perform the update step.
        self.filter
            .correct_nonlinear(&mut self.measurement, |state, measurement| {
                let reference = self.magnetic_field_ref;
                let rotated = Self::rotate_vector_internal(state, reference);
                measurement.set_row(0, rotated.x);
                measurement.set_row(1, rotated.y);
                measurement.set_row(2, rotated.z);
            });

        self.normalize_state_quaternion();
        self.panic_if_nan();
    }

    fn rotate_vector_internal<V>(state: &V, vec: Vector3<T>) -> Vector3<T>
    where
        V: RowVectorMut<STATES, T>,
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

        let y = vec.x * two * (q1 * q2 + q0 * q3)
            + vec.y * (one - two * (q1 * q1 + q3 * q3))
            + vec.z * two * (q2 * q3 - q0 * q1);

        let z = vec.x * two * (q1 * q3 - q0 * q2)
            + vec.y * two * (q2 * q3 + q0 * q1)
            + vec.z * (one - two * (q1 * q1 + q2 * q2));

        Vector3::new(x, y, z)
    }

    /// Rotate a vector in the body frame.
    ///
    /// For display purposes you likely want to use [`rotate_vector_world`](Self::rotate_vector_world) instead.
    pub fn rotate_vector_body(&self, vector: Vector3<T>) -> Vector3<T>
    where
        T: MatrixDataType,
    {
        Self::rotate_vector_internal(self.filter.state_vector(), vector)
    }

    /// Rotates a vector in the world frame.
    pub fn rotate_vector_world(&self, vec: Vector3<T>) -> Vector3<T>
    where
        T: MatrixDataType,
    {
        let state = self.filter.state_vector();
        let q0 = state.get_row(0);
        let q1 = state.get_row(1);
        let q2 = state.get_row(2);
        let q3 = state.get_row(3);

        let one = T::one();
        let two = one + one;
        let x = vec.x * (one - two * (q2 * q2 + q3 * q3))
            + vec.y * two * (q1 * q2 + q0 * q3)
            + vec.z * two * (q1 * q3 - q0 * q2);

        let y = vec.x * two * (q1 * q2 - q0 * q3)
            + vec.y * (one - two * (q1 * q1 + q3 * q3))
            + vec.z * two * (q2 * q3 + q0 * q1);

        let z = vec.x * two * (q1 * q3 + q0 * q2)
            + vec.y * two * (q2 * q3 - q0 * q1)
            + vec.z * (one - two * (q1 * q1 + q2 * q2));

        Vector3::new(x, y, z)
    }

    fn normalize_state_quaternion(&mut self)
    where
        T: MatrixDataType,
    {
        self.filter.state_vector_mut().apply(|vec| {
            let w = vec.get_row(0);
            let x = vec.get_row(1);
            let y = vec.get_row(2);
            let z = vec.get_row(3);

            let (w, x, y, z) = if w >= T::zero() {
                (w, x, y, z)
            } else {
                (-w, -x, -y, -z)
            };

            let norm_sq = w * w + x * x + y * y + z * z;
            let norm = norm_sq.square_root();
            let w = w / norm;
            let x = x / norm;
            let y = y / norm;
            let z = z / norm;
            vec.set_row(0, w);
            vec.set_row(1, x);
            vec.set_row(2, y);
            vec.set_row(3, z);
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
    fn estimated_quaternion_internal(&self) -> (T, T, T, T)
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
        let (w, x, y, z) = self.estimated_quaternion_internal();
        let one = T::one();
        let two = one + one;
        let sinr_cosp = two * (w * x + y * z);
        let cosr_cosp = one - two * (x * x + y * y);
        let roll = sinr_cosp.atan2(cosr_cosp);
        roll.normalize_angle()
    }

    /// Obtains the current estimate of the pitch angle θ (theta), in radians.
    ///
    /// The pitch angle is defined as the amount rotation around the x-axis (right).
    pub fn pitch(&self) -> T
    where
        T: MatrixDataType + Abs<T, Output = T> + ArcSin<T, Output = T> + NormalizeAngle<Output = T>,
    {
        let (w, x, y, z) = self.estimated_quaternion_internal();
        let one = T::one();
        let two = one + one;
        let sinp = two * (w * y - z * x);
        // TODO: If sin >= 1.0 || sin <= -1.0, clamp to +/- pi/2 instead
        let pitch = sinp.arcsin();
        pitch.normalize_angle()
    }

    /// Obtains the current estimate of the yaw angle ψ (psi), in radians.
    ///
    /// The yaw angle is defined as the amount rotation around the z-axis (up).
    pub fn yaw(&self) -> T
    where
        T: MatrixDataType + ArcTan<T, Output = T> + NormalizeAngle<Output = T>,
    {
        let (w, x, y, z) = self.estimated_quaternion_internal();
        let one = T::one();
        let two = one + one;
        let siny_cosp = two * (w * z + x * y);
        let cosy_cosp = one - two * (y * y + z * z);
        let yaw = siny_cosp.atan2(cosy_cosp);
        yaw.normalize_angle()
    }
}

impl OwnedOrientationEstimator<f32> {
    /// Provides the estimated orientation as a [`Quaternion`](micromath::Quaternion).
    #[cfg(feature = "micromath")]
    #[cfg_attr(docsrs, doc(cfg(feature = "micromath")))]
    pub fn estimated_quaternion(&self) -> micromath::Quaternion {
        let (w, x, y, z) = self.estimated_quaternion_internal();
        micromath::Quaternion::new(w, x, y, z)
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
    fn build_measurement() -> OwnedVector3Observation<T>
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
        noise_covariance.make_identity();

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
