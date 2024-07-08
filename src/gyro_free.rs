mod types;

use crate::gyro_free::types::*;
use crate::vector3::Vector3;
use crate::{
    AccelerometerNoise, AccelerometerReading, IsNaN, MagnetometerNoise, MagnetometerReading,
    NormalizeAngle,
};
use coordinate_frame::ZeroOne;
use minikalman::buffers::types::*;
use minikalman::extended::{ExtendedKalmanBuilder, ExtendedObservationBuilder};
use minikalman::matrix::MatrixDataType;
use minikalman::prelude::*;
use num_traits::One;

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
    magnetic_field_ref: MagnetometerReading<T>,
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
    /// * `epsilon` - A small bias term to avoid divisions by zero. Set to e.g. `1e-6`.
    pub fn new(
        accelerometer_noise: AccelerometerNoise<T>,
        magnetometer_noise: MagnetometerNoise<T>,
        magnetic_field_ref: MagnetometerReading<T>,
        epsilon: T,
    ) -> Self
    where
        T: MatrixDataType + Default,
    {
        let filter = Self::build_filter(epsilon);
        let mag_measurement =
            Self::build_mag_measurement(&accelerometer_noise, &magnetometer_noise);
        let acc_measurement =
            Self::build_accel_measurement(&accelerometer_noise, &magnetometer_noise);

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
        T: MatrixDataType + Default + NormalizeAngle<T, Output = T> + IsNaN,
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
        T: MatrixDataType + core::fmt::Debug + IsNaN,
    {
        // Normalize the vectors.
        let a = Vector3::from(accelerometer).normalized();
        self.acc_measurement.measurement_vector_mut().apply(|vec| {
            vec.set_row(0, a.x);
            vec.set_row(1, a.y);
            vec.set_row(2, a.z);
        });

        // TODO: Set Jacobian!

        // Perform the update step.
        self.filter
            .correct_nonlinear(&mut self.acc_measurement, |state, measurement| {
                let down = Vector3::new(T::zero(), T::zero(), T::one());
                let rotated = Self::rotate_vector(state, &down);
                measurement.set_row(0, rotated.x);
                measurement.set_row(1, rotated.y);
                measurement.set_row(2, rotated.z);
            });
        self.panic_if_nan();
    }

    /// Performs a correction step using accelerometer readings.
    ///
    /// ## Arguments
    /// * `magnetometer` - The magnetometer reading.
    pub fn correct_magnetometer(&mut self, magnetometer: &MagnetometerReading<T>)
    where
        T: MatrixDataType + core::fmt::Debug + IsNaN,
    {
        // Normalize the vector.
        let m = Vector3::from(magnetometer).normalized();
        self.mag_measurement.measurement_vector_mut().apply(|vec| {
            vec.set_row(0, m.x);
            vec.set_row(1, m.y);
            vec.set_row(2, m.z);
        });

        // TODO: Set Jacobian!

        // Perform the update step.
        self.filter
            .correct_nonlinear(&mut self.mag_measurement, |state, measurement| {
                let reference = Vector3::from(self.magnetic_field_ref);
                let rotated = Self::rotate_vector(state, &reference);
                measurement.set_row(0, rotated.x);
                measurement.set_row(1, rotated.y);
                measurement.set_row(2, rotated.z);
            });
        self.panic_if_nan();
    }

    fn rotate_vector(state: &StateVectorBufferOwnedType<STATES, T>, vec: &Vector3<T>) -> Vector3<T>
    where
        T: Copy
            + One<Output = T>
            + core::ops::Add<T, Output = T>
            + core::ops::Mul<T, Output = T>
            + core::ops::Sub<T, Output = T>,
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
        accelerometer_noise: &AccelerometerNoise<T>,
        magnetometer_noise: &MagnetometerNoise<T>,
    ) -> OwnedVector3Observation<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // Measurement vector
        let measurement =
            MeasurementVectorBuffer::<MAG_OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                MAG_OBSERVATIONS,
                1,
                MAG_OBSERVATIONS,
                T,
            >(
                [zero; MAG_OBSERVATIONS]
            ));

        // Observation matrix
        let mut observation_matrix =
            ObservationMatrixMutBuffer::<MAG_OBSERVATIONS, STATES, T, _>::new(
                MatrixData::new_array::<MAG_OBSERVATIONS, STATES, { MAG_OBSERVATIONS * STATES }, T>(
                    [zero; { MAG_OBSERVATIONS * STATES }],
                ),
            );
        observation_matrix.apply(|mat| {
            mat.set_at(0, 0, T::one());
            mat.set_at(1, 1, T::one());
            mat.set_at(2, 2, T::one());
        });

        // Measurement noise covariance
        let mut noise_covariance =
            MeasurementNoiseCovarianceMatrixBuffer::<MAG_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    MAG_OBSERVATIONS,
                    MAG_OBSERVATIONS,
                    { MAG_OBSERVATIONS * MAG_OBSERVATIONS },
                    T,
                >([zero; { MAG_OBSERVATIONS * MAG_OBSERVATIONS }]),
            );

        let noise_covariance_value = accelerometer_noise.x * magnetometer_noise.x
            + accelerometer_noise.y * magnetometer_noise.y
            + accelerometer_noise.z * magnetometer_noise.z;
        noise_covariance.make_scalar(noise_covariance_value);

        // Innovation vector
        let innovation_vector =
            InnovationVectorBuffer::<MAG_OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                MAG_OBSERVATIONS,
                1,
                MAG_OBSERVATIONS,
                T,
            >(
                [zero; MAG_OBSERVATIONS]
            ));

        // Innovation covariance matrix
        let innovation_covariance = InnovationCovarianceMatrixBuffer::<MAG_OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<
                MAG_OBSERVATIONS,
                MAG_OBSERVATIONS,
                { MAG_OBSERVATIONS * MAG_OBSERVATIONS },
                T,
            >([zero; { MAG_OBSERVATIONS * MAG_OBSERVATIONS }]),
        );

        // Kalman Gain matrix
        let kalman_gain =
            KalmanGainMatrixBuffer::<STATES, MAG_OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                STATES,
                MAG_OBSERVATIONS,
                { STATES * MAG_OBSERVATIONS },
                T,
            >(
                [zero; { STATES * MAG_OBSERVATIONS }],
            ));

        // Temporary residual covariance inverted matrix
        let temp_sinv =
            TemporaryResidualCovarianceInvertedMatrixBuffer::<MAG_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    MAG_OBSERVATIONS,
                    MAG_OBSERVATIONS,
                    { MAG_OBSERVATIONS * MAG_OBSERVATIONS },
                    T,
                >([zero; { MAG_OBSERVATIONS * MAG_OBSERVATIONS }]),
            );

        // Temporary H×P matrix
        let temp_hp = TemporaryHPMatrixBuffer::<MAG_OBSERVATIONS, STATES, T, _>::new(
            MatrixData::new_array::<MAG_OBSERVATIONS, STATES, { MAG_OBSERVATIONS * STATES }, T>(
                [zero; { MAG_OBSERVATIONS * STATES }],
            ),
        );

        // Temporary P×Hᵀ matrix
        let temp_pht = TemporaryPHTMatrixBuffer::<STATES, MAG_OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<STATES, MAG_OBSERVATIONS, { STATES * MAG_OBSERVATIONS }, T>(
                [zero; { STATES * MAG_OBSERVATIONS }],
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

        ExtendedObservationBuilder::new::<STATES, MAG_OBSERVATIONS, T>(
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
        magnetometer_noise: &MagnetometerNoise<T>,
    ) -> OwnedVector3Observation<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // Measurement vector
        let measurement =
            MeasurementVectorBuffer::<ACCEL_OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                ACCEL_OBSERVATIONS,
                1,
                ACCEL_OBSERVATIONS,
                T,
            >(
                [zero; ACCEL_OBSERVATIONS]
            ));

        // Observation matrix
        let mut observation_matrix =
            ObservationMatrixMutBuffer::<ACCEL_OBSERVATIONS, STATES, T, _>::new(
                MatrixData::new_array::<
                    ACCEL_OBSERVATIONS,
                    STATES,
                    { ACCEL_OBSERVATIONS * STATES },
                    T,
                >([zero; { ACCEL_OBSERVATIONS * STATES }]),
            );
        observation_matrix.apply(|mat| {
            mat.set_at(0, 0, T::one());
            mat.set_at(1, 1, T::one());
            mat.set_at(2, 2, T::one());
        });

        // Measurement noise covariance
        let mut noise_covariance =
            MeasurementNoiseCovarianceMatrixBuffer::<ACCEL_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    ACCEL_OBSERVATIONS,
                    ACCEL_OBSERVATIONS,
                    { ACCEL_OBSERVATIONS * ACCEL_OBSERVATIONS },
                    T,
                >([zero; { ACCEL_OBSERVATIONS * ACCEL_OBSERVATIONS }]),
            );

        let noise_covariance_value = accelerometer_noise.x * magnetometer_noise.x
            + accelerometer_noise.y * magnetometer_noise.y
            + accelerometer_noise.z * magnetometer_noise.z;
        noise_covariance.make_scalar(noise_covariance_value);

        // Innovation vector
        let innovation_vector =
            InnovationVectorBuffer::<ACCEL_OBSERVATIONS, T, _>::new(MatrixData::new_array::<
                ACCEL_OBSERVATIONS,
                1,
                ACCEL_OBSERVATIONS,
                T,
            >(
                [zero; ACCEL_OBSERVATIONS]
            ));

        // Innovation covariance matrix
        let innovation_covariance =
            InnovationCovarianceMatrixBuffer::<ACCEL_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    ACCEL_OBSERVATIONS,
                    ACCEL_OBSERVATIONS,
                    { ACCEL_OBSERVATIONS * ACCEL_OBSERVATIONS },
                    T,
                >([zero; { ACCEL_OBSERVATIONS * ACCEL_OBSERVATIONS }]),
            );

        // Kalman Gain matrix
        let kalman_gain = KalmanGainMatrixBuffer::<STATES, ACCEL_OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<STATES, ACCEL_OBSERVATIONS, { STATES * ACCEL_OBSERVATIONS }, T>(
                [zero; { STATES * ACCEL_OBSERVATIONS }],
            ),
        );

        // Temporary residual covariance inverted matrix
        let temp_sinv =
            TemporaryResidualCovarianceInvertedMatrixBuffer::<ACCEL_OBSERVATIONS, T, _>::new(
                MatrixData::new_array::<
                    ACCEL_OBSERVATIONS,
                    ACCEL_OBSERVATIONS,
                    { ACCEL_OBSERVATIONS * ACCEL_OBSERVATIONS },
                    T,
                >([zero; { ACCEL_OBSERVATIONS * ACCEL_OBSERVATIONS }]),
            );

        // Temporary H×P matrix
        let temp_hp = TemporaryHPMatrixBuffer::<ACCEL_OBSERVATIONS, STATES, T, _>::new(
            MatrixData::new_array::<ACCEL_OBSERVATIONS, STATES, { ACCEL_OBSERVATIONS * STATES }, T>(
                [zero; { ACCEL_OBSERVATIONS * STATES }],
            ),
        );

        // Temporary P×Hᵀ matrix
        let temp_pht = TemporaryPHTMatrixBuffer::<STATES, ACCEL_OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<STATES, ACCEL_OBSERVATIONS, { STATES * ACCEL_OBSERVATIONS }, T>(
                [zero; { STATES * ACCEL_OBSERVATIONS }],
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

        ExtendedObservationBuilder::new::<STATES, ACCEL_OBSERVATIONS, T>(
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
