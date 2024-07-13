use crate::vector3::Vector3;
use crate::{
    AccelerometerNoise, AccelerometerReading, ArcSin, ArcTan, DetectGimbalLock, EulerAngles,
    GyroscopeBias, GyroscopeNoise, GyroscopeReading, IsNaN, MagnetometerNoise, MagnetometerReading,
    NormalizeAngle,
};
use core::ops::Neg;
use minikalman::buffers::types::*;
use minikalman::matrix::MatrixDataType;
use minikalman::prelude::*;
use minikalman::regular::{
    Control, ControlBuilder, RegularKalman, RegularKalmanBuilder, RegularObservation,
    RegularObservationBuilder,
};

const STATES: usize = 6; // roll rate, pitch rate, yaw rate, as well as gyro bias (drift) terms
const CONTROLS: usize = 3; // roll rate, pitch rate, yaw rate
const OBSERVATIONS: usize = 3; // roll, pitch, yaw

/// A MARG (Magnetic, Angular Rate, and Gravity) orientation estimator with generic type `T`.
pub struct OwnedOrientationEstimator<T> {
    filter: OwnedKalmanFilter<T>,
    control: OwnedControlInput<T>,
    measurement: OwnedObservation<T>,
    /// The angular tolerance, in radians, to detect a Gimbal Lock.
    gimbal_lock_tolerance: T,
    /// The autocovariances of the noise terms.
    accelerometer_noise: AccelerometerNoise<T>,
    /// The autocovariances of the noise terms.
    magnetometer_noise: MagnetometerNoise<T>,
    /// A bias term to avoid divisions by zero.
    epsilon: T,
}

impl<T> OwnedOrientationEstimator<T> {
    /// Initializes a new instance of the [`OwnedOrientationEstimator`] struct.
    ///
    /// ## Arguments
    /// * `gimbal_lock_tolerance` - The angular tolerance, in radians, to detect a Gimbal Lock.
    /// * `accelerometer_noise` - The accelerometer noise values (sigma-squared) for each axis.
    /// * `gyroscope_noise` - The gyroscope noise values (sigma-squared) for each axis.
    /// * `magnetometer_noise` - The magnetometer noise values (sigma-squared) for each axis.
    /// * `epsilon` - A small bias term to avoid divisions by zero. Set to e.g. `1e-6`.
    pub fn new(
        gimbal_lock_tolerance: T,
        accelerometer_noise: AccelerometerNoise<T>,
        gyroscope_noise: GyroscopeNoise<T>,
        gyroscope_bias: GyroscopeBias<T>,
        magnetometer_noise: MagnetometerNoise<T>,
        epsilon: T,
    ) -> Self
    where
        T: MatrixDataType + Default,
    {
        let filter = Self::build_filter(&gyroscope_noise, &gyroscope_bias, epsilon);
        let control = Self::build_control(&gyroscope_noise, epsilon);
        let measurement = Self::build_measurement(&accelerometer_noise, &magnetometer_noise);

        Self {
            filter,
            control,
            measurement,
            gimbal_lock_tolerance,
            accelerometer_noise,
            magnetometer_noise,
            epsilon,
        }
    }
}

impl<T> OwnedOrientationEstimator<T> {
    /// Obtains the current estimates of the roll, pitch and yaw angles.
    pub fn estimated_angles(&self) -> EulerAngles<T>
    where
        T: Copy,
    {
        EulerAngles::new(self.roll(), self.pitch(), self.yaw())
    }

    /// Obtains the current estimate of the roll angle φ (phi), in radians.
    ///
    /// The roll angle is defined as the amount rotation around the y-axis (forward).
    pub fn roll(&self) -> T
    where
        T: Copy,
    {
        self.filter.state_vector().get_row(0)
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
        T: Copy,
    {
        self.filter.estimate_covariance().get_at(0, 0)
    }

    /// Obtains the current estimate of the bias term of the roll rate, in radians.
    pub fn roll_rate_bias(&self) -> T
    where
        T: Copy,
    {
        self.filter.state_vector().get_row(3)
    }

    /// Obtains the current estimate of the pitch angle θ (theta), in radians.
    ///
    /// The pitch angle is defined as the amount rotation around the x-axis (right).
    pub fn pitch(&self) -> T
    where
        T: Copy,
    {
        self.filter.state_vector().get_row(1)
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
        T: Copy,
    {
        self.filter.estimate_covariance().get_at(1, 1)
    }

    /// Obtains the current estimate of the bias term of the pitch rate, in radians.
    pub fn pitch_rate_bias(&self) -> T
    where
        T: Copy,
    {
        self.filter.state_vector().get_row(4)
    }

    /// Obtains the current estimate of the yaw angle ψ (psi), in radians.
    ///
    /// The yaw angle is defined as the amount rotation around the z-axis (up).
    pub fn yaw(&self) -> T
    where
        T: Copy,
    {
        self.filter.state_vector().get_row(2)
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
        T: Copy,
    {
        self.filter.estimate_covariance().get_at(2, 2)
    }

    /// Obtains the current estimate of the bias term of the yaw rate, in radians.
    pub fn yaw_rate_bias(&self) -> T
    where
        T: Copy,
    {
        self.filter.state_vector().get_row(5)
    }
}

impl<T> OwnedOrientationEstimator<T> {
    /// Performs a prediction step to obtain new orientation estimates.
    ///
    /// ## Arguments
    /// * `delta_t` - The time step for the prediction.
    /// * `angular_rates` - The angular rates measured by the gyroscope.
    pub fn predict(&mut self, delta_t: T, angular_rates: &GyroscopeReading<T>)
    where
        T: MatrixDataType + Default + NormalizeAngle<T, Output = T> + IsNaN,
    {
        // Update the Kalman filter components.
        self.update_state_transition_matrix(delta_t, angular_rates);
        self.update_control_input(delta_t, angular_rates);

        // Perform a regular Kalman Filter prediction step.
        self.filter.predict();
        self.panic_if_nan();

        // Apply the Gyro control input.
        self.filter.control(&mut self.control);
        self.panic_if_nan();

        // Normalize the state estimates.
        self.normalize_angles();
    }

    /// Performs a correction step using accelerometer and magnetometer readings.
    ///
    /// ## Arguments
    /// * `accelerometer` - The accelerometer reading.
    /// * `magnetometer` - The magnetometer reading.
    pub fn correct(
        &mut self,
        accelerometer: &AccelerometerReading<T>,
        magnetometer: &MagnetometerReading<T>,
    ) where
        T: MatrixDataType
            + ArcSin<T, Output = T>
            + ArcTan<T, Output = T>
            + NormalizeAngle<T, Output = T>
            + DetectGimbalLock<T>
            + core::fmt::Debug
            + IsNaN,
    {
        // Normalize the vectors.
        let a = Vector3::from(accelerometer).normalized();
        let m = Vector3::from(magnetometer).normalized();

        // Obtain the Euler Angles and apply them to the measurement.
        let angles = self.build_triad(a, m);
        self.measurement.measurement_vector_mut().apply(|vec| {
            vec.set_row(0, angles.roll_phi);
            vec.set_row(1, angles.pitch_theta);
            vec.set_row(2, angles.yaw_psi);
        });

        // Update the measurement noise.
        self.update_measurement_noise(a, m);

        // Perform the update step.
        self.filter.correct(&mut self.measurement);
        self.panic_if_nan();

        // Normalize the state estimates.
        self.normalize_angles();
    }

    // Normalizes the state estimates.
    fn normalize_angles(&mut self)
    where
        T: Copy + NormalizeAngle<T, Output = T>,
    {
        self.filter.state_vector_mut().apply(|vec| {
            vec.set_row(0, vec.get_row(0).normalize_angle());
            vec.set_row(1, vec.get_row(1).normalize_angle());
            vec.set_row(2, vec.get_row(2).normalize_angle());
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

    /// Computes the Jacobian matrices and measurement noise matrix for orientation estimation
    ///
    /// This is done by propagating the sensor noise through the TRIAD method
    /// The final measurement noise covariance matrix R is computed using J_a, R_a, J_m, and R_m
    /// R represents the combined effect of accelerometer and magnetometer noise on the orientation estimates,
    /// where J_a and J_m are the Jacobians of the respective measurement noise.
    ///
    /// ## Arguments
    /// * `accelerometer` - The normalized accelerometer vector.
    /// * `magnetometer` - The normalized magnetometer vector.
    #[allow(unused)]
    fn update_measurement_noise(&mut self, accelerometer: Vector3<T>, magnetometer: Vector3<T>)
    where
        T: MatrixDataType + core::fmt::Debug,
    {
        /*
        // Easy access to accelerometer noise.
        let sa11 = self.accelerometer_noise.x; // sigma_a_xx
        let sa22 = self.accelerometer_noise.y; // sigma_a_yy
        let sa33 = self.accelerometer_noise.z; // sigma_a_zz

        // Easy access to magnetometer noise.
        let sm11 = self.magnetometer_noise.x; // sigma_m_xx
        let sm22 = self.magnetometer_noise.y; // sigma_m_yy
        let sm33 = self.magnetometer_noise.z; // sigma_m_zz

        // Helper "constants".
        let two = T::one() + T::one();
        let epsilon = self.epsilon;

        // Easy access to accelerometer components.
        let ax = accelerometer.x;
        let ay = accelerometer.y;
        let az = accelerometer.z;

        // Easy access to magnetometer components.
        let mx = magnetometer.x;
        let my = magnetometer.y;
        let mz = magnetometer.z;

        // Common squares of the accelerometer components.
        let ax2 = accelerometer.x * accelerometer.x;
        let ay2 = accelerometer.y * accelerometer.y;
        let az2 = accelerometer.z * accelerometer.z;

        // Common squares of the magnetometer components.
        let mx2 = magnetometer.x * magnetometer.x;
        let my2 = magnetometer.y * magnetometer.y;
        let mz2 = magnetometer.z * magnetometer.z;

        // Noise matrix.
        let r = self.measurement.measurement_noise_covariance_mut();

        // Common denominator of the matrix components.
        #[rustfmt::skip]
        let denom =
            ax2 * az2 * mx2
                + ax2 * my2
                + two * ax2 * ay * az2 * mx * my
                - two * ax * ay * mx * my
                + two * ax * az2 * az * mx * mz
                - two * ax * az * mx * mz
                + ay2 * az2 * my2
                + ay2 * mx2
                + two * ay * az2 * az * my * mz
                - two * ay * az * my * mz
                + az2 * az2 * mz2
                - two * az2 * mz2
                + mz2;

        let one_minus_az2 = T::one() - az2;
        let one_minus_az2_sqrt = one_minus_az2.square_root();

        // Row 1, column 1.
        let r11_nom = sa11 * sq(ay * az * mx2 + ay * az * my2 + az2 * my * mz - my * mz)
            + sa22 * sq(ax * az * mx2 + ax * az * my2 + az2 * mx * mz - mx * mz)
            + sa33
                * sq(
                    ax2 * mx * my - ax * my * mx2 + ax * ay * my2 + two * ax * az * my * mz
                        - ay2 * mx * my
                        - two * ay * az * mx * mz,
                )
            + sm11 * sq(ax2 * az * my + ay2 * az * my + ay * az2 * mz - ay * mz)
            + sm22 * sq(ax2 * az * mx + ax * az2 * mz - ax * mz + ay2 * az * mx)
            + sm33 * sq(ax * az2 * my - ax * my - ay * az2 * mx + ay * mx);
        let r11 = r11_nom / (sq(denom) + epsilon);
        r.set_at(0, 0, r11);

        // Row 1, column 2 and row 2, column 1.
        let r12_nom = sa33
            * (-ax2 * mx * my + ax * ay * mx2 - ax * ay * my2 - two * ax * az * my * mz
                + ay2 * mx * my
                + two * ay * az * mx * mz);
        let r12 = r12_nom / (one_minus_az2_sqrt * denom + epsilon);
        r.set_symmetric(0, 1, r12);

        // Row 1, column 3 and row 3, column 1.
        let r13_nom = ax * sa22 * (ax * az * mx2 + ax * az * my2 + az2 * mx * mz - mx * mz)
            + ay * sa11 * (ay * az * mx2 + ay * az * my2 + az2 * my * mz - my * mz);
        let r13 = r13_nom / ((ax2 + ay2) * denom + epsilon);
        r.set_symmetric(0, 2, r13);

        // Row 2, column 2.
        let r22 = -sa33 / (ax2 - T::one() + epsilon);
        r.set_at(1, 1, r22);

        // Row 2, column 3 and row 3, column 2.
        r.set_symmetric(1, 2, T::zero());

        // Row 3, column 3.
        let r33 = (ax2 * sa22 + ay2 * sa11) / (sq(ax2 + ay2) + epsilon);
        r.set_at(2, 2, r33);
        */
    }

    /// Constructs the state transition matrix based on the angular rates.
    fn update_state_transition_matrix(&mut self, delta_t: T, angular_rates: &GyroscopeReading<T>)
    where
        T: MatrixDataType + Default,
    {
        let rates = *angular_rates * delta_t;
        let x_bias = self.roll_rate_bias() * delta_t;
        let y_bias = self.pitch_rate_bias() * delta_t;
        let z_bias = self.yaw_rate_bias() * delta_t;

        self.filter.state_transition_mut().apply(|mat| {
            mat.set_at(0, 0, T::one());
            mat.set_at(0, 1, -(rates.omega_z - z_bias));
            mat.set_at(0, 2, rates.omega_y - y_bias);
            mat.set_at(0, 3, -delta_t);
            mat.set_at(0, 4, T::zero());
            mat.set_at(0, 5, T::zero());

            mat.set_at(1, 0, rates.omega_z - z_bias);
            mat.set_at(1, 1, T::one());
            mat.set_at(1, 2, -(rates.omega_x - x_bias));
            mat.set_at(1, 3, T::zero());
            mat.set_at(1, 4, -delta_t);
            mat.set_at(1, 5, T::zero());

            mat.set_at(2, 0, -(rates.omega_y - y_bias));
            mat.set_at(2, 1, rates.omega_x - x_bias);
            mat.set_at(2, 2, T::one());
            mat.set_at(2, 3, T::zero());
            mat.set_at(2, 4, T::zero());
            mat.set_at(2, 5, -delta_t);

            mat.set_at(3, 3, T::one());
            mat.set_at(4, 4, T::one());
            mat.set_at(5, 5, T::one());
        });
    }

    /// Constructs the state transition matrix based on the angular rates.
    fn update_control_input(&mut self, delta_t: T, angular_rates: &GyroscopeReading<T>)
    where
        T: MatrixDataType + Default,
    {
        self.control.control_matrix_mut().apply(|mat| {
            mat.set_at(0, 0, delta_t);
            mat.set_at(1, 1, delta_t);
            mat.set_at(2, 2, delta_t);
        });

        let rates = *angular_rates * delta_t;
        self.control.control_vector_mut().apply(|vec| {
            vec.set_row(0, rates.omega_x);
            vec.set_row(1, rates.omega_y);
            vec.set_row(2, rates.omega_z);
        });
    }

    /// Builds the TRIAD rotation matrix and obtains the Euler angles from it.
    ///
    /// ## Arguments
    /// * `a` - The normalized accelerometer vector.
    /// * `m` - The normalized magnetometer vector.
    fn build_triad(&self, a: Vector3<T>, m: Vector3<T>) -> EulerAngles<T>
    where
        T: MatrixDataType
            + ArcTan<T, Output = T>
            + ArcSin<T, Output = T>
            + Neg<Output = T>
            + DetectGimbalLock<T>
            + NormalizeAngle<T, Output = T>,
    {
        // Define base vectors (i.e., TRIAD).
        let z = -a; // up
        let x = m; // forward
        let y = m.cross(z).normalized(); // left
        let x = y.cross(z).normalized(); // forward

        // TRIAD rotation matrix: [b1, b2, b3], stacked columns

        // Derive Euler angles from TRIAD rotation matrix (Trait-Bryan XYZ order).
        let theta = (-ArcSin::arcsin(z.x)).normalize_angle(); // pitch

        // Handle Gimbal lock situations.
        if theta.close_to_zenith_or_nadir(self.gimbal_lock_tolerance) {
            todo!("test this");
            let psi = ArcTan::atan2(-x.y, y.y); // yaw
            let phi = T::zero(); // roll
            EulerAngles::new(phi.normalize_angle(), theta, psi.normalize_angle())
        } else {
            let phi = ArcTan::atan2(z.y, z.z); // roll
            let psi = ArcTan::atan2(y.x, x.x); // yaw
            EulerAngles::new(phi.normalize_angle(), theta, psi.normalize_angle())
        }
    }
}

impl<T> OwnedOrientationEstimator<T> {
    /// Builds the Kalman filter used for prediction.
    fn build_filter(
        gyroscope_noise: &GyroscopeNoise<T>,
        gyroscope_bias: &GyroscopeBias<T>,
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
        state_vec.set_row(3, gyroscope_bias.omega_x);
        state_vec.set_row(4, gyroscope_bias.omega_y);
        state_vec.set_row(5, gyroscope_bias.omega_z);

        // State transition matrix.
        let state_transition =
            StateTransitionMatrixMutBuffer::<STATES, T, _>::new(MatrixData::new_array::<
                STATES,
                STATES,
                { STATES * STATES },
                T,
            >(
                [zero; { STATES * STATES }]
            ));

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
            mat.set_at(0, 0, gyroscope_noise.x);
            // mat.set_at(0, 1, gyroscope_noise.z * gyroscope_noise.x);
            // mat.set_at(0, 2, gyroscope_noise.y * gyroscope_noise.x);

            // mat.set_at(1, 0, gyroscope_noise.z * gyroscope_noise.y);
            mat.set_at(1, 1, gyroscope_noise.y);
            // mat.set_at(1, 2, gyroscope_noise.x * gyroscope_noise.y);

            // mat.set_at(2, 0, gyroscope_noise.y * gyroscope_noise.z);
            // mat.set_at(2, 1, gyroscope_noise.x * gyroscope_noise.z);
            mat.set_at(2, 2, gyroscope_noise.z);

            mat.set_at(3, 3, gyroscope_noise.x * gyroscope_noise.x);
            mat.set_at(4, 4, gyroscope_noise.y * gyroscope_noise.y);
            mat.set_at(5, 5, gyroscope_noise.z * gyroscope_noise.z);
        });

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

        RegularKalmanBuilder::new::<STATES, T>(
            state_transition,
            state_vec,
            estimate_covariance,
            process_noise,
            predicted_state,
            temp_state_matrix,
        )
    }

    /// Builds the Kalman filter control input.
    fn build_control(
        gyroscope_noise: &GyroscopeNoise<T>,
        process_noise_value: T,
    ) -> OwnedControlInput<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // Control vector.
        let control_vector = ControlVectorBuffer::<CONTROLS, T, _>::new(MatrixData::new_array::<
            CONTROLS,
            1,
            CONTROLS,
            T,
        >([zero; CONTROLS]));

        // Control matrix.
        let mut control_matrix =
            ControlMatrixMutBuffer::<STATES, CONTROLS, T, _>::new(MatrixData::new_array::<
                STATES,
                CONTROLS,
                { STATES * CONTROLS },
                T,
            >(
                [zero; STATES * CONTROLS]
            ));
        control_matrix.apply(|mat| {
            mat.set_at(0, 0, T::one());
            mat.set_at(1, 1, T::one());
            mat.set_at(2, 2, T::one());
        });

        // Process noise matrix.
        let mut process_noise = ControlProcessNoiseCovarianceMatrixMutBuffer::<CONTROLS, T, _>::new(
            MatrixData::new_array::<CONTROLS, CONTROLS, { CONTROLS * CONTROLS }, T>(
                [zero; CONTROLS * CONTROLS],
            ),
        );
        process_noise.make_scalar(process_noise_value);

        // Temporary matrix.
        let temp = TemporaryBQMatrixBuffer::<STATES, CONTROLS, T, _>::new(MatrixData::new_array::<
            STATES,
            CONTROLS,
            { STATES * CONTROLS },
            T,
        >(
            [zero; STATES * CONTROLS],
        ));

        ControlBuilder::new::<STATES, CONTROLS, T>(
            control_matrix,
            control_vector,
            process_noise,
            temp,
        )
    }

    /// Builds the Kalman filter observation used for the prediction.
    fn build_measurement(
        accelerometer_noise: &AccelerometerNoise<T>,
        magnetometer_noise: &MagnetometerNoise<T>,
    ) -> OwnedObservation<T>
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
            mat.set_at(0, 0, T::one());
            mat.set_at(1, 1, T::one());
            mat.set_at(2, 2, T::one());
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

        let noise_covariance_value = accelerometer_noise.x * magnetometer_noise.x
            + accelerometer_noise.y * magnetometer_noise.y
            + accelerometer_noise.z * magnetometer_noise.z;
        noise_covariance.make_scalar(noise_covariance_value);

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

type OwnedKalmanFilter<T> = RegularKalman<
    STATES,
    T,
    StateTransitionMatrixMutBuffer<
        STATES,
        T,
        MatrixDataArray<STATES, STATES, { STATES * STATES }, T>,
    >,
    StateVectorBuffer<STATES, T, MatrixDataArray<STATES, 1, STATES, T>>,
    EstimateCovarianceMatrixBuffer<
        STATES,
        T,
        MatrixDataArray<STATES, STATES, { STATES * STATES }, T>,
    >,
    DirectProcessNoiseCovarianceMatrixMutBuffer<
        STATES,
        T,
        MatrixDataArray<STATES, STATES, { STATES * STATES }, T>,
    >,
    PredictedStateEstimateVectorBuffer<STATES, T, MatrixDataArray<STATES, 1, STATES, T>>,
    TemporaryStateMatrixBuffer<STATES, T, MatrixDataArray<STATES, STATES, { STATES * STATES }, T>>,
>;

type OwnedControlInput<T> = Control<
    STATES,
    CONTROLS,
    T,
    ControlMatrixMutBuffer<STATES, 3, T, MatrixDataArray<STATES, 3, { STATES * CONTROLS }, T>>,
    ControlVectorBuffer<CONTROLS, T, MatrixDataArray<CONTROLS, 1, CONTROLS, T>>,
    ControlProcessNoiseCovarianceMatrixMutBuffer<
        CONTROLS,
        T,
        MatrixDataArray<CONTROLS, CONTROLS, { CONTROLS * CONTROLS }, T>,
    >,
    TemporaryBQMatrixBuffer<
        STATES,
        CONTROLS,
        T,
        MatrixDataArray<STATES, CONTROLS, { STATES * CONTROLS }, T>,
    >,
>;

type OwnedObservation<T> = RegularObservation<
    STATES,
    OBSERVATIONS,
    T,
    ObservationMatrixMutBuffer<
        OBSERVATIONS,
        STATES,
        T,
        MatrixDataArray<OBSERVATIONS, STATES, { OBSERVATIONS * STATES }, T>,
    >,
    MeasurementVectorBuffer<OBSERVATIONS, T, MatrixDataArray<OBSERVATIONS, 1, OBSERVATIONS, T>>,
    MeasurementNoiseCovarianceMatrixBuffer<
        OBSERVATIONS,
        T,
        MatrixDataArray<OBSERVATIONS, OBSERVATIONS, 9, T>,
    >,
    InnovationVectorBuffer<OBSERVATIONS, T, MatrixDataArray<OBSERVATIONS, 1, OBSERVATIONS, T>>,
    InnovationCovarianceMatrixBuffer<
        OBSERVATIONS,
        T,
        MatrixDataArray<OBSERVATIONS, OBSERVATIONS, { OBSERVATIONS * OBSERVATIONS }, T>,
    >,
    KalmanGainMatrixBuffer<
        STATES,
        OBSERVATIONS,
        T,
        MatrixDataArray<STATES, OBSERVATIONS, { STATES * OBSERVATIONS }, T>,
    >,
    TemporaryResidualCovarianceInvertedMatrixBuffer<
        OBSERVATIONS,
        T,
        MatrixDataArray<OBSERVATIONS, OBSERVATIONS, { OBSERVATIONS * OBSERVATIONS }, T>,
    >,
    TemporaryHPMatrixBuffer<
        OBSERVATIONS,
        STATES,
        T,
        MatrixDataArray<OBSERVATIONS, STATES, { OBSERVATIONS * STATES }, T>,
    >,
    TemporaryPHTMatrixBuffer<
        STATES,
        OBSERVATIONS,
        T,
        MatrixDataArray<STATES, OBSERVATIONS, { STATES * OBSERVATIONS }, T>,
    >,
    TemporaryKHPMatrixBuffer<STATES, T, MatrixDataArray<STATES, STATES, { STATES * STATES }, T>>,
>;

#[cfg(test)]
#[cfg(feature = "std")]
mod tests {
    use super::*;
    use crate::accelerometer_reading::AccelerometerReading;

    trait Round<T> {
        /// Rounds to two decimal places.
        fn round2(self) -> T;

        /// Rounds to seven decimal places.
        fn round7(self) -> T;
    }

    impl Round<f32> for f32 {
        fn round2(self) -> f32 {
            (self * 100.0).round() * 0.01
        }

        fn round7(self) -> f32 {
            (self * 1000000.0).round() * 0.000001
        }
    }

    #[test]
    fn test_state_transition_matrix() {
        let gimbal_lock_tolerance = 0.01;
        let accelerometer_noise = AccelerometerNoise::new(0.01, 0.02, 0.03);
        let gyroscope_noise = GyroscopeNoise::new(0.04, 0.05, 0.06);
        let gyroscope_bias = GyroscopeBias::default();
        let magnetometer_noise = MagnetometerNoise::new(0.07, 0.08, 0.09);
        let epsilon = 1e-6;

        let mut estimator: OwnedOrientationEstimator<f32> = OwnedOrientationEstimator::new(
            gimbal_lock_tolerance,
            accelerometer_noise,
            gyroscope_noise,
            gyroscope_bias,
            magnetometer_noise,
            epsilon,
        );

        // Set up example error covariances (sigma phi/theta/psi).
        estimator.filter.estimate_covariance_mut().make_scalar(0.1);

        // Set up example process noise.
        // TODO: Pass as argument
        estimator
            .filter
            .direct_process_noise_mut()
            .make_scalar(0.01);

        // Set up example measurement noise.
        // TODO: Pass as argument
        estimator
            .measurement
            .measurement_noise_covariance_mut()
            .make_scalar(0.01);

        let delta_t: f32 = 0.1;
        let angular_rates = GyroscopeReading::new(0.01, 0.02, -0.01);
        estimator.predict(delta_t, &angular_rates);

        estimator.filter.state_transition().inspect(|mat| {
            assert_eq!(mat.get_at(0, 0), 1.0);
            assert_eq!(mat.get_at(0, 1), 0.001);
            assert_eq!(mat.get_at(0, 2), 0.002);

            assert_eq!(mat.get_at(1, 0), -0.001);
            assert_eq!(mat.get_at(1, 1), 1.0);
            assert_eq!(mat.get_at(1, 2), -0.001);

            assert_eq!(mat.get_at(2, 0), -0.002);
            assert_eq!(mat.get_at(2, 1), 0.001);
            assert_eq!(mat.get_at(2, 2), 1.0);
        });

        estimator.filter.state_vector().inspect(|vec| {
            assert_eq!(vec.get_row(0), 0.001);
            assert_eq!(vec.get_row(1), 0.002);
            assert_eq!(vec.get_row(2), -0.001);
        });

        estimator.filter.estimate_covariance().inspect(|mat| {
            assert_eq!(mat.get_at(0, 0).round2(), 0.11);
            assert_eq!(mat.get_at(0, 1).round2(), -0.0);
            assert_eq!(mat.get_at(0, 2).round2(), 0.0);

            assert_eq!(mat.get_at(1, 0).round2(), -0.0);
            assert_eq!(mat.get_at(1, 1).round2(), 0.11);
            assert_eq!(mat.get_at(1, 2).round2(), 0.0);

            assert_eq!(mat.get_at(2, 0).round2(), 0.0);
            assert_eq!(mat.get_at(2, 1).round2(), 0.0);
            assert_eq!(mat.get_at(2, 2).round2(), 0.11);
        });

        // Incorporate accelerometer vector.
        let accelerometer = AccelerometerReading::new(0.0, 0.0, -9.81);
        let magnetometer = MagnetometerReading::new(0.2, 0.0, 0.4);

        estimator.correct(&accelerometer, &magnetometer);

        estimator.filter.state_vector().inspect(|vec| {
            assert_eq!(vec.get_row(0).round7(), 0.0); // replace with approximate tests
            assert_eq!(vec.get_row(1).round2(), 1.23); // replace with approximate tests
            assert_eq!(vec.get_row(2).round2(), -1.5699999); // replace with approximate tests
        })
    }
}
