// Enable no_std mode.
#![cfg_attr(not(feature = "std"), no_std)]
// Ensure allow(unsafe_code) / forbid(unsafe_code) markers.
#![cfg_attr(feature = "unsafe", allow(unsafe_code))]
#![cfg_attr(not(feature = "unsafe"), forbid(unsafe_code))]
// Only enables the `doc_cfg` feature when the `docsrs` configuration attribute is defined.
#![cfg_attr(docsrs, feature(doc_cfg))]

mod accelerometer_reading;
mod euler_angles;
mod gyroscope_reading;
mod macros;
mod magnetometer_reading;
mod num_traits;
mod vector3;

use crate::accelerometer_reading::AccelerometerReading;
use crate::euler_angles::EulerAngles;
use crate::magnetometer_reading::MagnetometerReading;
use crate::vector3::Vector3;
use core::ops::Neg;
use minikalman::buffers::types::*;
use minikalman::prelude::*;
use minikalman::regular::{
    Control, ControlBuilder, RegularKalman, RegularKalmanBuilder, RegularObservation,
    RegularObservationBuilder,
};

pub use crate::num_traits::*;

const STATES: usize = 3; // roll rate, pitch rate, yaw rate
const CONTROLS: usize = 3; // roll rate, pitch rate, yaw rate
const OBSERVATIONS: usize = 3; // roll, pitch, yaw

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
    ControlProcessNoiseCovarianceMatrixBuffer<
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
        MatrixDataArray<OBSERVATIONS, OBSERVATIONS, 9, T>,
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
        MatrixDataArray<OBSERVATIONS, OBSERVATIONS, 9, T>,
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

/// A MARG orientation estimator.
pub struct OrientationEstimator {}

pub struct OwnedOrientationEstimator<T> {
    filter: OwnedKalmanFilter<T>,
    control: OwnedControlInput<T>,
    measurement: OwnedObservation<T>,
    /// The angular tolerance, in radians, to detect a Gimbal Lock.
    gimbal_lock_tolerance: T,
}

impl<T> OwnedOrientationEstimator<T> {
    /// Initializes a new instance of the [`OwnedOrientationEstimator`] struct.
    ///
    /// ## Arguments
    /// * `gimbal_lock_tolerance` - The angular tolerance, in radians, to detect a Gimbal Lock.
    pub fn new(gimbal_lock_tolerance: T) -> Self
    where
        T: MatrixDataType + Default,
    {
        // TODO: Pass as argument
        // TODO: Add sensor covariances

        let filter = Self::build_filter();
        let control = Self::build_control();
        let measurement = Self::build_measurement();

        Self {
            filter,
            control,
            measurement,
            gimbal_lock_tolerance,
        }
    }
}

impl<T> OwnedOrientationEstimator<T> {
    pub fn predict(&mut self, delta_t: T, angular_rates: &MagnetometerReading<T>)
    where
        T: MatrixDataType + Default,
    {
        // Update the Kalman filter components.
        self.update_state_transition_matrix(delta_t, angular_rates);
        self.update_control_input(delta_t, angular_rates);

        // Perform a regular Kalman Filter prediction step.
        self.filter.predict();
        self.filter.control(&mut self.control);

        // TODO: Clamp any angles?
    }

    pub fn correct(
        &mut self,
        accelerometer: &AccelerometerReading<T>,
        magnetometer: &MagnetometerReading<T>,
    ) where
        T: MatrixDataType + ArcSin<T, Output = T> + ArcTan<T, Output = T> + DetectGimbalLock<T>,
    {
        // Obtain the Euler Angles and apply them to the measurement.
        let angles = self.build_triad(accelerometer, magnetometer);
        self.measurement.measurement_vector_mut().apply(|vec| {
            vec.set_row(0, angles.roll_phi);
            vec.set_row(1, angles.pitch_theta);
            vec.set_row(2, angles.yaw_psi);
        });

        // Perform the update step.
        self.filter.correct(&mut self.measurement);

        // TODO: Clamp any angles?
    }

    /// Constructs the state transition matrix based on the angular rates.
    fn update_state_transition_matrix(&mut self, delta_t: T, angular_rates: &MagnetometerReading<T>)
    where
        T: MatrixDataType + Default,
    {
        let rates = *angular_rates * delta_t;
        self.filter.state_transition_mut().apply(|mat| {
            mat.set_at(0, 0, T::one());
            mat.set_at(0, 1, -rates.z);
            mat.set_at(0, 2, rates.y);

            mat.set_at(1, 0, rates.z);
            mat.set_at(1, 1, T::one());
            mat.set_at(1, 2, -rates.x);

            mat.set_at(2, 0, -rates.y);
            mat.set_at(2, 1, rates.x);
            mat.set_at(2, 2, T::one());
        });
    }

    /// Constructs the state transition matrix based on the angular rates.
    fn update_control_input(&mut self, delta_t: T, angular_rates: &MagnetometerReading<T>)
    where
        T: MatrixDataType + Default,
    {
        let rates = *angular_rates * delta_t;
        self.control.control_vector_mut().apply(|vec| {
            vec.set_row(0, rates.x);
            vec.set_row(1, rates.y);
            vec.set_row(2, rates.z);
        });
    }

    fn build_triad(
        &self,
        accelerometer: &AccelerometerReading<T>,
        magnetometer: &MagnetometerReading<T>,
    ) -> EulerAngles<T>
    where
        T: MatrixDataType
            + ArcTan<T, Output = T>
            + ArcSin<T, Output = T>
            + Neg<Output = T>
            + DetectGimbalLock<T>,
    {
        let a = Vector3::from(accelerometer).normalized();
        let m = Vector3::from(magnetometer).normalized();

        // Calculate TRIAD vectors.
        let b1 = a;
        let b2 = (m - a * (m * a)).normalized();
        let b3 = b2.cross(b1);
        // TRIAD rotation matrix: [b1, b2, b3], stacked columns

        // Derive Euler angles from TRIAD rotation matrix (ZYX sequence).
        let theta = -ArcSin::arcsin(b1.z); // pitch

        // Handle Gimbal lock situations.
        if theta.close_to_zenith_or_nadir(self.gimbal_lock_tolerance) {
            let psi = ArcTan::atan2(-b3.y, b2.y); // yaw
            let phi = T::zero(); // roll
            EulerAngles::new(phi, theta, psi)
        } else {
            let psi = ArcTan::atan2(b1.y, b1.x); // yaw
            let phi = ArcTan::atan2(b2.z, b3.z); // roll
            EulerAngles::new(phi, theta, psi)
        }
    }
}

impl<T> OwnedOrientationEstimator<T> {
    /// Builds the Kalman filter used for prediction.
    fn build_filter() -> OwnedKalmanFilter<T>
    where
        T: MatrixDataType + Default,
    {
        let zero = T::default();

        // State vector.
        let state_vec =
            StateVectorBuffer::<STATES, T, _>::new(MatrixData::new_array::<STATES, 1, STATES, T>(
                [zero; STATES],
            ));

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
        estimate_covariance.make_identity();

        // Process noise matrix.
        let process_noise = DirectProcessNoiseCovarianceMatrixMutBuffer::<STATES, T, _>::new(
            MatrixData::new_array::<STATES, STATES, { STATES * STATES }, T>(
                [zero; { STATES * STATES }],
            ),
        );

        // Predicted state vector.
        let mut predicted_state =
            PredictedStateEstimateVectorBuffer::<STATES, T, _>::new(MatrixData::new_array::<
                STATES,
                1,
                STATES,
                T,
            >([zero; STATES]));
        predicted_state.set_all(T::one());

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

    /// Builds the Kalman filter used for prediction.
    fn build_control() -> OwnedControlInput<T>
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
        control_matrix.make_identity();

        // Process noise matrix.
        let process_noise = ControlProcessNoiseCovarianceMatrixBuffer::<CONTROLS, T, _>::new(
            MatrixData::new_array::<CONTROLS, CONTROLS, { CONTROLS * CONTROLS }, T>(
                [zero; CONTROLS * CONTROLS],
            ),
        );

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

    /// Builds the Kalman filter used for prediction.
    fn build_measurement() -> OwnedObservation<T>
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
        observation_matrix.make_identity();

        // Measurement noise covariance
        let noise_covariance = MeasurementNoiseCovarianceMatrixBuffer::<OBSERVATIONS, T, _>::new(
            MatrixData::new_array::<OBSERVATIONS, OBSERVATIONS, { OBSERVATIONS * OBSERVATIONS }, T>(
                [zero; { OBSERVATIONS * OBSERVATIONS }],
            ),
        );
        // TODO: Apply measurement noise!

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

#[cfg(test)]
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
        let mut estimator: OwnedOrientationEstimator<f32> =
            OwnedOrientationEstimator::new(gimbal_lock_tolerance);

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
        let angular_rates = MagnetometerReading::new(0.01, 0.02, -0.01);
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
            assert_eq!(vec.get_row(0).round7(), 0.000083); // replace with approximate tests
            assert_eq!(vec.get_row(1).round2(), 1.4399999); // replace with approximate tests
            assert_eq!(vec.get_row(2).round2(), -1.4399999); // replace with approximate tests
        })
    }
}
