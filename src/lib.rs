// Enable no_std mode.
#![cfg_attr(not(feature = "std"), no_std)]
// Ensure allow(unsafe_code) / forbid(unsafe_code) markers.
#![cfg_attr(feature = "unsafe", allow(unsafe_code))]
#![cfg_attr(not(feature = "unsafe"), forbid(unsafe_code))]
// Only enables the `doc_cfg` feature when the `docsrs` configuration attribute is defined.
#![cfg_attr(docsrs, feature(doc_cfg))]

mod accelerometer_reading;
mod gyroscope_reading;
mod macros;
mod magnetometer_reading;

use crate::magnetometer_reading::MagnetometerReading;
use minikalman::buffers::types::*;
use minikalman::prelude::*;
use minikalman::regular::{Control, ControlBuilder, RegularKalman, RegularKalmanBuilder};

const STATES: usize = 3; // roll, pitch, yaw
const CONTROLS: usize = 3; // roll, pitch, yaw

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

/// A MARG orientation estimator.
pub struct OrientationEstimator {}

pub struct OwnedOrientationEstimator<T> {
    filter: OwnedKalmanFilter<T>,
    control: OwnedControlInput<T>,
}

impl<T> OwnedOrientationEstimator<T> {
    pub fn new() -> Self
    where
        T: MatrixDataType + Default,
    {
        let filter = Self::build_filter();
        let control = Self::build_control();

        Self { filter, control }
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
}

#[cfg(test)]
mod tests {
    use super::*;

    trait Round<T> {
        /// Rounds to two decimal places.
        fn round2(self) -> T;
    }

    impl Round<f32> for f32 {
        fn round2(self) -> f32 {
            (self * 100.0).round() * 0.01
        }
    }

    #[test]
    fn test_state_transition_matrix() {
        let mut estimator: OwnedOrientationEstimator<f32> = OwnedOrientationEstimator::new();

        // Set up example error covariances (sigma phi/theta/psi).
        estimator.filter.estimate_covariance_mut().make_scalar(0.1);

        // Set up example process noise.
        estimator
            .filter
            .direct_process_noise_mut()
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
    }
}
