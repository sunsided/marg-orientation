use minikalman::buffers::types::*;
use minikalman::extended::{ExtendedKalman, ExtendedObservation};
use minikalman::prelude::*;

pub const STATES: usize = 4; // quaternion components
pub const ACCEL_OBSERVATIONS: usize = 3; // x, y, z
pub const MAG_OBSERVATIONS: usize = 3; // x, y, z

// A Kalman filter of four states, using owned buffers.
pub type OwnedKalmanFilter<T> = ExtendedKalman<
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

/// On observation of three states, using owned buffers.
pub type OwnedVector3Observation<T> = ExtendedObservation<
    STATES,
    ACCEL_OBSERVATIONS,
    T,
    ObservationMatrixMutBuffer<
        ACCEL_OBSERVATIONS,
        STATES,
        T,
        MatrixDataArray<ACCEL_OBSERVATIONS, STATES, { ACCEL_OBSERVATIONS * STATES }, T>,
    >,
    MeasurementVectorBuffer<
        ACCEL_OBSERVATIONS,
        T,
        MatrixDataArray<ACCEL_OBSERVATIONS, 1, ACCEL_OBSERVATIONS, T>,
    >,
    MeasurementNoiseCovarianceMatrixBuffer<
        ACCEL_OBSERVATIONS,
        T,
        MatrixDataArray<ACCEL_OBSERVATIONS, ACCEL_OBSERVATIONS, 9, T>,
    >,
    InnovationVectorBuffer<
        ACCEL_OBSERVATIONS,
        T,
        MatrixDataArray<ACCEL_OBSERVATIONS, 1, ACCEL_OBSERVATIONS, T>,
    >,
    InnovationCovarianceMatrixBuffer<
        ACCEL_OBSERVATIONS,
        T,
        MatrixDataArray<ACCEL_OBSERVATIONS, ACCEL_OBSERVATIONS, 9, T>,
    >,
    KalmanGainMatrixBuffer<
        STATES,
        ACCEL_OBSERVATIONS,
        T,
        MatrixDataArray<STATES, ACCEL_OBSERVATIONS, { STATES * ACCEL_OBSERVATIONS }, T>,
    >,
    TemporaryResidualCovarianceInvertedMatrixBuffer<
        ACCEL_OBSERVATIONS,
        T,
        MatrixDataArray<ACCEL_OBSERVATIONS, ACCEL_OBSERVATIONS, 9, T>,
    >,
    TemporaryHPMatrixBuffer<
        ACCEL_OBSERVATIONS,
        STATES,
        T,
        MatrixDataArray<ACCEL_OBSERVATIONS, STATES, { ACCEL_OBSERVATIONS * STATES }, T>,
    >,
    TemporaryPHTMatrixBuffer<
        STATES,
        ACCEL_OBSERVATIONS,
        T,
        MatrixDataArray<STATES, ACCEL_OBSERVATIONS, { STATES * ACCEL_OBSERVATIONS }, T>,
    >,
    TemporaryKHPMatrixBuffer<STATES, T, MatrixDataArray<STATES, STATES, { STATES * STATES }, T>>,
>;
