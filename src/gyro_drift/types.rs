use minikalman::buffers::types::*;
use minikalman::prelude::*;
use minikalman::regular::{RegularKalman, RegularObservation};

pub const STATES: usize = 2; // angular velocity, bias
pub const OBSERVATIONS: usize = 1; // angular velocity

// A Kalman filter of four states, using owned buffers.
pub type OwnedKalmanFilter<T> = RegularKalman<
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
pub type OwnedObservation<T> = RegularObservation<
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
        MatrixDataArray<OBSERVATIONS, OBSERVATIONS, { OBSERVATIONS * OBSERVATIONS }, T>,
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
