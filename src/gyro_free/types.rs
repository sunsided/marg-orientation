use minikalman::buffers::types::*;
use minikalman::extended::{ExtendedKalman, ExtendedObservation};
use minikalman::prelude::*;

pub const STATES: usize = 4; // quaternion components
pub const VEC3_OBSERVATIONS: usize = 3; // x, y, z

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
    VEC3_OBSERVATIONS,
    T,
    ObservationMatrixMutBuffer<
        VEC3_OBSERVATIONS,
        STATES,
        T,
        MatrixDataArray<VEC3_OBSERVATIONS, STATES, { VEC3_OBSERVATIONS * STATES }, T>,
    >,
    MeasurementVectorBuffer<
        VEC3_OBSERVATIONS,
        T,
        MatrixDataArray<VEC3_OBSERVATIONS, 1, VEC3_OBSERVATIONS, T>,
    >,
    MeasurementNoiseCovarianceMatrixBuffer<
        VEC3_OBSERVATIONS,
        T,
        MatrixDataArray<
            VEC3_OBSERVATIONS,
            VEC3_OBSERVATIONS,
            { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS },
            T,
        >,
    >,
    InnovationVectorBuffer<
        VEC3_OBSERVATIONS,
        T,
        MatrixDataArray<VEC3_OBSERVATIONS, 1, VEC3_OBSERVATIONS, T>,
    >,
    InnovationCovarianceMatrixBuffer<
        VEC3_OBSERVATIONS,
        T,
        MatrixDataArray<VEC3_OBSERVATIONS, VEC3_OBSERVATIONS, 9, T>,
    >,
    KalmanGainMatrixBuffer<
        STATES,
        VEC3_OBSERVATIONS,
        T,
        MatrixDataArray<STATES, VEC3_OBSERVATIONS, { STATES * VEC3_OBSERVATIONS }, T>,
    >,
    TemporaryResidualCovarianceInvertedMatrixBuffer<
        VEC3_OBSERVATIONS,
        T,
        MatrixDataArray<
            VEC3_OBSERVATIONS,
            VEC3_OBSERVATIONS,
            { VEC3_OBSERVATIONS * VEC3_OBSERVATIONS },
            T,
        >,
    >,
    TemporaryHPMatrixBuffer<
        VEC3_OBSERVATIONS,
        STATES,
        T,
        MatrixDataArray<VEC3_OBSERVATIONS, STATES, { VEC3_OBSERVATIONS * STATES }, T>,
    >,
    TemporaryPHTMatrixBuffer<
        STATES,
        VEC3_OBSERVATIONS,
        T,
        MatrixDataArray<STATES, VEC3_OBSERVATIONS, { STATES * VEC3_OBSERVATIONS }, T>,
    >,
    TemporaryKHPMatrixBuffer<STATES, T, MatrixDataArray<STATES, STATES, { STATES * STATES }, T>>,
>;
