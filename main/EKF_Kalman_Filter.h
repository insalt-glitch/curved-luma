#pragma once

#include "LinearAlgebra.h"
#include "Sensors.h"

namespace Kalman {

template <int n>
void PredictionStepEKF(
    const StateVector& mu_prior, const BLA::Matrix<n, n>& cov_prior,
    const MeasurementVector& measurement, const BLA::Matrix<n, n>& cov_motion_noise,
    const f32 dt,
    void (*f_motion)(
        const MeasurementVector&, const StateVector&,
        const f32, StateVector*, BLA::Matrix<n, n>*),
    StateVector* const mu_prediction, BLA::Matrix<n, n>* const cov_prediction)
    // Calculates mean and covariance of predicted state
    // density using a EKF model.
{
    // TODO(Nils): Gyroscope noise is usually time dependent (and obviously orientation-dependend)!
    BLA::Matrix<n, n> jacobian;
    // Evaluate motion model
    (*f_motion)(measurement, mu_prior, dt, mu_prediction, &jacobian);
    // Covariance prediciton
    *cov_prediction = jacobian * cov_prior * (~jacobian) + cov_motion_noise;
    // Make sure P is symmetric
    *cov_prediction = 0.5f * (*cov_prediction + (~(*cov_prediction)));
}

template <int n, int m>
void UpdateStepEKF(
    const StateVector& mu_prior, const BLA::Matrix<n, n>& cov_prior,
    const BLA::Matrix<m, 1>& mu_measurement, const BLA::Matrix<m, m>& cov_measurement_noise,
    void (*h_measurement)(const StateVector&, BLA::Matrix<m, 1>*, BLA::Matrix<m, n>*),
    StateVector* mu_update, BLA::Matrix<n, n>* cov_update)
    // Calculates mean and covariance of predicted state
    // density using a EKF model
{
    // Evaluate measurement model
    BLA::Matrix<m, 1> h_x;
    BLA::Matrix<m, n> jacobian;
    (*h_measurement)(mu_prior, &h_x, &jacobian);

    // Innovation covariance
    const BLA::Matrix<m, m> S = jacobian * cov_prior * (~jacobian) + cov_measurement_noise;
    // Kalman gain
    const BLA::Matrix<n, m> K = cov_prior * (~jacobian) * BLA::Inverse(S);

    // State update
    mu_update->vec = mu_prior.vec + K * (mu_measurement - h_x);
    // Covariance update
    *cov_update = cov_prior - K * S * (~K);
    // Make sure that the covariance is symmetric
    *cov_update = 0.5f * ((*cov_update) + (~(*cov_update)));
}

template <int n>
void UpdateStepRTS(
    const BLA::Matrix<n, 1>& mu_smoother_kplus1  , const BLA::Matrix<n, n>& cov_smoother_kplus1,
    const BLA::Matrix<n, 1>& mu_filter_k         , const BLA::Matrix<n, n>& cov_filter_k,
    const BLA::Matrix<n, 1>& mu_prediction_kplus1, const BLA::Matrix<n, n>& cov_prediction_kplus1,
    void (*f_motion)(const BLA::Matrix<n, 1>&, BLA::Matrix<n, 1>*, BLA::Matrix<n, n>*),
    BLA::Matrix<n, 1>* mu_smoother, BLA::Matrix<n, n>* cov_smoother)
    // Calculates mean and covariance of smoothed state
    // density, using a non-linear Gaussian model.
{
    BLA::Matrix<n, n> jacobian;
    (*f_motion)(mu_filter_k, nullptr, &jacobian);
    const BLA::Matrix<n, n> G_k = cov_filter_k * (~jacobian) * BLA::Inverse(cov_prediction_kplus1);
    *mu_smoother = mu_filter_k + G_k * (mu_smoother_kplus1 - mu_prediction_kplus1);
    *cov_smoother = cov_filter_k - G_k * (cov_prediction_kplus1 - cov_smoother_kplus1) * (~G_k);
}

};
