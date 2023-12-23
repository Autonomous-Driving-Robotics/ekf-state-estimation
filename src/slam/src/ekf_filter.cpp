#ifndef EKF_FILTER_CPP
#define EKF_FILTER_CPP

#include "ekf_filter.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
    // Set control input
    control_input(4, 0) = 0.1;
    control_input(5, 0) = 0.1;

    // Set measurement matrix
    measurement_matrix(0, 0) = 1;
    measurement_matrix(1, 1) = 1;
}

void ExtendedKalmanFilter::SetTransitionMatrixValues(double delta_t)
{
    // Set state transition matrix
    state_transition_matrix(0, 2) = delta_t;
    state_transition_matrix(0, 4) = 0.5 * delta_t * delta_t;
    state_transition_matrix(1, 4) = delta_t;
    state_transition_matrix(1, 5) = 0.5 * delta_t * delta_t;
    state_transition_matrix(2, 4) = delta_t;
    state_transition_matrix(3, 5) = delta_t;
}

void ExtendedKalmanFilter::Predict()
{
    state_vector = state_transition_matrix * state_vector + control_input_matrix * control_input;
    prediction_covariance =
        state_transition_matrix * prediction_covariance * state_transition_matrix.transpose() +
        process_noise;
}

void ExtendedKalmanFilter::Update()
{
    MatrixXd measurement_state{measurement_matrix * state_vector};
    MatrixXd innovation_covariance{measurement_noise + measurement_matrix * prediction_covariance *
                                                           measurement_matrix.transpose()};
    kalman_gain =
        prediction_covariance * measurement_matrix.transpose() * innovation_covariance.inverse();
    state_vector = state_vector + kalman_gain * (measurement - measurement_state);
    prediction_covariance =
        prediction_covariance - kalman_gain * innovation_covariance * kalman_gain.transpose();
}

void ExtendedKalmanFilter::SetMeasurement(double x, double y)
{
    measurement(0, 0) = x;
    measurement(1, 0) = y;
}

#endif
