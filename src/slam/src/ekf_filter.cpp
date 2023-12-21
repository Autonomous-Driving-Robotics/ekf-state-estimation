#ifndef EKF_FILTER_CPP
#define EKF_FILTER_CPP

#include "ekf_filter.hpp"

void ExtendedKalmanFilter::Predict()
{
    state_vector = state_transition_matrix * state_vector + control_input_matrix * control_input +
                   process_noise;
}

void ExtendedKalmanFilter::Update()
{
    measurement = measurement_matrix * measurement + measurement_noise;
}

void ExtendedKalmanFilter::SetMeasurement(double x, double y)
{
    measurement(0, 0) = x;
    measurement(1, 0) = y;
}

#endif
