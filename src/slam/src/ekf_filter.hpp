#ifndef EKF_FILTER_HPP
#define EKF_FILTER_HPP

#include "eigen3/Eigen/Dense"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

class ExtendedKalmanFilter
{
  public:
    ExtendedKalmanFilter();
    void Predict();
    void Update();
    void SetMeasurement(double x, double y);
    void SetTransitionMatrixValues(double delta_t);
    MatrixXd GetStateEstimation()
    {
        return measurement;
    }

  private:
    MatrixXd state_vector{MatrixXd::Zero(6, 1)};  // x, y, vx, vy, ax, ay
    MatrixXd state_transition_matrix{MatrixXd::Identity(6, 6)};
    MatrixXd control_input{MatrixXd::Zero(6, 1)};  // 0, 0, 0, 0, ax, ay
    MatrixXd control_input_matrix{MatrixXd::Identity(6, 6)};
    MatrixXd process_noise{MatrixXd::Identity(6, 6)};
    MatrixXd prediction_covariance{MatrixXd::Identity(6, 6) * 0.01};

    MatrixXd measurement{MatrixXd::Zero(2, 1)};  // x, y
    MatrixXd measurement_matrix{MatrixXd::Zero(2, 6)};
    MatrixXd measurement_noise{MatrixXd::Identity(2, 2)};
    MatrixXd kalman_gain{6, 2};
};

#endif