#ifndef EKF_FILTER_HPP
#define EKF_FILTER_HPP

#include "eigen3/Eigen/Dense"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

class ExtendedKalmanFilter
{
  public:
    ExtendedKalmanFilter() {}
    void Predict();
    void Update();
    void SetMeasurement(double x, double y);
    MatrixXd GetStateEstimation()
    {
        return measurement;
    }

  private:
    double delta_t{};
    MatrixXd state_vector{4, 1};  // x, y, vx, vy
    MatrixXd state_transition_matrix{4, 4};
    MatrixXd control_input{4, 1};  //
    MatrixXd control_input_matrix{4, 4};
    MatrixXd process_noise{4, 1};

    MatrixXd measurement{2, 1};
    MatrixXd measurement_matrix{2, 4};
    MatrixXd measurement_noise{2, 1};
};

#endif