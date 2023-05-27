#include "iostream"
#include "Eigen/Dense"
#include "opencv4/opencv2/opencv.hpp"

using namespace cv;

int main()
{
    Eigen::MatrixXd m(2, 2);

    m(0, 0) = 1;
    m(0, 1) = 0;
    m(1, 0) = 0;
    m(1, 1) = 2;

    std::cout << m << std::endl;

    Eigen::MatrixXd m2(2, 2);
    std::cout << m2 << std::endl;
    return 0;
}