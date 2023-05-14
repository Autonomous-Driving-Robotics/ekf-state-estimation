cmake_minimum_required(VERSION 3.5)
project(ekf-slam)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(utils src/utils.cpp)
target_include_directories(utils PUBLIC inc/)

add_executable(ekfslam src/main.cpp)
target_link_libraries(ekfslam utils)
target_link_libraries(ekfslam Eigen3::Eigen)