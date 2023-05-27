cmake_minimum_required(VERSION 3.5)
project(ekf-slam)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

add_library(utils src/utils.cpp)
target_include_directories(utils PUBLIC inc/)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ekfslam src/main.cpp)
target_link_libraries(ekfslam utils Eigen3::Eigen ${OpenCV_LIBS})