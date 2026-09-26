cmake_minimum_required(VERSION 3.10)
project(assignment2_fit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)   # 线性最小二乘
find_package(Ceres REQUIRED)    # 非线性精修

add_executable(task2_fit pro18.cpp)
target_include_directories(task2_fit PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR})
target_link_libraries(task2_fit PRIVATE
    ${OpenCV_LIBS}
    Ceres::ceres)
