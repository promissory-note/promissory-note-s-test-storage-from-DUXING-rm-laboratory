cmake_minimum_required(VERSION 3.10)
project(assignment1_image)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(task1_image pro16.cpp)
target_include_directories(task1_image PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(task1_image PRIVATE ${OpenCV_LIBS})
