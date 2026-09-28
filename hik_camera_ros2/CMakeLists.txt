cmake_minimum_required(VERSION 3.8)
project(hik_camera_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---- 海康 MVS SDK（rosdep 无法安装，需先手动安装，见 README）----
set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root")
find_path(MVS_INCLUDE_DIR MvCameraControl.h
          PATHS ${MVS_ROOT}/include NO_DEFAULT_PATH)
find_library(MVS_LIBRARY NAMES MvCameraControl
             PATHS ${MVS_ROOT}/lib/64 ${MVS_ROOT}/lib/x64 ${MVS_ROOT}/lib NO_DEFAULT_PATH)
if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR "未找到 MVS SDK（${MVS_ROOT}）。请先按 README 安装海康 MVS，或设置 -DMVS_ROOT=<路径>")
endif()

add_executable(hik_camera_node src/hik_camera_node.cpp)
target_include_directories(hik_camera_node PRIVATE ${MVS_INCLUDE_DIR})
ament_target_dependencies(hik_camera_node rclcpp rcl_interfaces sensor_msgs)
target_link_directories(hik_camera_node PRIVATE ${MVS_ROOT}/lib/64)
target_link_libraries(hik_camera_node ${MVS_LIBRARY})
# MVS 依赖同目录下的其它 .so，设置 rpath 保证运行时能找到
set_target_properties(hik_camera_node PROPERTIES
  BUILD_RPATH "${MVS_ROOT}/lib/64"
  INSTALL_RPATH "${MVS_ROOT}/lib/64")

install(TARGETS hik_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz2 DESTINATION share/${PROJECT_NAME})

ament_package()
