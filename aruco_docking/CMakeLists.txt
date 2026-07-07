cmake_minimum_required(VERSION 3.8)
project(aruco_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    nodes/dock_pose_estimator.py
    nodes/charging_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
