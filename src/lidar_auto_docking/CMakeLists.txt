cmake_minimum_required(VERSION 3.5)
project(lidar_auto_docking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lidar_auto_docking_messages REQUIRED)
find_package(rclcpp_action REQUIRED)



add_executable(dock_coordinates
  src/dock_coordinates.cpp
  src/perception.cpp
  src/laser_processor.cpp
  src/scan_rotation.cpp
  src/linear_pose_filter_2d.cpp
  src/icp_2d.cpp
  src/tf2listener.cpp
)
ament_target_dependencies(dock_coordinates
  geometry_msgs
  tf2
  sensor_msgs
  rclcpp
  Eigen3
  angles
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  lidar_auto_docking_messages
)
#allow target to access header files defined in include
target_include_directories(dock_coordinates
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)



add_executable(auto_dock
  src/autodock.cpp
  src/controller.cpp
  src/perception.cpp
  src/laser_processor.cpp
  src/scan_rotation.cpp
  src/linear_pose_filter_2d.cpp
  src/icp_2d.cpp
  src/tf2listener.cpp
)
ament_target_dependencies(auto_dock
  nav_msgs
  geometry_msgs
  tf2
  sensor_msgs
  rclcpp
  rclcpp_action
  Eigen3
  angles
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  lidar_auto_docking_messages
)
target_include_directories(auto_dock
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/dock_robot.py
  scripts/dock_ws_auto_start.py
  scripts/undock_robot.py
  scripts/dock_saver.py
  scripts/dock_pose_saver_cli
  scripts/nav2_dock.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
  config
  launch
  initial_dock_pose
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES Dock.stl
  DESTINATION share/${PROJECT_NAME}
)






install(TARGETS
auto_dock
dock_coordinates
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_pytest_test(test_dock_pose_saver
    test/test_dock_pose_saver.py
  )

  ament_add_gtest(test_scan_rotation
    test/test_scan_rotation.cpp
    src/scan_rotation.cpp
    src/laser_processor.cpp
  )
  if(TARGET test_scan_rotation)
    ament_target_dependencies(test_scan_rotation sensor_msgs geometry_msgs tf2 std_msgs)
    target_include_directories(test_scan_rotation
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
  endif()

  ament_add_gtest(test_dock_detection
    test/test_dock_detection.cpp
  )
  if(TARGET test_dock_detection)
    ament_target_dependencies(test_dock_detection
      geometry_msgs
      lidar_auto_docking_messages
      std_msgs
    )
    target_include_directories(test_dock_detection
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
  endif()

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
