cmake_minimum_required(VERSION 3.8)
project(wheel_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(joy REQUIRED)

include_directories(
  includes
  /opt/ros/humble/include
  /opt/ros/humble/include/rclcpp
  /opt/ros/humble/include/sensor_msgs/sensor_msgs/msg
  /opt/ros/humble/include/gazebo_msgs/gazebo_msgs/msg
)

file(GLOB_RECURSE SRC_PATH CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*)

add_executable(Wheel_robot ${SRC_PATH})
ament_target_dependencies(Wheel_robot rclcpp std_msgs sensor_msgs)
target_include_directories(Wheel_robot PUBLIC ${CMAKE_SOURCE_DIR}/includes)

install(TARGETS Wheel_robot
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  model
  config
  rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
