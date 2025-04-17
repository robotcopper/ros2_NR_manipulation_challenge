cmake_minimum_required(VERSION 3.8)
project(ros2_nr_motion_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Create executable
add_executable(joint_angle_publisher src/joint_angle_publisher.cpp)
ament_target_dependencies(joint_angle_publisher rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs)

add_executable(joint_sine_publisher src/joint_sine_publisher.cpp)
ament_target_dependencies(joint_sine_publisher rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs)


install(TARGETS
  joint_angle_publisher
  joint_sine_publisher
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  ${action_msgs_INCLUDE_DIRS}
)

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
