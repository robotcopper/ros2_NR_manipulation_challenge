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
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

# Create executable
add_executable(joint_angle_publisher src/joint_angle_publisher.cpp)
ament_target_dependencies(joint_angle_publisher rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs)

add_executable(joint_sine_publisher src/joint_sine_publisher.cpp)
ament_target_dependencies(joint_sine_publisher rclcpp trajectory_msgs sensor_msgs)

add_executable(joint_space_trajectory_node src/joint_space_trajectory_node.cpp)
ament_target_dependencies(joint_space_trajectory_node rclcpp trajectory_msgs)

add_executable(cartesian_space_trajectory_node src/cartesian_space_trajectory_node.cpp)
ament_target_dependencies(cartesian_space_trajectory_node rclcpp orocos_kdl kdl_parser urdf trajectory_msgs sensor_msgs )

install(TARGETS
  joint_angle_publisher
  joint_sine_publisher
  joint_space_trajectory_node
  cartesian_space_trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(joint_space_trajectory_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(joint_space_trajectory_node
  orocos-kdl
)

install(DIRECTORY launch include
  DESTINATION share/${PROJECT_NAME}
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
