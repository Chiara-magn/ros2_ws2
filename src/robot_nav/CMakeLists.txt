cmake_minimum_required(VERSION 3.8)
project(robot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(include)

# navigation server 
add_library(navigation_server SHARED
  src/robot_nav_server.cpp
)

# server dependencies
ament_target_dependencies(navigation_server
  rclcpp
  rclcpp_components
  rclcpp_action
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

rosidl_target_interfaces(navigation_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# movetopose client
add_library(move_to_pose_client SHARED
  src/move_to_pose_client.cpp
)

# client dependencies
ament_target_dependencies(move_to_pose_client
  rclcpp
  rclcpp_components
  rclcpp_action
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

rosidl_target_interfaces(move_to_pose_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Components registrations
rclcpp_components_register_nodes(navigation_server
  "robot_nav::NavigationServer"
)

rclcpp_components_register_nodes(move_to_pose_client
  "robot_nav::MoveToPoseClient"
)

# installations
install(TARGETS
  navigation_server
  move_to_pose_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
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
