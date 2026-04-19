cmake_minimum_required(VERSION 3.8)
project(robot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

# action generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

# server component
add_library(navigation_server SHARED
  src/robot_nav_server.cpp
)

ament_target_dependencies(navigation_server
  rclcpp
  rclcpp_components
  rclcpp_action
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

# link generated action types
rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(navigation_server
  "${cpp_typesupport_target}"
)

rclcpp_components_register_nodes(navigation_server
  "robot_nav::NavigationServer"
)

# client component
add_library(move_to_pose_client SHARED
  src/move_to_pose_client.cpp
)

ament_target_dependencies(move_to_pose_client
  rclcpp
  rclcpp_components
  rclcpp_action
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

target_link_libraries(move_to_pose_client
  "${cpp_typesupport_target}"
)

rclcpp_components_register_nodes(move_to_pose_client
  "robot_nav::MoveToPoseClient"
)

# install
install(TARGETS
  navigation_server
  move_to_pose_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/navigation_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()