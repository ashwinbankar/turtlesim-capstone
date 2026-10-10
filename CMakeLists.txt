cmake_minimum_required(VERSION 3.20)
project(turtlesim_project_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtlesim_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Spawner Node
add_executable(spawner src/turtle_spawner.cpp)
target_link_libraries(spawner
  rclcpp::rclcpp
  turtlesim_msgs::turtlesim_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  my_robot_interfaces::my_robot_interfaces__rosidl_typesupport_cpp
)

# Controller Node
add_executable(controller src/turtle_controller.cpp)
target_link_libraries(controller
  rclcpp::rclcpp
  turtlesim_msgs::turtlesim_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  my_robot_interfaces::my_robot_interfaces__rosidl_typesupport_cpp
)

# Install Rules
install(TARGETS
  controller
  spawner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()