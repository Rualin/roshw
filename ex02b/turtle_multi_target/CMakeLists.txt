cmake_minimum_required(VERSION 3.8)
project(turtle_multi_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtle_multi_target_intefaces REQUIRED)

add_executable(target_switcher src/target_switcher.cpp)
ament_target_dependencies(
  target_switcher
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(
  turtle_controller
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
  turtle_multi_target_intefaces
)

install(TARGETS
  target_switcher
  turtle_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
