cmake_minimum_required(VERSION 3.8)
project(cat_robot_bring_up)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(circle_publisher src/circle_movement.cpp)
ament_target_dependencies(circle_publisher rclcpp geometry_msgs)

add_executable(eight_publisher src/eight_movement.cpp)
ament_target_dependencies(eight_publisher rclcpp geometry_msgs)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS
  circle_publisher
  eight_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
