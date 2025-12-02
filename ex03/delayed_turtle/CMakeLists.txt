cmake_minimum_required(VERSION 3.8)
project(delayed_turtle)

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

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
    turtle_tf2_broadcaster
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    turtlesim
)

add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
ament_target_dependencies(
    turtle_tf2_listener
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    turtlesim
)

install(TARGETS
    turtle_tf2_broadcaster
    turtle_tf2_listener
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
