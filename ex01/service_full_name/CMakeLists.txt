cmake_minimum_required(VERSION 3.8)
project(service_full_name)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SummFullName.srv"
)

add_executable(service_name src/service_name.cpp)
ament_target_dependencies(service_name rclcpp std_msgs)

add_executable(client_name src/client_name.cpp)
ament_target_dependencies(client_name rclcpp std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(service_name "${cpp_typesupport_target}")
target_link_libraries(client_name "${cpp_typesupport_target}")

install(TARGETS
    service_name
    client_name
  DESTINATION lib/${PROJECT_NAME})

ament_package()
