cmake_minimum_required(VERSION 3.8)
project(roboracer_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(safety_brake_node src/safety_brake_node.cpp)
ament_target_dependencies(safety_brake_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  control_node
  safety_brake_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
