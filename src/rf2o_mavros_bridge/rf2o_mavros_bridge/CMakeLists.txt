cmake_minimum_required(VERSION 3.8)
project(rf2o_mavros_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

add_executable(rf2o_mavros_bridge_node src/rf2o_mavros_bridge_node.cpp)
target_compile_features(rf2o_mavros_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(rf2o_mavros_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  geographic_msgs
)

install(TARGETS rf2o_mavros_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
