cmake_minimum_required(VERSION 3.8)
project(rover_mavlink_communication_updated)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(
    rover_mavlink_node
    src/main.cpp
    src/rover_mavlink_node.cpp
    src/ardupilot_interface.cpp
    src/external_pose_publisher.cpp
)

target_include_directories(
    rover_mavlink_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(rover_mavlink_node PUBLIC cxx_std_17)

ament_target_dependencies(
    rover_mavlink_node
    geometry_msgs
    mavros_msgs
    nav_msgs
    rclcpp
    std_msgs
    tf2
    tf2_ros
)

install(TARGETS rover_mavlink_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
