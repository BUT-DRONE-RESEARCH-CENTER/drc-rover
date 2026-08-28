cmake_minimum_required(VERSION 3.8)

project(rover_mavlink_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mavros_msgs REQUIRED)


# ============================================================
# Executable
# ============================================================

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


ament_target_dependencies(
    rover_mavlink_node

    rclcpp
    geometry_msgs
    tf2
    tf2_ros
    mavros_msgs
)


# ============================================================
# Install
# ============================================================

install(
    TARGETS rover_mavlink_node

    DESTINATION lib/${PROJECT_NAME}
)


install(
    DIRECTORY include/

    DESTINATION include
)


# ============================================================
# Testing
# ============================================================

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

endif()


ament_package()