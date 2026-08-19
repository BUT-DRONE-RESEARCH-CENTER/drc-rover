cmake_minimum_required(VERSION 3.8)
project(drone_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# 1. Executable: marker_detector
add_executable(marker_detector src/marker_detector.cpp)
target_include_directories(marker_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(marker_detector PUBLIC cxx_std_17)
ament_target_dependencies(marker_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)
target_link_libraries(marker_detector ${OpenCV_LIBRARIES})

# 2. Executable: tracker_control
add_executable(tracker_control src/tracker_control.cpp)
target_include_directories(tracker_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(tracker_control PUBLIC cxx_std_17)
ament_target_dependencies(tracker_control
  rclcpp
  geometry_msgs
)

# Instalace obou binárek
install(TARGETS 
  marker_detector
  tracker_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()