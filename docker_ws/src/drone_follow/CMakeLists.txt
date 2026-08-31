cmake_minimum_required(VERSION 3.8)
project(drone_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Vyhledání závislostí
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco core highgui imgproc)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 1. Spustitelný cíl: aruco_detector
add_executable(aruco_detector src/aruco_detector.cpp)
target_include_directories(aruco_detector PRIVATE include)
ament_target_dependencies(aruco_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
  OpenCV
)
target_link_libraries(aruco_detector ${OpenCV_LIBRARIES})

# 2. Spustitelný cíl: tracker_control
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

# Instalace binárních souborů
install(TARGETS 
  aruco_detector
  tracker_control
  DESTINATION lib/${PROJECT_NAME}
)

# Instalace hlavičkových souborů
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()