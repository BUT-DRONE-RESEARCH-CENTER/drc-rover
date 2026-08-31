cmake_minimum_required(VERSION 3.8)
project(drone_follow)

# Globální nastavení C++ standardu a varování kompilátoru
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# 1. Vyhledání závislostí
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco core highgui imgproc)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Globální zpřístupnění adresáře 'include' pro všechny uzly
include_directories(include)

# ==============================================================================
# 2. Generování vlastních ROS 2 zpráv
# ==============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoMarker.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Načtení cíle pro C++ typovou podporu (volá se pouze JEDNOU pro celý projekt)
rosidl_get_typesupport_target(msg_typesupport "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# ==============================================================================
# 3. Spustitelné cíle (Uzly)
# ==============================================================================

# Node 1: ArUco Detector
add_executable(aruco_detector src/aruco_detector.cpp)
target_link_libraries(aruco_detector ${msg_typesupport} ${OpenCV_LIBRARIES})
ament_target_dependencies(aruco_detector
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
  OpenCV
)

# Node 2: Tracker Control
add_executable(tracker_control src/tracker_control.cpp)
target_link_libraries(tracker_control ${msg_typesupport})
ament_target_dependencies(tracker_control
  rclcpp
  geometry_msgs
)

# Node 3: MAVROS Handler
add_executable(mavros_handler src/mavros_handler.cpp)
target_link_libraries(mavros_handler ${msg_typesupport})
ament_target_dependencies(mavros_handler
  rclcpp
  geometry_msgs
  mavros_msgs
)

# ==============================================================================
# 4. Instalace
# ==============================================================================
install(TARGETS 
  aruco_detector
  tracker_control
  mavros_handler
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()