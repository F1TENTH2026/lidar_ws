cmake_minimum_required(VERSION 3.5)
project(f1tenth_lidar)

if(WIN32)
  message(WARNING "${PROJECT_NAME} is currently not supported on Windows")
  ament_package()
  return()
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── C++ 의존성 ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Boost REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(laser_proc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urg_c REQUIRED)
find_package(urg_node_msgs REQUIRED)

# ── C++ 라이브러리 (Hokuyo LiDAR 드라이버) ─────────────────────────────────
add_library(f1tenth_lidar_cpp SHARED
  src/urg_c_wrapper.cpp
  src/urg_node.cpp
)
target_include_directories(f1tenth_lidar_cpp
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(f1tenth_lidar_cpp
  diagnostic_updater
  laser_proc
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  urg_c
  urg_node_msgs
  Boost
)
rclcpp_components_register_node(f1tenth_lidar_cpp
  PLUGIN "urg_node::UrgNode"
  EXECUTABLE urg_node)

# ── 유틸리티 실행파일 ───────────────────────────────────────────────────────
add_executable(getID src/get_id.cpp)
target_link_libraries(getID f1tenth_lidar_cpp)

# ── 설치 ───────────────────────────────────────────────────────────────────
install(TARGETS f1tenth_lidar_cpp getID
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS scripts/set_urg_ip.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch meshes urdf config
  DESTINATION share/${PROJECT_NAME}
)

# ── Python (DBSCAN 클러스터링 노드) ────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  f1tenth_lidar/clustering_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_clustering
)

# ── 익스포트 ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(urg_node_msgs diagnostic_updater laser_proc rclcpp sensor_msgs std_srvs urg_c)
ament_export_include_directories(include)
ament_export_libraries(f1tenth_lidar_cpp)
ament_package()
