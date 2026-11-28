cmake_minimum_required(VERSION 3.16)
project(relocalizer LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP QUIET)

add_library(relocalizer_core STATIC
  src/aligner.cpp
  src/surface_atlas.cpp
  src/engine.cpp
  src/finder.cpp
  src/plan_atlas.cpp
  src/inspector.cpp
  src/sweep_prep.cpp
)
target_include_directories(relocalizer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(relocalizer_core PUBLIC Eigen3::Eigen yaml-cpp Threads::Threads)
if(OpenMP_CXX_FOUND)
  target_link_libraries(relocalizer_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(relocalizer_node src/relocalizer_node.cpp)
target_link_libraries(relocalizer_node PUBLIC relocalizer_core)
ament_target_dependencies(relocalizer_node PUBLIC
  rclcpp std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros)

add_executable(relocalizer_probe src/relocalizer_probe.cpp)
target_link_libraries(relocalizer_probe PUBLIC relocalizer_core)

add_executable(livox_body_cloud_relay src/livox_body_cloud_relay.cpp)
ament_target_dependencies(livox_body_cloud_relay
  rclcpp rcpputils sensor_msgs tf2 tf2_ros tf2_geometry_msgs livox_ros_driver2)

install(TARGETS relocalizer_node relocalizer_probe livox_body_cloud_relay
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_body_cloud.cpp)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_body_cloud test/test_body_cloud.cpp)
  ament_target_dependencies(test_body_cloud rcpputils sensor_msgs tf2)
  ament_add_gtest(test_odom_correction test/test_odom_correction.cpp)
  target_include_directories(test_odom_correction PRIVATE include)
  ament_target_dependencies(test_odom_correction rclcpp geometry_msgs)
  ament_add_gtest(test_relocalizer
    test/test_relocalizer.cpp)
  target_link_libraries(test_relocalizer relocalizer_core)
endif()

ament_package()
