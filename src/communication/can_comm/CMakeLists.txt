cmake_minimum_required(VERSION 3.8)
project(can_comm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sentry_msgs REQUIRED)

add_executable(can_send src/can_send.cpp)
target_include_directories(can_send PRIVATE include)
ament_target_dependencies(can_send rclcpp geometry_msgs std_msgs sentry_msgs)

add_executable(can_receive src/can_receive.cpp)
target_include_directories(can_receive PRIVATE include)
ament_target_dependencies(can_receive rclcpp std_msgs)

install(TARGETS can_send can_receive
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)
endif()

ament_package()
