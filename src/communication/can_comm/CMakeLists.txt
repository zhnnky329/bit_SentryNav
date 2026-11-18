cmake_minimum_required(VERSION 3.8)
project(can_comm)
find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/can_send.py
  DESTINATION lib/${PROJECT_NAME} RENAME can_send)
install(PROGRAMS scripts/can_receive.py
  DESTINATION lib/${PROJECT_NAME} RENAME can_receive)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(can_send_test test/test_can_send.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  ament_add_pytest_test(can_receive_test test/test_can_receive.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
endif()

ament_package()
