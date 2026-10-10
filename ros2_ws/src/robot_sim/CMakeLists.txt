cmake_minimum_required(VERSION 3.16)
project(robot_sim LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(balance_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_protocol REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The firmware's portable application code, compiled for the host: the virtual ESP32 runs the
# same ControlLoop and CommsService as the robot.
set(FIRMWARE_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../../../firmware/esp32/robot-firmware/lib)
add_library(firmware_app STATIC
  ${FIRMWARE_LIB}/app/src/control_loop.cpp
  ${FIRMWARE_LIB}/app/src/link.cpp
  ${FIRMWARE_LIB}/logging/src/logger.cpp
)
target_include_directories(firmware_app PUBLIC
  ${FIRMWARE_LIB}/app/include
  ${FIRMWARE_LIB}/drivers/include
  ${FIRMWARE_LIB}/hal/include
  ${FIRMWARE_LIB}/logging/include
)
target_link_libraries(firmware_app PUBLIC
  balance_core::balance_core
  robot_protocol::robot_protocol
)
set_target_properties(firmware_app PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(virtual_esp32 src/virtual_esp32.cpp)
target_link_libraries(virtual_esp32 firmware_app util)
ament_target_dependencies(virtual_esp32 rclcpp sensor_msgs std_msgs)

install(TARGETS virtual_esp32 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
