cmake_minimum_required(VERSION 3.16)
project(robot_protocol VERSION 1.0.0 LANGUAGES CXX)

# Built three ways from the same sources:
#   - standalone CMake via shared/CMakeLists.txt (host unit tests, coverage, sanitizers)
#   - colcon/ament as a ROS 2 package (used by robot_bridge)
#   - PlatformIO via library.json (ESP32 firmware)

add_library(robot_protocol
  src/cobs.cpp
  src/codec.cpp
  src/crc16.cpp
  src/frame_parser.cpp
  src/messages.cpp
)
add_library(robot_protocol::robot_protocol ALIAS robot_protocol)
target_compile_features(robot_protocol PUBLIC cxx_std_17)
target_include_directories(robot_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(robot_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(robot_protocol PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

set(ROBOT_PROTOCOL_TESTS
  test/test_byte_io.cpp
  test/test_cobs.cpp
  test/test_codec.cpp
  test/test_crc16.cpp
  test/test_frame_parser.cpp
)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS robot_protocol EXPORT export_robot_protocol
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_targets(export_robot_protocol HAS_LIBRARY_TARGET)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_robot_protocol ${ROBOT_PROTOCOL_TESTS})
    target_link_libraries(test_robot_protocol robot_protocol)
  endif()

  ament_package()
elseif(BUILD_TESTING AND TARGET GTest::gtest_main)
  add_executable(test_robot_protocol ${ROBOT_PROTOCOL_TESTS})
  target_link_libraries(test_robot_protocol PRIVATE robot_protocol GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(test_robot_protocol)
endif()
