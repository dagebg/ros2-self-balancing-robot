cmake_minimum_required(VERSION 3.16)
project(balance_core VERSION 1.0.0 LANGUAGES CXX)

# Built three ways from the same sources:
#   - standalone CMake via shared/CMakeLists.txt (host unit tests, coverage, sanitizers)
#   - colcon/ament as a ROS 2 package (used by robot_bridge and the simulation)
#   - PlatformIO via library.json (ESP32 firmware)

add_library(balance_core
  src/attitude.cpp
  src/filters.cpp
  src/imu_fusion.cpp
)
add_library(balance_core::balance_core ALIAS balance_core)
target_compile_features(balance_core PUBLIC cxx_std_17)
target_include_directories(balance_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(balance_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(balance_core PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

set(BALANCE_CORE_TESTS
  test/test_attitude.cpp
  test/test_filters.cpp
  test/test_imu_fusion.cpp
)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS balance_core EXPORT export_balance_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_targets(export_balance_core HAS_LIBRARY_TARGET)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_balance_core ${BALANCE_CORE_TESTS})
    target_link_libraries(test_balance_core balance_core)
  endif()

  ament_package()
elseif(BUILD_TESTING AND TARGET GTest::gtest_main)
  add_executable(test_balance_core ${BALANCE_CORE_TESTS})
  target_link_libraries(test_balance_core PRIVATE balance_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(test_balance_core)
endif()
