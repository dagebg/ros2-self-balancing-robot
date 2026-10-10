# Standalone host build of the shared, hardware-independent C++ libraries.
#
#   cmake -S shared -B build/shared -G Ninja
#   cmake --build build/shared && ctest --test-dir build/shared --output-on-failure
#
# Options: -DROBOT_COVERAGE=ON (gcov instrumentation), -DROBOT_SANITIZERS=ON (ASan + UBSan).
cmake_minimum_required(VERSION 3.16)
project(robot_shared LANGUAGES CXX)

option(ROBOT_COVERAGE "Instrument for code coverage (gcov)" OFF)
option(ROBOT_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(ROBOT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(ROBOT_COVERAGE)
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()
if(ROBOT_SANITIZERS)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
endif()
if(ROBOT_WARNINGS_AS_ERRORS)
  add_compile_options(-Werror)
endif()

include(CTest)
if(BUILD_TESTING)
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.18.0
    GIT_SHALLOW TRUE
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

add_subdirectory(balance_core)
add_subdirectory(robot_protocol)
