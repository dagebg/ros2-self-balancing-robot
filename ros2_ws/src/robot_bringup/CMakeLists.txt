cmake_minimum_required(VERSION 3.16)
project(robot_bringup NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_params test/test_robot_params.py)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_robot_launch.py TIMEOUT 120)
endif()

ament_package()
