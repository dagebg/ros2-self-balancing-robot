cmake_minimum_required(VERSION 3.16)
project(robot_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf DESTINATION share/${PROJECT_NAME})

# The repository's robot parameters ship with the model: config/robot_params.yaml stays the single
# source of truth for geometry and masses (see docs: ADR-0004).
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/robot_params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
endif()

ament_package()
