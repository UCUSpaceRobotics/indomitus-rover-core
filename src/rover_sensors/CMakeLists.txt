cmake_minimum_required(VERSION 3.5)
project(rover_sensors)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md rplidar_s2.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
