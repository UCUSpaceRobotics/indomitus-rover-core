cmake_minimum_required(VERSION 3.8)
project(arm_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# arm_bringup.launch_utils, shared with arm_sim's launch files.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
