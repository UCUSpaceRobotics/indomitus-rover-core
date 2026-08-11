cmake_minimum_required(VERSION 3.8)
project(panel_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# copy urdf and meshes to share folder, so they can be found by rviz/gazebo
install(
  DIRECTORY
    urdf
    meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
