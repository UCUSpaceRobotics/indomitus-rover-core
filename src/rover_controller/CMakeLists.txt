cmake_minimum_required(VERSION 3.8)
project(rover_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)  
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/swerve_controller_test.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if("$ENV{ROS_DISTRO}" STREQUAL "jazzy" OR "$ENV{ROS_DISTRO}" STREQUAL "rolling")
    target_compile_definitions(rover_controller PRIVATE JAZZY_OR_LATER)
endif()

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME} SYSTEM PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
  geometry_msgs
  std_srvs
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
  geometry_msgs
  std_srvs
  realtime_tools
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
