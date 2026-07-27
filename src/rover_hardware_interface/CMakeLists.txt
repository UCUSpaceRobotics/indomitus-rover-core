cmake_minimum_required(VERSION 3.8)
project(rover_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(indomitus_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rover_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if($ENV{ROS_DISTRO} STREQUAL "jazzy")
    target_compile_definitions(${PROJECT_NAME} PRIVATE JAZZY_OR_LATER)
endif()

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  can_msgs
  sensor_msgs
  diagnostic_msgs
  std_srvs
  indomitus_interfaces
)

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  can_msgs
  sensor_msgs
  diagnostic_msgs
  std_srvs
  indomitus_interfaces
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)

#   ament_add_gtest(test_protocols
#     test/test_protocols.cpp
#   )
#   target_include_directories(test_protocols PRIVATE
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   )
#   ament_target_dependencies(test_protocols can_msgs)
# endif()

ament_package()
