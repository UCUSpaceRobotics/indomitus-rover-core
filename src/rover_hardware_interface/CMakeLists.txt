cmake_minimum_required(VERSION 3.8)
project(rover_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(indomitus_interfaces REQUIRED)

set(NODE_NAME "chassis_driver_node_exe")

add_executable(${NODE_NAME}
  src/chassis_driver_node.cpp
)

target_include_directories(${NODE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${NODE_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${NODE_NAME}
  rclcpp
  can_msgs
  sensor_msgs
  diagnostic_msgs
  std_srvs
  indomitus_interfaces
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  can_msgs
  sensor_msgs
  diagnostic_msgs
  std_srvs
  indomitus_interfaces
)

install(TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_protocols
    test/test_protocols.cpp
  )
  target_include_directories(test_protocols PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_protocols can_msgs)
endif()

ament_package()