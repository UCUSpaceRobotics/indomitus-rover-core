cmake_minimum_required(VERSION 3.8)
project(damiao_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(indomitus_msgs REQUIRED)

add_executable(damiao_driver_node
  src/damiao_driver_node.cpp
)

target_include_directories(damiao_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(damiao_driver_node PUBLIC cxx_std_17)

ament_target_dependencies(damiao_driver_node
  rclcpp
  can_msgs
  sensor_msgs
  diagnostic_msgs
  indomitus_msgs
)

install(TARGETS damiao_driver_node
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
