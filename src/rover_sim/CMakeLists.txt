cmake_minimum_required(VERSION 3.8)
project(rover_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(indomitus_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)

include_directories(include)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

install(
    DIRECTORY launch worlds config urdf
    DESTINATION share/${PROJECT_NAME}
)

add_library(rover_sim_controller SHARED
  src/rocker_diff_controller.cpp
)

target_include_directories(rover_sim_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(rover_sim_controller PUBLIC cxx_std_17)

if($ENV{ROS_DISTRO} STREQUAL "jazzy")
    target_compile_definitions(rover_sim_controller PRIVATE JAZZY_OR_LATER)
endif()

ament_target_dependencies(rover_sim_controller
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
)

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(rover_sim_controller)
ament_export_dependencies(
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
)

install(TARGETS rover_sim_controller
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
