cmake_minimum_required(VERSION 3.22)
project(arm_hardware_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(can_msgs REQUIRED)

# Create the shared library for the plugin
add_library(arm_hardware_interface SHARED
  src/arm_hardware_interface.cpp
)

# Set include directories
target_include_directories(arm_hardware_interface PRIVATE include)

# Link dependencies
ament_target_dependencies(arm_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  can_msgs
)

# Export the plugin description file so ros2_control can find it
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

# Install targets
install(
  TARGETS arm_hardware_interface
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()