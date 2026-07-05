cmake_minimum_required(VERSION 3.8)
project(rover_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(indomitus_interfaces REQUIRED)

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

add_executable(sim_diff_bar_node src/sim_diff_bar_node.cpp)
ament_target_dependencies(sim_diff_bar_node rclcpp sensor_msgs std_msgs)
install(TARGETS sim_diff_bar_node DESTINATION lib/${PROJECT_NAME})

ament_package()
