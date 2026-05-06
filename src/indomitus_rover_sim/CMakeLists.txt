cmake_minimum_required(VERSION 3.8)
project(indomitus_rover_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


# find_package(joint_state_publisher REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(indomitus_msgs REQUIRED)

include_directories(include)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()


install(
    DIRECTORY launch worlds parameters urdf
    DESTINATION share/${PROJECT_NAME}
)

add_executable(icr_controller_node src/icr_controller_node.cpp)
ament_target_dependencies(icr_controller_node rclcpp geometry_msgs std_msgs indomitus_msgs)
install(TARGETS icr_controller_node DESTINATION lib/${PROJECT_NAME})

add_executable(rocker_soft_mimic src/rocker_soft_mimic.cpp)
ament_target_dependencies(rocker_soft_mimic rclcpp sensor_msgs std_msgs)
install(TARGETS rocker_soft_mimic DESTINATION lib/${PROJECT_NAME})

ament_package()
