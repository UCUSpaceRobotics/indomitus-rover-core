cmake_minimum_required(VERSION 3.5)
project(rover_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_tilt_math test/test_tilt_math.cpp)
    target_include_directories(test_tilt_math PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_tilt_math tf2)
endif()

add_executable(tilt_broadcaster_node src/tilt_broadcaster_node.cpp)
target_include_directories(tilt_broadcaster_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(tilt_broadcaster_node rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS tilt_broadcaster_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch config resource
    DESTINATION share/${PROJECT_NAME}
)

ament_package()