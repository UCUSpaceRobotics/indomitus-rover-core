cmake_minimum_required(VERSION 3.5)
project(rover_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch config maps
    DESTINATION share/${PROJECT_NAME}
)

# Install package resource marker for ament index visibility
install(DIRECTORY resource
    DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()
