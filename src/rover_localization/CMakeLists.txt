cmake_minimum_required(VERSION 3.5)
project(rover_localization)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Install launch and config directories
install(DIRECTORY launch config resource
    DESTINATION share/${PROJECT_NAME}
)

ament_package()