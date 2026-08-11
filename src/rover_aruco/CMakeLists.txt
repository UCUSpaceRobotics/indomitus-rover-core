cmake_minimum_required(VERSION 3.5)
project(rover_aruco)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS launch/calibrate_camera.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install package resource marker for ament index visibility
install(DIRECTORY resource
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
