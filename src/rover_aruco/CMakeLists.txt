cmake_minimum_required(VERSION 3.5)
project(rover_aruco)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Install launch/config/resource as subdirectories (no trailing slash) --
# aruco.launch.py and aruco_debug.launch.py both look up their yaml params
# at get_package_share_directory("rover_aruco") + "config/<file>.yaml";
# trailing slashes here flatten each directory's contents into share/
# root instead, which builds fine but leaves those os.path.join() lookups
# pointing at a config/ subdirectory that was never actually installed.
install(DIRECTORY launch config resource
    DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

install(PROGRAMS launch/calibrate_camera.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
