cmake_minimum_required(VERSION 3.8)
project(indomitus_launch_utils NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_graphics test/test_graphics.py)
endif()

ament_package()
