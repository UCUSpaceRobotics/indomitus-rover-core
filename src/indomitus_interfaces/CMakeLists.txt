cmake_minimum_required(VERSION 3.8)
project(indomitus_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelTargets.msg"
  "msg/MotorStatus.msg"
  "msg/ChassisStatus.msg"
  "action/ContainerLid.action"
  "srv/GetWeight.srv"
  DEPENDENCIES std_msgs
)

ament_package()
