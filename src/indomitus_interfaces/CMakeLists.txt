cmake_minimum_required(VERSION 3.8)
project(indomitus_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/ChassisStatus.msg"
  "msg/FaultEvent.msg"
  "msg/EndEffectorState.msg"
  "action/ContainerLid.action"
  "srv/GetWeight.srv"
  "srv/SetSteerZero.srv"
  "srv/SetTrafficLight.srv"
  DEPENDENCIES std_msgs action_msgs
)

ament_package()
