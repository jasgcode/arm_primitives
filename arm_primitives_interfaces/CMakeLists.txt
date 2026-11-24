cmake_minimum_required(VERSION 3.10)
project(arm_primitives_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetArmState.srv"
  "srv/Gripper.srv"
  "srv/MoveRelative.srv"
  "srv/MoveToJoints.srv"
  "srv/MoveToNamed.srv"
  "srv/MoveToPose.srv"
  "srv/PickPlace.srv"
  "srv/SceneObject.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
