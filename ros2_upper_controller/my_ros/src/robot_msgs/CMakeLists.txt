cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/ArmStatus.msg
  msg/Detection.msg
  msg/DetectionArray.msg
  msg/JointTarget.msg
  msg/ObjectPose.msg
  msg/ObjectOnPlane.msg
  msg/PlaneHeight.msg
  msg/RobotState.msg
)

set(action_files
  action/CleanArea.action
  action/NavigateToObject.action
  action/PickObject.action
  action/PlaceObject.action
  action/CalibratePlaneHeight.action
  action/ReturnHome.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES
    action_msgs
    builtin_interfaces
    geometry_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
