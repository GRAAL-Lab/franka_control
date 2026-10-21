cmake_minimum_required(VERSION 3.16)
project(fer_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fer_model REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StateData.msg" "msg/ControlData.msg" "msg/GripperState.msg"
  "msg/TaskStatus.msg" "srv/ControlCommand.srv" "srv/DriverCommand.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime fer_model)
ament_package()
