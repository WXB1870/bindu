cmake_minimum_required(VERSION 3.16)
project(bindu_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCommand.msg"
  "msg/ExecutionState.msg"
  "msg/SimulationCommand.msg"
  "msg/SimulationFeedback.msg"
  "msg/RuntimeEvent.msg"
  "msg/RecorderHealth.msg"
  "msg/ObjectObservation.msg"
  "srv/Lease.srv"
  "srv/SubmitMotion.srv"
  "srv/ControlExecution.srv"
  "srv/LocateObject.srv"
  "srv/InjectFault.srv"
  "msg/PiObservation.msg"
  "action/PiSession.action"
  "msg/NavigationVelocity.msg"
  "msg/NavigationPose.msg"
  "msg/VRInput.msg"
  "action/TeleopSession.action"
  "action/FetchDrink.action"
  "action/NavigateToSite.action"
  DEPENDENCIES builtin_interfaces geometry_msgs trajectory_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
