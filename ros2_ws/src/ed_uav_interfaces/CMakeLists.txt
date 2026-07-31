cmake_minimum_required(VERSION 3.8)
project(ed_uav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FcuState.msg"
  "msg/LocalizationStatus.msg"
  "msg/BoundaryObservation.msg"
  "msg/VehicleTelemetry.msg"
  "msg/TargetObservation.msg"
  "msg/MissionStatus.msg"
  "msg/PayloadContactState.msg"
  "msg/H7GpioCommand.msg"
  "action/FlightCommand.action"
  "action/ExecuteMission.action"
  "srv/StartMapSession.srv"
  "srv/SelectDTaskMission.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

install(
  DIRECTORY contracts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.local.yaml" EXCLUDE
)
install(PROGRAMS
  tools/check_contract.py
  tools/check_d_task_config.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES tools/d_task_models.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(contract_checker test/test_contract_checker.py)
  ament_add_pytest_test(d_task_contracts test/test_d_task_contracts.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
