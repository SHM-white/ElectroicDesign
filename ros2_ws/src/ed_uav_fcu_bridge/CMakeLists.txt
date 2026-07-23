cmake_minimum_required(VERSION 3.8)
project(ed_uav_fcu_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/ed_uav_fcu_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(v7_codec test/test_v7_codec.py)
  ament_add_pytest_test(telemetry_cache test/test_telemetry_cache.py)
  ament_add_pytest_test(actions test/test_actions.py)
  ament_add_pytest_test(pty_surface test/test_pty_surface.py)
endif()

ament_package()
