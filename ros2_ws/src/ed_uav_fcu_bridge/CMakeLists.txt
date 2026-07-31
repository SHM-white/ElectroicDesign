cmake_minimum_required(VERSION 3.8)
project(ed_uav_fcu_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/ed_uav_fcu_bridge
  scripts/ed_uav_v7_characterize
  scripts/ed_uav_h7_gpio_bridge
  scripts/ed_uav_h7_gpio_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(v7_codec test/test_v7_codec.py)
  ament_add_pytest_test(telemetry_cache test/test_telemetry_cache.py)
  ament_add_pytest_test(actions test/test_actions.py)
  ament_add_pytest_test(programmable_commands test/test_programmable_commands.py)
  ament_add_pytest_test(capability_gate test/test_capability_gate.py)
  ament_add_pytest_test(characterize_cli test/test_characterize_cli.py)
  ament_add_pytest_test(verifier_blockers test/test_verifier_blockers.py)
  ament_add_pytest_test(pty_surface test/test_pty_surface.py)
  ament_add_pytest_test(flight_command_authority test/test_flight_command_authority.py)
  ament_add_pytest_test(flight_command_boundaries test/test_flight_command_boundaries.py)
  ament_add_pytest_test(serial_ownership_security test/test_serial_ownership_security.py)
  ament_add_pytest_test(h7_gpio_protocol test/test_h7_gpio_protocol.py)
  ament_add_pytest_test(h7_gpio_serial test/test_h7_gpio_serial.py)
endif()

ament_package()
