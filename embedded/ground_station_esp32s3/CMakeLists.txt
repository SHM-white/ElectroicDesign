cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(ground_station_esp32s3_native LANGUAGES CXX)
    enable_testing()
endif()

add_library(ground_station_protocol
    src/navigation.cpp
    src/telemetry.cpp
    src/v7.cpp
    src/view_model.cpp
)

target_include_directories(ground_station_protocol PUBLIC include)
target_compile_features(ground_station_protocol PUBLIC cxx_std_17)
target_compile_options(ground_station_protocol PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)

add_executable(ground_station_native_tests tests/native_tests.cpp)
target_link_libraries(ground_station_native_tests PRIVATE ground_station_protocol)
target_compile_options(ground_station_native_tests PRIVATE -Wall -Wextra -Werror -pedantic)

add_test(NAME ground_station_native_tests COMMAND ground_station_native_tests)

add_test(
    NAME ground_station_firmware_static_checks
    COMMAND ${CMAKE_COMMAND}
        -DPROJECT_ROOT=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware_static_checks.cmake
)

add_test(
    NAME ground_station_documentation_contract
    COMMAND ${CMAKE_COMMAND}
        -DPROJECT_ROOT=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/documentation_contract.cmake
)
