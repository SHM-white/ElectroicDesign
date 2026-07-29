cmake_minimum_required(VERSION 3.20)
project(ed_ground_station_arduino_core LANGUAGES CXX)

add_library(ed_hmi_core STATIC src/hmi_state.cpp)
target_include_directories(ed_hmi_core PUBLIC include)
target_compile_features(ed_hmi_core PUBLIC cxx_std_17)

add_executable(ed_hmi_core_tests tests/hmi_state_test.cpp)
target_link_libraries(ed_hmi_core_tests PRIVATE ed_hmi_core)
add_test(NAME ed_hmi_core_tests COMMAND ed_hmi_core_tests)
