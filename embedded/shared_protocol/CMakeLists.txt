cmake_minimum_required(VERSION 3.20)
project(ed_shared_protocol LANGUAGES CXX)

add_library(ed_shared_protocol STATIC
    src/codec.cpp
    src/crypto.cpp
    src/payloads.cpp
    src/session.cpp
)

target_include_directories(ed_shared_protocol PUBLIC include)
target_compile_features(ed_shared_protocol PUBLIC cxx_std_17)

add_executable(ed_shared_protocol_tests
    tests/protocol_test.cpp
)
target_link_libraries(ed_shared_protocol_tests PRIVATE ed_shared_protocol)
add_test(NAME ed_shared_protocol_tests COMMAND ed_shared_protocol_tests)
