cmake_minimum_required(VERSION 3.20)

project(esp32s3_cam_i2c_reference LANGUAGES CXX)

include(CTest)

set(ESP32S3_CAM_STRICT_WARNINGS
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Wall>
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Wextra>
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Werror>
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-pedantic>
)

add_library(esp32s3_cam_i2c_core
  src/i2c_service.cpp
)

target_compile_features(esp32s3_cam_i2c_core PUBLIC cxx_std_17)
set_target_properties(esp32s3_cam_i2c_core PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(esp32s3_cam_i2c_core
  PRIVATE ${ESP32S3_CAM_STRICT_WARNINGS}
)
target_include_directories(esp32s3_cam_i2c_core PUBLIC include)

add_executable(esp32s3_cam_i2c_tests
  tests/i2c_service_test.cpp
)

target_compile_features(esp32s3_cam_i2c_tests PRIVATE cxx_std_17)
set_target_properties(esp32s3_cam_i2c_tests PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(esp32s3_cam_i2c_tests
  PRIVATE ${ESP32S3_CAM_STRICT_WARNINGS}
)
target_link_libraries(esp32s3_cam_i2c_tests PRIVATE esp32s3_cam_i2c_core)

add_test(NAME esp32s3_cam_i2c_tests COMMAND esp32s3_cam_i2c_tests)

add_executable(esp32s3_cam_arduino_adapter_tests
  tests/arduino_i2c_adapter_test.cpp
  tests/stubs/Wire.cpp
  src/arduino_i2c_adapter.cpp
)

target_compile_features(esp32s3_cam_arduino_adapter_tests PRIVATE cxx_std_17)
set_target_properties(esp32s3_cam_arduino_adapter_tests PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(esp32s3_cam_arduino_adapter_tests
  PRIVATE ${ESP32S3_CAM_STRICT_WARNINGS}
)
target_include_directories(esp32s3_cam_arduino_adapter_tests PRIVATE
  tests/stubs
)
target_link_libraries(esp32s3_cam_arduino_adapter_tests
  PRIVATE esp32s3_cam_i2c_core
)

add_test(
  NAME esp32s3_cam_arduino_adapter_tests
  COMMAND esp32s3_cam_arduino_adapter_tests
)
