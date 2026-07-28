cmake_minimum_required(VERSION 3.20)

project(ed_embedded LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_subdirectory(ground_station_esp32s3)
add_subdirectory(esp32s3_cam)
