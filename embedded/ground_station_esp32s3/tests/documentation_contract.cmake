cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED PROJECT_ROOT)
    get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

file(READ "${PROJECT_ROOT}/README.md" readme)

function(require_readme needle description)
    string(FIND "${readme}" "${needle}" match_index)
    if(match_index EQUAL -1)
        message(FATAL_ERROR "${description}: expected '${needle}'")
    endif()
endfunction()

function(reject_readme needle description)
    string(FIND "${readme}" "${needle}" match_index)
    if(NOT match_index EQUAL -1)
        message(FATAL_ERROR "${description}: forbidden '${needle}'")
    endif()
endfunction()

foreach(required_text IN ITEMS
        "## Windows-native host validation"
        "winget install --id Espressif.EIM-CLI"
        "eim install -i v5.5.2"
        "Microsoft.v5.5.2.PowerShell_profile.ps1"
        "idf.py -C embedded/ground_station_esp32s3/firmware set-target esp32s3"
        "idf.py -C embedded/ground_station_esp32s3/firmware build"
        "ed_ground_station_waveshare.bin"
        "bootloader.bin"
        "partition-table.bin"
        "https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/"
        "https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Development-Environment-Setup-ESP-IDF"
        "https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Instructions-For-Use"
        "https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Firmware-Flashing"
        "https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Resources-And-Documents"
        "GPIO44"
        "GPIO43"
        "common GND"
        "3.3 V TTL"
        "500000 baud"
        "8N1"
        "UART2"
        "BOOT"
        "RESET"
        "Bluetooth"
        "0x08"
        "0x51"
        "0.20 s"
        "0.50 s"
        "Overview"
        "Detail"
        "display-only")
    require_readme("${required_text}" "ground-station documentation contract")
endforeach()

reject_readme("WSL" "ground-station development instructions must be Windows-native")

message(STATUS "Ground-station documentation contract passed")
