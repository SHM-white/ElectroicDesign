cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED PROJECT_ROOT)
    get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

set(FIRMWARE_ROOT "${PROJECT_ROOT}/firmware")
set(FIRMWARE_FILES
    "${FIRMWARE_ROOT}/.gitignore"
    "${FIRMWARE_ROOT}/CMakeLists.txt"
    "${FIRMWARE_ROOT}/sdkconfig.defaults"
    "${FIRMWARE_ROOT}/main/CMakeLists.txt"
    "${FIRMWARE_ROOT}/main/idf_component.yml"
    "${FIRMWARE_ROOT}/main/include/board_support.hpp"
    "${FIRMWARE_ROOT}/main/board_support.cpp"
    "${PROJECT_ROOT}/include/ground_station/ui_layout.hpp"
    "${FIRMWARE_ROOT}/main/include/dashboard_lvgl.hpp"
    "${FIRMWARE_ROOT}/main/dashboard_lvgl.cpp"
    "${FIRMWARE_ROOT}/main/main.cpp"
)

foreach(required_file IN LISTS FIRMWARE_FILES)
    if(NOT EXISTS "${required_file}")
        message(FATAL_ERROR "missing required firmware file: ${required_file}")
    endif()
endforeach()

function(require_contains variable_name needle description)
    string(FIND "${${variable_name}}" "${needle}" match_index)
    if(match_index EQUAL -1)
        message(FATAL_ERROR "${description}: expected '${needle}'")
    endif()
endfunction()

function(reject_contains variable_name needle description)
    string(FIND "${${variable_name}}" "${needle}" match_index)
    if(NOT match_index EQUAL -1)
        message(FATAL_ERROR "${description}: forbidden '${needle}'")
    endif()
endfunction()

file(READ "${FIRMWARE_ROOT}/CMakeLists.txt" firmware_cmake)
require_contains(firmware_cmake "include(\$ENV{IDF_PATH}/tools/cmake/project.cmake)" "firmware must use ESP-IDF project CMake")
require_contains(firmware_cmake "project(ed_ground_station_waveshare)" "firmware project name must be exact")

file(READ "${FIRMWARE_ROOT}/main/CMakeLists.txt" component_cmake)
require_contains(component_cmake "board_support.cpp" "main component must compile board support")
require_contains(component_cmake "main.cpp" "main component must compile app entrypoint")
require_contains(component_cmake "dashboard_lvgl.cpp" "main component must compile LVGL dashboard")
require_contains(component_cmake "../../src/navigation.cpp" "main component must compile portable navigation")
require_contains(component_cmake "../../src/telemetry.cpp" "main component must compile portable telemetry")
require_contains(component_cmake "../../src/v7.cpp" "main component must compile portable V7")
require_contains(component_cmake "../../src/view_model.cpp" "main component must compile portable view model")
require_contains(component_cmake "INCLUDE_DIRS \"include\" \"../../include\"" "main component must expose local and shared headers")
require_contains(component_cmake "REQUIRES driver esp_timer esp_lcd esp_lcd_touch_gt911 esp_lvgl_port lvgl" "main component must declare required ESP-IDF components")
require_contains(component_cmake "cxx_std_17" "main component must use C++17")
require_contains(component_cmake "-Wall -Wextra -Werror -pedantic" "main component must use strict warnings")
reject_contains(component_cmake "../managed_components" "main component may not vendor managed sources")

file(READ "${PROJECT_ROOT}/include/ground_station/ui_layout.hpp" layout_header)
foreach(layout_fact IN ITEMS
        "Rect root{0, 0, canvas_width, canvas_height}"
        "Rect masthead{0, 0, canvas_width, 64}"
        "Rect content{0, 64, canvas_width, 352}"
        "Rect footer{0, 416, canvas_width, 64}"
        "Rect overview_detail_button{680, 424, 96, 48}"
        "Rect detail_back_button{24, 424, 96, 48}"
        "minimum_touch_target_px = 48")
    require_contains(layout_header "${layout_fact}" "layout geometry contract")
endforeach()

file(READ "${FIRMWARE_ROOT}/main/include/dashboard_lvgl.hpp" dashboard_header)
file(READ "${FIRMWARE_ROOT}/main/dashboard_lvgl.cpp" dashboard_source)
file(READ "${FIRMWARE_ROOT}/main/main.cpp" firmware_main)
foreach(dashboard_fact IN ITEMS
        "class DashboardLvgl"
        "void initialize()"
        "void apply(const DashboardViewModel&")
    require_contains(dashboard_header "${dashboard_fact}" "dashboard LVGL API contract")
endforeach()
foreach(dashboard_fact IN ITEMS
        "LV_EVENT_RELEASED"
        "navigation_target"
        "NavigationIntent"
        "LV_STATE_PRESSED"
        "LV_STATE_FOCUSED"
        "LV_STATE_DISABLED"
        "gs_color_text_muted"
        "gs_color_success"
        "gs_color_warning"
        "gs_color_error"
        "gs_color_info"
        "lv_label_set_text(label, field.text.c_str())")
    require_contains(dashboard_source "${dashboard_fact}" "dashboard LVGL contract")
endforeach()
foreach(lvgl8_constructor IN ITEMS
        "lv_obj_create(nullptr)"
        "lv_label_create(parent)"
        "lv_label_create(button)"
        "lv_btn_create(overview_screen_)"
        "lv_btn_create(detail_screen_)")
    require_contains(dashboard_source "${lvgl8_constructor}" "dashboard must use LVGL8 one-argument constructors")
endforeach()
foreach(legacy_constructor IN ITEMS
        "lv_obj_create(nullptr, nullptr)"
        "lv_label_create(parent, nullptr)"
        "lv_label_create(button, nullptr)"
        "lv_btn_create(overview_screen_, nullptr)"
        "lv_btn_create(detail_screen_, nullptr)")
    reject_contains(dashboard_source "${legacy_constructor}" "dashboard must reject legacy two-argument constructors")
endforeach()
string(REGEX MATCHALL "lv_obj_create\\(nullptr\\)" screen_creation_paths "${dashboard_source}")
list(LENGTH screen_creation_paths screen_creation_count)
if(NOT screen_creation_count EQUAL 2)
    message(FATAL_ERROR "dashboard must have exactly two LVGL8 screen creation paths")
endif()
foreach(main_fact IN ITEMS
        "initialize_board"
        "initialize_telemetry_uart"
        "TelemetryCache cache"
        "V7StreamDecoder decoder"
        "lvgl_port_lock"
        "uart_read_bytes"
        "uart_buffer"
        "pdMS_TO_TICKS(20)"
        "pdMS_TO_TICKS(100)"
        "make_dashboard_view_model")
    require_contains(firmware_main "${main_fact}" "firmware app loop contract")
endforeach()
require_contains(firmware_main "lvgl_port_lock(0)" "firmware must use an indefinite LVGL lock wait")
reject_contains(firmware_main "lvgl_port_lock(portMAX_DELAY)" "firmware must not pass FreeRTOS ticks to LVGL lock")
foreach(forbidden_keyword IN ITEMS "arm" "takeoff" "land" "RTL" "radio" "flight_command")
    reject_contains(dashboard_source "${forbidden_keyword}" "dashboard must remain display-only")
    reject_contains(firmware_main "${forbidden_keyword}" "firmware must remain display-only")
endforeach()

file(READ "${FIRMWARE_ROOT}/main/idf_component.yml" manifest)
require_contains(manifest "version: \">=5.2\"" "ESP-IDF version constraint must be exact")
require_contains(manifest "espressif/esp_lvgl_port: \"2.8.0~1\"" "esp_lvgl_port pin must be exact")
require_contains(manifest "lvgl/lvgl:" "LVGL must be declared explicitly")
require_contains(manifest "version: \"8.4.0\"" "LVGL pin must be exact")
require_contains(manifest "public: true" "LVGL dependency must be public")
require_contains(manifest "espressif/esp_lcd_touch_gt911: \"1.1.1~1\"" "GT911 pin must be exact")

file(READ "${FIRMWARE_ROOT}/sdkconfig.defaults" sdkconfig)
foreach(required_setting IN ITEMS
        "CONFIG_IDF_TARGET=\"esp32s3\""
        "CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y"
        "CONFIG_ESPTOOLPY_FLASHMODE_QIO=y"
        "CONFIG_ESPTOOLPY_FLASHFREQ_80M=y"
        "CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y"
        "CONFIG_SPIRAM=y"
        "CONFIG_SPIRAM_MODE_OCT=y"
        "CONFIG_SPIRAM_SPEED_80M=y"
        "CONFIG_SPIRAM_SIZE=8388608"
        "CONFIG_SPIRAM_FETCH_INSTRUCTIONS=y"
        "CONFIG_SPIRAM_RODATA=y"
        "CONFIG_ESP32S3_DATA_CACHE_LINE_64B=y"
        "CONFIG_FREERTOS_HZ=1000"
        "# CONFIG_COMPILER_CXX_EXCEPTIONS is not set"
        "# CONFIG_COMPILER_CXX_RTTI is not set"
        "CONFIG_LV_COLOR_DEPTH_16=y"
        "CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y")
    require_contains(sdkconfig "${required_setting}" "sdkconfig.defaults hardware/runtime contract")
endforeach()
foreach(font_size IN ITEMS 12 14 16 20 24 32 40 48)
    require_contains(sdkconfig "CONFIG_LV_FONT_MONTSERRAT_${font_size}=y" "required LVGL built-in font")
endforeach()
reject_contains(sdkconfig "CONFIG_LV_USE_DEMO" "LVGL demos must remain disabled")
reject_contains(sdkconfig "CONFIG_ESP_CONSOLE_UART_DEFAULT=y" "console must not consume UART0")

file(READ "${PROJECT_ROOT}/include/ground_station/board_config.hpp" board_config)
foreach(shared_fact IN ITEMS
        "display_width_px = 800"
        "display_height_px = 480"
        "i2c_port = 0"
        "i2c_sda_pin = 8"
        "i2c_scl_pin = 9"
        "rgb_vsync_pin = 3"
        "rgb_hsync_pin = 46"
        "rgb_data_enable_pin = 5"
        "rgb_pixel_clock_pin = 7"
        "14, 38, 18, 17, 10, 39, 0, 45, 48, 47, 21, 1, 2, 42, 41, 40"
        "rgb_pixel_clock_hz = 16000000"
        "rgb_pixel_clock_active_low = true"
        "rgb_hsync_pulse_width = 4"
        "rgb_hsync_back_porch = 8"
        "rgb_hsync_front_porch = 8"
        "rgb_vsync_pulse_width = 4"
        "rgb_vsync_back_porch = 8"
        "rgb_vsync_front_porch = 8"
        "telemetry_uart_port = 0"
        "telemetry_uart_tx_pin = 43"
        "telemetry_uart_rx_pin = 44"
        "telemetry_uart_baud = 500000")
    require_contains(board_config "${shared_fact}" "shared board constants")
endforeach()

file(READ "${FIRMWARE_ROOT}/main/include/board_support.hpp" board_header)
require_contains(board_header "esp_err_t initialize_board" "board API must return esp_err_t")
require_contains(board_header "esp_err_t initialize_display" "display API must return esp_err_t")
require_contains(board_header "esp_err_t initialize_touch" "touch API must return esp_err_t")
require_contains(board_header "esp_err_t initialize_telemetry_uart" "telemetry API must return esp_err_t")
require_contains(board_header "esp_lcd_panel_handle_t" "board API must expose typed LCD handles")
require_contains(board_header "esp_lcd_touch_handle_t" "board API must expose typed touch handles")
require_contains(board_header "lv_disp_t" "board API must expose typed LVGL display handles")
require_contains(board_header "lv_indev_t" "board API must expose typed LVGL input handles")

file(READ "${FIRMWARE_ROOT}/main/board_support.cpp" board_source)
foreach(shared_use IN ITEMS
        "board_config::display_width_px"
        "board_config::display_height_px"
        "board_config::i2c_port"
        "board_config::i2c_sda_pin"
        "board_config::i2c_scl_pin"
        "board_config::rgb_data_pins"
        "board_config::rgb_pixel_clock_hz"
        "board_config::rgb_pixel_clock_active_low"
        "board_config::rgb_vsync_pin"
        "board_config::rgb_hsync_pin"
        "board_config::rgb_data_enable_pin"
        "board_config::rgb_pixel_clock_pin"
        "board_config::rgb_hsync_pulse_width"
        "board_config::rgb_hsync_back_porch"
        "board_config::rgb_hsync_front_porch"
        "board_config::rgb_vsync_pulse_width"
        "board_config::rgb_vsync_back_porch"
        "board_config::rgb_vsync_front_porch"
        "board_config::telemetry_uart_port"
        "board_config::telemetry_uart_tx_pin"
        "board_config::telemetry_uart_rx_pin"
        "board_config::telemetry_uart_baud")
    require_contains(board_source "${shared_use}" "firmware must consume shared board constants")
endforeach()
foreach(required_board_fact IN ITEMS
        "CH422G_MODE_ADDRESS = 0x24"
        "CH422G_OUTPUT_ADDRESS = 0x38"
        "CH422G_OUTPUT_ENABLE = 0x01"
        "CH422G_USB_MODE_NATIVE"
        "static_assert(CH422G_RESET_ASSERTED_SHADOW == 0x0C"
        "static_assert(CH422G_FINAL_OUTPUT_SHADOW == 0x1E"
        "I2C_CLK_SRC_DEFAULT"
        "400000"
        "esp_lcd_new_rgb_panel"
        "esp_lcd_new_panel_io_i2c"
        "esp_lcd_touch_new_i2c_gt911"
        "lvgl_port_add_disp_rgb"
        "lvgl_port_add_touch"
        "panel_config.num_fbs = 2"
        "panel_config.flags.fb_in_psram = true"
        "display_config.buffer_size = static_cast<std::size_t>(board_config::display_width_px) *"
        "display_config.double_buffer = true"
        "display_config.flags.buff_spiram = true"
        "display_config.flags.direct_mode = true"
        "rgb_config.flags.avoid_tearing = true"
        "touch_config.rst_gpio_num = GPIO_NUM_NC"
        "touch_config.int_gpio_num = GPIO_NUM_4"
        "UART_DATA_8_BITS"
        "UART_PARITY_DISABLE"
        "UART_STOP_BITS_1"
        "UART_HW_FLOWCTRL_DISABLE"
        "2048")
    require_contains(board_source "${required_board_fact}" "board support hardware contract")
endforeach()
require_contains(board_source "#include \"esp_idf_version.h\"" "board support must use ESP-IDF version macros")
require_contains(board_source
    "#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)\n    panel_config.sram_trans_align = 64;\n    panel_config.psram_trans_align = 64;\n#else\n    panel_config.dma_burst_size = 64;\n#endif"
    "RGB DMA configuration must support the complete IDF >=5.2 range")
require_contains(board_source
    "#if LVGL_VERSION_MAJOR >= 9\n    display_config.flags.swap_bytes = false;\n#endif"
    "swap_bytes must be guarded because LVGL 8.4 does not expose it")
reject_contains(board_source "LV_DISPLAY_RENDER_MODE_DIRECT" "LVGL9-only render mode must not be used with LVGL 8.4")

file(READ "${FIRMWARE_ROOT}/.gitignore" firmware_gitignore)
foreach(ignore_entry IN ITEMS "build/" "managed_components/" "dependencies.lock" "sdkconfig" ".vscode/" ".idea/")
    require_contains(firmware_gitignore "${ignore_entry}" "firmware generated artifact ignore list")
endforeach()
reject_contains(firmware_gitignore "sdkconfig.defaults" "sdkconfig.defaults must remain visible")

message(STATUS "Firmware static hardware/configuration checks passed")
