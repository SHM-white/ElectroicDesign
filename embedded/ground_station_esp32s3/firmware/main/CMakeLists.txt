idf_component_register(
    SRCS "main.cpp" "board_support.cpp" "dashboard_lvgl.cpp"
         "../../src/navigation.cpp" "../../src/telemetry.cpp" "../../src/v7.cpp"
         "../../src/view_model.cpp"
    REQUIRES driver esp_timer esp_lcd esp_lcd_touch_gt911 esp_lvgl_port lvgl
    INCLUDE_DIRS "include" "../../include"
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror -pedantic)
