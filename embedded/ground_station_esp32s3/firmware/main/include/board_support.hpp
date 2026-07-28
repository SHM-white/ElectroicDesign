#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

namespace ground_station::firmware {

struct BoardSupport {
    i2c_master_bus_handle_t i2c_bus = nullptr;
    i2c_master_dev_handle_t ch422g_mode = nullptr;
    i2c_master_dev_handle_t ch422g_output = nullptr;
    esp_lcd_panel_handle_t panel = nullptr;
    esp_lcd_panel_io_handle_t touch_io = nullptr;
    esp_lcd_touch_handle_t touch = nullptr;
    lv_disp_t* display = nullptr;
    lv_indev_t* touch_input = nullptr;
};

esp_err_t initialize_board(BoardSupport& board);
esp_err_t initialize_display(BoardSupport& board);
esp_err_t initialize_touch(BoardSupport& board);
esp_err_t initialize_telemetry_uart();

}  // namespace ground_station::firmware
