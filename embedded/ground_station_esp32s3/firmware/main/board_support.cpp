#include "board_support.hpp"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_idf_version.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ground_station/board_config.hpp"

#include <cstddef>
#include <cstdint>

namespace ground_station::firmware {
namespace {

namespace board_config = ground_station::board_config;

constexpr std::uint16_t CH422G_MODE_ADDRESS = 0x24;
constexpr std::uint16_t CH422G_OUTPUT_ADDRESS = 0x38;
constexpr std::uint8_t CH422G_OUTPUT_ENABLE = 0x01;
constexpr std::uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr TickType_t GT911_RESET_ASSERT_DELAY = pdMS_TO_TICKS(100);
constexpr TickType_t GT911_RESET_RELEASE_DELAY = pdMS_TO_TICKS(100);
constexpr TickType_t GT911_STARTUP_DELAY = pdMS_TO_TICKS(200);
constexpr int TELEMETRY_RX_BUFFER_SIZE = 2048;

constexpr std::uint8_t CH422G_EXIO1_GT911_RESET = 1U << 1U;
constexpr std::uint8_t CH422G_EXIO2_BACKLIGHT = 1U << 2U;
constexpr std::uint8_t CH422G_EXIO3_LCD_RESET = 1U << 3U;
constexpr std::uint8_t CH422G_EXIO4_TF_DESELECT = 1U << 4U;
constexpr std::uint8_t CH422G_EXIO5_USB_CAN_SELECT = 1U << 5U;
constexpr std::uint8_t CH422G_USB_MODE_NATIVE = 0x00;

constexpr std::uint8_t CH422G_RESET_ASSERTED_SHADOW =
    CH422G_EXIO2_BACKLIGHT | CH422G_EXIO3_LCD_RESET | CH422G_USB_MODE_NATIVE;
constexpr std::uint8_t CH422G_RESET_RELEASED_SHADOW =
    CH422G_RESET_ASSERTED_SHADOW | CH422G_EXIO1_GT911_RESET;
constexpr std::uint8_t CH422G_FINAL_OUTPUT_SHADOW =
    CH422G_RESET_RELEASED_SHADOW | CH422G_EXIO4_TF_DESELECT;

static_assert(CH422G_RESET_ASSERTED_SHADOW == 0x0C,
              "GT911 reset asserted keeps EXIO1 low while board outputs remain valid");
static_assert(CH422G_RESET_RELEASED_SHADOW == 0x0E,
              "GT911 reset release raises only EXIO1");
static_assert(CH422G_FINAL_OUTPUT_SHADOW == 0x1E,
              "final shadow enables backlight, releases resets, and deselects TF");
static_assert((CH422G_FINAL_OUTPUT_SHADOW & CH422G_EXIO5_USB_CAN_SELECT) ==
                  CH422G_USB_MODE_NATIVE,
              "EXIO5 must stay low to select native USB instead of CAN");

esp_err_t add_i2c_device(i2c_master_bus_handle_t bus, const std::uint16_t address,
                         i2c_master_dev_handle_t* const device) {
    i2c_device_config_t config{};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = address;
    config.scl_speed_hz = I2C_FREQUENCY_HZ;
    return i2c_master_bus_add_device(bus, &config, device);
}

esp_err_t write_ch422g(const i2c_master_dev_handle_t device, const std::uint8_t value) {
    return i2c_master_transmit(device, &value, sizeof(value), -1);
}

esp_err_t initialize_i2c(BoardSupport& board) {
    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = static_cast<i2c_port_num_t>(board_config::i2c_port);
    bus_config.sda_io_num = static_cast<gpio_num_t>(board_config::i2c_sda_pin);
    bus_config.scl_io_num = static_cast<gpio_num_t>(board_config::i2c_scl_pin);
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t result = i2c_new_master_bus(&bus_config, &board.i2c_bus);
    if (result != ESP_OK) {
        return result;
    }
    result = add_i2c_device(board.i2c_bus, CH422G_MODE_ADDRESS, &board.ch422g_mode);
    if (result != ESP_OK) {
        return result;
    }
    return add_i2c_device(board.i2c_bus, CH422G_OUTPUT_ADDRESS, &board.ch422g_output);
}

esp_err_t reset_board_peripherals(BoardSupport& board) {
    gpio_config_t interrupt_pin{};
    interrupt_pin.pin_bit_mask = 1ULL << GPIO_NUM_4;
    interrupt_pin.mode = GPIO_MODE_OUTPUT;
    interrupt_pin.pull_up_en = GPIO_PULLUP_DISABLE;
    interrupt_pin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    interrupt_pin.intr_type = GPIO_INTR_DISABLE;

    esp_err_t result = gpio_config(&interrupt_pin);
    if (result != ESP_OK) {
        return result;
    }
    result = gpio_set_level(GPIO_NUM_4, 0);
    if (result != ESP_OK) {
        return result;
    }
    result = write_ch422g(board.ch422g_mode, CH422G_OUTPUT_ENABLE);
    if (result != ESP_OK) {
        return result;
    }
    result = write_ch422g(board.ch422g_output, CH422G_RESET_ASSERTED_SHADOW);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(GT911_RESET_ASSERT_DELAY);

    result = write_ch422g(board.ch422g_output, CH422G_RESET_RELEASED_SHADOW);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(GT911_RESET_RELEASE_DELAY);

    result = gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(GT911_STARTUP_DELAY);
    return write_ch422g(board.ch422g_output, CH422G_FINAL_OUTPUT_SHADOW);
}

}  // namespace

esp_err_t initialize_display(BoardSupport& board) {
    esp_lcd_rgb_panel_config_t panel_config{};
    panel_config.clk_src = LCD_CLK_SRC_DEFAULT;
    panel_config.timings.pclk_hz = board_config::rgb_pixel_clock_hz;
    panel_config.timings.h_res = board_config::display_width_px;
    panel_config.timings.v_res = board_config::display_height_px;
    panel_config.timings.hsync_pulse_width = board_config::rgb_hsync_pulse_width;
    panel_config.timings.hsync_back_porch = board_config::rgb_hsync_back_porch;
    panel_config.timings.hsync_front_porch = board_config::rgb_hsync_front_porch;
    panel_config.timings.vsync_pulse_width = board_config::rgb_vsync_pulse_width;
    panel_config.timings.vsync_back_porch = board_config::rgb_vsync_back_porch;
    panel_config.timings.vsync_front_porch = board_config::rgb_vsync_front_porch;
    panel_config.timings.flags.pclk_active_neg = board_config::rgb_pixel_clock_active_low;
    panel_config.data_width = 16;
    panel_config.bits_per_pixel = 16;
    panel_config.num_fbs = 2;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)
    panel_config.sram_trans_align = 64;
    panel_config.psram_trans_align = 64;
#else
    panel_config.dma_burst_size = 64;
#endif
    panel_config.hsync_gpio_num = board_config::rgb_hsync_pin;
    panel_config.vsync_gpio_num = board_config::rgb_vsync_pin;
    panel_config.de_gpio_num = board_config::rgb_data_enable_pin;
    panel_config.pclk_gpio_num = board_config::rgb_pixel_clock_pin;
    panel_config.disp_gpio_num = GPIO_NUM_NC;
    for (std::size_t index = 0; index < board_config::rgb_data_pins.size(); ++index) {
        panel_config.data_gpio_nums[index] = board_config::rgb_data_pins[index];
    }
    panel_config.flags.fb_in_psram = true;

    esp_err_t result = esp_lcd_new_rgb_panel(&panel_config, &board.panel);
    if (result != ESP_OK) {
        return result;
    }
    result = esp_lcd_panel_reset(board.panel);
    if (result != ESP_OK) {
        return result;
    }
    result = esp_lcd_panel_init(board.panel);
    if (result != ESP_OK) {
        return result;
    }

    lvgl_port_display_cfg_t display_config{};
    display_config.panel_handle = board.panel;
    display_config.buffer_size = static_cast<std::size_t>(board_config::display_width_px) *
                                 board_config::display_height_px;
    display_config.double_buffer = true;
    display_config.hres = board_config::display_width_px;
    display_config.vres = board_config::display_height_px;
    display_config.monochrome = false;
    display_config.flags.buff_dma = false;
    display_config.flags.buff_spiram = true;
    display_config.flags.sw_rotate = false;
#if LVGL_VERSION_MAJOR >= 9
    display_config.flags.swap_bytes = false;
#endif
    display_config.flags.full_refresh = false;
    display_config.flags.direct_mode = true;

    lvgl_port_display_rgb_cfg_t rgb_config{};
    rgb_config.flags.bb_mode = false;
    rgb_config.flags.avoid_tearing = true;
    board.display = lvgl_port_add_disp_rgb(&display_config, &rgb_config);
    return board.display == nullptr ? ESP_FAIL : ESP_OK;
}

esp_err_t initialize_touch(BoardSupport& board) {
    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    esp_err_t result = esp_lcd_new_panel_io_i2c(board.i2c_bus, &io_config, &board.touch_io);
    if (result != ESP_OK) {
        return result;
    }

    esp_lcd_touch_config_t touch_config{};
    touch_config.x_max = board_config::display_width_px;
    touch_config.y_max = board_config::display_height_px;
    touch_config.rst_gpio_num = GPIO_NUM_NC;
    touch_config.int_gpio_num = GPIO_NUM_4;
    touch_config.levels.reset = 0;
    touch_config.levels.interrupt = 0;
    touch_config.flags.swap_xy = false;
    touch_config.flags.mirror_x = false;
    touch_config.flags.mirror_y = false;
    result = esp_lcd_touch_new_i2c_gt911(board.touch_io, &touch_config, &board.touch);
    if (result != ESP_OK) {
        return result;
    }

    lvgl_port_touch_cfg_t port_touch_config{};
    port_touch_config.disp = board.display;
    port_touch_config.handle = board.touch;
    board.touch_input = lvgl_port_add_touch(&port_touch_config);
    return board.touch_input == nullptr ? ESP_FAIL : ESP_OK;
}

esp_err_t initialize_board(BoardSupport& board) {
    if (board.i2c_bus != nullptr || board.panel != nullptr || board.touch != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = initialize_i2c(board);
    if (result != ESP_OK) {
        return result;
    }
    result = reset_board_peripherals(board);
    if (result != ESP_OK) {
        return result;
    }

    const lvgl_port_cfg_t lvgl_config = ESP_LVGL_PORT_INIT_CONFIG();
    result = lvgl_port_init(&lvgl_config);
    if (result != ESP_OK) {
        return result;
    }
    result = initialize_display(board);
    if (result != ESP_OK) {
        return result;
    }
    return initialize_touch(board);
}

esp_err_t initialize_telemetry_uart() {
    static_assert(board_config::telemetry_uart_data_bits == 8);
    static_assert(board_config::telemetry_uart_stop_bits == 1);
    static_assert(board_config::telemetry_uart_parity == board_config::UartParity::none);
    static_assert(board_config::telemetry_uart_flow_control == board_config::UartFlowControl::none);

    const auto port = static_cast<uart_port_t>(board_config::telemetry_uart_port);
    uart_config_t uart_config{};
    uart_config.baud_rate = board_config::telemetry_uart_baud;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t result = uart_param_config(port, &uart_config);
    if (result != ESP_OK) {
        return result;
    }
    result = uart_set_pin(port, board_config::telemetry_uart_tx_pin,
                          board_config::telemetry_uart_rx_pin, UART_PIN_NO_CHANGE,
                          UART_PIN_NO_CHANGE);
    if (result != ESP_OK) {
        return result;
    }
    return uart_driver_install(port, TELEMETRY_RX_BUFFER_SIZE, 0, 0, nullptr, 0);
}

}  // namespace ground_station::firmware
