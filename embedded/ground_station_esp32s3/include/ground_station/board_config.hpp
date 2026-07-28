#pragma once

#include <array>

namespace ground_station::board_config {

inline constexpr int display_width_px = 800;
inline constexpr int display_height_px = 480;

inline constexpr int i2c_port = 0;
inline constexpr int i2c_sda_pin = 8;
inline constexpr int i2c_scl_pin = 9;

inline constexpr int rgb_vsync_pin = 3;
inline constexpr int rgb_hsync_pin = 46;
inline constexpr int rgb_data_enable_pin = 5;
inline constexpr int rgb_pixel_clock_pin = 7;
inline constexpr std::array<int, 16> rgb_data_pins{
    14, 38, 18, 17, 10, 39, 0, 45, 48, 47, 21, 1, 2, 42, 41, 40};
inline constexpr int rgb_pixel_clock_hz = 16000000;
inline constexpr bool rgb_pixel_clock_active_low = true;
inline constexpr int rgb_hsync_pulse_width = 4;
inline constexpr int rgb_hsync_back_porch = 8;
inline constexpr int rgb_hsync_front_porch = 8;
inline constexpr int rgb_vsync_pulse_width = 4;
inline constexpr int rgb_vsync_back_porch = 8;
inline constexpr int rgb_vsync_front_porch = 8;

enum class UartParity {
    none,
};

enum class UartFlowControl {
    none,
};

inline constexpr int telemetry_uart_port = 0;
inline constexpr int telemetry_uart_tx_pin = 43;
inline constexpr int telemetry_uart_rx_pin = 44;
inline constexpr int telemetry_uart_baud = 500000;
inline constexpr int telemetry_uart_data_bits = 8;
inline constexpr int telemetry_uart_stop_bits = 1;
inline constexpr UartParity telemetry_uart_parity = UartParity::none;
inline constexpr UartFlowControl telemetry_uart_flow_control = UartFlowControl::none;

}  // namespace ground_station::board_config
