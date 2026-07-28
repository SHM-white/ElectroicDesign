#include "board_support.hpp"
#include "dashboard_lvgl.hpp"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ground_station/board_config.hpp"
#include "ground_station/telemetry.hpp"
#include "ground_station/v7.hpp"
#include "ground_station/view_model.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace {

constexpr std::size_t uart_read_capacity = 512U;
constexpr TickType_t uart_read_timeout = pdMS_TO_TICKS(20);
constexpr TickType_t dashboard_refresh_period = pdMS_TO_TICKS(100);

double monotonic_seconds() {
    return static_cast<double>(esp_timer_get_time()) / 1000000.0;
}

}  // namespace

extern "C" void app_main() {
    using ground_station::Bytes;
    using ground_station::TelemetryCache;
    using ground_station::V7StreamDecoder;
    using ground_station::firmware::BoardSupport;
    using ground_station::firmware::DashboardLvgl;

    BoardSupport board;
    ESP_ERROR_CHECK(ground_station::firmware::initialize_board(board));
    ESP_ERROR_CHECK(ground_station::firmware::initialize_telemetry_uart());

    TelemetryCache cache;
    V7StreamDecoder decoder;
    DashboardLvgl dashboard(board.display);
    const auto initial_view = ground_station::make_dashboard_view_model(cache.snapshot(0.0));
    if (!lvgl_port_lock(0)) {
        return;
    }
    dashboard.initialize();
    dashboard.apply(initial_view);
    lvgl_port_unlock();

    std::array<std::uint8_t, uart_read_capacity> uart_buffer{};
    TickType_t last_refresh = xTaskGetTickCount();
    while (true) {
        const int bytes_read = uart_read_bytes(
            static_cast<uart_port_t>(ground_station::board_config::telemetry_uart_port),
            uart_buffer.data(), uart_buffer.size(), uart_read_timeout);
        if (bytes_read > 0) {
            const Bytes chunk(uart_buffer.begin(), uart_buffer.begin() + bytes_read);
            for (const auto& frame : decoder.feed(chunk)) {
                cache.ingest(frame, monotonic_seconds());
            }
        }

        if (xTaskGetTickCount() - last_refresh >= dashboard_refresh_period) {
            last_refresh += dashboard_refresh_period;
            const auto view = ground_station::make_dashboard_view_model(
                cache.snapshot(monotonic_seconds()));
            if (lvgl_port_lock(0)) {
                dashboard.apply(view);
                lvgl_port_unlock();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
