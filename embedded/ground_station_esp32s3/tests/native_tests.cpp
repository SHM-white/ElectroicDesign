#include "ground_station/board_config.hpp"
#include "ground_station/navigation.hpp"
#include "ground_station/telemetry.hpp"
#include "ground_station/ui_layout.hpp"
#include "ground_station/v7.hpp"
#include "ground_station/view_model.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

using ground_station::TelemetryCache;
using ground_station::V7Frame;
using ground_station::SemanticState;
using Bytes = std::vector<std::uint8_t>;

int failures = 0;

void check(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

template <typename Value, std::size_t Size>
constexpr bool arrays_equal(const std::array<Value, Size>& left,
                            const std::array<Value, Size>& right) {
    for (std::size_t index = 0U; index < Size; ++index) {
        if (left[index] != right[index]) {
            return false;
        }
    }
    return true;
}

std::uint8_t hex_nibble(const char value) {
    if (value >= '0' && value <= '9') {
        return static_cast<std::uint8_t>(value - '0');
    }
    return static_cast<std::uint8_t>(value - 'A' + 10);
}

Bytes from_hex(const std::string_view text) {
    Bytes result;
    result.reserve(text.size() / 2U);
    for (std::size_t index = 0; index < text.size(); index += 2U) {
        const auto high = static_cast<std::uint8_t>(hex_nibble(text[index]) << 4U);
        result.push_back(static_cast<std::uint8_t>(high | hex_nibble(text[index + 1U])));
    }
    return result;
}

void append_le32(Bytes& bytes, const std::int32_t value) {
    const auto bits = static_cast<std::uint32_t>(value);
    bytes.push_back(static_cast<std::uint8_t>(bits & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((bits >> 8U) & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((bits >> 16U) & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((bits >> 24U) & 0xFFU));
}

Bytes position_frame(const std::int32_t x_cm, const std::int32_t y_cm) {
    Bytes data;
    append_le32(data, x_cm);
    append_le32(data, y_cm);
    return ground_station::build_v7_frame(0xFFU, 0x08U, data);
}

V7Frame decoded(const Bytes& raw) {
    V7Frame frame;
    check(ground_station::decode_v7_frame(raw, frame), "test fixture must decode");
    return frame;
}

void test_v7_framing_and_resynchronization() {
    struct Vector {
        Bytes data;
        std::string_view expected;
    };
    const std::vector<Vector> vectors{
        {{0x10U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U},
         "AAFFE00B1000010000000000000000A585"},
        {{0x10U, 0x00U, 0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U},
         "AAFFE00B1000020000000000000000A68E"},
        {{0x01U, 0x01U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U},
         "AAFFE00B01010103000000000000009A02"},
        {{0x10U, 0x00U, 0x05U, 0x96U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U},
         "AAFFE00B10000596000000000000003F59"},
        {{0x10U, 0x00U, 0x06U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U},
         "AAFFE00B1000060000000000000000AAB2"},
        {{0x10U, 0x02U, 0x03U, 0x64U, 0x00U, 0x1EU, 0x00U, 0x5AU, 0x00U, 0x00U, 0x00U},
         "AAFFE00B10020364001E005A00000085E7"},
    };
    for (const auto& vector : vectors) {
        check(ground_station::build_v7_frame(0xFFU, 0xE0U, vector.data) == from_hex(vector.expected),
              "canonical V7 vector is byte-identical");
    }

    auto bad = from_hex("AAFFE00B1000020000000000000000A68E");
    bad.back() ^= 0x01U;
    V7Frame rejected;
    check(!ground_station::decode_v7_frame(bad, rejected), "direct decoder rejects checksum error");

    const auto good = position_frame(125, -180);
    ground_station::V7StreamDecoder decoder;
    Bytes first{0x00U, 0x55U, 0xAAU};
    first.insert(first.end(), bad.begin() + 1, bad.end());
    first.insert(first.end(), good.begin(), good.begin() + 5);
    check(decoder.feed(first).empty(), "fragmented V7 frame is not delivered early");
    const Bytes rest(good.begin() + 5, good.end());
    const auto frames = decoder.feed(rest);
    check(frames.size() == 1U && frames.front().raw == good,
          "V7 stream rejects corruption and resynchronizes to the next frame");
    check(decoder.rejected_frames() == 1U, "V7 stream counts one checksum rejection");
}

void test_telemetry_authority_and_freshness() {
    TelemetryCache cache;
    cache.ingest(decoded(position_frame(100, -200)), 10.0);
    Bytes diagnostic(15U, 0U);
    diagnostic[0] = 2U;
    diagnostic[1] = 1U;
    diagnostic[10] = 0x30U;
    diagnostic[11] = 0x75U;
    diagnostic[12] = 0xD0U;
    diagnostic[13] = 0x8AU;
    diagnostic[14] = 200U;
    cache.ingest(decoded(ground_station::build_v7_frame(0xFFU, 0x51U, diagnostic)), 10.15);

    const auto isolated = cache.snapshot(10.21);
    check(isolated.position.has_value(), "0x08 creates position state");
    check(isolated.position.has_value() && isolated.position->x_cm == 100 &&
              isolated.position->y_cm == -200 && isolated.position->sequence == 1U,
          "0x51 never changes position value or sequence");
    check(isolated.position.has_value() && !isolated.position->fresh,
          "0x51 never refreshes stale position");
    check(isolated.diagnostic_51.has_value() &&
              isolated.diagnostic_51->integrated_x_cm.has_value() &&
              *isolated.diagnostic_51->integrated_x_cm == 30000 &&
              isolated.diagnostic_51->integrated_y_cm.has_value() &&
              *isolated.diagnostic_51->integrated_y_cm == -30000 &&
              isolated.diagnostic_51->quality == 200U,
          "0x51 is retained as a separate diagnostic");
    check(isolated.link.fresh && isolated.link.sequence == 2U,
          "any verified V7 frame independently refreshes link state");

    TelemetryCache boundary;
    boundary.ingest(decoded(position_frame(1, 2)), 3.0);
    check(boundary.snapshot(3.20).position->fresh, "position is fresh through 0.20 seconds");
    check(!boundary.snapshot(3.201).position->fresh, "position is stale after 0.20 seconds");
    boundary.ingest(decoded(ground_station::build_v7_frame(0xFFU, 0x06U, {3U, 1U})), 4.0);
    check(boundary.snapshot(4.50).status->fresh, "status is fresh through 0.50 seconds");
    check(!boundary.snapshot(4.501).status->fresh, "status is stale after 0.50 seconds");
    check(boundary.snapshot(4.50).link.fresh, "link is fresh through 0.50 seconds");
    check(!boundary.snapshot(4.501).link.fresh, "link is stale after 0.50 seconds");
}

void test_generic_navigation() {
    using ground_station::NavigationIntent;
    using ground_station::Page;

    const auto detail = ground_station::navigation_target(Page::overview, NavigationIntent::show_detail);
    check(detail.has_value() && *detail == ground_station::Page::detail,
          "show-detail intent navigates from Overview to Detail");

    const auto overview = ground_station::navigation_target(Page::detail, NavigationIntent::show_overview);
    check(overview.has_value() && *overview == ground_station::Page::overview,
          "show-overview intent navigates from Detail to Overview");

    check(!ground_station::navigation_target(Page::overview, NavigationIntent::show_overview).has_value(),
          "show-overview intent is ignored on Overview");
    check(!ground_station::navigation_target(Page::detail, NavigationIntent::show_detail).has_value(),
          "show-detail intent is ignored on Detail");
}

void check_field(const ground_station::ViewField& field, const std::string_view expected_text,
                 const SemanticState expected_state, const std::string_view message) {
    check(field.text == expected_text && field.state == expected_state, message);
}

void test_unknown_view_model() {
    const auto view = ground_station::make_dashboard_view_model(TelemetryCache{}.snapshot(1.0));
    check_field(view.link, "LINK UNKNOWN", SemanticState::neutral, "unknown link is neutral and exact");
    check_field(view.status, "STATUS UNKNOWN", SemanticState::neutral,
                "unknown status is neutral and exact");
    check_field(view.position, "POSITION UNKNOWN", SemanticState::neutral,
                "unknown position is neutral and exact");
    check_field(view.position_age, "POSITION AGE UNKNOWN", SemanticState::neutral,
                "unknown position age is neutral and exact");
    check_field(view.diagnostic_51, "0x51 UNKNOWN", SemanticState::neutral,
                "unknown diagnostic is neutral and exact");
    check_field(view.quality, "QUALITY UNKNOWN", SemanticState::neutral,
                "unknown quality is neutral and exact");
}

void test_fresh_and_stale_view_model() {
    TelemetryCache cache;
    cache.ingest(decoded(position_frame(125, -180)), 20.0);
    cache.ingest(decoded(ground_station::build_v7_frame(0xFFU, 0x06U, {3U, 1U})), 20.0);
    Bytes diagnostic(15U, 0U);
    diagnostic[0] = 2U;
    diagnostic[1] = 1U;
    diagnostic[14] = 200U;
    cache.ingest(decoded(ground_station::build_v7_frame(0xFFU, 0x51U, diagnostic)), 20.0);

    const auto fresh = ground_station::make_dashboard_view_model(cache.snapshot(20.05));
    check_field(fresh.link, "LINK OK", SemanticState::success, "fresh link is success and exact");
    check_field(fresh.status, "STATUS OK - MODE 3 ARMED", SemanticState::success,
                "fresh armed status is success and exact");
    check_field(fresh.position, "POSITION OK - X +1.25 m Y -1.80 m", SemanticState::success,
                "fresh signed position is success and exact");
    check_field(fresh.position_age, "POSITION AGE 0.050 s", SemanticState::success,
                "fresh position age is success and exact");
    check_field(fresh.diagnostic_51, "0x51 MODE 2 STATE 1", SemanticState::info,
                "diagnostic is informational and exact");
    check_field(fresh.quality, "QUALITY 200", SemanticState::info,
                "quality is informational and exact");

    const auto stale = ground_station::make_dashboard_view_model(cache.snapshot(20.501));
    check_field(stale.link, "LINK LOST", SemanticState::error, "lost link is error and exact");
    check_field(stale.status, "STATUS STALE", SemanticState::warning,
                "stale status is warning and exact");
    check_field(stale.position, "POSITION STALE", SemanticState::warning,
                "stale position is warning and exact");
    check_field(stale.position_age, "POSITION AGE STALE", SemanticState::warning,
                "stale position age is warning and exact");
    check_field(stale.diagnostic_51, "0x51 MODE 2 STATE 1", SemanticState::info,
                "diagnostic remains informational when samples age");
    check_field(stale.quality, "QUALITY 200", SemanticState::info,
                "quality remains informational when samples age");

    TelemetryCache locked_cache;
    locked_cache.ingest(decoded(ground_station::build_v7_frame(0xFFU, 0x06U, {4U, 0U})), 5.0);
    const auto locked = ground_station::make_dashboard_view_model(locked_cache.snapshot(5.1));
    check_field(locked.status, "STATUS OK - MODE 4 LOCKED", SemanticState::success,
                "fresh locked status is success and exact");

    TelemetryCache no_quality_cache;
    no_quality_cache.ingest(decoded(ground_station::build_v7_frame(0xFFU, 0x51U, {7U, 9U})), 8.0);
    const auto no_quality = ground_station::make_dashboard_view_model(no_quality_cache.snapshot(8.1));
    check_field(no_quality.diagnostic_51, "0x51 MODE 7 STATE 9", SemanticState::info,
                "short diagnostic text is informational and exact");
    check_field(no_quality.quality, "QUALITY UNKNOWN", SemanticState::neutral,
                "missing diagnostic quality remains neutral and exact");
}

void test_diagnostic_does_not_change_position_view() {
    TelemetryCache cache;
    cache.ingest(decoded(position_frame(100, -200)), 10.0);
    cache.ingest(decoded(ground_station::build_v7_frame(0xFFU, 0x51U, {2U, 1U, 200U})), 10.15);

    const auto fresh = ground_station::make_dashboard_view_model(cache.snapshot(10.19));
    check_field(fresh.position, "POSITION OK - X +1.00 m Y -2.00 m", SemanticState::success,
                "0x51 does not alter fresh authoritative position text");

    const auto stale = ground_station::make_dashboard_view_model(cache.snapshot(10.21));
    check_field(stale.position, "POSITION STALE", SemanticState::warning,
                "0x51 does not refresh stale authoritative position text");
    check_field(stale.position_age, "POSITION AGE STALE", SemanticState::warning,
                "0x51 does not refresh stale authoritative position age");
}

void test_board_config() {
    namespace board = ground_station::board_config;

    static_assert(board::display_width_px == 800 && board::display_height_px == 480);
    static_assert(board::i2c_port == 0 && board::i2c_sda_pin == 8 && board::i2c_scl_pin == 9);
    static_assert(board::rgb_vsync_pin == 3 && board::rgb_hsync_pin == 46 &&
                  board::rgb_data_enable_pin == 5 && board::rgb_pixel_clock_pin == 7);
    constexpr std::array<int, 16> expected_data_pins{
        14, 38, 18, 17, 10, 39, 0, 45, 48, 47, 21, 1, 2, 42, 41, 40};
    static_assert(arrays_equal(board::rgb_data_pins, expected_data_pins));
    static_assert(board::rgb_pixel_clock_hz == 16000000 && board::rgb_pixel_clock_active_low);
    static_assert(board::rgb_hsync_pulse_width == 4 && board::rgb_hsync_back_porch == 8 &&
                  board::rgb_hsync_front_porch == 8);
    static_assert(board::rgb_vsync_pulse_width == 4 && board::rgb_vsync_back_porch == 8 &&
                  board::rgb_vsync_front_porch == 8);
    static_assert(board::telemetry_uart_port == 0 && board::telemetry_uart_tx_pin == 43 &&
                  board::telemetry_uart_rx_pin == 44 && board::telemetry_uart_baud == 500000);
    static_assert(board::telemetry_uart_data_bits == 8 && board::telemetry_uart_stop_bits == 1 &&
                  board::telemetry_uart_parity == board::UartParity::none &&
                  board::telemetry_uart_flow_control == board::UartFlowControl::none);
    static_assert(board::telemetry_uart_tx_pin != 17 && board::telemetry_uart_tx_pin != 18 &&
                  board::telemetry_uart_rx_pin != 17 && board::telemetry_uart_rx_pin != 18);

    check(board::rgb_data_pins == expected_data_pins, "RGB data pin order is exact");
}

void check_rect(const ground_station::ui_layout::Rect& rect, const int x, const int y,
                const int width, const int height, const std::string_view message) {
    check(rect.x == x && rect.y == y && rect.width == width && rect.height == height, message);
}

bool within(const ground_station::ui_layout::Rect& rect,
            const ground_station::ui_layout::Rect& bounds) {
    return rect.x >= bounds.x && rect.y >= bounds.y &&
           rect.x + rect.width <= bounds.x + bounds.width &&
           rect.y + rect.height <= bounds.y + bounds.height;
}

bool separated(const ground_station::ui_layout::Rect& left,
               const ground_station::ui_layout::Rect& right) {
    return left.x + left.width <= right.x || right.x + right.width <= left.x ||
           left.y + left.height <= right.y || right.y + right.height <= left.y;
}

void test_fixed_dashboard_layout() {
    using namespace ground_station::ui_layout;

    check_rect(root, 0, 0, 800, 480, "root bounds are exact");
    check_rect(masthead, 0, 0, 800, 64, "masthead bounds are exact");
    check_rect(content, 0, 64, 800, 352, "content bounds are exact");
    check_rect(footer, 0, 416, 800, 64, "footer bounds are exact");

    check_rect(overview_title, 24, 8, 320, 48, "overview title bounds are exact");
    check_rect(overview_link, 600, 16, 176, 32, "overview link bounds are exact");
    check_rect(overview_status_row, 24, 88, 752, 80, "overview status row bounds are exact");
    check_rect(overview_status_label, 24, 88, 752, 24, "overview status label bounds are exact");
    check_rect(overview_status_value, 24, 120, 752, 48, "overview status value bounds are exact");
    check_rect(overview_position_row, 24, 192, 752, 144, "overview position row bounds are exact");
    check_rect(overview_position_label, 24, 192, 752, 24,
               "overview position label bounds are exact");
    check_rect(overview_position_value, 24, 224, 752, 56,
               "overview position value bounds are exact");
    check_rect(overview_position_state, 24, 288, 752, 32,
               "overview position state bounds are exact");
    check_rect(overview_detail_button, 680, 424, 96, 48,
               "overview detail button bounds are exact");

    check_rect(detail_title, 24, 8, 320, 48, "detail title bounds are exact");
    check_rect(detail_position_age_row, 24, 88, 752, 72,
               "detail position age row bounds are exact");
    check_rect(detail_position_age_label, 24, 88, 752, 24,
               "detail position age label bounds are exact");
    check_rect(detail_position_age_value, 24, 112, 752, 48,
               "detail position age value bounds are exact");
    check_rect(detail_diagnostic_row, 24, 184, 752, 88,
               "detail diagnostic row bounds are exact");
    check_rect(detail_diagnostic_label, 24, 184, 752, 24,
               "detail diagnostic label bounds are exact");
    check_rect(detail_diagnostic_value, 24, 208, 752, 64,
               "detail diagnostic value bounds are exact");
    check_rect(detail_quality_row, 24, 296, 752, 72, "detail quality row bounds are exact");
    check_rect(detail_quality_label, 24, 296, 752, 24, "detail quality label bounds are exact");
    check_rect(detail_quality_value, 24, 320, 752, 48,
               "detail quality value bounds are exact");
    check_rect(detail_back_button, 24, 424, 96, 48, "detail back button bounds are exact");

    check(minimum_touch_target_px == 48, "minimum touch target is 48 pixels");
    check(overview_detail_button.width == 96 && overview_detail_button.height == 48 &&
              detail_back_button.width == 96 && detail_back_button.height == 48,
          "navigation buttons are exactly 96 by 48 pixels");

    const std::array<ground_station::ui_layout::Rect, 2> regions{masthead, footer};
    const std::array<ground_station::ui_layout::Rect, 21> elements{
        overview_title, overview_link, overview_status_row, overview_status_label,
        overview_status_value, overview_position_row, overview_position_label,
        overview_position_value, overview_position_state, overview_detail_button, detail_title,
        detail_position_age_row, detail_position_age_label, detail_position_age_value,
        detail_diagnostic_row, detail_diagnostic_label, detail_diagnostic_value, detail_quality_row,
        detail_quality_label, detail_quality_value, detail_back_button};
    for (const auto& element : elements) {
        check(within(element, root), "every layout element stays within the canvas");
        check(within(element, content) || within(element, masthead) || within(element, footer),
              "every layout element stays within its declared region");
    }
    for (const auto& region : regions) {
        check(separated(region, content), "content does not overlap masthead or footer");
    }
    check(separated(overview_detail_button, overview_position_state),
          "overview detail button does not overlap content");
    check(separated(detail_back_button, detail_quality_value),
          "detail back button does not overlap content");
}

}  // namespace

int main() {
    test_v7_framing_and_resynchronization();
    test_telemetry_authority_and_freshness();
    test_generic_navigation();
    test_unknown_view_model();
    test_fresh_and_stale_view_model();
    test_diagnostic_does_not_change_position_view();
    test_board_config();
    test_fixed_dashboard_layout();
    if (failures != 0) {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All ground-station native tests passed\n";
    return 0;
}
