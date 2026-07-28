#include "ground_station/telemetry.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace ground_station {
namespace {

constexpr double kFreshnessToleranceS = 1.0e-12;

std::int32_t read_le_i32(const Bytes& bytes, const std::size_t offset) {
    const auto value = static_cast<std::uint32_t>(bytes[offset]) |
                       (static_cast<std::uint32_t>(bytes[offset + 1U]) << 8U) |
                       (static_cast<std::uint32_t>(bytes[offset + 2U]) << 16U) |
                       (static_cast<std::uint32_t>(bytes[offset + 3U]) << 24U);
    const auto signed_value = value <= 0x7FFFFFFFU
                                  ? static_cast<std::int64_t>(value)
                                  : static_cast<std::int64_t>(value) - 0x100000000LL;
    return static_cast<std::int32_t>(signed_value);
}

std::int16_t read_le_i16(const Bytes& bytes, const std::size_t offset) {
    const std::uint16_t value = static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(bytes[offset]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U));
    const auto signed_value = value <= 0x7FFFU
                                  ? static_cast<std::int32_t>(value)
                                  : static_cast<std::int32_t>(value) - 0x10000;
    return static_cast<std::int16_t>(signed_value);
}

bool is_fresh(const double age_s, const double limit_s) {
    return age_s >= -kFreshnessToleranceS && age_s <= limit_s + kFreshnessToleranceS;
}

}  // namespace

TelemetryCache::TelemetryCache(const FreshnessPolicy policy) : policy_(policy) {}

void TelemetryCache::ingest(const V7Frame& frame, const double steady_now_s) {
    ++link_sequence_;
    last_link_s_ = steady_now_s;

    if (frame.id == 0x08U && frame.data.size() >= 8U) {
        ++position_sequence_;
        position_ = PositionSample{position_sequence_, steady_now_s, read_le_i32(frame.data, 0U),
                                   read_le_i32(frame.data, 4U), 0.0, true};
        return;
    }
    if (frame.id == 0x06U && frame.data.size() >= 2U) {
        ++status_sequence_;
        status_ = StatusSample{status_sequence_, steady_now_s, frame.data[0], frame.data[1] == 1U,
                               0.0, true};
        return;
    }
    if (frame.id == 0x51U && frame.data.size() >= 2U) {
        ++diagnostic_sequence_;
        Diagnostic51 diagnostic;
        diagnostic.sequence = diagnostic_sequence_;
        diagnostic.received_s = steady_now_s;
        diagnostic.mode = frame.data[0];
        diagnostic.state = frame.data[1];
        if (frame.data.size() >= 5U) {
            diagnostic.quality = frame.data.back();
        }
        if (diagnostic.mode == 2U && frame.data.size() >= 15U) {
            diagnostic.integrated_x_cm = read_le_i16(frame.data, 10U);
            diagnostic.integrated_y_cm = read_le_i16(frame.data, 12U);
            diagnostic.quality = frame.data[14];
        }
        diagnostic_51_ = diagnostic;
    }
}

TelemetrySnapshot TelemetryCache::snapshot(const double steady_now_s) const {
    TelemetrySnapshot result;
    result.position = position_;
    if (result.position.has_value()) {
        result.position->age_s = steady_now_s - result.position->received_s;
        result.position->fresh = is_fresh(result.position->age_s, policy_.position_max_age_s);
    }
    result.status = status_;
    if (result.status.has_value()) {
        result.status->age_s = steady_now_s - result.status->received_s;
        result.status->fresh = is_fresh(result.status->age_s, policy_.status_max_age_s);
    }
    result.diagnostic_51 = diagnostic_51_;
    result.link.sequence = link_sequence_;
    result.link.received_s = last_link_s_;
    if (last_link_s_.has_value()) {
        result.link.age_s = steady_now_s - *last_link_s_;
        result.link.fresh = is_fresh(result.link.age_s, policy_.link_max_age_s);
    } else {
        result.link.age_s = std::numeric_limits<double>::infinity();
    }
    return result;
}

}  // namespace ground_station
