#pragma once

#include "ground_station/v7.hpp"

#include <cstdint>
#include <optional>

namespace ground_station {

struct FreshnessPolicy {
    double position_max_age_s{0.20};
    double status_max_age_s{0.50};
    double link_max_age_s{0.50};
};

struct PositionSample {
    std::uint32_t sequence{0U};
    double received_s{0.0};
    std::int32_t x_cm{0};
    std::int32_t y_cm{0};
    double age_s{0.0};
    bool fresh{false};
};

struct StatusSample {
    std::uint32_t sequence{0U};
    double received_s{0.0};
    std::uint8_t mode{0U};
    bool armed{false};
    double age_s{0.0};
    bool fresh{false};
};

struct Diagnostic51 {
    std::uint32_t sequence{0U};
    double received_s{0.0};
    std::uint8_t mode{0U};
    std::uint8_t state{0U};
    std::optional<std::uint8_t> quality;
    std::optional<std::int16_t> integrated_x_cm;
    std::optional<std::int16_t> integrated_y_cm;
};

struct LinkSample {
    std::uint32_t sequence{0U};
    std::optional<double> received_s;
    double age_s{0.0};
    bool fresh{false};
};

struct TelemetrySnapshot {
    std::optional<PositionSample> position;
    std::optional<StatusSample> status;
    std::optional<Diagnostic51> diagnostic_51;
    LinkSample link;
};

class TelemetryCache {
public:
    explicit TelemetryCache(FreshnessPolicy policy = {});

    void ingest(const V7Frame& frame, double steady_now_s);
    TelemetrySnapshot snapshot(double steady_now_s) const;

private:
    FreshnessPolicy policy_;
    std::optional<PositionSample> position_;
    std::optional<StatusSample> status_;
    std::optional<Diagnostic51> diagnostic_51_;
    std::uint32_t position_sequence_{0U};
    std::uint32_t status_sequence_{0U};
    std::uint32_t diagnostic_sequence_{0U};
    std::uint32_t link_sequence_{0U};
    std::optional<double> last_link_s_;
};

}  // namespace ground_station
