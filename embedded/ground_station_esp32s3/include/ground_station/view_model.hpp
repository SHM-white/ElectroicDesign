#pragma once

#include "ground_station/telemetry.hpp"

#include <string>

namespace ground_station {

enum class SemanticState {
    neutral,
    success,
    warning,
    error,
    info,
};

struct ViewField {
    const std::string text;
    const SemanticState state;
};

struct DashboardViewModel {
    const ViewField link;
    const ViewField status;
    const ViewField position;
    const ViewField position_age;
    const ViewField diagnostic_51;
    const ViewField quality;
};

DashboardViewModel make_dashboard_view_model(const TelemetrySnapshot& snapshot);

}  // namespace ground_station
