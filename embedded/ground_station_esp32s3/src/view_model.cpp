#include "ground_station/view_model.hpp"

#include <cstdio>
#include <string>

namespace ground_station {
namespace {

ViewField link_field(const LinkSample& link) {
    if (!link.received_s.has_value()) {
        return {"LINK UNKNOWN", SemanticState::neutral};
    }
    if (link.fresh) {
        return {"LINK OK", SemanticState::success};
    }
    return {"LINK LOST", SemanticState::error};
}

ViewField status_field(const std::optional<StatusSample>& status) {
    if (!status.has_value()) {
        return {"STATUS UNKNOWN", SemanticState::neutral};
    }
    if (!status->fresh) {
        return {"STATUS STALE", SemanticState::warning};
    }
    char text[64]{};
    static_cast<void>(std::snprintf(text, sizeof(text), "STATUS OK - MODE %u %s",
                                    static_cast<unsigned int>(status->mode),
                                    status->armed ? "ARMED" : "LOCKED"));
    return {std::string(text), SemanticState::success};
}

ViewField position_field(const std::optional<PositionSample>& position) {
    if (!position.has_value()) {
        return {"POSITION UNKNOWN", SemanticState::neutral};
    }
    if (!position->fresh) {
        return {"POSITION STALE", SemanticState::warning};
    }
    char text[96]{};
    static_cast<void>(std::snprintf(text, sizeof(text), "POSITION OK - X %+.2f m Y %+.2f m",
                                    static_cast<double>(position->x_cm) / 100.0,
                                    static_cast<double>(position->y_cm) / 100.0));
    return {std::string(text), SemanticState::success};
}

ViewField position_age_field(const std::optional<PositionSample>& position) {
    if (!position.has_value()) {
        return {"POSITION AGE UNKNOWN", SemanticState::neutral};
    }
    if (!position->fresh) {
        return {"POSITION AGE STALE", SemanticState::warning};
    }
    char text[64]{};
    static_cast<void>(std::snprintf(text, sizeof(text), "POSITION AGE %.3f s", position->age_s));
    return {std::string(text), SemanticState::success};
}

ViewField diagnostic_field(const std::optional<Diagnostic51>& diagnostic) {
    if (!diagnostic.has_value()) {
        return {"0x51 UNKNOWN", SemanticState::neutral};
    }
    char text[64]{};
    static_cast<void>(std::snprintf(text, sizeof(text), "0x51 MODE %u STATE %u",
                                    static_cast<unsigned int>(diagnostic->mode),
                                    static_cast<unsigned int>(diagnostic->state)));
    return {std::string(text), SemanticState::info};
}

ViewField quality_field(const std::optional<Diagnostic51>& diagnostic) {
    if (!diagnostic.has_value() || !diagnostic->quality.has_value()) {
        return {"QUALITY UNKNOWN", SemanticState::neutral};
    }
    char text[32]{};
    static_cast<void>(std::snprintf(text, sizeof(text), "QUALITY %u",
                                    static_cast<unsigned int>(*diagnostic->quality)));
    return {std::string(text), SemanticState::info};
}

}  // namespace

DashboardViewModel make_dashboard_view_model(const TelemetrySnapshot& snapshot) {
    return {link_field(snapshot.link), status_field(snapshot.status), position_field(snapshot.position),
            position_age_field(snapshot.position), diagnostic_field(snapshot.diagnostic_51),
            quality_field(snapshot.diagnostic_51)};
}

}  // namespace ground_station
