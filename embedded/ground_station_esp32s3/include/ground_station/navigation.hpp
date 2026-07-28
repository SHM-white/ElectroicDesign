#pragma once

#include <optional>

namespace ground_station {

enum class Page {
    overview,
    detail,
};

enum class NavigationIntent {
    show_detail,
    show_overview,
};

std::optional<Page> navigation_target(Page current_page, NavigationIntent intent) noexcept;

}  // namespace ground_station
