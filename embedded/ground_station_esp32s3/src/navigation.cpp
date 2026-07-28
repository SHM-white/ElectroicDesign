#include "ground_station/navigation.hpp"

namespace ground_station {

std::optional<Page> navigation_target(const Page current_page, const NavigationIntent intent) noexcept {
    if (current_page == Page::overview && intent == NavigationIntent::show_detail) {
        return Page::detail;
    }
    if (current_page == Page::detail && intent == NavigationIntent::show_overview) {
        return Page::overview;
    }
    return std::nullopt;
}

}  // namespace ground_station
