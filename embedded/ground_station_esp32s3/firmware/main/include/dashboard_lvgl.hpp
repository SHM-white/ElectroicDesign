#pragma once

#include "ground_station/navigation.hpp"
#include "ground_station/view_model.hpp"
#include "lvgl.h"

namespace ground_station {
namespace firmware {

class DashboardLvgl {
public:
    explicit DashboardLvgl(lv_disp_t* display) noexcept;

    void initialize() noexcept;
    void apply(const DashboardViewModel& view_model) noexcept;

private:
    static void on_navigation_event(lv_event_t* event);

    void create_overview() noexcept;
    void create_detail() noexcept;
    void load_page(Page page) noexcept;

    lv_disp_t* display_;
    lv_obj_t* overview_screen_ = nullptr;
    lv_obj_t* detail_screen_ = nullptr;
    lv_obj_t* overview_link_ = nullptr;
    lv_obj_t* overview_status_ = nullptr;
    lv_obj_t* overview_position_ = nullptr;
    lv_obj_t* overview_position_state_ = nullptr;
    lv_obj_t* detail_position_age_ = nullptr;
    lv_obj_t* detail_diagnostic_51_ = nullptr;
    lv_obj_t* detail_quality_ = nullptr;
    lv_obj_t* detail_button_ = nullptr;
    lv_obj_t* back_button_ = nullptr;
    Page current_page_ = Page::overview;
    bool initialized_ = false;
};

}  // namespace firmware
}  // namespace ground_station
