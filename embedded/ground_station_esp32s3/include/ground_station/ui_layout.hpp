#pragma once

namespace ground_station {
namespace ui_layout {

struct Rect {
    int x;
    int y;
    int width;
    int height;
};

static constexpr int canvas_width = 800;
static constexpr int canvas_height = 480;
static constexpr int minimum_touch_target_px = 48;

static constexpr Rect root{0, 0, canvas_width, canvas_height};
static constexpr Rect masthead{0, 0, canvas_width, 64};
static constexpr Rect content{0, 64, canvas_width, 352};
static constexpr Rect footer{0, 416, canvas_width, 64};

static constexpr Rect overview_title{24, 8, 320, 48};
static constexpr Rect overview_link{600, 16, 176, 32};
static constexpr Rect overview_status_row{24, 88, 752, 80};
static constexpr Rect overview_status_label{24, 88, 752, 24};
static constexpr Rect overview_status_value{24, 120, 752, 48};
static constexpr Rect overview_position_row{24, 192, 752, 144};
static constexpr Rect overview_position_label{24, 192, 752, 24};
static constexpr Rect overview_position_value{24, 224, 752, 56};
static constexpr Rect overview_position_state{24, 288, 752, 32};
static constexpr Rect overview_detail_button{680, 424, 96, 48};

static constexpr Rect detail_title{24, 8, 320, 48};
static constexpr Rect detail_position_age_row{24, 88, 752, 72};
static constexpr Rect detail_position_age_label{24, 88, 752, 24};
static constexpr Rect detail_position_age_value{24, 112, 752, 48};
static constexpr Rect detail_diagnostic_row{24, 184, 752, 88};
static constexpr Rect detail_diagnostic_label{24, 184, 752, 24};
static constexpr Rect detail_diagnostic_value{24, 208, 752, 64};
static constexpr Rect detail_quality_row{24, 296, 752, 72};
static constexpr Rect detail_quality_label{24, 296, 752, 24};
static constexpr Rect detail_quality_value{24, 320, 752, 48};
static constexpr Rect detail_back_button{24, 424, 96, 48};

static_assert(root.width == canvas_width && root.height == canvas_height, "canvas is exact");
static_assert(masthead.y + masthead.height == content.y, "masthead ends at content start");
static_assert(content.y + content.height == footer.y, "content ends at footer start");
static_assert(footer.y + footer.height == canvas_height, "footer ends at canvas end");
static_assert(overview_detail_button.width == 2 * minimum_touch_target_px &&
                  overview_detail_button.height == minimum_touch_target_px,
              "overview target meets minimum touch geometry");
static_assert(detail_back_button.width == 2 * minimum_touch_target_px &&
                  detail_back_button.height == minimum_touch_target_px,
              "detail target meets minimum touch geometry");

}  // namespace ui_layout
}  // namespace ground_station
