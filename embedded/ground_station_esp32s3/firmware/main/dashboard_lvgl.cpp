#include "dashboard_lvgl.hpp"

#include "ground_station/ui_layout.hpp"

#include <cstdint>

namespace ground_station {
namespace firmware {
namespace {

constexpr std::uint32_t gs_color_canvas = 0xF4F4F4U;
constexpr std::uint32_t gs_color_surface = 0xFFFFFFU;
constexpr std::uint32_t gs_color_surface_subtle = 0xE8E8E8U;
constexpr std::uint32_t gs_color_border = 0xC6C6C6U;
constexpr std::uint32_t gs_color_masthead = 0x161616U;
constexpr std::uint32_t gs_color_masthead_text = 0xFFFFFFU;
constexpr std::uint32_t gs_color_text_secondary = 0x525252U;
constexpr std::uint32_t gs_color_text_muted = 0x6F6F6FU;
constexpr std::uint32_t gs_color_interactive = 0x0F62FEU;
constexpr std::uint32_t gs_color_interactive_pressed = 0x0353E9U;
constexpr std::uint32_t gs_color_success = 0x198038U;
constexpr std::uint32_t gs_color_warning = 0xF1C21BU;
constexpr std::uint32_t gs_color_error = 0xDA1E28U;
constexpr std::uint32_t gs_color_info = 0x0043CEU;

lv_color_t semantic_color(const SemanticState state) {
    switch (state) {
    case SemanticState::success:
        return lv_color_hex(gs_color_success);
    case SemanticState::warning:
        return lv_color_hex(gs_color_warning);
    case SemanticState::error:
        return lv_color_hex(gs_color_error);
    case SemanticState::info:
        return lv_color_hex(gs_color_info);
    case SemanticState::neutral:
    default:
        return lv_color_hex(gs_color_text_muted);
    }
}

void set_flat_surface(lv_obj_t* object, const std::uint32_t color) {
    lv_obj_set_style_bg_opa(object, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(object, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(object, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(object, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(object, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
}

lv_obj_t* create_label(lv_obj_t* parent, const ui_layout::Rect& bounds, const char* text,
                       const lv_font_t* font, const std::uint32_t color,
                       const lv_text_align_t align = LV_TEXT_ALIGN_LEFT) {
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_set_pos(label, bounds.x, bounds.y);
    lv_obj_set_size(label, bounds.width, bounds.height);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(label, align, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text_static(label, text);
    return label;
}

void style_navigation_button(lv_obj_t* button, const char* text) {
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(button, lv_color_hex(gs_color_surface), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(button, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(button, lv_color_hex(gs_color_interactive),
                                  LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(button, lv_color_hex(gs_color_interactive),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(button, lv_color_hex(gs_color_interactive_pressed),
                              LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_text_color(button, lv_color_hex(gs_color_masthead_text),
                                LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_width(button, 2, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_color(button, lv_color_hex(gs_color_interactive),
                                  LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_bg_color(button, lv_color_hex(gs_color_surface_subtle),
                              LV_PART_MAIN | LV_STATE_DISABLED);
    lv_obj_set_style_border_color(button, lv_color_hex(gs_color_border),
                                  LV_PART_MAIN | LV_STATE_DISABLED);
    lv_obj_set_style_text_color(button, lv_color_hex(gs_color_text_muted),
                                LV_PART_MAIN | LV_STATE_DISABLED);
    lv_obj_set_style_radius(button, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(button, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(button, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t* label = lv_label_create(button);
    lv_label_set_text_static(label, text);
    lv_obj_center(label);
}

void set_field(lv_obj_t* label, const ViewField& field) {
    lv_label_set_text(label, field.text.c_str());
    lv_obj_set_style_text_color(label, semantic_color(field.state), LV_PART_MAIN | LV_STATE_DEFAULT);
}

}  // namespace

DashboardLvgl::DashboardLvgl(lv_disp_t* display) noexcept : display_(display) {}

void DashboardLvgl::initialize() noexcept {
    if (initialized_) {
        return;
    }
    if (display_ != nullptr) {
        lv_disp_set_default(display_);
    }
    overview_screen_ = lv_obj_create(nullptr);
    detail_screen_ = lv_obj_create(nullptr);
    create_overview();
    create_detail();
    initialized_ = true;
    load_page(Page::overview);
}

void DashboardLvgl::create_overview() noexcept {
    using namespace ui_layout;
    set_flat_surface(overview_screen_, gs_color_canvas);
    lv_obj_set_size(overview_screen_, root.width, root.height);
    lv_obj_t* masthead_background = create_label(overview_screen_, masthead, "",
                                                 &lv_font_montserrat_12, gs_color_masthead);
    set_flat_surface(masthead_background, gs_color_masthead);
    lv_obj_t* content_background = create_label(overview_screen_, content, "",
                                                &lv_font_montserrat_12, gs_color_surface);
    set_flat_surface(content_background, gs_color_surface);
    lv_obj_t* footer_background = create_label(overview_screen_, footer, "",
                                               &lv_font_montserrat_12, gs_color_surface);
    set_flat_surface(footer_background, gs_color_surface);
    lv_obj_t* title = create_label(overview_screen_, overview_title, "OVERVIEW",
                                   &lv_font_montserrat_40, gs_color_masthead_text);
    set_flat_surface(title, gs_color_masthead);
    overview_link_ = create_label(overview_screen_, overview_link, "",
                                  &lv_font_montserrat_20, gs_color_text_muted,
                                  LV_TEXT_ALIGN_RIGHT);
    overview_status_ = create_label(overview_screen_, overview_status_value, "",
                                    &lv_font_montserrat_20, gs_color_text_muted);
    create_label(overview_screen_, overview_status_label, "STATUS", &lv_font_montserrat_16,
                 gs_color_text_secondary);
    overview_position_ = create_label(overview_screen_, overview_position_value, "",
                                     &lv_font_montserrat_32, gs_color_text_muted);
    create_label(overview_screen_, overview_position_label, "POSITION", &lv_font_montserrat_16,
                 gs_color_text_secondary);
    overview_position_state_ = create_label(overview_screen_, overview_position_state, "",
                                            &lv_font_montserrat_16, gs_color_text_muted);
    detail_button_ = lv_btn_create(overview_screen_);
    lv_obj_set_pos(detail_button_, overview_detail_button.x, overview_detail_button.y);
    lv_obj_set_size(detail_button_, overview_detail_button.width, overview_detail_button.height);
    style_navigation_button(detail_button_, "DETAIL");
    lv_obj_add_event_cb(detail_button_, on_navigation_event, LV_EVENT_RELEASED, this);
}

void DashboardLvgl::create_detail() noexcept {
    using namespace ui_layout;
    set_flat_surface(detail_screen_, gs_color_canvas);
    lv_obj_set_size(detail_screen_, root.width, root.height);
    lv_obj_t* masthead_background = create_label(detail_screen_, masthead, "",
                                                 &lv_font_montserrat_12, gs_color_masthead);
    set_flat_surface(masthead_background, gs_color_masthead);
    lv_obj_t* content_background = create_label(detail_screen_, content, "",
                                                &lv_font_montserrat_12, gs_color_surface);
    set_flat_surface(content_background, gs_color_surface);
    lv_obj_t* footer_background = create_label(detail_screen_, footer, "",
                                               &lv_font_montserrat_12, gs_color_surface);
    set_flat_surface(footer_background, gs_color_surface);
    lv_obj_t* title = create_label(detail_screen_, detail_title, "DETAIL", &lv_font_montserrat_40,
                                   gs_color_masthead_text);
    set_flat_surface(title, gs_color_masthead);
    create_label(detail_screen_, detail_position_age_label, "POSITION AGE", &lv_font_montserrat_16,
                 gs_color_text_secondary);
    detail_position_age_ = create_label(detail_screen_, detail_position_age_value, "",
                                        &lv_font_montserrat_20,
                                        gs_color_text_muted);
    create_label(detail_screen_, detail_diagnostic_label, "DIAGNOSTIC 0x51", &lv_font_montserrat_16,
                 gs_color_text_secondary);
    detail_diagnostic_51_ = create_label(detail_screen_, detail_diagnostic_value, "",
                                         &lv_font_montserrat_20, gs_color_text_muted);
    create_label(detail_screen_, detail_quality_label, "QUALITY", &lv_font_montserrat_16,
                 gs_color_text_secondary);
    detail_quality_ = create_label(detail_screen_, detail_quality_value, "",
                                   &lv_font_montserrat_20, gs_color_text_muted);
    back_button_ = lv_btn_create(detail_screen_);
    lv_obj_set_pos(back_button_, detail_back_button.x, detail_back_button.y);
    lv_obj_set_size(back_button_, detail_back_button.width, detail_back_button.height);
    style_navigation_button(back_button_, "BACK");
    lv_obj_add_event_cb(back_button_, on_navigation_event, LV_EVENT_RELEASED, this);
}

void DashboardLvgl::apply(const DashboardViewModel& view_model) noexcept {
    if (!initialized_) {
        return;
    }
    set_field(overview_link_, view_model.link);
    set_field(overview_status_, view_model.status);
    set_field(overview_position_, view_model.position);
    if (view_model.position.state == SemanticState::neutral ||
        view_model.position.state == SemanticState::warning) {
        set_field(overview_position_state_, view_model.position);
    } else {
        lv_label_set_text_static(overview_position_state_, "");
    }
    set_field(detail_position_age_, view_model.position_age);
    set_field(detail_diagnostic_51_, view_model.diagnostic_51);
    set_field(detail_quality_, view_model.quality);
}

void DashboardLvgl::on_navigation_event(lv_event_t* event) {
    if (lv_event_get_code(event) != LV_EVENT_RELEASED) {
        return;
    }
    auto* dashboard = static_cast<DashboardLvgl*>(lv_event_get_user_data(event));
    const auto* target = lv_event_get_target(event);
    const NavigationIntent intent = target == dashboard->detail_button_
                                        ? NavigationIntent::show_detail
                                        : NavigationIntent::show_overview;
    const auto page = navigation_target(dashboard->current_page_, intent);
    if (page.has_value()) {
        dashboard->load_page(*page);
    }
}

void DashboardLvgl::load_page(const Page page) noexcept {
    current_page_ = page;
    lv_scr_load(page == Page::overview ? overview_screen_ : detail_screen_);
}

}  // namespace firmware
}  // namespace ground_station
