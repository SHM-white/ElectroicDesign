#include "ground_station_arduino/hmi_state.hpp"

#include <cstring>

namespace ed::ground_station {

namespace {
constexpr std::uint32_t kFreshAfterMs = 750;
}

HmiStateMachine::HmiStateMachine() { reset(); }

void HmiStateMachine::reset() {
    view_ = HmiView{};
    view_.state = HmiState::BootLocked;
    view_.controls_locked = true;
    std::strncpy(view_.reason, "WAITING_FOR_CURRENT_PRESTART_EPOCH", sizeof(view_.reason) - 1);
    ap_ = {}; car_ = {}; ros_ = {}; vision_ = {};
}

bool HmiStateMachine::authority_prestart(std::uint64_t car_boot_epoch, std::uint32_t now_ms) {
    if (car_boot_epoch == 0 || view_.state == HmiState::CarRunning) return false;
    view_.state = HmiState::Prestart; view_.controls_locked = false; view_.car_boot_epoch = car_boot_epoch; view_.reason[0] = '\0'; update_ros(now_ms); return true;
}

bool HmiStateMachine::can_select() const { return view_.state == HmiState::Prestart && !view_.controls_locked; }

bool HmiStateMachine::choose(Task task) {
    if (!can_select() || task == Task::None) return false;
    view_.selected_task = task; return true;
}

bool HmiStateMachine::confirm_selection(std::uint32_t now_ms) {
    if (!can_select() || view_.selected_task == Task::None) return false;
    ++view_.selection_id; if (view_.selection_id == 0) ++view_.selection_id;
    view_.state = HmiState::SelectPending; view_.controls_locked = true; view_.reason[0] = '\0'; update_ros(now_ms); return true;
}

bool HmiStateMachine::receive_ack(std::uint64_t selection_id, std::uint64_t car_boot_epoch, bool accepted,
                                  HmiState acknowledged_state, const char* reason) {
    if (view_.state != HmiState::SelectPending || selection_id != view_.selection_id || car_boot_epoch != view_.car_boot_epoch) return false;
    if (!accepted) { set_fault(reason == nullptr ? "SELECTION_REJECTED" : reason); return true; }
    if (acknowledged_state != HmiState::Selected && acknowledged_state != HmiState::ArmedReady) return false;
    view_.state = acknowledged_state; view_.controls_locked = acknowledged_state != HmiState::Selected ? true : false;
    if (reason != nullptr) std::strncpy(view_.reason, reason, sizeof(view_.reason) - 1);
    return true;
}

bool HmiStateMachine::receive_car_start(std::uint64_t car_boot_epoch, std::uint32_t now_ms) {
    if (car_boot_epoch != view_.car_boot_epoch) { set_fault("CAR_EPOCH_MISMATCH"); return false; }
    if (view_.state != HmiState::ArmedReady) { set_fault("NO_COMMITTED_SELECTION"); return false; }
    view_.state = HmiState::CarRunning; view_.controls_locked = true; view_.car_started = true; update_car(now_ms); return true;
}

void HmiStateMachine::receive_status(std::uint8_t route_stage, std::uint32_t now_ms) {
    view_.route_stage = route_stage; update_ros(now_ms);
}

void HmiStateMachine::update_ap(bool connected, std::uint32_t now_ms) { ap_.connected = connected; ap_.last_update_ms = now_ms; }
void HmiStateMachine::update_car(std::uint32_t now_ms) { car_.connected = true; car_.last_update_ms = now_ms; }
void HmiStateMachine::update_ros(std::uint32_t now_ms) { ros_.connected = true; ros_.last_update_ms = now_ms; }
void HmiStateMachine::update_vision(std::uint32_t now_ms) { vision_.connected = true; vision_.last_update_ms = now_ms; }

void HmiStateMachine::authority_lost() {
    if (view_.state != HmiState::CarRunning) {
        view_.state = HmiState::BootLocked;
        view_.controls_locked = true;
        std::strncpy(view_.reason, "WAITING_FOR_CURRENT_PRESTART_EPOCH", sizeof(view_.reason) - 1);
    }
    else set_fault("AUTHORITY_LOST_AFTER_START");
}

void HmiStateMachine::tick(std::uint32_t now_ms) {
    now_ms_ = now_ms;
    age(ap_, now_ms, view_.ap_connected, view_.ap_age_ms);
    age(car_, now_ms, view_.car_fresh, view_.car_age_ms);
    age(ros_, now_ms, view_.ros_fresh, view_.ros_age_ms);
    age(vision_, now_ms, view_.vision_fresh, view_.vision_age_ms);
    if (!view_.ros_fresh && view_.state != HmiState::BootLocked && view_.state != HmiState::CarRunning) authority_lost();
}

void HmiStateMachine::set_fault(const char* reason) {
    view_.state = HmiState::Fault; view_.controls_locked = true;
    std::strncpy(view_.reason, reason == nullptr ? "FAULT" : reason, sizeof(view_.reason) - 1);
}

void HmiStateMachine::age(LinkStatus link, std::uint32_t now_ms, bool& fresh, std::uint32_t& age_ms) const {
    age_ms = link.connected ? now_ms - link.last_update_ms : now_ms;
    fresh = link.connected && age_ms <= kFreshAfterMs;
}

}  // namespace ed::ground_station
