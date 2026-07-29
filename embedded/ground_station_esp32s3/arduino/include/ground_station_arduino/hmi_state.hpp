#pragma once

#include <cstdint>

namespace ed::ground_station {

enum class HmiState : std::uint8_t { BootLocked, Prestart, SelectPending, Selected, ArmedReady, CarRunning, Fault };
enum class Task : std::uint8_t { None = 0, Task1 = 1, Task2 = 2 };

struct LinkStatus {
    bool connected = false;
    std::uint32_t last_update_ms = 0;
};

struct HmiView {
    HmiState state = HmiState::BootLocked;
    Task selected_task = Task::None;
    std::uint64_t selection_id = 0;
    std::uint64_t car_boot_epoch = 0;
    bool controls_locked = true;
    bool ap_connected = false;
    bool car_fresh = false;
    bool ros_fresh = false;
    bool vision_fresh = false;
    std::uint32_t ap_age_ms = 0;
    std::uint32_t car_age_ms = 0;
    std::uint32_t ros_age_ms = 0;
    std::uint32_t vision_age_ms = 0;
    std::uint8_t route_stage = 0;
    bool car_started = false;
    char reason[97]{};
};

class HmiStateMachine {
public:
    HmiStateMachine();

    void reset();
    bool authority_prestart(std::uint64_t car_boot_epoch, std::uint32_t now_ms);
    bool choose(Task task);
    bool confirm_selection(std::uint32_t now_ms);
    bool receive_ack(std::uint64_t selection_id, std::uint64_t car_boot_epoch, bool accepted,
                     HmiState acknowledged_state, const char* reason);
    bool receive_car_start(std::uint64_t car_boot_epoch, std::uint32_t now_ms);
    void receive_status(std::uint8_t route_stage, std::uint32_t now_ms);
    void update_ap(bool connected, std::uint32_t now_ms);
    void update_car(std::uint32_t now_ms);
    void update_ros(std::uint32_t now_ms);
    void update_vision(std::uint32_t now_ms);
    void authority_lost();
    void tick(std::uint32_t now_ms);
    HmiView view() const { return view_; }

private:
    bool can_select() const;
    void set_fault(const char* reason);
    void age(LinkStatus link, std::uint32_t now_ms, bool& fresh, std::uint32_t& age) const;

    HmiView view_{};
    LinkStatus ap_{};
    LinkStatus car_{};
    LinkStatus ros_{};
    LinkStatus vision_{};
    std::uint32_t now_ms_ = 0;
};

}  // namespace ed::ground_station
