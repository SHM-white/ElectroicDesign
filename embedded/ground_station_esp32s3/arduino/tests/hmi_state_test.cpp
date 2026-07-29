#include "ground_station_arduino/hmi_state.hpp"

#include <cassert>

using namespace ed::ground_station;

void test_happy_selection_and_lock() {
    HmiStateMachine hmi;
    assert(hmi.view().state == HmiState::BootLocked);
    assert(hmi.authority_prestart(42, 0));
    hmi.update_ap(true, 0); hmi.update_vision(0);
    assert(hmi.choose(Task::Task1));
    assert(hmi.confirm_selection(10));
    assert(hmi.view().state == HmiState::SelectPending && hmi.view().controls_locked);
    assert(hmi.receive_ack(1, 42, true, HmiState::ArmedReady, "ACK"));
    assert(hmi.view().state == HmiState::ArmedReady);
    assert(hmi.receive_car_start(42, 20));
    assert(hmi.view().state == HmiState::CarRunning && hmi.view().controls_locked);
    assert(!hmi.choose(Task::Task2));
}

void test_pending_and_reboot_stay_locked() {
    HmiStateMachine hmi;
    hmi.authority_prestart(77, 0); hmi.choose(Task::Task2); hmi.confirm_selection(1);
    assert(!hmi.receive_car_start(77, 2));
    assert(hmi.view().state == HmiState::Fault);
    hmi.reset();
    assert(hmi.view().state == HmiState::BootLocked && hmi.view().controls_locked);
    assert(!hmi.receive_car_start(77, 3));
}

void test_stale_status_is_visible() {
    HmiStateMachine hmi;
    hmi.authority_prestart(88, 0); hmi.update_ap(true, 0); hmi.update_car(0); hmi.update_ros(0); hmi.update_vision(0); hmi.tick(751);
    assert(!hmi.view().ap_connected && !hmi.view().car_fresh && !hmi.view().ros_fresh && !hmi.view().vision_fresh);
    assert(hmi.view().ap_age_ms == 751 && hmi.view().car_age_ms == 751);
}

int main() {
    test_happy_selection_and_lock();
    test_pending_and_reboot_stay_locked();
    test_stale_status_is_visible();
    return 0;
}
