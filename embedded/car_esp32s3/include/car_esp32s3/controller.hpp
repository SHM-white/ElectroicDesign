#pragma once

#include "car_esp32s3/ports.hpp"
#include "ed_shared_protocol/session.hpp"

namespace ed::car {

enum class CarState : std::uint8_t { Ready, Running, SafeStop };
enum class Fault : std::uint8_t { None, WifiTimeout, MissedLine, EncoderDisagreement, Brownout, MotorFault, StuckButton, PidOverrun, InvalidRoute };

struct ControllerConfig {
    std::uint32_t telemetry_period_ms = 50;
    std::uint32_t wifi_grace_ms = 1000;
    std::uint32_t stuck_button_ms = 1500;
    float encoder_disagreement_m = 0.10F;
    float small_turn_error = 0.22F;
    float large_turn_error = 0.55F;
    float kp = 0.85F;
    float kd = 0.10F;
    float base_speed = 0.35F;
};

class CarController {
public:
    CarController(CarPorts ports, ControllerConfig config = {});

    void tick(std::uint32_t now_ms, bool wifi_connected);
    bool mark_route_event(ed::shared_protocol::RouteStage stage);
    void physical_reset();
    void fault(Fault reason);
    CarState state() const { return state_; }
    Fault fault_reason() const { return fault_; }
    ed::shared_protocol::RouteStage route_stage() const { return route_.stage(); }

private:
    void stop(Fault reason);
    void publish(std::uint32_t now_ms);
    std::uint8_t classify_turn(float error) const;

    CarPorts ports_;
    ControllerConfig config_;
    CarState state_ = CarState::Ready;
    Fault fault_ = Fault::None;
    ed::shared_protocol::RouteValidator route_;
    std::uint32_t last_tick_ms_ = 0;
    std::uint32_t last_telemetry_ms_ = 0;
    std::uint32_t wifi_lost_ms_ = 0;
    std::uint32_t button_down_ms_ = 0;
    float previous_error_ = 0.0F;
    float previous_displacement_m_ = 0.0F;
    float displacement_m_ = 0.0F;
    float speed_m_s_ = 0.0F;
    bool button_was_down_ = false;
    bool start_event_pending_ = false;
};

}  // namespace ed::car
