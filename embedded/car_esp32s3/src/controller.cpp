#include "car_esp32s3/controller.hpp"

#include <cmath>

namespace ed::car {

CarController::CarController(CarPorts ports, ControllerConfig config) : ports_(ports), config_(config) {}

void CarController::tick(std::uint32_t now_ms, bool wifi_connected) {
    const std::uint32_t elapsed = now_ms - last_tick_ms_;
    if (last_tick_ms_ != 0 && elapsed > 100) {
        stop(Fault::PidOverrun);
        publish(now_ms);
        return;
    }
    last_tick_ms_ = now_ms;
    const bool button_down = ports_.start_button.pressed();
    if (button_down && !button_was_down_) button_down_ms_ = now_ms;
    if (button_down && now_ms - button_down_ms_ > config_.stuck_button_ms) stop(Fault::StuckButton);
    const bool rising_edge = button_down && !button_was_down_;
    button_was_down_ = button_down;

    if (ports_.health.brownout()) stop(Fault::Brownout);
    if (ports_.motor.faulted()) stop(Fault::MotorFault);
    if (!wifi_connected) {
        if (wifi_lost_ms_ == 0) wifi_lost_ms_ = now_ms;
        if (now_ms - wifi_lost_ms_ > config_.wifi_grace_ms) stop(Fault::WifiTimeout);
    } else {
        wifi_lost_ms_ = 0;
    }

    if (state_ == CarState::Ready && rising_edge && fault_ == Fault::None) {
        if (!route_.accept(true, ed::shared_protocol::RouteStage::Start, false)) stop(Fault::InvalidRoute);
        else { state_ = CarState::Running; start_event_pending_ = true; }
    }
    if (state_ == CarState::Running) {
        const LineSample line = ports_.line_sensors.sample();
        const EncoderSample encoders = ports_.encoders.sample();
        if (!line.valid) stop(Fault::MissedLine);
        if (!encoders.valid || std::fabs(encoders.left_m - encoders.right_m) > config_.encoder_disagreement_m) stop(Fault::EncoderDisagreement);
        if (state_ == CarState::Running) {
            const float derivative = elapsed == 0 ? 0.0F : (line.lateral_error - previous_error_) / (elapsed / 1000.0F);
            const float correction = config_.kp * line.lateral_error + config_.kd * derivative;
            ports_.motor.drive(config_.base_speed - correction, config_.base_speed + correction);
            previous_error_ = line.lateral_error;
            displacement_m_ = (encoders.left_m + encoders.right_m) * 0.5F;
            speed_m_s_ = elapsed == 0 ? speed_m_s_ : (displacement_m_ - previous_displacement_m_) / (elapsed / 1000.0F);
            previous_displacement_m_ = displacement_m_;
        }
    }
    if (now_ms - last_telemetry_ms_ >= config_.telemetry_period_ms) publish(now_ms);
}

bool CarController::mark_route_event(ed::shared_protocol::RouteStage stage) {
    const bool complete = stage == ed::shared_protocol::RouteStage::Complete;
    if (!route_.accept(false, stage, complete)) { stop(Fault::InvalidRoute); return false; }
    return true;
}

void CarController::physical_reset() {
    ports_.motor.brake();
    state_ = CarState::Ready; fault_ = Fault::None; route_.reset();
    last_tick_ms_ = 0; last_telemetry_ms_ = 0; wifi_lost_ms_ = 0; button_down_ms_ = 0; previous_error_ = 0.0F; previous_displacement_m_ = 0.0F; displacement_m_ = 0.0F; speed_m_s_ = 0.0F; button_was_down_ = false; start_event_pending_ = false;
}

void CarController::fault(Fault reason) { stop(reason); }

void CarController::stop(Fault reason) {
    if (state_ == CarState::SafeStop) return;
    state_ = CarState::SafeStop; fault_ = reason; ports_.motor.brake();
}

void CarController::publish(std::uint32_t now_ms) {
    TelemetrySnapshot snapshot;
    snapshot.start_event = start_event_pending_;
    snapshot.heartbeat_alive = state_ != CarState::SafeStop;
    snapshot.lap_complete = route_.stage() == ed::shared_protocol::RouteStage::Complete;
    snapshot.turn_class = classify_turn(previous_error_);
    snapshot.route_stage = static_cast<std::uint8_t>(route_.stage());
    snapshot.displacement_m = displacement_m_;
    snapshot.wheel_speed_m_s = speed_m_s_;
    ports_.telemetry.publish(snapshot);
    last_telemetry_ms_ = now_ms; start_event_pending_ = false;
}

std::uint8_t CarController::classify_turn(float error) const {
    const float magnitude = std::fabs(error);
    if (magnitude >= config_.large_turn_error) return static_cast<std::uint8_t>(ed::shared_protocol::TurnClass::Large);
    if (magnitude >= config_.small_turn_error) return static_cast<std::uint8_t>(ed::shared_protocol::TurnClass::Small);
    return static_cast<std::uint8_t>(ed::shared_protocol::TurnClass::Straight);
}

}  // namespace ed::car
