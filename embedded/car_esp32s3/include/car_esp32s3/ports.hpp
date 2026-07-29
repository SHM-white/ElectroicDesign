#pragma once

#include <cstdint>

namespace ed::car {

struct LineSample {
    bool valid = false;
    float lateral_error = 0.0F;
    std::uint8_t quality = 0;
};

struct EncoderSample {
    bool valid = false;
    float left_m = 0.0F;
    float right_m = 0.0F;
};

class LineSensors {
public:
    virtual ~LineSensors() = default;
    virtual LineSample sample() = 0;
};

class MotorDriver {
public:
    virtual ~MotorDriver() = default;
    virtual void drive(float left, float right) = 0;
    virtual void brake() = 0;
    virtual bool faulted() const = 0;
};

class Encoders {
public:
    virtual ~Encoders() = default;
    virtual EncoderSample sample() = 0;
};

class StartButton {
public:
    virtual ~StartButton() = default;
    virtual bool pressed() = 0;
};

class HealthMonitor {
public:
    virtual ~HealthMonitor() = default;
    virtual bool brownout() const = 0;
};

struct TelemetrySnapshot {
    bool start_event = false;
    bool heartbeat_alive = false;
    bool lap_complete = false;
    std::uint8_t turn_class = 0;
    std::uint8_t route_stage = 0;
    float displacement_m = 0.0F;
    float wheel_speed_m_s = 0.0F;
};

class TelemetrySink {
public:
    virtual ~TelemetrySink() = default;
    virtual void publish(const TelemetrySnapshot& snapshot) = 0;
};

struct CarPorts {
    LineSensors& line_sensors;
    MotorDriver& motor;
    Encoders& encoders;
    StartButton& start_button;
    HealthMonitor& health;
    TelemetrySink& telemetry;
};

}  // namespace ed::car
