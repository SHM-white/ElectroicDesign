#include "car_esp32s3/controller.hpp"

#include <cassert>

using namespace ed::car;
using ed::shared_protocol::RouteStage;

namespace {

struct FakeLine : LineSensors { LineSample value{true, 0.1F, 100}; LineSample sample() override { return value; } };
struct FakeMotor : MotorDriver { float left = 0.0F; float right = 0.0F; bool stopped = false; bool failed = false; void drive(float l, float r) override { left = l; right = r; } void brake() override { stopped = true; } bool faulted() const override { return failed; } };
struct FakeEncoders : Encoders { EncoderSample value{true, 0.0F, 0.0F}; EncoderSample sample() override { return value; } };
struct FakeButton : StartButton { bool value = false; bool pressed() override { return value; } };
struct FakeHealth : HealthMonitor { bool value = false; bool brownout() const override { return value; } };
struct FakeTelemetry : TelemetrySink { TelemetrySnapshot last{}; int count = 0; void publish(const TelemetrySnapshot& value) override { last = value; ++count; } };

void test_local_run_and_route() {
    FakeLine line; FakeMotor motor; FakeEncoders encoders; FakeButton button; FakeHealth health; FakeTelemetry telemetry;
    CarController car({line, motor, encoders, button, health, telemetry});
    car.tick(0, true);
    button.value = true; car.tick(50, true);
    assert(car.state() == CarState::Running && telemetry.last.start_event);
    button.value = false; car.tick(100, true);
    assert(motor.left != 0.0F && motor.right != 0.0F);
    assert(car.mark_route_event(RouteStage::B));
    assert(car.mark_route_event(RouteStage::D));
    assert(car.mark_route_event(RouteStage::A));
    assert(car.mark_route_event(RouteStage::Complete));
    assert(!car.mark_route_event(RouteStage::B));
}

void test_wifi_fault_latches_until_reset() {
    FakeLine line; FakeMotor motor; FakeEncoders encoders; FakeButton button; FakeHealth health; FakeTelemetry telemetry;
    CarController car({line, motor, encoders, button, health, telemetry});
    button.value = true; car.tick(0, true); button.value = false; car.tick(50, true);
    car.tick(100, false);
    for (std::uint32_t now = 200; now <= 1100; now += 100) car.tick(now, false);
    car.tick(1101, false);
    assert(car.state() == CarState::SafeStop && car.fault_reason() == Fault::WifiTimeout && motor.stopped);
    button.value = true; car.tick(1200, true);
    assert(car.state() == CarState::SafeStop);
    car.physical_reset();
    assert(car.state() == CarState::Ready);
    button.value = false;
    car.tick(1300, true);
    assert(car.state() == CarState::Ready);
    button.value = true; car.tick(1350, true);
    assert(car.state() == CarState::Running);
}

void test_sensor_fault() {
    FakeLine line; FakeMotor motor; FakeEncoders encoders; FakeButton button; FakeHealth health; FakeTelemetry telemetry;
    CarController car({line, motor, encoders, button, health, telemetry});
    button.value = true; car.tick(0, true); button.value = false; line.value.valid = false; car.tick(50, true);
    assert(car.state() == CarState::SafeStop && car.fault_reason() == Fault::MissedLine);
}

}  // namespace

int main() {
    test_local_run_and_route();
    test_wifi_fault_latches_until_reset();
    test_sensor_fault();
    return 0;
}
