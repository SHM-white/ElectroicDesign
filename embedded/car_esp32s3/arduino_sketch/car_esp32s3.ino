#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstring>
#include <esp_system.h>

#include <ed_shared_protocol/codec.hpp>
#include <ed_shared_protocol/payloads.hpp>

#include <car_esp32s3/controller.hpp>

#if __has_include("config_local.h")
#include "config_local.h"
#else
#define ED_CAR_CONFIG_READY 0
#define ED_WIFI_SSID ""
#define ED_WIFI_PASSWORD ""
#define ED_CAR_SENDER_ID "CAR-01"
#define ED_ROS_PEER_IP "0.0.0.0"
#define ED_ROS_PEER_PORT 0
#define ED_HMAC_KEY_HEX ""
#endif

namespace {

class UnwiredLineSensors final : public ed::car::LineSensors {
public:
    ed::car::LineSample sample() override { return {}; }
};
class UnwiredMotorDriver final : public ed::car::MotorDriver {
public:
    void drive(float, float) override {}
    void brake() override {}
    bool faulted() const override { return false; }
};
class UnwiredEncoders final : public ed::car::Encoders {
public:
    ed::car::EncoderSample sample() override { return {}; }
};
class UnwiredStartButton final : public ed::car::StartButton {
public:
    bool pressed() override { return false; }
};
class UnwiredHealth final : public ed::car::HealthMonitor {
public:
    bool brownout() const override { return false; }
};

class UdpTelemetry final : public ed::car::TelemetrySink {
public:
    void begin() {
#if ED_CAR_CONFIG_READY
        udp_.begin(0);
        for (std::size_t index = 0; index < sizeof(key_); ++index) {
            const int high = hex_digit(ED_HMAC_KEY_HEX[index * 2]);
            const int low = hex_digit(ED_HMAC_KEY_HEX[index * 2 + 1]);
            if (high < 0 || low < 0) { configured_ = false; return; }
            key_[index] = static_cast<std::uint8_t>((high << 4) | low);
        }
#endif
    }
    void publish(const ed::car::TelemetrySnapshot& snapshot) override {
#if ED_CAR_CONFIG_READY
        if (!configured_) return;
        ed::shared_protocol::TelemetryPayload payload;
        payload.start_event = snapshot.start_event;
        payload.heartbeat_alive = snapshot.heartbeat_alive;
        payload.lap_complete = snapshot.lap_complete;
        payload.turn_class = static_cast<ed::shared_protocol::TurnClass>(snapshot.turn_class);
        payload.route_stage = static_cast<ed::shared_protocol::RouteStage>(snapshot.route_stage);
        payload.displacement_m = snapshot.displacement_m;
        payload.wheel_speed_m_s = snapshot.wheel_speed_m_s;
        ed::shared_protocol::set_text(payload.vehicle_id, ED_CAR_SENDER_ID);
        ed::shared_protocol::set_text(payload.frame_id, "telemetry");
        ed::shared_protocol::Frame frame;
        if (ed::shared_protocol::encode_telemetry(payload, frame) != ed::shared_protocol::PayloadError::Ok) return;
        frame.boot_epoch = boot_epoch_;
        frame.sequence = sequence_++;
        frame.source_millis = millis();
        strncpy(frame.sender_id, ED_CAR_SENDER_ID, sizeof(frame.sender_id) - 1);
        std::uint8_t packet[ed::shared_protocol::kMaximumDatagramBytes]{};
        std::size_t packet_length = 0;
        if (ed::shared_protocol::encode_frame(frame, key_, sizeof(key_), packet, sizeof(packet), packet_length) != ed::shared_protocol::CodecError::Ok) return;
        udp_.beginPacket(ED_ROS_PEER_IP, ED_ROS_PEER_PORT);
        udp_.write(packet, packet_length);
        udp_.endPacket();
#else
        (void)snapshot;
#endif
    }
    void set_epoch(std::uint64_t epoch) { boot_epoch_ = epoch == 0 ? 1 : epoch; }
private:
    static int hex_digit(char value) {
        if (value >= '0' && value <= '9') return value - '0';
        if (value >= 'a' && value <= 'f') return value - 'a' + 10;
        if (value >= 'A' && value <= 'F') return value - 'A' + 10;
        return -1;
    }
    WiFiUDP udp_;
    std::uint8_t key_[32]{};
    std::uint64_t boot_epoch_ = 1;
    std::uint32_t sequence_ = 1;
    bool configured_ = true;
};

UnwiredLineSensors line_sensors;
UnwiredMotorDriver motor;
UnwiredEncoders encoders;
UnwiredStartButton start_button;
UnwiredHealth health;
UdpTelemetry telemetry;
ed::car::CarController* controller = nullptr;

}  // namespace

void setup() {
    Serial.begin(115200);
    telemetry.set_epoch((static_cast<std::uint64_t>(esp_random()) << 32) | esp_random());
    telemetry.begin();
    static ed::car::CarController car({line_sensors, motor, encoders, start_button, health, telemetry});
    controller = &car;
#if ED_CAR_CONFIG_READY
    WiFi.mode(WIFI_STA);
    WiFi.begin(ED_WIFI_SSID, ED_WIFI_PASSWORD);
#endif
}

void loop() {
    if (controller == nullptr) return;
    controller->tick(millis(), WiFi.status() == WL_CONNECTED);
}
