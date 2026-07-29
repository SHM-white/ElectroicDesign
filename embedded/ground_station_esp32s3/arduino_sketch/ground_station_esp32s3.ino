#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstring>
#include <esp_system.h>

#include <ed_shared_protocol/codec.hpp>
#include <ed_shared_protocol/payloads.hpp>
#include <ground_station_arduino/hmi_state.hpp>

#if __has_include("config_local.h")
#include "config_local.h"
#else
#define ED_STATION_CONFIG_READY 0
#define ED_WIFI_SSID ""
#define ED_WIFI_PASSWORD ""
#define ED_ROS_PEER_IP "0.0.0.0"
#define ED_ROS_PEER_PORT 0
#define ED_LISTEN_PORT 0
#define ED_HMAC_KEY_HEX ""
#endif

namespace {

class DisplayPort {
public:
    virtual ~DisplayPort() = default;
    virtual void render(const ed::ground_station::HmiView& view) = 0;
};

class TouchPort {
public:
    virtual ~TouchPort() = default;
    virtual bool task1_released() = 0;
    virtual bool task2_released() = 0;
    virtual bool confirm_released() = 0;
};

class SerialPreviewDisplay final : public DisplayPort {
public:
    void render(const ed::ground_station::HmiView& view) override {
        Serial.printf("HMI state=%u task=%u locked=%u AP age=%lu CAR age=%lu ROS age=%lu VISION age=%lu reason=%s\n",
                      static_cast<unsigned>(view.state), static_cast<unsigned>(view.selected_task),
                      view.controls_locked, static_cast<unsigned long>(view.ap_age_ms),
                      static_cast<unsigned long>(view.car_age_ms), static_cast<unsigned long>(view.ros_age_ms),
                      static_cast<unsigned long>(view.vision_age_ms), view.reason);
    }
};

class UnwiredTouch final : public TouchPort {
public:
    bool task1_released() override { return false; }
    bool task2_released() override { return false; }
    bool confirm_released() override { return false; }
};

class UdpBridge final {
public:
    void begin() {
#if ED_STATION_CONFIG_READY
        udp_.begin(ED_LISTEN_PORT);
        for (std::size_t index = 0; index < sizeof(key_); ++index) {
            const int high = hex_digit(ED_HMAC_KEY_HEX[index * 2]);
            const int low = hex_digit(ED_HMAC_KEY_HEX[index * 2 + 1]);
            if (high < 0 || low < 0) { configured_ = false; return; }
            key_[index] = static_cast<std::uint8_t>((high << 4) | low);
        }
#endif
    }
    void send_selection(const ed::ground_station::HmiView& view) {
#if ED_STATION_CONFIG_READY
        if (!configured_ || view.selected_task == ed::ground_station::Task::None) return;
        ed::shared_protocol::SelectionPayload payload;
        payload.selection_id = view.selection_id;
        payload.car_boot_epoch = view.car_boot_epoch;
        payload.task = view.selected_task == ed::ground_station::Task::Task1 ? ed::shared_protocol::DTask::PayloadDrop : ed::shared_protocol::DTask::DynamicLanding;
        if (!ed::shared_protocol::set_text(payload.mission_id, ED_MISSION_ID) || !ed::shared_protocol::set_text(payload.mission_profile_id, ED_MISSION_PROFILE_ID) || !ed::shared_protocol::set_text(payload.deployment_preset_id, ED_DEPLOYMENT_PRESET_ID) || !ed::shared_protocol::set_text(payload.target_revision, ED_TARGET_REVISION)) return;
        ed::shared_protocol::Frame frame;
        if (ed::shared_protocol::encode_selection(payload, frame) != ed::shared_protocol::PayloadError::Ok) return;
        frame.boot_epoch = boot_epoch_; frame.sequence = sequence_++; frame.source_millis = millis();
        strncpy(frame.sender_id, ED_STATION_SENDER_ID, sizeof(frame.sender_id) - 1);
        std::uint8_t packet[ed::shared_protocol::kMaximumDatagramBytes]{}; std::size_t packet_length = 0;
        if (ed::shared_protocol::encode_frame(frame, key_, sizeof(key_), packet, sizeof(packet), packet_length) != ed::shared_protocol::CodecError::Ok) return;
        udp_.beginPacket(ED_ROS_PEER_IP, ED_ROS_PEER_PORT); udp_.write(packet, packet_length); udp_.endPacket();
#else
        (void)view;
#endif
    }
    void poll(ed::ground_station::HmiStateMachine& hmi, std::uint32_t now_ms) {
#if ED_STATION_CONFIG_READY
        const int packet_size = udp_.parsePacket();
        if (packet_size <= 0 || packet_size > static_cast<int>(ed::shared_protocol::kMaximumDatagramBytes)) return;
        std::uint8_t packet[ed::shared_protocol::kMaximumDatagramBytes]{};
        const int received = udp_.read(packet, sizeof(packet));
        ed::shared_protocol::Frame frame;
        if (received <= 0 || ed::shared_protocol::decode_frame(packet, static_cast<std::size_t>(received), key_, sizeof(key_), frame) != ed::shared_protocol::CodecError::Ok) return;
        if (frame.message_type == ed::shared_protocol::MessageType::SelectionAck) {
            ed::shared_protocol::AckPayload ack;
            if (ed::shared_protocol::decode_ack(frame, ack) == ed::shared_protocol::PayloadError::Ok) hmi.receive_ack(ack.selection_id, ack.car_boot_epoch, ack.accepted, static_cast<ed::ground_station::HmiState>(ack.state), ack.reason.value);
        } else if (frame.message_type == ed::shared_protocol::MessageType::MissionStatus) {
            ed::shared_protocol::StatusPayload status;
            if (ed::shared_protocol::decode_status(frame, status) == ed::shared_protocol::PayloadError::Ok) hmi.receive_status(static_cast<std::uint8_t>(status.route_stage), now_ms);
        }
#else
        (void)hmi; (void)now_ms;
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

ed::ground_station::HmiStateMachine hmi;
SerialPreviewDisplay display;
UnwiredTouch touch;
UdpBridge bridge;

}  // namespace

void setup() {
    Serial.begin(115200);
    hmi.reset();
    bridge.set_epoch((static_cast<std::uint64_t>(esp_random()) << 32) | esp_random());
    bridge.begin();
#if ED_STATION_CONFIG_READY
    WiFi.mode(WIFI_STA);
    WiFi.begin(ED_WIFI_SSID, ED_WIFI_PASSWORD);
#endif
}

void loop() {
    const std::uint32_t now = millis();
    hmi.update_ap(WiFi.status() == WL_CONNECTED, now);
    bridge.poll(hmi, now);
    if (touch.task1_released()) hmi.choose(ed::ground_station::Task::Task1);
    if (touch.task2_released()) hmi.choose(ed::ground_station::Task::Task2);
    if (touch.confirm_released() && hmi.confirm_selection(now)) bridge.send_selection(hmi.view());
    hmi.tick(now);
    display.render(hmi.view());
    delay(50);
}
