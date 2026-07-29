#pragma once

#include "ed_shared_protocol/codec.hpp"

namespace ed::shared_protocol {

constexpr std::size_t kTelemetryVehicleIdBytes = 32;
constexpr std::size_t kTelemetryFrameIdBytes = 32;
constexpr std::size_t kMissionIdBytes = 64;
constexpr std::size_t kProfileIdBytes = 64;
constexpr std::size_t kPresetIdBytes = 64;
constexpr std::size_t kRevisionBytes = 32;
constexpr std::size_t kReasonBytes = 96;

struct TextBuffer {
    char value[97]{};
    std::uint8_t length = 0;
};

enum class MotionKind : std::uint8_t { Displacement = 1, WheelSpeed = 2 };
enum class TurnClass : std::uint8_t { Straight = 0, Small = 1, Large = 2 };
enum class RouteStage : std::uint8_t { Start = 0, B = 1, D = 2, A = 3, Complete = 4 };
enum class DTask : std::uint8_t { PayloadDrop = 1, DynamicLanding = 2 };
enum class AuthorityState : std::uint8_t {
    BootLocked = 0,
    Prestart = 1,
    SelectPending = 2,
    Selected = 3,
    ArmedReady = 4,
    CarRunning = 5,
    Fault = 6,
};

struct TelemetryPayload {
    std::uint16_t contract_version = 1;
    bool start_event = false;
    bool heartbeat_alive = false;
    bool lap_complete = false;
    MotionKind motion_kind = MotionKind::Displacement;
    float displacement_m = 0.0F;
    float wheel_speed_m_s = 0.0F;
    TurnClass turn_class = TurnClass::Straight;
    RouteStage route_stage = RouteStage::Start;
    TextBuffer vehicle_id;
    TextBuffer frame_id;
};

struct SelectionPayload {
    std::uint16_t contract_version = 1;
    std::uint64_t selection_id = 0;
    std::uint64_t car_boot_epoch = 0;
    DTask task = DTask::PayloadDrop;
    TextBuffer mission_id;
    TextBuffer mission_profile_id;
    TextBuffer deployment_preset_id;
    TextBuffer target_revision;
};

struct AckPayload {
    std::uint16_t contract_version = 1;
    std::uint64_t selection_id = 0;
    std::uint64_t car_boot_epoch = 0;
    bool accepted = false;
    AuthorityState state = AuthorityState::BootLocked;
    TextBuffer reason;
};

struct StatusPayload {
    std::uint16_t contract_version = 1;
    std::uint32_t source_sequence = 0;
    std::uint8_t state = 0;
    RouteStage route_stage = RouteStage::Start;
    bool complete = false;
    TextBuffer mission_id;
    TextBuffer reason;
};

enum class PayloadError : std::uint8_t { Ok, BufferTooSmall, Invalid, TrailingBytes };

bool set_text(TextBuffer& destination, const char* value);
PayloadError encode_telemetry(const TelemetryPayload& value, Frame& frame);
PayloadError decode_telemetry(const Frame& frame, TelemetryPayload& value);
PayloadError encode_selection(const SelectionPayload& value, Frame& frame);
PayloadError decode_selection(const Frame& frame, SelectionPayload& value);
PayloadError encode_ack(const AckPayload& value, Frame& frame);
PayloadError decode_ack(const Frame& frame, AckPayload& value);
PayloadError encode_status(const StatusPayload& value, Frame& frame);
PayloadError decode_status(const Frame& frame, StatusPayload& value);

}  // namespace ed::shared_protocol
