#include "ed_shared_protocol/payloads.hpp"

#include <cmath>
#include <cstring>

namespace ed::shared_protocol {
namespace {

class Writer {
public:
    explicit Writer(Frame& frame) : frame_(frame) {}
    bool bytes(const std::uint8_t* data, std::size_t length) {
        if (offset_ + length > kMaxPayloadBytes) return false;
        std::memcpy(frame_.payload + offset_, data, length);
        offset_ += length;
        return true;
    }
    bool u8(std::uint8_t value) { return bytes(&value, 1); }
    bool u16(std::uint16_t value) { const std::uint8_t b[] = {static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value)}; return bytes(b, 2); }
    bool u32(std::uint32_t value) { const std::uint8_t b[] = {static_cast<std::uint8_t>(value >> 24), static_cast<std::uint8_t>(value >> 16), static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value)}; return bytes(b, 4); }
    bool u64(std::uint64_t value) { std::uint8_t b[8]{}; for (int shift = 56, index = 0; shift >= 0; shift -= 8, ++index) b[index] = static_cast<std::uint8_t>(value >> shift); return bytes(b, 8); }
    bool text(const TextBuffer& value, std::size_t maximum) { return value.length >= 1 && value.length <= maximum && u8(value.length) && bytes(reinterpret_cast<const std::uint8_t*>(value.value), value.length); }
    std::size_t finish() { frame_.payload_length = static_cast<std::uint16_t>(offset_); return offset_; }
private:
    Frame& frame_;
    std::size_t offset_ = 0;
};

class Reader {
public:
    explicit Reader(const Frame& frame) : frame_(frame) {}
    bool bytes(std::uint8_t* output, std::size_t length) { if (offset_ + length > frame_.payload_length) return false; std::memcpy(output, frame_.payload + offset_, length); offset_ += length; return true; }
    bool u8(std::uint8_t& value) { return bytes(&value, 1); }
    bool u16(std::uint16_t& value) { std::uint8_t b[2]{}; if (!bytes(b, 2)) return false; value = static_cast<std::uint16_t>((b[0] << 8) | b[1]); return true; }
    bool u32(std::uint32_t& value) { std::uint8_t b[4]{}; if (!bytes(b, 4)) return false; value = (static_cast<std::uint32_t>(b[0]) << 24) | (static_cast<std::uint32_t>(b[1]) << 16) | (static_cast<std::uint32_t>(b[2]) << 8) | b[3]; return true; }
    bool u64(std::uint64_t& value) { std::uint8_t b[8]{}; if (!bytes(b, 8)) return false; value = 0; for (std::uint8_t byte : b) value = (value << 8) | byte; return true; }
    bool text(TextBuffer& output, std::size_t maximum) { std::uint8_t length = 0; if (!u8(length) || length < 1 || length > maximum || length > sizeof(output.value) - 1) return false; if (!bytes(reinterpret_cast<std::uint8_t*>(output.value), length)) return false; output.value[length] = '\0'; output.length = length; return true; }
    bool done() const { return offset_ == frame_.payload_length; }
private:
    const Frame& frame_;
    std::size_t offset_ = 0;
};

std::uint32_t float_bits(float value) { std::uint32_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); return bits; }
float bits_float(std::uint32_t bits) { float value = 0.0F; std::memcpy(&value, &bits, sizeof(value)); return value; }

bool valid_version(std::uint16_t version) { return version == 1; }
bool finite(float value) { return std::isfinite(value); }

}  // namespace

bool set_text(TextBuffer& destination, const char* value) {
    if (value == nullptr) return false;
    const std::size_t length = std::strlen(value);
    if (length == 0 || length > sizeof(destination.value) - 1) return false;
    std::memcpy(destination.value, value, length + 1);
    destination.length = static_cast<std::uint8_t>(length);
    return true;
}

PayloadError encode_telemetry(const TelemetryPayload& value, Frame& frame) {
    if (!valid_version(value.contract_version) || !finite(value.displacement_m) || !finite(value.wheel_speed_m_s)) return PayloadError::Invalid;
    frame.message_type = MessageType::CarTelemetry;
    Writer writer(frame);
    const std::uint8_t flags = static_cast<std::uint8_t>(value.start_event) | static_cast<std::uint8_t>(value.heartbeat_alive << 1) | static_cast<std::uint8_t>(value.lap_complete << 2);
    const bool ok = writer.u16(value.contract_version) && writer.u8(flags) && writer.u8(static_cast<std::uint8_t>(value.motion_kind)) && writer.u32(float_bits(value.displacement_m)) && writer.u32(float_bits(value.wheel_speed_m_s)) && writer.u8(static_cast<std::uint8_t>(value.turn_class)) && writer.u8(static_cast<std::uint8_t>(value.route_stage)) && writer.text(value.vehicle_id, kTelemetryVehicleIdBytes) && writer.text(value.frame_id, kTelemetryFrameIdBytes);
    writer.finish();
    return ok ? PayloadError::Ok : PayloadError::BufferTooSmall;
}

PayloadError decode_telemetry(const Frame& frame, TelemetryPayload& value) {
    if (frame.message_type != MessageType::CarTelemetry) return PayloadError::Invalid;
    Reader reader(frame); std::uint8_t flags = 0, motion = 0, turn = 0, route = 0; std::uint16_t version = 0; std::uint32_t displacement = 0, speed = 0;
    if (!reader.u16(version) || !reader.u8(flags) || !reader.u8(motion) || !reader.u32(displacement) || !reader.u32(speed) || !reader.u8(turn) || !reader.u8(route) || !reader.text(value.vehicle_id, kTelemetryVehicleIdBytes) || !reader.text(value.frame_id, kTelemetryFrameIdBytes) || !reader.done() || !valid_version(version) || (flags & ~0x07U) != 0 || motion < 1 || motion > 2 || turn > 2 || route > 4) return PayloadError::Invalid;
    value.contract_version = version; value.start_event = (flags & 1) != 0; value.heartbeat_alive = (flags & 2) != 0; value.lap_complete = (flags & 4) != 0; value.motion_kind = static_cast<MotionKind>(motion); value.displacement_m = bits_float(displacement); value.wheel_speed_m_s = bits_float(speed); value.turn_class = static_cast<TurnClass>(turn); value.route_stage = static_cast<RouteStage>(route);
    return finite(value.displacement_m) && finite(value.wheel_speed_m_s) ? PayloadError::Ok : PayloadError::Invalid;
}

PayloadError encode_selection(const SelectionPayload& value, Frame& frame) {
    if (!valid_version(value.contract_version) || value.selection_id == 0 || value.car_boot_epoch == 0) return PayloadError::Invalid;
    frame.message_type = MessageType::HmiSelection; Writer writer(frame);
    const bool ok = writer.u16(value.contract_version) && writer.u64(value.selection_id) && writer.u64(value.car_boot_epoch) && writer.u8(static_cast<std::uint8_t>(value.task)) && writer.text(value.mission_id, kMissionIdBytes) && writer.text(value.mission_profile_id, kProfileIdBytes) && writer.text(value.deployment_preset_id, kPresetIdBytes) && writer.text(value.target_revision, kRevisionBytes);
    writer.finish(); return ok ? PayloadError::Ok : PayloadError::BufferTooSmall;
}

PayloadError decode_selection(const Frame& frame, SelectionPayload& value) {
    if (frame.message_type != MessageType::HmiSelection) return PayloadError::Invalid;
    Reader reader(frame); std::uint16_t version = 0; std::uint8_t task = 0;
    if (!reader.u16(version) || !reader.u64(value.selection_id) || !reader.u64(value.car_boot_epoch) || !reader.u8(task) || !reader.text(value.mission_id, kMissionIdBytes) || !reader.text(value.mission_profile_id, kProfileIdBytes) || !reader.text(value.deployment_preset_id, kPresetIdBytes) || !reader.text(value.target_revision, kRevisionBytes) || !reader.done() || !valid_version(version) || value.selection_id == 0 || value.car_boot_epoch == 0 || task < 1 || task > 2) return PayloadError::Invalid;
    value.contract_version = version; value.task = static_cast<DTask>(task); return PayloadError::Ok;
}

PayloadError encode_ack(const AckPayload& value, Frame& frame) {
    if (!valid_version(value.contract_version)) return PayloadError::Invalid;
    frame.message_type = MessageType::SelectionAck; Writer writer(frame);
    const bool ok = writer.u16(value.contract_version) && writer.u64(value.selection_id) && writer.u64(value.car_boot_epoch) && writer.u8(static_cast<std::uint8_t>(value.accepted)) && writer.u8(static_cast<std::uint8_t>(value.state)) && writer.u8(0) && writer.text(value.reason, kReasonBytes);
    writer.finish(); return ok ? PayloadError::Ok : PayloadError::BufferTooSmall;
}

PayloadError decode_ack(const Frame& frame, AckPayload& value) {
    if (frame.message_type != MessageType::SelectionAck) return PayloadError::Invalid;
    Reader reader(frame); std::uint16_t version = 0; std::uint8_t accepted = 0, state = 0, reserved = 0;
    if (!reader.u16(version) || !reader.u64(value.selection_id) || !reader.u64(value.car_boot_epoch) || !reader.u8(accepted) || !reader.u8(state) || !reader.u8(reserved) || !reader.text(value.reason, kReasonBytes) || !reader.done() || !valid_version(version) || accepted > 1 || state > 6 || reserved != 0) return PayloadError::Invalid;
    value.contract_version = version; value.accepted = accepted != 0; value.state = static_cast<AuthorityState>(state); return PayloadError::Ok;
}

PayloadError encode_status(const StatusPayload& value, Frame& frame) {
    if (!valid_version(value.contract_version) || value.state > 10) return PayloadError::Invalid;
    frame.message_type = MessageType::MissionStatus; Writer writer(frame);
    const bool ok = writer.u16(value.contract_version) && writer.u32(value.source_sequence) && writer.u8(value.state) && writer.u8(static_cast<std::uint8_t>(value.route_stage)) && writer.u8(static_cast<std::uint8_t>(value.complete)) && writer.text(value.mission_id, kMissionIdBytes) && writer.text(value.reason, kReasonBytes);
    writer.finish(); return ok ? PayloadError::Ok : PayloadError::BufferTooSmall;
}

PayloadError decode_status(const Frame& frame, StatusPayload& value) {
    if (frame.message_type != MessageType::MissionStatus) return PayloadError::Invalid;
    Reader reader(frame); std::uint16_t version = 0; std::uint8_t route = 0, complete = 0;
    if (!reader.u16(version) || !reader.u32(value.source_sequence) || !reader.u8(value.state) || !reader.u8(route) || !reader.u8(complete) || !reader.text(value.mission_id, kMissionIdBytes) || !reader.text(value.reason, kReasonBytes) || !reader.done() || !valid_version(version) || value.state > 10 || route > 4 || complete > 1) return PayloadError::Invalid;
    value.contract_version = version; value.route_stage = static_cast<RouteStage>(route); value.complete = complete != 0; return PayloadError::Ok;
}

}  // namespace ed::shared_protocol
