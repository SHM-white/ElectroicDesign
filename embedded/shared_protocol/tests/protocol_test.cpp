#include "ed_shared_protocol/codec.hpp"
#include "ed_shared_protocol/payloads.hpp"
#include "ed_shared_protocol/session.hpp"

#include <array>
#include <cassert>
#include <cstring>

using namespace ed::shared_protocol;

namespace {

constexpr std::array<std::uint8_t, 32> kKey = [] {
    std::array<std::uint8_t, 32> value{};
    for (std::size_t index = 0; index < value.size(); ++index) value[index] = static_cast<std::uint8_t>(index);
    return value;
}();

Frame golden_frame() {
    Frame frame;
    frame.message_type = MessageType::CarTelemetry;
    std::memcpy(frame.sender_id, "CAR-01", 6);
    frame.boot_epoch = 0x0102030405060708ULL;
    frame.sequence = 0xFFFFFFFEU;
    frame.source_millis = 0x10203040U;
    frame.payload_length = 3;
    frame.payload[0] = 1;
    frame.payload[1] = 2;
    frame.payload[2] = 3;
    return frame;
}

void test_golden_vector() {
    const Frame frame = golden_frame();
    std::array<std::uint8_t, kMaximumDatagramBytes> packet{};
    std::size_t length = 0;
    assert(encode_frame(frame, kKey.data(), kKey.size(), packet.data(), packet.size(), length) == CodecError::Ok);
    constexpr char expected[] =
        "45445531010100034341522d303100000102030405060708fffffffe10203040"
        "0102034450affe1d99aa17475115930f8a10f67f52";
    for (std::size_t index = 0; index < length; ++index) {
        const char hex[] = "0123456789abcdef";
        assert(hex[packet[index] >> 4] == expected[index * 2]);
        assert(hex[packet[index] & 0x0F] == expected[index * 2 + 1]);
    }
    Frame decoded;
    assert(decode_frame(packet.data(), length, kKey.data(), kKey.size(), decoded) == CodecError::Ok);
    assert(decoded.boot_epoch == frame.boot_epoch && decoded.sequence == frame.sequence);
    assert(decoded.payload_length == 3 && decoded.payload[2] == 3);
}

void test_authenticated_failures() {
    Frame frame = golden_frame();
    std::array<std::uint8_t, kMaximumDatagramBytes> packet{};
    std::size_t length = 0;
    assert(encode_frame(frame, kKey.data(), kKey.size(), packet.data(), packet.size(), length) == CodecError::Ok);
    packet[length - 1] ^= 1;
    Frame ignored;
    assert(decode_frame(packet.data(), length, kKey.data(), kKey.size(), ignored) == CodecError::BadHmac);

    assert(encode_frame(frame, kKey.data(), kKey.size(), packet.data(), packet.size(), length) == CodecError::Ok);
    packet[length - 18] ^= 1;
    std::array<std::uint8_t, 32> wrong_key{};
    assert(decode_frame(packet.data(), length, wrong_key.data(), wrong_key.size(), ignored) == CodecError::BadHmac);
}

void test_session_and_route_guards() {
    SessionTracker tracker("CAR-01", {0x0A000001U, 4001});
    Frame frame = golden_frame();
    assert(tracker.accept(frame, {0x0A000001U, 4001}, 10) == SessionDecision::Accepted);
    assert(tracker.accept(frame, {0x0A000001U, 4001}, 11) == SessionDecision::Replay);
    frame.sequence = 0xFFFFFFFDU;
    assert(tracker.accept(frame, {0x0A000001U, 4001}, 12) == SessionDecision::Reordered);
    frame.sequence = 0x00001000U;
    assert(tracker.accept(frame, {0x0A000001U, 4001}, 13) == SessionDecision::SequenceGap);
    frame.sequence = 0xFFFFFFFFU;
    assert(tracker.accept(frame, {0x0A000001U, 4001}, 14) == SessionDecision::Accepted);
    assert(tracker.stale(765));

    RouteValidator route;
    assert(!route.accept(false, RouteStage::B, false));
    assert(route.accept(true, RouteStage::Start, false));
    assert(route.accept(false, RouteStage::B, false));
    assert(route.accept(false, RouteStage::D, false));
    assert(route.accept(false, RouteStage::A, false));
    assert(route.accept(false, RouteStage::Complete, true));
    assert(!route.accept(true, RouteStage::Complete, true));
}

void test_payload_round_trip() {
    TelemetryPayload telemetry;
    telemetry.start_event = true;
    telemetry.heartbeat_alive = true;
    telemetry.motion_kind = MotionKind::WheelSpeed;
    telemetry.displacement_m = 1.25F;
    telemetry.wheel_speed_m_s = -0.5F;
    telemetry.turn_class = TurnClass::Large;
    telemetry.route_stage = RouteStage::B;
    assert(set_text(telemetry.vehicle_id, "CAR-01"));
    assert(set_text(telemetry.frame_id, "frame-7"));
    Frame frame;
    frame.message_type = MessageType::CarTelemetry;
    assert(encode_telemetry(telemetry, frame) == PayloadError::Ok);
    TelemetryPayload decoded;
    assert(decode_telemetry(frame, decoded) == PayloadError::Ok);
    assert(decoded.start_event && decoded.motion_kind == MotionKind::WheelSpeed);
    assert(decoded.displacement_m == 1.25F && decoded.vehicle_id.length == 6);
}

}  // namespace

int main() {
    test_golden_vector();
    test_authenticated_failures();
    test_session_and_route_guards();
    test_payload_round_trip();
    return 0;
}
