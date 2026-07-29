#pragma once

#include <cstddef>
#include <cstdint>

namespace ed::shared_protocol {

constexpr std::size_t kMaxPayloadBytes = 256;
constexpr std::size_t kHeaderBytes = 32;
constexpr std::size_t kCrcBytes = 2;
constexpr std::size_t kHmacTagBytes = 16;
constexpr std::size_t kMinimumDatagramBytes = kHeaderBytes + kCrcBytes + kHmacTagBytes;
constexpr std::size_t kMaximumDatagramBytes = kMinimumDatagramBytes + kMaxPayloadBytes;

enum class MessageType : std::uint8_t {
    CarTelemetry = 1,
    HmiSelection = 2,
    SelectionAck = 3,
    MissionStatus = 4,
};

enum class CodecError : std::uint8_t {
    Ok,
    BufferTooSmall,
    DatagramTooShort,
    DatagramTooLarge,
    BadMagic,
    BadVersion,
    BadMessageType,
    BadLength,
    BadSender,
    BadBootEpoch,
    KeyTooShort,
    BadHmac,
    BadCrc,
};

struct Frame {
    MessageType message_type = MessageType::CarTelemetry;
    char sender_id[9]{};
    std::uint64_t boot_epoch = 0;
    std::uint32_t sequence = 0;
    std::uint32_t source_millis = 0;
    std::uint16_t payload_length = 0;
    std::uint8_t payload[kMaxPayloadBytes]{};
};

std::uint16_t crc16_ccitt_false(const std::uint8_t* data, std::size_t length);

CodecError encode_frame(const Frame& frame, const std::uint8_t* key, std::size_t key_length,
                        std::uint8_t* output, std::size_t capacity, std::size_t& output_length);

CodecError decode_frame(const std::uint8_t* data, std::size_t length, const std::uint8_t* key,
                        std::size_t key_length, Frame& output);

const char* codec_error_name(CodecError error);

}  // namespace ed::shared_protocol
