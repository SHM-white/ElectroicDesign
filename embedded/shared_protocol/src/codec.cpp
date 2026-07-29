#include "ed_shared_protocol/codec.hpp"

#include "crypto.hpp"

#include <cstring>

namespace ed::shared_protocol {
namespace {

constexpr std::uint8_t kVersion = 1;
constexpr std::uint8_t kMagic[4] = {'E', 'D', 'U', '1'};
constexpr std::size_t kHmacBytes = 32;

void put_u16(std::uint8_t* output, std::uint16_t value) {
    output[0] = static_cast<std::uint8_t>(value >> 8);
    output[1] = static_cast<std::uint8_t>(value);
}

void put_u32(std::uint8_t* output, std::uint32_t value) {
    output[0] = static_cast<std::uint8_t>(value >> 24);
    output[1] = static_cast<std::uint8_t>(value >> 16);
    output[2] = static_cast<std::uint8_t>(value >> 8);
    output[3] = static_cast<std::uint8_t>(value);
}

void put_u64(std::uint8_t* output, std::uint64_t value) {
    for (int shift = 56; shift >= 0; shift -= 8) *output++ = static_cast<std::uint8_t>(value >> shift);
}

std::uint16_t get_u16(const std::uint8_t* input) {
    return static_cast<std::uint16_t>((input[0] << 8) | input[1]);
}

std::uint32_t get_u32(const std::uint8_t* input) {
    return (static_cast<std::uint32_t>(input[0]) << 24) | (static_cast<std::uint32_t>(input[1]) << 16) |
           (static_cast<std::uint32_t>(input[2]) << 8) | input[3];
}

std::uint64_t get_u64(const std::uint8_t* input) {
    std::uint64_t value = 0;
    for (std::size_t index = 0; index < 8; ++index) value = (value << 8) | input[index];
    return value;
}

bool valid_type(std::uint8_t value) { return value >= 1 && value <= 4; }

bool constant_time_equal(const std::uint8_t* left, const std::uint8_t* right, std::size_t length) {
    std::uint8_t difference = 0;
    for (std::size_t index = 0; index < length; ++index) difference |= left[index] ^ right[index];
    return difference == 0;
}

bool valid_sender(const char sender[9], std::size_t& length) {
    length = 0;
    while (length < 8 && sender[length] != '\0') {
        if (static_cast<unsigned char>(sender[length]) > 0x7F) return false;
        ++length;
    }
    return length > 0 && (length == 8 || sender[length] == '\0');
}

}  // namespace

std::uint16_t crc16_ccitt_false(const std::uint8_t* data, std::size_t length) {
    std::uint16_t checksum = 0xFFFF;
    for (std::size_t index = 0; index < length; ++index) {
        checksum ^= static_cast<std::uint16_t>(data[index]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            checksum = (checksum & 0x8000) != 0 ? static_cast<std::uint16_t>((checksum << 1) ^ 0x1021)
                                                 : static_cast<std::uint16_t>(checksum << 1);
        }
    }
    return checksum;
}

CodecError encode_frame(const Frame& frame, const std::uint8_t* key, std::size_t key_length,
                        std::uint8_t* output, std::size_t capacity, std::size_t& output_length) {
    output_length = 0;
    if (key == nullptr || key_length < 32) return CodecError::KeyTooShort;
    std::size_t sender_length = 0;
    if (!valid_sender(frame.sender_id, sender_length)) return CodecError::BadSender;
    if (frame.boot_epoch == 0) return CodecError::BadBootEpoch;
    if (!valid_type(static_cast<std::uint8_t>(frame.message_type))) return CodecError::BadMessageType;
    if (frame.payload_length > kMaxPayloadBytes) return CodecError::DatagramTooLarge;
    const std::size_t total = kMinimumDatagramBytes + frame.payload_length;
    if (capacity < total) return CodecError::BufferTooSmall;
    std::memcpy(output, kMagic, sizeof(kMagic));
    output[4] = kVersion;
    output[5] = static_cast<std::uint8_t>(frame.message_type);
    put_u16(output + 6, frame.payload_length);
    std::memset(output + 8, 0, 8);
    std::memcpy(output + 8, frame.sender_id, sender_length);
    put_u64(output + 16, frame.boot_epoch);
    put_u32(output + 24, frame.sequence);
    put_u32(output + 28, frame.source_millis);
    std::memcpy(output + kHeaderBytes, frame.payload, frame.payload_length);
    put_u16(output + kHeaderBytes + frame.payload_length,
            crc16_ccitt_false(output, kHeaderBytes + frame.payload_length));
    std::uint8_t digest[kHmacBytes]{};
    crypto::hmac_sha256(key, key_length, output, total - kHmacTagBytes, digest);
    std::memcpy(output + total - kHmacTagBytes, digest, kHmacTagBytes);
    output_length = total;
    return CodecError::Ok;
}

CodecError decode_frame(const std::uint8_t* data, std::size_t length, const std::uint8_t* key,
                        std::size_t key_length, Frame& output) {
    if (key == nullptr || key_length < 32) return CodecError::KeyTooShort;
    if (data == nullptr || length < kMinimumDatagramBytes) return CodecError::DatagramTooShort;
    if (length > kMaximumDatagramBytes) return CodecError::DatagramTooLarge;
    if (std::memcmp(data, kMagic, sizeof(kMagic)) != 0) return CodecError::BadMagic;
    if (data[4] != kVersion) return CodecError::BadVersion;
    if (!valid_type(data[5])) return CodecError::BadMessageType;
    const std::uint16_t payload_length = get_u16(data + 6);
    if (payload_length > kMaxPayloadBytes || length != kMinimumDatagramBytes + payload_length) return CodecError::BadLength;
    std::size_t sender_length = 0;
    char sender[9]{};
    std::memcpy(sender, data + 8, 8);
    if (!valid_sender(sender, sender_length)) return CodecError::BadSender;
    if (get_u64(data + 16) == 0) return CodecError::BadBootEpoch;
    std::uint8_t digest[32]{};
    crypto::hmac_sha256(key, key_length, data, length - kHmacTagBytes, digest);
    if (!constant_time_equal(data + length - kHmacTagBytes, digest, kHmacTagBytes)) return CodecError::BadHmac;
    const std::size_t crc_offset = kHeaderBytes + payload_length;
    if (get_u16(data + crc_offset) != crc16_ccitt_false(data, crc_offset)) return CodecError::BadCrc;
    output = Frame{};
    output.message_type = static_cast<MessageType>(data[5]);
    std::memcpy(output.sender_id, sender, sender_length);
    output.boot_epoch = get_u64(data + 16);
    output.sequence = get_u32(data + 24);
    output.source_millis = get_u32(data + 28);
    output.payload_length = payload_length;
    std::memcpy(output.payload, data + kHeaderBytes, payload_length);
    return CodecError::Ok;
}

const char* codec_error_name(CodecError error) {
    constexpr const char* names[] = {"OK", "BUFFER_TOO_SMALL", "DATAGRAM_TOO_SHORT", "DATAGRAM_TOO_LARGE",
                                     "BAD_MAGIC", "BAD_VERSION", "BAD_MESSAGE_TYPE", "BAD_LENGTH", "BAD_SENDER",
                                     "BAD_BOOT_EPOCH", "KEY_TOO_SHORT", "BAD_HMAC", "BAD_CRC"};
    return names[static_cast<std::size_t>(error)];
}

}  // namespace ed::shared_protocol
