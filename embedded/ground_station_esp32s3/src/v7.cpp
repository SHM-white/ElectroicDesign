#include "ground_station/v7.hpp"

#include <algorithm>
#include <utility>

namespace ground_station {
namespace {

constexpr std::uint8_t kHeader = 0xAAU;
constexpr std::size_t kFixedBytes = 6U;
constexpr std::size_t kMaxPayloadBytes = 255U;

std::pair<std::uint8_t, std::uint8_t> checksums(const Bytes& bytes, const std::size_t count) {
    std::uint8_t sum = 0U;
    std::uint8_t add = 0U;
    for (std::size_t index = 0U; index < count; ++index) {
        sum = static_cast<std::uint8_t>(sum + bytes[index]);
        add = static_cast<std::uint8_t>(add + sum);
    }
    return {sum, add};
}

}  // namespace

Bytes build_v7_frame(const std::uint8_t address, const std::uint8_t id, const Bytes& data) {
    if (data.size() > kMaxPayloadBytes) {
        return {};
    }
    Bytes raw;
    raw.reserve(data.size() + kFixedBytes);
    raw.push_back(kHeader);
    raw.push_back(address);
    raw.push_back(id);
    raw.push_back(static_cast<std::uint8_t>(data.size()));
    raw.insert(raw.end(), data.begin(), data.end());
    const auto checksum = checksums(raw, raw.size());
    raw.push_back(checksum.first);
    raw.push_back(checksum.second);
    return raw;
}

bool decode_v7_frame(const Bytes& raw, V7Frame& output) {
    if (raw.size() < kFixedBytes || raw.front() != kHeader) {
        return false;
    }
    const auto expected_size = static_cast<std::size_t>(raw[3]) + kFixedBytes;
    if (raw.size() != expected_size) {
        return false;
    }
    const auto checksum = checksums(raw, raw.size() - 2U);
    if (checksum.first != raw[raw.size() - 2U] || checksum.second != raw.back()) {
        return false;
    }
    output.address = raw[1];
    output.id = raw[2];
    output.data.assign(raw.begin() + 4, raw.end() - 2);
    output.sum_check = raw[raw.size() - 2U];
    output.add_check = raw.back();
    output.raw = raw;
    return true;
}

std::vector<V7Frame> V7StreamDecoder::feed(const Bytes& chunk) {
    buffer_.insert(buffer_.end(), chunk.begin(), chunk.end());
    std::vector<V7Frame> frames;
    while (true) {
        const auto header = std::find(buffer_.begin(), buffer_.end(), kHeader);
        if (header == buffer_.end()) {
            buffer_.clear();
            break;
        }
        buffer_.erase(buffer_.begin(), header);
        if (buffer_.size() < 4U) {
            break;
        }
        const auto frame_size = static_cast<std::size_t>(buffer_[3]) + kFixedBytes;
        if (buffer_.size() < frame_size) {
            break;
        }
        const Bytes candidate(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
        V7Frame frame;
        if (!decode_v7_frame(candidate, frame)) {
            ++rejected_frames_;
            buffer_.erase(buffer_.begin());
            continue;
        }
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
        frames.push_back(std::move(frame));
    }
    return frames;
}

std::size_t V7StreamDecoder::rejected_frames() const noexcept {
    return rejected_frames_;
}

}  // namespace ground_station
