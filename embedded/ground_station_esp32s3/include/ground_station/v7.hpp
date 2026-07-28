#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ground_station {

using Bytes = std::vector<std::uint8_t>;

struct V7Frame {
    std::uint8_t address{0U};
    std::uint8_t id{0U};
    Bytes data;
    std::uint8_t sum_check{0U};
    std::uint8_t add_check{0U};
    Bytes raw;
};

Bytes build_v7_frame(std::uint8_t address, std::uint8_t id, const Bytes& data);
bool decode_v7_frame(const Bytes& raw, V7Frame& output);

class V7StreamDecoder {
public:
    std::vector<V7Frame> feed(const Bytes& chunk);
    std::size_t rejected_frames() const noexcept;

private:
    Bytes buffer_;
    std::size_t rejected_frames_{0U};
};

}  // namespace ground_station
