#include "crypto.hpp"

#include <array>
#include <cstring>

namespace ed::shared_protocol::crypto {
namespace {

constexpr std::uint32_t kRoundConstants[64] = {
    0x428A2F98U, 0x71374491U, 0xB5C0FBCFU, 0xE9B5DBA5U, 0x3956C25BU, 0x59F111F1U, 0x923F82A4U,
    0xAB1C5ED5U, 0xD807AA98U, 0x12835B01U, 0x243185BEU, 0x550C7DC3U, 0x72BE5D74U, 0x80DEB1FEU,
    0x9BDC06A7U, 0xC19BF174U, 0xE49B69C1U, 0xEFBE4786U, 0x0FC19DC6U, 0x240CA1CCU, 0x2DE92C6FU,
    0x4A7484AAU, 0x5CB0A9DCU, 0x76F988DAU, 0x983E5152U, 0xA831C66DU, 0xB00327C8U, 0xBF597FC7U,
    0xC6E00BF3U, 0xD5A79147U, 0x06CA6351U, 0x14292967U, 0x27B70A85U, 0x2E1B2138U, 0x4D2C6DFCU,
    0x53380D13U, 0x650A7354U, 0x766A0ABBU, 0x81C2C92EU, 0x92722C85U, 0xA2BFE8A1U, 0xA81A664BU,
    0xC24B8B70U, 0xC76C51A3U, 0xD192E819U, 0xD6990624U, 0xF40E3585U, 0x106AA070U, 0x19A4C116U,
    0x1E376C08U, 0x2748774CU, 0x34B0BCB5U, 0x391C0CB3U, 0x4ED8AA4AU, 0x5B9CCA4FU, 0x682E6FF3U,
    0x748F82EEU, 0x78A5636FU, 0x84C87814U, 0x8CC70208U, 0x90BEFFFAU, 0xA4506CEBU, 0xBEF9A3F7U,
    0xC67178F2U,
};

std::uint32_t rotate_right(std::uint32_t value, std::uint32_t count) {
    return (value >> count) | (value << (32U - count));
}

class Sha256 {
public:
    Sha256() : state_{0x6A09E667U, 0xBB67AE85U, 0x3C6EF372U, 0xA54FF53AU,
                      0x510E527FU, 0x9B05688CU, 0x1F83D9ABU, 0x5BE0CD19U} {}

    void update(const std::uint8_t* data, std::size_t length) {
        while (length != 0) {
            const std::size_t take = (length < block_.size() - used_) ? length : block_.size() - used_;
            std::memcpy(block_.data() + used_, data, take);
            used_ += take;
            data += take;
            length -= take;
            total_bytes_ += take;
            if (used_ == block_.size()) {
                transform(block_.data());
                used_ = 0;
            }
        }
    }

    void finish(std::uint8_t output[32]) {
        block_[used_++] = 0x80;
        if (used_ > 56) {
            while (used_ < 64) block_[used_++] = 0;
            transform(block_.data());
            used_ = 0;
        }
        while (used_ < 56) block_[used_++] = 0;
        const std::uint64_t bits = total_bytes_ * 8U;
        for (int shift = 56; shift >= 0; shift -= 8) block_[used_++] = static_cast<std::uint8_t>(bits >> shift);
        transform(block_.data());
        for (std::size_t index = 0; index < 8; ++index) {
            for (int shift = 24; shift >= 0; shift -= 8) *output++ = static_cast<std::uint8_t>(state_[index] >> shift);
        }
    }

private:
    void transform(const std::uint8_t block[64]) {
        std::uint32_t words[64]{};
        for (std::size_t index = 0; index < 16; ++index) {
            words[index] = (static_cast<std::uint32_t>(block[index * 4]) << 24) |
                           (static_cast<std::uint32_t>(block[index * 4 + 1]) << 16) |
                           (static_cast<std::uint32_t>(block[index * 4 + 2]) << 8) | block[index * 4 + 3];
        }
        for (std::size_t index = 16; index < 64; ++index) {
            const std::uint32_t s0 = rotate_right(words[index - 15], 7) ^ rotate_right(words[index - 15], 18) ^ (words[index - 15] >> 3);
            const std::uint32_t s1 = rotate_right(words[index - 2], 17) ^ rotate_right(words[index - 2], 19) ^ (words[index - 2] >> 10);
            words[index] = words[index - 16] + s0 + words[index - 7] + s1;
        }
        std::uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
        std::uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
        for (std::size_t index = 0; index < 64; ++index) {
            const std::uint32_t s1 = rotate_right(e, 6) ^ rotate_right(e, 11) ^ rotate_right(e, 25);
            const std::uint32_t choose = (e & f) ^ (~e & g);
            const std::uint32_t temp1 = h + s1 + choose + kRoundConstants[index] + words[index];
            const std::uint32_t s0 = rotate_right(a, 2) ^ rotate_right(a, 13) ^ rotate_right(a, 22);
            const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
            const std::uint32_t temp2 = s0 + majority;
            h = g; g = f; f = e; e = d + temp1; d = c; c = b; b = a; a = temp1 + temp2;
        }
        state_[0] += a; state_[1] += b; state_[2] += c; state_[3] += d;
        state_[4] += e; state_[5] += f; state_[6] += g; state_[7] += h;
    }

    std::uint32_t state_[8];
    std::array<std::uint8_t, 64> block_{};
    std::size_t used_ = 0;
    std::uint64_t total_bytes_ = 0;
};

}  // namespace

void hmac_sha256(const std::uint8_t* key, std::size_t key_length, const std::uint8_t* data,
                 std::size_t data_length, std::uint8_t output[32]) {
    std::uint8_t key_block[64]{};
    if (key_length > sizeof(key_block)) {
        Sha256 key_hash;
        key_hash.update(key, key_length);
        key_hash.finish(key_block);
    } else {
        std::memcpy(key_block, key, key_length);
    }
    std::uint8_t inner_pad[64]{}, outer_pad[64]{};
    for (std::size_t index = 0; index < 64; ++index) {
        inner_pad[index] = static_cast<std::uint8_t>(key_block[index] ^ 0x36);
        outer_pad[index] = static_cast<std::uint8_t>(key_block[index] ^ 0x5C);
    }
    Sha256 inner;
    inner.update(inner_pad, sizeof(inner_pad));
    inner.update(data, data_length);
    std::uint8_t inner_hash[32]{};
    inner.finish(inner_hash);
    Sha256 outer;
    outer.update(outer_pad, sizeof(outer_pad));
    outer.update(inner_hash, sizeof(inner_hash));
    outer.finish(output);
}

}  // namespace ed::shared_protocol::crypto
