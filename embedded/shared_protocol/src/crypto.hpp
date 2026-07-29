#pragma once

#include <cstddef>
#include <cstdint>

namespace ed::shared_protocol::crypto {

void hmac_sha256(const std::uint8_t* key, std::size_t key_length, const std::uint8_t* data,
                 std::size_t data_length, std::uint8_t output[32]);

}  // namespace ed::shared_protocol::crypto
