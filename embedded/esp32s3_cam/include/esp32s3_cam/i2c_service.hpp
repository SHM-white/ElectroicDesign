#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace esp32s3_cam {

inline constexpr std::uint8_t kI2cAddress = 0x52U;
inline constexpr int kSdaPin = 47;
inline constexpr int kSclPin = 48;
inline constexpr std::uint32_t kI2cFrequencyHz = 100000U;

enum class Profile : std::uint8_t {
  color,
  face,
};

struct Detection {
  std::uint8_t center_x;
  std::uint8_t center_y;
  std::uint8_t width;
  std::uint8_t length;
};

using Payload = std::array<std::uint8_t, 4U>;

class I2cRegisterService {
 public:
  explicit constexpr I2cRegisterService(const Profile profile) noexcept
      : profile_{profile} {}

  void receive(const std::uint8_t* bytes, std::size_t length) noexcept;
  [[nodiscard]] Payload request() const noexcept;
  [[nodiscard]] bool set_detection(std::uint8_t selector,
                                   Detection detection) noexcept;

 private:
  [[nodiscard]] bool supports(std::uint8_t selector) const noexcept;

  Profile profile_;
  std::uint8_t selected_register_{0xFFU};
  Detection register_zero_{};
  Detection register_one_{};
};

}  // namespace esp32s3_cam
