#include "esp32s3_cam/i2c_service.hpp"

namespace esp32s3_cam {
namespace {

constexpr Payload to_payload(const Detection detection) noexcept {
  return {detection.center_x, detection.center_y, detection.width,
          detection.length};
}

}  // namespace

void I2cRegisterService::receive(const std::uint8_t* const bytes,
                                 const std::size_t length) noexcept {
  if (bytes != nullptr && length != 0U) {
    selected_register_ = bytes[length - 1U];
  }
}

Payload I2cRegisterService::request() const noexcept {
  if (!supports(selected_register_)) {
    return {};
  }

  return to_payload(selected_register_ == 0x00U ? register_zero_
                                                : register_one_);
}

bool I2cRegisterService::set_detection(const std::uint8_t selector,
                                       const Detection detection) noexcept {
  if (!supports(selector)) {
    return false;
  }

  if (selector == 0x00U) {
    register_zero_ = detection;
  } else {
    register_one_ = detection;
  }
  return true;
}

bool I2cRegisterService::supports(const std::uint8_t selector) const noexcept {
  if (profile_ == Profile::color) {
    return selector == 0x00U || selector == 0x01U;
  }
  return selector == 0x01U;
}

}  // namespace esp32s3_cam
