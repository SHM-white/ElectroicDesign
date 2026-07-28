#include "esp32s3_cam/arduino_i2c_adapter.hpp"

#include <Wire.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>

int main() {
  Wire.reset();
  esp32s3_cam::I2cRegisterService service{esp32s3_cam::Profile::color};
  if (!service.set_detection(
          0x01U, esp32s3_cam::Detection{0U, 127U, 128U, 255U})) {
    std::cerr << "FAIL: adapter test could not populate register 0x01\n";
    return 1;
  }

  esp32s3_cam::begin_arduino_i2c(service);

  if (!Wire.begin_called || Wire.address != esp32s3_cam::kI2cAddress ||
      Wire.sda != esp32s3_cam::kSdaPin || Wire.scl != esp32s3_cam::kSclPin ||
      Wire.frequency != esp32s3_cam::kI2cFrequencyHz ||
      Wire.receive_handler == nullptr || Wire.request_handler == nullptr) {
    std::cerr << "FAIL: adapter did not initialize the verified I2C contract\n";
    return 1;
  }

  const std::array<std::uint8_t, 3U> received{0x00U, 0xFFU, 0x01U};
  Wire.simulateReceive(received.data(), received.size());
  Wire.simulateRequest();

  const std::array<std::uint8_t, 4U> expected{0U, 127U, 128U, 255U};
  if (Wire.slave_write_calls != 1U ||
      Wire.outgoing_length != expected.size() || Wire.outgoing != expected) {
    std::cerr << "FAIL: adapter did not slave-write exactly four bytes\n";
    return 1;
  }

  std::cout << "ESP32 Arduino I2C adapter contract passed\n";
  return 0;
}
