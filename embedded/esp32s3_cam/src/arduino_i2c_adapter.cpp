#include "esp32s3_cam/arduino_i2c_adapter.hpp"

#include <Wire.h>

#include <cstdint>

namespace esp32s3_cam {
namespace {

I2cRegisterService* active_service = nullptr;

void on_receive(int) {
  std::uint8_t final_byte = 0U;
  bool received = false;
  while (Wire.available() > 0) {
    final_byte = static_cast<std::uint8_t>(Wire.read());
    received = true;
  }

  if (received && active_service != nullptr) {
    active_service->receive(&final_byte, 1U);
  }
}

void on_request() {
  if (active_service == nullptr) {
    return;
  }

  const Payload response = active_service->request();
  Wire.slaveWrite(response.data(), response.size());
}

}  // namespace

void begin_arduino_i2c(I2cRegisterService& service) {
  active_service = &service;
  Wire.begin(kI2cAddress, kSdaPin, kSclPin, kI2cFrequencyHz);
  Wire.onReceive(on_receive);
  Wire.onRequest(on_request);
}

}  // namespace esp32s3_cam
