#pragma once

#include "esp32s3_cam/i2c_service.hpp"

namespace esp32s3_cam {

void begin_arduino_i2c(I2cRegisterService& service);

}  // namespace esp32s3_cam
