#include <Arduino.h>

#include "esp32s3_cam/arduino_i2c_adapter.hpp"
#include "esp32s3_cam/i2c_service.hpp"

#if defined(ESP32S3_CAM_PROFILE_COLOR) == \
    defined(ESP32S3_CAM_PROFILE_FACE)
#error "Define exactly one ESP32S3-Cam I2C profile"
#endif

namespace {

#if defined(ESP32S3_CAM_PROFILE_COLOR)
constexpr esp32s3_cam::Profile kProfile = esp32s3_cam::Profile::color;
#else
constexpr esp32s3_cam::Profile kProfile = esp32s3_cam::Profile::face;
#endif

esp32s3_cam::I2cRegisterService service{kProfile};

}  // namespace

void setup() { esp32s3_cam::begin_arduino_i2c(service); }

void loop() { delay(1000); }
