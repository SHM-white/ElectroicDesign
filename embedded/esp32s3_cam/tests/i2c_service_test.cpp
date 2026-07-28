#include "esp32s3_cam/i2c_service.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <tuple>
#include <type_traits>

namespace {

using esp32s3_cam::Detection;
using esp32s3_cam::I2cRegisterService;
using esp32s3_cam::Payload;
using esp32s3_cam::Profile;

int failures = 0;

template <typename Actual, typename Expected>
void expect_equal(const Actual& actual, const Expected& expected,
                  const std::string_view message) {
  if (actual == expected) {
    return;
  }

  std::cerr << "FAIL: " << message << '\n';
  ++failures;
}

void select_register(I2cRegisterService& service, const std::uint8_t selector) {
  service.receive(&selector, 1U);
}

void selectors_map_to_profile_registers() {
  I2cRegisterService color{Profile::color};
  const Detection red{1U, 2U, 3U, 4U};
  const Detection blue{5U, 6U, 7U, 8U};

  expect_equal(color.set_detection(0x00U, red), true,
               "color register 0x00 accepts red data");
  expect_equal(color.set_detection(0x01U, blue), true,
               "color register 0x01 accepts blue data");

  select_register(color, 0x00U);
  expect_equal(color.request(), Payload{1U, 2U, 3U, 4U},
               "color register 0x00 selects red");
  select_register(color, 0x01U);
  expect_equal(color.request(), Payload{5U, 6U, 7U, 8U},
               "color register 0x01 selects blue");

  I2cRegisterService face{Profile::face};
  const Detection face_box{9U, 10U, 11U, 12U};
  expect_equal(face.set_detection(0x01U, face_box), true,
               "face register 0x01 accepts face data");
  expect_equal(face.set_detection(0x00U, face_box), false,
               "face profile rejects undocumented register 0x00");
  select_register(face, 0x01U);
  expect_equal(face.request(), Payload{9U, 10U, 11U, 12U},
               "face register 0x01 selects face");
}

void final_received_byte_selects_register() {
  I2cRegisterService color{Profile::color};
  expect_equal(color.set_detection(0x00U, Detection{10U, 20U, 30U, 40U}),
               true, "color register 0x00 accepts test data");
  expect_equal(color.set_detection(0x01U, Detection{50U, 60U, 70U, 80U}),
               true, "color register 0x01 accepts test data");
  const std::array<std::uint8_t, 3U> received{0x00U, 0xFFU, 0x01U};

  color.receive(received.data(), received.size());

  expect_equal(color.request(), Payload{50U, 60U, 70U, 80U},
               "only the final received byte selects the register");
}

void response_is_exactly_four_unsigned_bytes() {
  static_assert(std::tuple_size_v<Payload> == 4U);
  static_assert(sizeof(Payload::value_type) == 1U);
  static_assert(std::is_same_v<Payload::value_type, std::uint8_t>);

  I2cRegisterService color{Profile::color};
  expect_equal(color.set_detection(0x00U, Detection{0U, 127U, 128U, 255U}),
               true, "color register accepts unsigned boundary values");
  select_register(color, 0x00U);
  const Payload response = color.request();

  expect_equal(response.size(), std::size_t{4U},
               "request returns exactly four bytes");
  expect_equal(response, Payload{0U, 127U, 128U, 255U},
               "0, 127, 128, and 255 remain unsigned and ordered");
}

void documented_registers_start_at_no_detection_zero() {
  I2cRegisterService color{Profile::color};
  for (const std::uint8_t selector : {std::uint8_t{0x00U},
                                      std::uint8_t{0x01U}}) {
    select_register(color, selector);
    expect_equal(color.request(), Payload{0U, 0U, 0U, 0U},
                 "color register starts with no-detection zeros");
  }

  I2cRegisterService face{Profile::face};
  select_register(face, 0x01U);
  expect_equal(face.request(), Payload{0U, 0U, 0U, 0U},
               "face register starts with no-detection zeros");
}

void hardware_contract_is_fixed() {
  static_assert(esp32s3_cam::kI2cAddress == 0x52U);
  static_assert(esp32s3_cam::kSdaPin == 47);
  static_assert(esp32s3_cam::kSclPin == 48);
  static_assert(esp32s3_cam::kI2cFrequencyHz == 100000U);

  expect_equal(esp32s3_cam::kI2cAddress, std::uint8_t{0x52U},
               "I2C address is 0x52");
  expect_equal(esp32s3_cam::kSdaPin, 47, "SDA is GPIO47");
  expect_equal(esp32s3_cam::kSclPin, 48, "SCL is GPIO48");
  expect_equal(esp32s3_cam::kI2cFrequencyHz, std::uint32_t{100000U},
               "I2C frequency is 100000 Hz");
}

}  // namespace

int main() {
  selectors_map_to_profile_registers();
  final_received_byte_selects_register();
  response_is_exactly_four_unsigned_bytes();
  documented_registers_start_at_no_detection_zero();
  hardware_contract_is_fixed();

  if (failures != 0) {
    std::cerr << failures << " test assertion(s) failed\n";
    return 1;
  }

  std::cout << "All esp32s3_cam I2C contract tests passed\n";
  return 0;
}
