#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

class FakeWire {
 public:
  using ReceiveHandler = void (*)(int);
  using RequestHandler = void (*)();

  bool begin(std::uint8_t address, int sda, int scl,
             std::uint32_t frequency) noexcept;
  void onReceive(ReceiveHandler handler) noexcept;
  void onRequest(RequestHandler handler) noexcept;
  [[nodiscard]] int available() const noexcept;
  int read() noexcept;
  std::size_t slaveWrite(const std::uint8_t* bytes,
                         std::size_t length) noexcept;

  void reset() noexcept;
  void simulateReceive(const std::uint8_t* bytes, std::size_t length) noexcept;
  void simulateRequest() noexcept;

  bool begin_called{false};
  std::uint8_t address{0U};
  int sda{0};
  int scl{0};
  std::uint32_t frequency{0U};
  ReceiveHandler receive_handler{nullptr};
  RequestHandler request_handler{nullptr};
  std::array<std::uint8_t, 4U> outgoing{};
  std::size_t outgoing_length{0U};
  std::size_t slave_write_calls{0U};

 private:
  std::array<std::uint8_t, 16U> incoming_{};
  std::size_t incoming_length_{0U};
  std::size_t incoming_index_{0U};
};

extern FakeWire Wire;
