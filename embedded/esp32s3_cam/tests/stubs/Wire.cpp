#include "Wire.h"

#include <algorithm>

FakeWire Wire;

bool FakeWire::begin(const std::uint8_t requested_address,
                     const int requested_sda, const int requested_scl,
                     const std::uint32_t requested_frequency) noexcept {
  begin_called = true;
  address = requested_address;
  sda = requested_sda;
  scl = requested_scl;
  frequency = requested_frequency;
  return true;
}

void FakeWire::onReceive(const ReceiveHandler handler) noexcept {
  receive_handler = handler;
}

void FakeWire::onRequest(const RequestHandler handler) noexcept {
  request_handler = handler;
}

int FakeWire::available() const noexcept {
  return incoming_index_ < incoming_length_ ? 1 : 0;
}

int FakeWire::read() noexcept {
  if (incoming_index_ >= incoming_length_) {
    return -1;
  }
  return incoming_[incoming_index_++];
}

std::size_t FakeWire::slaveWrite(const std::uint8_t* const bytes,
                                 const std::size_t length) noexcept {
  const std::size_t copied = std::min(length, outgoing.size());
  std::copy_n(bytes, copied, outgoing.begin());
  outgoing_length = copied;
  ++slave_write_calls;
  return copied;
}

void FakeWire::reset() noexcept {
  begin_called = false;
  address = 0U;
  sda = 0;
  scl = 0;
  frequency = 0U;
  receive_handler = nullptr;
  request_handler = nullptr;
  outgoing.fill(0U);
  outgoing_length = 0U;
  slave_write_calls = 0U;
  incoming_.fill(0U);
  incoming_length_ = 0U;
  incoming_index_ = 0U;
}

void FakeWire::simulateReceive(const std::uint8_t* const bytes,
                               const std::size_t length) noexcept {
  incoming_length_ = std::min(length, incoming_.size());
  std::copy_n(bytes, incoming_length_, incoming_.begin());
  incoming_index_ = 0U;
  if (receive_handler != nullptr) {
    receive_handler(static_cast<int>(incoming_length_));
  }
}

void FakeWire::simulateRequest() noexcept {
  outgoing_length = 0U;
  if (request_handler != nullptr) {
    request_handler();
  }
}
