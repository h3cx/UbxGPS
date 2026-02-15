#pragma once

#include <cstddef>
#include <cstdint>

#include "drivers/BufferedSerial.h"

#include "ubx_transport.h"

namespace UbxGps {

class MbedBufferedSerialTransport final : public UbxTransport {
 public:
  explicit MbedBufferedSerialTransport(mbed::BufferedSerial* port) : port_(port) {}

  bool write(const uint8_t* data, size_t len) override {
    if (port_ == nullptr || data == nullptr || len == 0) {
      return false;
    }
    const auto written = port_->write(data, len);
    return written == static_cast<decltype(written)>(len);
  }

  bool readByte(uint8_t& out) override {
    if (port_ == nullptr) {
      return false;
    }
    const auto n = port_->read(&out, 1);
    return n == 1;
  }

 private:
  mbed::BufferedSerial* port_ = nullptr;
};

}  // namespace UbxGps
