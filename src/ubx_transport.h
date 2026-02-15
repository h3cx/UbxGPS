#pragma once

#include <cstddef>
#include <cstdint>

namespace UbxGps {

class UbxTransport {
 public:
  virtual ~UbxTransport() = default;
  virtual bool write(const uint8_t* data, size_t len) = 0;
  virtual bool readByte(uint8_t& out) = 0;
};

}  // namespace UbxGps
