#pragma once

#include <cstdint>

namespace UbxGps {

struct NavPvt {
  uint32_t iTOW = 0;
  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t min = 0;
  uint8_t sec = 0;
  uint8_t valid = 0;

  uint32_t tAcc = 0;
  int32_t nano = 0;

  uint8_t fixType = 0;
  uint8_t flags = 0;
  uint8_t flags2 = 0;
  uint8_t numSV = 0;

  int32_t lon = 0;
  int32_t lat = 0;
  int32_t height = 0;
  int32_t hMSL = 0;

  uint32_t hAcc = 0;
  uint32_t vAcc = 0;

  int32_t velN = 0;
  int32_t velE = 0;
  int32_t velD = 0;
  int32_t gSpeed = 0;

  int32_t headMot = 0;

  uint32_t sAcc = 0;
  uint32_t headAcc = 0;

  uint16_t pDOP = 0;

  uint16_t flags3 = 0;

  int32_t headVeh = 0;
  int16_t magDec = 0;
  uint16_t magAcc = 0;
};

class NavPvtDecoder {
 public:
  static bool decode(const uint8_t* payload, uint16_t len, NavPvt& out);

 private:
  static inline uint16_t rdU16(const uint8_t* p, uint16_t o) {
    return static_cast<uint16_t>(p[o] | (static_cast<uint16_t>(p[o + 1]) << 8));
  }

  static inline int16_t rdI16(const uint8_t* p, uint16_t o) {
    return static_cast<int16_t>(rdU16(p, o));
  }

  static inline uint32_t rdU32(const uint8_t* p, uint16_t o) {
    return static_cast<uint32_t>(p[o]) |
           (static_cast<uint32_t>(p[o + 1]) << 8) |
           (static_cast<uint32_t>(p[o + 2]) << 16) |
           (static_cast<uint32_t>(p[o + 3]) << 24);
  }

  static inline int32_t rdI32(const uint8_t* p, uint16_t o) {
    return static_cast<int32_t>(rdU32(p, o));
  }
};

}  // namespace UbxGps
