#pragma once

#include <cstdint>

namespace UbxGps {

struct UbxStatus {
  bool rx_queue_full = false;
  uint32_t dropped_count = 0;
};

struct UbxDiagnostics {
  uint32_t sync_errors = 0;
  uint32_t length_overflow = 0;
  uint32_t checksum_fail = 0;
  uint32_t frames_ok = 0;
  uint32_t navpvt_ok = 0;
  uint32_t navpvt_bad_len = 0;
};

}  // namespace UbxGps
