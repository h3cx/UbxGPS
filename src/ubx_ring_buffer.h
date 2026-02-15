#pragma once

#include <cstdint>

#include "ubx_nav_pvt.h"

namespace UbxGps {

struct NavPvtSlot {
  NavPvt data{};
  uint32_t rx_timestamp_ms = 0;
};

}  // namespace UbxGps
