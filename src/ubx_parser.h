#pragma once

#include <cstdint>

namespace UbxGps {

static constexpr uint8_t kSync1 = 0xB5;
static constexpr uint8_t kSync2 = 0x62;

enum class ParseState : uint8_t {
  WaitSync1,
  WaitSync2,
  ReadClass,
  ReadId,
  ReadLen1,
  ReadLen2,
  ReadPayload,
  ReadCkA,
  ReadCkB,
};

struct UbxFrameView {
  uint8_t msg_class = 0;
  uint8_t msg_id = 0;
  uint16_t len = 0;
  const uint8_t* payload = nullptr;
};

struct ParserContext {
  ParseState state = ParseState::WaitSync1;
  uint8_t msg_class = 0;
  uint8_t msg_id = 0;
  uint16_t msg_len = 0;
  uint16_t payload_pos = 0;
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  uint8_t rx_ck_a = 0;
  uint8_t rx_ck_b = 0;
};

}  // namespace UbxGps
