#pragma once

#include <cstdint>

#include "rtos/Mutex.h"
#include "rtos/Semaphore.h"
#include "rtos/Thread.h"

#include "ubx_diagnostics.h"
#include "ubx_nav_pvt.h"
#include "ubx_parser.h"
#include "ubx_ring_buffer.h"
#include "ubx_transport.h"

namespace UbxGps {

enum class RxMode : uint8_t {
  SingleSlot,
  RingBuffer,
};

struct UbxGpsConfig {
  RxMode rx_mode = RxMode::SingleSlot;
  uint8_t rx_queue_depth = 1;
  uint16_t max_payload_size = 100;

  bool accept_nav_pvt_only = true;

  uint32_t parser_thread_stack_bytes = 4096;
  uint32_t poll_interval_ms = 1;
  uint32_t stop_timeout_ms = 100;
};

using NavPvtCallback = void (*)(const NavPvt& pvt);
using FrameCallback = void (*)(const UbxFrameView& frame);

class UbxGpsNode {
 public:
  UbxGpsNode();
  explicit UbxGpsNode(const UbxGpsConfig& cfg);
  explicit UbxGpsNode(UbxTransport* transport, const UbxGpsConfig& cfg);
  ~UbxGpsNode();

  bool begin(UbxTransport* transport, const UbxGpsConfig& cfg);
  void stop();
  void poll();

  bool available();
  bool read(NavPvt& out);
  bool peek(NavPvt& out) const;

  bool latest(NavPvt& out) const;
  void onNavPvt(NavPvtCallback cb);
  void onFrame(FrameCallback cb);

  bool startParserThread();
  bool startParserThread(uint32_t poll_interval_ms, uint32_t stop_timeout_ms);
  bool stopParserThread();

  UbxStatus getStatus() const;
  UbxDiagnostics getDiagnostics() const;
  void resetDiagnostics();

 private:
  static constexpr uint8_t kNavClass = 0x01;
  static constexpr uint8_t kNavPvtId = 0x07;
  static constexpr uint16_t kAbsoluteMaxPayload = 512;

  void pollCore();
  bool parseOneByte(uint8_t b);
  bool finalizeFrame(UbxFrameView& out);
  bool handleFrame(const UbxFrameView& f);
  bool enqueueNavPvt(const NavPvt& pvt);
  bool dequeueNavPvt(NavPvt& out);
  void resetParserState();

  void lockState() const;
  void unlockState() const;

  void parserThreadMain();

  UbxTransport* transport_ = nullptr;
  UbxGpsConfig config_{};

  mutable rtos::Mutex state_mutex_{};

  ParserContext parser_ctx_{};
  uint8_t parser_payload_[kAbsoluteMaxPayload] = {};

  NavPvt latest_pvt_{};
  bool has_latest_ = false;

  NavPvtCallback navpvt_callback_ = nullptr;
  FrameCallback frame_callback_ = nullptr;

  NavPvtSlot* rx_slots_ = nullptr;
  uint8_t rx_queue_depth_ = 1;
  uint8_t rx_head_ = 0;
  uint8_t rx_tail_ = 0;
  uint8_t rx_count_ = 0;

  UbxStatus status_{};
  UbxDiagnostics diagnostics_{};

  rtos::Thread* parser_thread_ = nullptr;
  rtos::Semaphore* stopped_sem_ = nullptr;
  volatile bool parser_thread_running_ = false;
  uint32_t parser_poll_interval_ms_ = 1;
  uint32_t parser_stop_timeout_ms_ = 100;
};

}  // namespace UbxGps
