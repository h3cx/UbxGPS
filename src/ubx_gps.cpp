#include "ubx_gps.h"

#include <chrono>
#include <cstddef>
#include <new>

#include "Kernel.h"
#include "platform/Callback.h"
#include "rtos/ThisThread.h"

namespace UbxGps {

namespace {
constexpr uint32_t kParserStoppedReleaseCount = 1;
}

bool NavPvtDecoder::decode(const uint8_t* payload, uint16_t len, NavPvt& out) {
  if (payload == nullptr || (len != 84 && len != 92)) {
    return false;
  }

  out = {};
  out.iTOW = rdU32(payload, 0);
  out.year = rdU16(payload, 4);
  out.month = payload[6];
  out.day = payload[7];
  out.hour = payload[8];
  out.min = payload[9];
  out.sec = payload[10];
  out.valid = payload[11];
  out.tAcc = rdU32(payload, 12);
  out.nano = rdI32(payload, 16);
  out.fixType = payload[20];
  out.flags = payload[21];
  out.flags2 = payload[22];
  out.numSV = payload[23];
  out.lon = rdI32(payload, 24);
  out.lat = rdI32(payload, 28);
  out.height = rdI32(payload, 32);
  out.hMSL = rdI32(payload, 36);
  out.hAcc = rdU32(payload, 40);
  out.vAcc = rdU32(payload, 44);
  out.velN = rdI32(payload, 48);
  out.velE = rdI32(payload, 52);
  out.velD = rdI32(payload, 56);
  out.gSpeed = rdI32(payload, 60);
  out.headMot = rdI32(payload, 64);
  out.sAcc = rdU32(payload, 68);
  out.headAcc = rdU32(payload, 72);
  out.pDOP = rdU16(payload, 76);
  out.flags3 = rdU16(payload, 78);

  if (len == 92) {
    out.headVeh = rdI32(payload, 84);
    out.magDec = rdI16(payload, 88);
    out.magAcc = rdU16(payload, 90);
  } else {
    out.headVeh = out.headMot;
  }

  return true;
}

UbxGpsNode::UbxGpsNode() = default;

UbxGpsNode::UbxGpsNode(const UbxGpsConfig& cfg) : config_(cfg) {}

UbxGpsNode::UbxGpsNode(UbxTransport* transport, const UbxGpsConfig& cfg)
    : transport_(transport), config_(cfg) {}

UbxGpsNode::~UbxGpsNode() { stop(); }

bool UbxGpsNode::begin(UbxTransport* transport, const UbxGpsConfig& cfg) {
  stop();

  if (transport == nullptr) {
    return false;
  }

  if (cfg.max_payload_size > kAbsoluteMaxPayload || cfg.max_payload_size < 92) {
    return false;
  }

  transport_ = transport;
  config_ = cfg;
  parser_poll_interval_ms_ = config_.poll_interval_ms;
  parser_stop_timeout_ms_ = config_.stop_timeout_ms;

  if (config_.rx_mode == RxMode::RingBuffer) {
    if (config_.rx_queue_depth == 0) {
      return false;
    }

    rx_slots_ = new (std::nothrow) NavPvtSlot[config_.rx_queue_depth];
    if (rx_slots_ == nullptr) {
      return false;
    }
    rx_queue_depth_ = config_.rx_queue_depth;
  } else {
    rx_queue_depth_ = 1;
  }

  lockState();
  status_ = {};
  diagnostics_ = {};
  has_latest_ = false;
  rx_head_ = 0;
  rx_tail_ = 0;
  rx_count_ = 0;
  unlockState();
  resetParserState();

  return true;
}

void UbxGpsNode::stop() {
  (void)stopParserThread();

  lockState();
  transport_ = nullptr;
  has_latest_ = false;
  rx_head_ = 0;
  rx_tail_ = 0;
  rx_count_ = 0;
  unlockState();

  delete[] rx_slots_;
  rx_slots_ = nullptr;
}

void UbxGpsNode::poll() { pollCore(); }

bool UbxGpsNode::available() {
  if (config_.rx_mode != RxMode::RingBuffer) {
    return false;
  }

  lockState();
  const bool has_data = rx_count_ > 0;
  unlockState();
  return has_data;
}

bool UbxGpsNode::read(NavPvt& out) {
  if (config_.rx_mode != RxMode::RingBuffer) {
    return false;
  }

  return dequeueNavPvt(out);
}

bool UbxGpsNode::peek(NavPvt& out) const {
  if (config_.rx_mode != RxMode::RingBuffer) {
    return false;
  }

  lockState();
  if (rx_count_ == 0 || rx_slots_ == nullptr) {
    unlockState();
    return false;
  }
  out = rx_slots_[rx_tail_].data;
  unlockState();
  return true;
}

bool UbxGpsNode::latest(NavPvt& out) const {
  lockState();
  if (!has_latest_) {
    unlockState();
    return false;
  }

  out = latest_pvt_;
  unlockState();
  return true;
}

void UbxGpsNode::onNavPvt(NavPvtCallback cb) {
  lockState();
  navpvt_callback_ = cb;
  unlockState();
}

void UbxGpsNode::onFrame(FrameCallback cb) {
  lockState();
  frame_callback_ = cb;
  unlockState();
}

bool UbxGpsNode::startParserThread() {
  return startParserThread(config_.poll_interval_ms, config_.stop_timeout_ms);
}

bool UbxGpsNode::startParserThread(uint32_t poll_interval_ms, uint32_t stop_timeout_ms) {
  if (parser_thread_running_ || transport_ == nullptr) {
    return parser_thread_running_;
  }

  parser_poll_interval_ms_ = poll_interval_ms;
  parser_stop_timeout_ms_ = stop_timeout_ms;

  stopped_sem_ = new (std::nothrow) rtos::Semaphore(0, 1);
  if (stopped_sem_ == nullptr) {
    return false;
  }

  parser_thread_ = new (std::nothrow)
      rtos::Thread(osPriorityNormal, config_.parser_thread_stack_bytes, nullptr, "UbxGpsParser");
  if (parser_thread_ == nullptr) {
    delete stopped_sem_;
    stopped_sem_ = nullptr;
    return false;
  }

  parser_thread_running_ = true;
  parser_thread_->start(mbed::callback(this, &UbxGpsNode::parserThreadMain));
  return true;
}

bool UbxGpsNode::stopParserThread() {
  if (!parser_thread_running_) {
    return true;
  }

  parser_thread_running_ = false;

  bool stopped = true;
  if (stopped_sem_ != nullptr) {
    stopped = stopped_sem_->try_acquire_for(parser_stop_timeout_ms_);
  }

  if (parser_thread_ != nullptr) {
    if (stopped) {
      parser_thread_->join();
    }
    delete parser_thread_;
    parser_thread_ = nullptr;
  }

  delete stopped_sem_;
  stopped_sem_ = nullptr;

  return stopped;
}

UbxStatus UbxGpsNode::getStatus() const {
  lockState();
  const UbxStatus copy = status_;
  unlockState();
  return copy;
}

UbxDiagnostics UbxGpsNode::getDiagnostics() const {
  lockState();
  const UbxDiagnostics copy = diagnostics_;
  unlockState();
  return copy;
}

void UbxGpsNode::resetDiagnostics() {
  lockState();
  diagnostics_ = {};
  status_ = {};
  unlockState();
}

void UbxGpsNode::pollCore() {
  if (transport_ == nullptr) {
    return;
  }

  uint8_t b = 0;
  UbxFrameView frame{};
  while (transport_->readByte(b)) {
    if (parseOneByte(b) && finalizeFrame(frame)) {
      (void)handleFrame(frame);
    }
  }
}

bool UbxGpsNode::parseOneByte(uint8_t b) {
  switch (parser_ctx_.state) {
    case ParseState::WaitSync1:
      if (b == kSync1) {
        parser_ctx_.state = ParseState::WaitSync2;
      }
      return false;

    case ParseState::WaitSync2:
      if (b == kSync2) {
        parser_ctx_.state = ParseState::ReadClass;
      } else {
        lockState();
        ++diagnostics_.sync_errors;
        unlockState();
        parser_ctx_.state = ParseState::WaitSync1;
      }
      return false;

    case ParseState::ReadClass:
      parser_ctx_.msg_class = b;
      parser_ctx_.ck_a = b;
      parser_ctx_.ck_b = parser_ctx_.ck_a;
      parser_ctx_.state = ParseState::ReadId;
      return false;

    case ParseState::ReadId:
      parser_ctx_.msg_id = b;
      parser_ctx_.ck_a = static_cast<uint8_t>(parser_ctx_.ck_a + b);
      parser_ctx_.ck_b = static_cast<uint8_t>(parser_ctx_.ck_b + parser_ctx_.ck_a);
      parser_ctx_.state = ParseState::ReadLen1;
      return false;

    case ParseState::ReadLen1:
      parser_ctx_.msg_len = b;
      parser_ctx_.ck_a = static_cast<uint8_t>(parser_ctx_.ck_a + b);
      parser_ctx_.ck_b = static_cast<uint8_t>(parser_ctx_.ck_b + parser_ctx_.ck_a);
      parser_ctx_.state = ParseState::ReadLen2;
      return false;

    case ParseState::ReadLen2:
      parser_ctx_.msg_len |= static_cast<uint16_t>(b) << 8;
      parser_ctx_.ck_a = static_cast<uint8_t>(parser_ctx_.ck_a + b);
      parser_ctx_.ck_b = static_cast<uint8_t>(parser_ctx_.ck_b + parser_ctx_.ck_a);
      parser_ctx_.payload_pos = 0;

      if (parser_ctx_.msg_len > config_.max_payload_size) {
        lockState();
        ++diagnostics_.length_overflow;
        unlockState();
        parser_ctx_.state = ParseState::WaitSync1;
      } else if (parser_ctx_.msg_len == 0) {
        parser_ctx_.state = ParseState::ReadCkA;
      } else {
        parser_ctx_.state = ParseState::ReadPayload;
      }
      return false;

    case ParseState::ReadPayload:
      parser_payload_[parser_ctx_.payload_pos++] = b;
      parser_ctx_.ck_a = static_cast<uint8_t>(parser_ctx_.ck_a + b);
      parser_ctx_.ck_b = static_cast<uint8_t>(parser_ctx_.ck_b + parser_ctx_.ck_a);
      if (parser_ctx_.payload_pos >= parser_ctx_.msg_len) {
        parser_ctx_.state = ParseState::ReadCkA;
      }
      return false;

    case ParseState::ReadCkA:
      parser_ctx_.rx_ck_a = b;
      parser_ctx_.state = ParseState::ReadCkB;
      return false;

    case ParseState::ReadCkB:
      parser_ctx_.rx_ck_b = b;
      if (parser_ctx_.rx_ck_a == parser_ctx_.ck_a && parser_ctx_.rx_ck_b == parser_ctx_.ck_b) {
        parser_ctx_.state = ParseState::WaitSync1;
        return true;
      }
      lockState();
      ++diagnostics_.checksum_fail;
      unlockState();
      parser_ctx_.state = ParseState::WaitSync1;
      return false;
  }

  return false;
}

bool UbxGpsNode::finalizeFrame(UbxFrameView& out) {
  out.msg_class = parser_ctx_.msg_class;
  out.msg_id = parser_ctx_.msg_id;
  out.len = parser_ctx_.msg_len;
  out.payload = parser_payload_;

  lockState();
  ++diagnostics_.frames_ok;
  unlockState();
  return true;
}

bool UbxGpsNode::handleFrame(const UbxFrameView& f) {
  FrameCallback frame_cb = nullptr;
  lockState();
  frame_cb = frame_callback_;
  unlockState();
  if (frame_cb != nullptr) {
    frame_cb(f);
  }

  if (config_.accept_nav_pvt_only && (f.msg_class != kNavClass || f.msg_id != kNavPvtId)) {
    return false;
  }

  if (f.msg_class == kNavClass && f.msg_id == kNavPvtId) {
    NavPvt pvt{};
    if (!NavPvtDecoder::decode(f.payload, f.len, pvt)) {
      lockState();
      ++diagnostics_.navpvt_bad_len;
      unlockState();
      return false;
    }

    lockState();
    ++diagnostics_.navpvt_ok;
    unlockState();

    if (config_.rx_mode == RxMode::SingleSlot) {
      NavPvtCallback cb = nullptr;
      lockState();
      latest_pvt_ = pvt;
      has_latest_ = true;
      cb = navpvt_callback_;
      unlockState();
      if (cb != nullptr) {
        cb(pvt);
      }
      return true;
    }

    return enqueueNavPvt(pvt);
  }

  return false;
}

bool UbxGpsNode::enqueueNavPvt(const NavPvt& pvt) {
  lockState();
  if (rx_slots_ == nullptr || rx_count_ >= rx_queue_depth_) {
    status_.rx_queue_full = true;
    ++status_.dropped_count;
    unlockState();
    return false;
  }

  rx_slots_[rx_head_].data = pvt;
  rx_slots_[rx_head_].rx_timestamp_ms = static_cast<uint32_t>(mbed::Kernel::get_ms_count());
  rx_head_ = static_cast<uint8_t>((rx_head_ + 1) % rx_queue_depth_);
  ++rx_count_;
  status_.rx_queue_full = false;
  unlockState();
  return true;
}

bool UbxGpsNode::dequeueNavPvt(NavPvt& out) {
  lockState();
  if (rx_slots_ == nullptr || rx_count_ == 0) {
    unlockState();
    return false;
  }

  out = rx_slots_[rx_tail_].data;
  rx_tail_ = static_cast<uint8_t>((rx_tail_ + 1) % rx_queue_depth_);
  --rx_count_;
  unlockState();
  return true;
}

void UbxGpsNode::resetParserState() { parser_ctx_ = {}; }

void UbxGpsNode::lockState() const { state_mutex_.lock(); }

void UbxGpsNode::unlockState() const { state_mutex_.unlock(); }

void UbxGpsNode::parserThreadMain() {
  while (parser_thread_running_) {
    pollCore();
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(parser_poll_interval_ms_));
  }

  if (stopped_sem_ != nullptr) {
    stopped_sem_->release(kParserStoppedReleaseCount);
  }
}

}  // namespace UbxGps
