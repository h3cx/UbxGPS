# API Reference

## Namespace

All public types are in `namespace UbxGps`.

## Core structs

### `NavPvt`
Decoded UBX NAV-PVT fields with original UBX units.

### `UbxGpsConfig`
Configuration values for parser behavior and threading.

- `rx_mode`: `SingleSlot` or `RingBuffer`
- `rx_queue_depth`: ring depth when `RingBuffer`
- `max_payload_size`: UBX payload guard (`>=92` required for NAV-PVT)
- `accept_nav_pvt_only`: if true, ignore non NAV-PVT frames
- `parser_thread_stack_bytes`, `poll_interval_ms`, `stop_timeout_ms`

### `UbxStatus`
- `rx_queue_full`: last enqueue failed due to queue full.
- `dropped_count`: total dropped decoded NAV-PVT messages.

### `UbxDiagnostics`
- `sync_errors`
- `length_overflow`
- `checksum_fail`
- `frames_ok`
- `navpvt_ok`
- `navpvt_bad_len`

## Transport interface

### `class UbxTransport`

```cpp
virtual bool write(const uint8_t* data, size_t len) = 0;
virtual bool readByte(uint8_t& out) = 0;
```

### `class MbedBufferedSerialTransport`
Adapter around `mbed::BufferedSerial`.

## Node class

### Setup/lifecycle

- `bool begin(UbxTransport* transport, const UbxGpsConfig& cfg)`
- `void stop()`

### Parsing

- `void poll()`
- `bool startParserThread()`
- `bool startParserThread(uint32_t poll_interval_ms, uint32_t stop_timeout_ms)`
- `bool stopParserThread()`

### RingBuffer mode API

- `bool available()`
- `bool read(NavPvt& out)`
- `bool peek(NavPvt& out) const`

### SingleSlot mode API

- `bool latest(NavPvt& out) const`
- `void onNavPvt(NavPvtCallback cb)`

### Raw frame callback

- `void onFrame(FrameCallback cb)`

### Status + diagnostics

- `UbxStatus getStatus() const`
- `UbxDiagnostics getDiagnostics() const`
- `void resetDiagnostics()`
