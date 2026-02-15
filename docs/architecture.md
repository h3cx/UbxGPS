# Architecture

## Layers

1. **Transport** (`UbxTransport`)
   - Abstract, byte-wise interface for read/write.
   - `MbedBufferedSerialTransport` adapts `mbed::BufferedSerial`.

2. **Parser**
   - Stateful UBX parser (`ParseState`, `ParserContext`).
   - State flow: sync -> class/id -> length -> payload -> checksum.
   - Emits validated `UbxFrameView`.

3. **Decoder**
   - `NavPvtDecoder::decode(payload, len, out)` supports 84 and 92 byte variants.

4. **Node** (`UbxGpsNode`)
   - Orchestrates reading, parsing, decoding, buffering, callbacks, diagnostics.

## Threading model

- Shared state is guarded by `rtos::Mutex`.
- Parser can run from:
  - foreground `poll()` calls, or
  - background `rtos::Thread` loop.
- Callback invocation (`onNavPvt` / `onFrame`) is outside state mutex.

## Queue policy

RingBuffer mode stores decoded `NavPvtSlot` entries.

- Full queue behavior: **drop newest** and increment `dropped_count`.
- Queue metadata: head/tail/count with modulo indexing.

## Error tracking

Parser + decode fault paths increment counters:

- sync mismatch after sync1
- payload length overflow vs configured max
- checksum mismatch
- NAV-PVT bad payload length
