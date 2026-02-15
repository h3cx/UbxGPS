# UBX GPS (NAV-PVT) Mbed Library

A compact C++17 library for **Arduino Giga R1 / Mbed OS** that parses a UBX byte stream and decodes **NAV-PVT (0x01/0x07)** into a typed struct.

## Features

- UBX parser with sync detection, payload length checks, and checksum validation.
- NAV-PVT decoder for both 84-byte and 92-byte payload variants.
- Two receive modes:
  - **SingleSlot** (latest sample + callback)
  - **RingBuffer** (`available()` + `read()` / `peek()`)
- Optional RTOS parser thread (`rtos::Thread`) or manual polling (`poll()`).
- Diagnostics counters for parser and decode health.
- Transport abstraction with an Mbed `mbed::BufferedSerial` adapter.

## Repository layout

```text
src/
  ubx_gps.h                 # main API: UbxGpsNode, config, callbacks
  ubx_gps.cpp               # parser, thread loop, NAV-PVT handling
  ubx_transport.h           # abstract byte transport interface
  ubx_transport_mbed.h      # BufferedSerial transport adapter
  ubx_parser.h              # parser states and frame view structs
  ubx_nav_pvt.h             # NavPvt struct + decode helpers
  ubx_ring_buffer.h         # ring slot payload type
  ubx_diagnostics.h         # status + diagnostics counters

docs/
  usage.md                  # integration guide and runtime patterns
  api.md                    # full API reference
  architecture.md           # design and threading model
  troubleshooting.md        # common issues and debugging flow
```

## Quick start

### 1) Configure UART and transport

```cpp
#include "UbxGPS.h"

static constexpr uint32_t kGpsBaud = 115200;

mbed::BufferedSerial gps_uart(PA_9, PA_10, kGpsBaud);
UbxGps::MbedBufferedSerialTransport gps_transport(&gps_uart);
```

### 2) Initialize the node

```cpp
UbxGps::UbxGpsConfig cfg;
cfg.rx_mode = UbxGps::RxMode::RingBuffer;
cfg.rx_queue_depth = 8;

UbxGps::UbxGpsNode gps;

gps_uart.set_blocking(false);
(void)gps.begin(&gps_transport, cfg);
```

### 3) Poll and consume decoded NAV-PVT

```cpp
while (true) {
  gps.poll();

  UbxGps::NavPvt pvt;
  while (gps.available() && gps.read(pvt)) {
    // use pvt
  }

  rtos::ThisThread::sleep_for(1ms);
}
```

For callback-based usage and parser-thread usage, see [docs/usage.md](docs/usage.md).

## Build notes

- This library intentionally avoids `Arduino.h` and Arduino serial APIs.
- It uses Mbed headers directly (`mbed::` and `rtos::` namespaces).
- Ensure your GNSS module is already configured to output UBX NAV-PVT.

## Documentation index

- [Usage guide](docs/usage.md)
- [API reference](docs/api.md)
- [Architecture notes](docs/architecture.md)
- [Troubleshooting](docs/troubleshooting.md)
