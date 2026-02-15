# Usage Guide

## Requirements

- Mbed OS environment (Arduino Giga R1 compatible target).
- GNSS module outputting UBX NAV-PVT (`class=0x01`, `id=0x07`).
- Non-blocking UART read path.

## Basic polling mode (RingBuffer)

```cpp
#include "UbxGPS.h"

mbed::BufferedSerial gps_uart(PA_9, PA_10, 115200);
UbxGps::MbedBufferedSerialTransport transport(&gps_uart);

int main() {
  UbxGps::UbxGpsConfig cfg;
  cfg.rx_mode = UbxGps::RxMode::RingBuffer;
  cfg.rx_queue_depth = 8;

  UbxGps::UbxGpsNode gps;
  gps_uart.set_blocking(false);
  (void)gps.begin(&transport, cfg);

  while (true) {
    gps.poll();

    UbxGps::NavPvt pvt;
    while (gps.available() && gps.read(pvt)) {
      // consume fix
    }

    rtos::ThisThread::sleep_for(1ms);
  }
}
```

## SingleSlot + callback mode

```cpp
static void onFix(const UbxGps::NavPvt& pvt) {
  // fast callback path
}

int main() {
  UbxGps::UbxGpsConfig cfg;
  cfg.rx_mode = UbxGps::RxMode::SingleSlot;

  UbxGps::UbxGpsNode gps;
  gps.begin(&transport, cfg);
  gps.onNavPvt(onFix);

  while (true) {
    gps.poll();
    rtos::ThisThread::sleep_for(1ms);
  }
}
```

## Background parser thread mode

```cpp
UbxGps::UbxGpsConfig cfg;
cfg.rx_mode = UbxGps::RxMode::SingleSlot;
cfg.poll_interval_ms = 1;
cfg.stop_timeout_ms = 100;

UbxGps::UbxGpsNode gps;
gps.begin(&transport, cfg);
gps.startParserThread();

while (true) {
  UbxGps::NavPvt latest;
  if (gps.latest(latest)) {
    // read latest decoded state
  }
  rtos::ThisThread::sleep_for(10ms);
}
```

## Runtime recommendations

- Keep UART at 115200 unless your GNSS configuration says otherwise.
- Keep `set_blocking(false)` enabled on `mbed::BufferedSerial`.
- Prefer RingBuffer in polling designs where your main loop may stall occasionally.
- Keep callbacks short and non-blocking.
- If queue overflow occurs, inspect `getStatus().dropped_count`.

## Shutdown

- Call `stopParserThread()` before ending your app/task.
- `UbxGpsNode` destructor calls `stop()`, which attempts to stop the thread and release resources.
