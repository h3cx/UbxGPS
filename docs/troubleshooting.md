# Troubleshooting

## No fixes are decoded

- Confirm GNSS module is outputting UBX NAV-PVT (not only NMEA).
- Confirm baud rate and wiring (TX/RX crossed correctly).
- Ensure `mbed::BufferedSerial` is set to non-blocking mode.

## `checksum_fail` increases

- Check UART signal integrity and power stability.
- Confirm baud and framing settings match GNSS output.
- Check for line noise / level conversion issues.

## `length_overflow` increases

- Increase `UbxGpsConfig::max_payload_size` if you expect larger UBX frames.
- If NAV-PVT-only mode is expected, verify the module configuration.

## `dropped_count` increases

- Increase `rx_queue_depth` in RingBuffer mode.
- Drain queue more frequently.
- Lower parser poll interval and/or consume data in a dedicated task.

## Thread stop failure

- Increase `stop_timeout_ms`.
- Ensure no callback blocks indefinitely.
