# legacy/

Superseded modules kept for reference. **None of these are part of the
current build** (the active top is `../main.v`, a Modbus RTU ambient-light
hub for the DFRobot LTR390UV). Do not add them to the Vivado project.

| File | Was | Superseded because |
|------|-----|--------------------|
| `baud_tick_gen.v` / `_tb` | 16x oversample baud tick for `rx.v` | only fed `rx.v`; the Modbus path's `uart_tx`/`uart_rx` derive their own timing |
| `rx.v` / `rx_tb` | custom 8N1 receiver (one byte/word + error flags) | the sensor speaks Modbus RTU (multi-byte frames + CRC), handled by `modbus_rtu_master.v` |
| `fifo.v` | word buffer for the round-robin BT streamer | single slow-polled sensor, one reading at a time — no buffering needed |
| `crc16_modbus.v` / `_tb` | standalone Modbus CRC-16 | `modbus_rtu_master.v` computes CRC internally; this one is never instantiated |
| `basys3_sensors_top.xdc` | constraints for the old `basys3_uv_ir_top` | that top was removed; the build uses `../main.xdc` |

The original raw-UART pipeline was:
`baud_tick_gen -> rx -> fifo -> streamer -> BT`. It fit a sensor that
streams raw bytes; the LTR390UV is a Modbus register device, which needs a
fundamentally different (request/response) receiver.
