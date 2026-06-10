# UART-Sensor-Hub
This repo contains the Verilog that describes a 4 channel full-duplex sensor aggregation hub with onboard error detection and FIFO buffers. Each channel is made to have some configurability that one sets before synthesis. The Verilog in this repo is intended to be used with the Basys 3 FPGA Development Board

## UART Error Detection Mechanisms

---

### 1. Parity Bit Generation and Checking

Parity is the most common hardware mechanism used to detect single-bit errors in the data payload.

**The Hardware Mechanism:** The transmitting UART hardware passes the data byte through an internal XOR gate tree. This circuit automatically counts the number of logical `1`s.

- **Even Parity:** The transmitter forces the parity bit to a `1` or `0` so that the total count of `1`s (data + parity) is even.
- **Odd Parity:** The transmitter forces the parity bit to a `1` or `0` so that the total count of `1`s is odd.

**The Check:** On the receiving end, the RX hardware independently calculates the parity of the incoming data bits. It then compares its calculated result with the received parity bit. If they do not match, the hardware sets a **Parity Error (`PE`)** flag in its status register.

---

### 2. Framing Error Detection

A framing error occurs when the receiving hardware does not find the expected protocol structure on the physical wire.

**The Hardware Mechanism:** The receiver expects the transmission line to return to a high voltage state (logical `1`) at the exact moment the Stop Bit is scheduled to arrive.

**The Check:** The internal clock of the receiver counts out the bit periods based on the configured baud rate. If the receiver samples the line during the Stop Bit period and finds a low voltage level (logical `0`), the hardware recognizes that synchronization has broken. It instantly triggers a **Framing Error (`FE`)** flag. This is usually caused by mismatched baud rates or severe signal noise.

---

### 3. Data Overrun Detection

An overrun error happens when the hardware receives new data faster than the system software can process it.

**The Hardware Mechanism:** UART hardware features a small buffer — often a shift register coupled with a single-byte holding register or a hardware FIFO (First-In, First-Out) buffer.

**The Check:** When a complete data frame is shifted in from the wire, the hardware attempts to move it into the holding register. If the previous byte has not been read by the CPU or DMA controller yet, the new byte will overwrite the unread data. The hardware detects this conflict and raises an **Overrun Error (`OE`)** flag to alert the system that data was permanently lost.

---

## Summary of Error Detection Flags

| Error Type          | Triggering Condition                                          | Primary Root Cause                         |
|---------------------|---------------------------------------------------------------|--------------------------------------------|
| Parity Error (`PE`) | Calculated bit count does not match the parity bit.          | Electrical noise flipped a single bit.     |
| Framing Error (`FE`)| Stop bit is sampled as a logical `0` instead of `1`.         | Mismatched baud rates or clock drift.      |
| Overrun Error (`OE`)| New data arrives before the old data is read.                | CPU bottleneck or slow software loop.      |