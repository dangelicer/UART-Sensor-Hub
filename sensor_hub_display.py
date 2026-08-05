#!/usr/bin/env python3
"""
UART Sensor Hub - Host PC Receiver & Display  (BLE / HM-10 version)
==================================================================
Receives sensor data from the FPGA over the HM-10 Bluetooth Low Energy
module and shows a live dashboard.

WHY BLE AND NOT A COM PORT
--------------------------
The HM-10 (e.g. the "DSD TECH" module) is a Bluetooth LOW ENERGY device.
Windows only creates "Standard Serial over Bluetooth link" COM ports for
CLASSIC Bluetooth (SPP) devices -- BLE modules never get a COM port, so
pyserial cannot talk to them. Instead, the HM-10 exposes its serial stream
as a GATT characteristic:

    service        FFE0  (0000ffe0-0000-1000-8000-00805f9b34fb)
    characteristic FFE1  (0000ffe1-0000-1000-8000-00805f9b34fb)

Bytes the FPGA sends arrive as notifications on FFE1. Holding that
notification subscription open is also what keeps the BLE link from
idle-disconnecting after a few seconds.

WIRE FORMAT (what the FPGA actually sends)
------------------------------------------
Raw BINARY, one ambient-light reading per 4-byte frame (see bt_als_framer
in main.v). The UV sensor is a DFRobot SEN0540 (LTR390-UV) read over Modbus
by the FPGA; the FPGA forwards each reading as:

    byte 0 : 0xAA        sync
    byte 1 : als[23:16]  (MSB)
    byte 2 : als[15:8]
    byte 3 : als[7:0]    (LSB)

The reading is the raw ambient-light value (up to 20 bits):
    als = byte1 << 16 | byte2 << 8 | byte3
The dashboard converts that raw count to lux (see als_to_lux). The 0xAA
sync byte lets the receiver re-align if a byte is ever dropped.

USAGE
-----
    pip install bleak            # one-time setup (BLE library)

    # List nearby BLE devices to find yours:
    python sensor_hub_display.py --scan

    # Connect by name (default name is "DSD TECH"):
    python sensor_hub_display.py

    # Or connect directly by address (faster, skips the scan):
    python sensor_hub_display.py --address 84:C6:92:F1:3F:BB

    # Test the display with fake data (no hardware/BLE needed):
    python sensor_hub_display.py --simulate
"""

import argparse
import os
import sys
import time
import random
from datetime import datetime

# Enable ANSI escape processing on Windows 10+ consoles (no-op elsewhere).
if os.name == "nt":
    os.system("")

# The dashboard uses box-drawing/bullet characters; Windows stdout defaults to
# cp1252 and would crash on them, so force UTF-8 output where supported.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, ValueError):
    pass

# HM-10 "transparent UART" GATT service/characteristic.
HM10_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
HM10_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
DEFAULT_NAME = "DSD TECH"

SENSOR_LABEL = "Ambient Light"
SYNC = 0xAA

# --- Ambient-light (ALS) raw-count -> lux ---------------------------------
# The FPGA configures the LTR390UV for GAIN 3 and 18-bit resolution (100 ms
# integration) -- see the boot register writes in main.v. The LTR390 / DFRobot
# ALS-mode lux formula is:
#     lux = 0.6 * raw / (gain * int_factor)
# with int_factor = 1.0 for the 18-bit setting. If you change the GAIN or
# MEAS_RATE writes in main.v, update these constants to match.
ALS_GAIN = 3.0
ALS_INT_FACTOR = 1.0
ALS_WINDOW = 0.6


def als_to_lux(raw):
    return ALS_WINDOW * raw / (ALS_GAIN * ALS_INT_FACTOR)

# Live state: the latest ambient-light reading.
reading = {"als": None, "time": 0.0}
stats = {
    "bytes": 0,     # raw bytes received
    "frames": 0,    # complete 4-byte frames decoded
    "resync": 0,    # bytes skipped while hunting for a 0xAA sync
}

# Decoder state: None = waiting for the 0xAA sync; else the data bytes so far.
_buf = None


# --------------------------------------------------------------------------
# INGEST  ->  feed raw bytes; assemble 0xAA + 3-byte frames and self-align.
# While waiting for a frame we skip anything that isn't the 0xAA sync byte;
# once synced we collect the next 3 bytes as the 24-bit ambient-light value.
# --------------------------------------------------------------------------
def handle_bytes(chunk):
    global _buf
    for b in chunk:
        stats["bytes"] += 1
        if _buf is None:
            if b == SYNC:
                _buf = []
            else:
                stats["resync"] += 1      # not a sync byte; skip to re-align
        else:
            _buf.append(b)
            if len(_buf) == 3:
                als = (_buf[0] << 16) | (_buf[1] << 8) | _buf[2]
                reading["als"] = als
                reading["time"] = time.time()
                stats["frames"] += 1
                _buf = None


# --------------------------------------------------------------------------
# DISPLAY  ->  redraws a fixed dashboard in place using ANSI escape codes.
# Box content is kept plain (no color codes) so column widths stay aligned.
# --------------------------------------------------------------------------
def render(status_note=""):
    # \033[H moves cursor to top-left, \033[J clears from there down.
    out = ["\033[H\033[J"]
    width = 50
    now = time.time()

    out.append("╔" + "═" * width + "╗")
    out.append("║" + "UART SENSOR HUB".center(width) + "║")
    out.append("╠" + "═" * width + "╣")

    als = reading["als"]
    if als is None:
        line = f"   {SENSOR_LABEL:<14} -- no data --"
    else:
        lux = als_to_lux(als)
        age = now - reading["time"]
        age_mark = "●" if age < 3 else "○"   # fresh (<3s) vs stale
        line = f" {age_mark} {SENSOR_LABEL:<14} {lux:>9.1f} lux  (raw {als})"
    out.append("║" + line.ljust(width)[:width] + "║")

    out.append("╠" + "═" * width + "╣")
    status = (f" frames:{stats['frames']} bytes:{stats['bytes']} "
              f"resync:{stats['resync']}")
    out.append("║" + status.ljust(width)[:width] + "║")
    clock = " " + datetime.now().strftime("%H:%M:%S") + "   ● live   ○ stale"
    out.append("║" + clock.ljust(width)[:width] + "║")
    out.append("╚" + "═" * width + "╝")
    if status_note:
        out.append(status_note)
    out.append("Press Ctrl+C to quit.")

    sys.stdout.write("\n".join(out) + "\n")
    sys.stdout.flush()


# --------------------------------------------------------------------------
# BLE MODE  ->  real hardware over the HM-10 (Bluetooth Low Energy).
# --------------------------------------------------------------------------
def run_ble(address, name):
    try:
        import asyncio
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("bleak is not installed. Run:  pip install bleak")
        sys.exit(1)

    try:
        asyncio.run(_ble_main(address, name, BleakClient, BleakScanner))
    except KeyboardInterrupt:
        print("\nStopped.")


async def _ble_main(address, name, BleakClient, BleakScanner):
    import asyncio

    # Resolve a device if no explicit address was given.
    if not address:
        target = name or DEFAULT_NAME
        print(f"Scanning for BLE device named '{target}' (up to 15s)...")
        device = await BleakScanner.find_device_by_name(target, timeout=15.0)
        if device is None:
            print(f"Could not find '{target}'. Is the module powered and advertising?")
            print("Run  --scan  to list nearby devices, or pass  --address AA:BB:...")
            return
        address = device.address
        print(f"Found '{target}' at {address}.")

    disconnected = asyncio.Event()

    def on_disconnect(_client):
        disconnected.set()

    def on_notify(_sender, data):
        handle_bytes(bytes(data))

    print(f"Connecting to {address} ...")
    client = BleakClient(address, disconnected_callback=on_disconnect)
    async with client:
        await client.start_notify(HM10_CHAR_UUID, on_notify)
        note = f" BLE: connected to {address}"
        last_draw = 0.0
        while client.is_connected and not disconnected.is_set():
            now = time.time()
            if now - last_draw > 0.1:
                render(note)
                last_draw = now
            await asyncio.sleep(0.05)
    if disconnected.is_set():
        print("\nBLE link dropped. Re-run to reconnect.")


def run_scan():
    try:
        import asyncio
        from bleak import BleakScanner
    except ImportError:
        print("bleak is not installed. Run:  pip install bleak")
        sys.exit(1)

    async def _scan():
        print("Scanning 10s for BLE devices...\n")
        devices = await BleakScanner.discover(timeout=10.0)
        if not devices:
            print("No BLE devices found. Make sure the module is powered.")
            return
        print(f"{'ADDRESS':<20} NAME")
        for d in sorted(devices, key=lambda x: (x.name or "~")):
            print(f"{d.address:<20} {d.name or '(unknown)'}")
        print("\nUse:  python sensor_hub_display.py --address <ADDRESS>")

    try:
        asyncio.run(_scan())
    except KeyboardInterrupt:
        pass


# --------------------------------------------------------------------------
# SIMULATION MODE  ->  no hardware; synthesizes the SAME 4-byte frames the
# FPGA emits and feeds them through the real decode path.
# --------------------------------------------------------------------------
def make_frame(als):
    als &= 0xFFFFFF
    return bytes([SYNC, (als >> 16) & 0xFF, (als >> 8) & 0xFF, als & 0xFF])


def run_simulation():
    print("Simulation mode - generating fake ambient-light frames. Ctrl+C to quit.")
    time.sleep(1)
    als = 12000                        # raw ambient-light value
    last_draw = 0
    try:
        while True:
            als = max(0, min(0xFFFFF, als + random.randint(-800, 800)))
            handle_bytes(make_frame(als))
            if time.time() - last_draw > 0.1:
                render(" SIMULATION")
                last_draw = time.time()
            time.sleep(0.8)
    except KeyboardInterrupt:
        print("\nStopped.")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="UART Sensor Hub display (HM-10 BLE)")
    ap.add_argument("--address", help="BLE address of the HM-10, e.g. 84:C6:92:F1:3F:BB")
    ap.add_argument("--name", default=DEFAULT_NAME,
                    help=f"BLE device name to scan for (default: {DEFAULT_NAME!r})")
    ap.add_argument("--scan", action="store_true", help="List nearby BLE devices and exit")
    ap.add_argument("--simulate", action="store_true", help="Run with fake data (no BLE)")
    args = ap.parse_args()

    if args.scan:
        run_scan()
    elif args.simulate:
        run_simulation()
    else:
        run_ble(args.address, args.name)


if __name__ == "__main__":
    main()
