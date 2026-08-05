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
Raw BINARY, two bytes per reading (see fifo_bt_streamer in main.v),
round-robin between the sensors:

    byte 0 (high) : 0 0 0 | id1 id0 | PE | FE | OE
    byte 1 (low)  : data_byte[7:0]

    id (bits 4:3) : 0 = UV sensor  (the IR channel was removed from the FPGA)
    PE (bit 2)    : parity error flag
    FE (bit 1)    : framing error flag
    OE (bit 0)    : overrun error flag  (FIFO was full when this was queued)

The id is small, so a header byte's top nibble is always 0 (<= 0x0F);
that lets the receiver re-align if a byte is ever dropped.
The data byte is shown as a decimal value (with hex for reference).

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

# --------------------------------------------------------------------------
# Sensor metadata, keyed by the 2-bit channel id the FPGA tags each reading
# with (rx.v: UV_ID = 0, IR_ID = 1). Add entries here if you add channels.
# --------------------------------------------------------------------------
SENSORS = {
    0: {"label": "UV Sensor", "unit": ""},
}

# Live state: sensor_id -> {"data": int, "parity"/"framing"/"overrun": int, "time": float}
readings = {}
stats = {
    "bytes": 0,     # raw bytes received
    "words": 0,     # complete 2-byte readings decoded
    "resync": 0,    # bytes skipped to re-align to a packet boundary
    "parity": 0,    # readings that arrived with a parity error
    "framing": 0,   # ... framing error
    "overrun": 0,   # ... overrun error
}

# Byte-stream decoder state: the header byte awaiting its data byte.
_pending_hi = None


# --------------------------------------------------------------------------
# DECODE  ->  turn a (high, low) byte pair into a stored reading.
# --------------------------------------------------------------------------
def decode_word(hi, lo):
    sensor_id = (hi >> 3) & 0x03
    parity = (hi >> 2) & 0x01
    framing = (hi >> 1) & 0x01
    overrun = hi & 0x01

    readings[sensor_id] = {
        "data": lo,               # raw 8-bit value
        "parity": parity,
        "framing": framing,
        "overrun": overrun,
        "time": time.time(),
    }
    stats["words"] += 1
    if parity:
        stats["parity"] += 1
    if framing:
        stats["framing"] += 1
    if overrun:
        stats["overrun"] += 1


# --------------------------------------------------------------------------
# INGEST  ->  feed raw bytes; pairs them into readings and self-aligns.
# A valid header byte has a zero top nibble (only ids 0/1 are used, so
# bits 7:4 are always 0). Anything else while we expect a header is byte
# slippage -> skip it and try to re-sync on the next byte.
# --------------------------------------------------------------------------
def handle_bytes(chunk):
    global _pending_hi
    for b in chunk:
        stats["bytes"] += 1
        if _pending_hi is None:
            if (b & 0xF0) != 0x00:
                stats["resync"] += 1      # not a plausible header; drop to re-align
                continue
            _pending_hi = b
        else:
            decode_word(_pending_hi, b)
            _pending_hi = None


# --------------------------------------------------------------------------
# DISPLAY  ->  redraws a fixed dashboard in place using ANSI escape codes.
# Box content is kept plain (no color codes) so column widths stay aligned.
# --------------------------------------------------------------------------
def render(status_note=""):
    # \033[H moves cursor to top-left, \033[J clears from there down.
    out = ["\033[H\033[J"]
    width = 46
    now = time.time()

    out.append("╔" + "═" * width + "╗")
    out.append("║" + "UART SENSOR HUB".center(width) + "║")
    out.append("╠" + "═" * width + "╣")

    # Known channels in id order, plus any unexpected ids the FPGA sent.
    ids = list(SENSORS.keys())
    for sid in readings:
        if sid not in ids:
            ids.append(sid)

    for sid in ids:
        label = SENSORS.get(sid, {}).get("label", f"Channel {sid}")
        unit = SENSORS.get(sid, {}).get("unit", "")
        data = readings.get(sid)

        if data is None:
            line = f"   {label:<10} -- no data --"
        else:
            raw = data["data"]
            # Show the byte as a decimal number (hex kept in parens for reference).
            value_str = f"{raw:>3} (0x{raw:02X}) {unit}".rstrip()
            flags = []
            if data["parity"]:
                flags.append("PAR")
            if data["framing"]:
                flags.append("FRM")
            if data["overrun"]:
                flags.append("OVR")
            flag_str = ("ERR:" + ",".join(flags)) if flags else "ok"
            age = now - data["time"]
            age_mark = "●" if age < 3 else "○"   # fresh (<3s) vs stale
            line = f" {age_mark} {label:<10} {value_str:<15} {flag_str}"

        out.append("║" + line.ljust(width)[:width] + "║")

    out.append("╠" + "═" * width + "╣")
    status = (f" words:{stats['words']} bytes:{stats['bytes']} "
              f"resync:{stats['resync']}")
    out.append("║" + status.ljust(width)[:width] + "║")
    errs = (f" errs  parity:{stats['parity']} framing:{stats['framing']} "
            f"overrun:{stats['overrun']}")
    out.append("║" + errs.ljust(width)[:width] + "║")
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
# SIMULATION MODE  ->  no hardware; synthesizes the SAME 2-byte binary
# packets the FPGA emits and feeds them through the real decode path.
# --------------------------------------------------------------------------
def make_packet(sensor_id, data, parity=0, framing=0, overrun=0):
    hi = ((sensor_id & 0x03) << 3) | (parity << 2) | (framing << 1) | overrun
    return bytes([hi & 0xFF, data & 0xFF])


def run_simulation():
    print("Simulation mode - generating fake sensor packets. Ctrl+C to quit.")
    time.sleep(1)
    state = {0: 40}   # id -> current raw value (UV only)
    last_draw = 0
    try:
        while True:
            for sid in state:
                state[sid] = max(0, min(255, state[sid] + random.randint(-8, 8)))
                # Inject an occasional error flag so the dashboard shows them off.
                parity = 1 if random.random() < 0.05 else 0
                framing = 1 if random.random() < 0.02 else 0
                handle_bytes(make_packet(sid, state[sid], parity, framing))
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
