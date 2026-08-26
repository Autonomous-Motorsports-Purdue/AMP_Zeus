#!/usr/bin/env python3
"""
Waveshare USB-CAN-A monitor/sender for macOS (and Linux).

Uses the adapter's variable-length serial protocol (not SocketCAN, not
python-can) since the USB-CAN-A speaks its own framed protocol over a
plain virtual serial port.

Usage:
    python3 waveshare_can_monitor.py --port /dev/tty.usbserial-XXXX --can-baud 500000

Then either just watch traffic scroll by, or type at the prompt:
    send 123 11 22 33 44          -> send standard frame ID 0x123, data bytes
    send 1abcdef 11 22 33 44 -e   -> send extended frame ID 0x1abcdef
    quit                          -> exit
"""

import argparse
import sys
import threading
import time

import serial

# --- CAN baud rate codes, per Waveshare's config protocol ---
CAN_BAUD_CODES = {
    1000000: 0x01,
    800000: 0x02,
    500000: 0x03,
    400000: 0x04,
    250000: 0x05,
    200000: 0x06,
    125000: 0x07,
    100000: 0x08,
    50000: 0x09,
    20000: 0x0A,
    10000: 0x0B,
    5000: 0x0C,
}

HEADER = 0xAA
FOOTER = 0x55


def build_config_frame(can_baud, extended_default=False):
    """Fixed 20-byte config frame: sets CAN bitrate, accepts all IDs, normal mode."""
    if can_baud not in CAN_BAUD_CODES:
        raise ValueError(
            f"Unsupported CAN baud {can_baud}. Choices: {sorted(CAN_BAUD_CODES)}"
        )

    frame = [HEADER, FOOTER, 0x12]  # header, header, Type=Set
    frame.append(CAN_BAUD_CODES[can_baud])  # CAN baud rate code
    frame.append(0x02 if extended_default else 0x01)  # frame type
    frame += [0x00, 0x00, 0x00, 0x00]  # filter ID (accept all)
    frame += [0x00, 0x00, 0x00, 0x00]  # mask ID (accept all)
    frame.append(0x00)  # CAN mode: normal
    frame.append(0x00)  # auto-retransmit: enabled
    frame += [0x00, 0x00, 0x00, 0x00]  # reserved
    checksum = sum(frame[2:19]) & 0xFF
    frame.append(checksum)
    return bytes(frame)


def build_data_frame(can_id, data, extended=False, remote=False):
    """Variable-length data frame: AA <type> <ID> <data...> 55."""
    dlc = len(data)
    if dlc > 8:
        raise ValueError("CAN data payload must be 0-8 bytes")

    type_byte = 0xC0
    if extended:
        type_byte |= 0x20
    if remote:
        type_byte |= 0x10
    type_byte |= dlc

    id_bytes = 4 if extended else 2
    id_field = [(can_id >> (8 * i)) & 0xFF for i in range(id_bytes)]  # little-endian

    frame = [HEADER, type_byte] + id_field + list(data) + [FOOTER]
    return bytes(frame)


def parse_stream(buf):
    """
    Try to pull one complete frame out of the front of buf.
    Returns (parsed_dict_or_None, remaining_buf).
    """
    # Resync: drop bytes until we find a header
    while buf and buf[0] != HEADER:
        buf = buf[1:]

    if len(buf) < 3:
        return None, buf  # not enough data yet

    type_byte = buf[1]
    if (type_byte & 0xC0) != 0xC0:
        # Not a valid type byte right after header; drop the header and resync
        return None, buf[1:]

    extended = bool(type_byte & 0x20)
    remote = bool(type_byte & 0x10)
    dlc = type_byte & 0x0F
    id_bytes = 4 if extended else 2
    total_len = 2 + id_bytes + dlc + 1  # header + type + id + data + footer

    if len(buf) < total_len:
        return None, buf  # wait for more bytes

    if buf[total_len - 1] != FOOTER:
        # Malformed frame; drop just the header byte and try to resync
        return None, buf[1:]

    id_field = buf[2 : 2 + id_bytes]
    can_id = 0
    for i, b in enumerate(id_field):
        can_id |= b << (8 * i)

    data = buf[2 + id_bytes : 2 + id_bytes + dlc]

    parsed = {
        "id": can_id,
        "extended": extended,
        "remote": remote,
        "dlc": dlc,
        "data": bytes(data),
    }
    return parsed, buf[total_len:]


def format_frame(parsed):
    id_str = f"{parsed['id']:08X}" if parsed["extended"] else f"{parsed['id']:03X}"
    kind = "EXT" if parsed["extended"] else "STD"
    rtr = "RTR" if parsed["remote"] else "   "
    data_str = " ".join(f"{b:02X}" for b in parsed["data"])
    ts = time.strftime("%H:%M:%S")
    return f"[{ts}] {kind} {rtr} ID={id_str}  DLC={parsed['dlc']}  DATA={data_str}"


def reader_thread(ser, stop_event, raw=False):
    buf = bytearray()
    while not stop_event.is_set():
        try:
            chunk = ser.read(64)
        except serial.SerialException as e:
            print(f"\n[serial error] {e}")
            stop_event.set()
            break
        if chunk:
            if raw:
                # Just dump exactly what came in, no parsing at all.
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] RAW ({len(chunk)} bytes): {chunk.hex(' ').upper()}")
                continue
            buf += chunk
            while True:
                parsed, buf_new = parse_stream(bytes(buf))
                if parsed is None and len(buf_new) == len(buf):
                    break  # need more data, nothing consumed
                buf = bytearray(buf_new)
                if parsed is not None:
                    print(format_frame(parsed))
                if not buf:
                    break


def main():
    ap = argparse.ArgumentParser(description="Waveshare USB-CAN-A monitor")
    ap.add_argument("--port", required=True, help="e.g. /dev/tty.usbserial-XXXX")
    ap.add_argument(
        "--serial-baud",
        type=int,
        default=2000000,
        help="USB-side baud (default 2000000)",
    )
    ap.add_argument(
        "--can-baud", type=int, default=500000, help="CAN bus bitrate (default 500000)"
    )
    ap.add_argument(
        "--extended", action="store_true", help="Default frame type = extended"
    )
    ap.add_argument(
        "--raw",
        action="store_true",
        help="Skip parsing entirely, just hex-dump every byte received (debug mode)",
    )
    ap.add_argument(
        "--no-config",
        action="store_true",
        help="Don't send the config/bitrate-set frame on startup (use adapter's current mode)",
    )
    args = ap.parse_args()

    print(f"Opening {args.port} @ {args.serial_baud} ...")
    ser = serial.Serial(args.port, args.serial_baud, timeout=0.2)

    if not args.no_config:
        cfg = build_config_frame(args.can_baud, extended_default=args.extended)
        ser.write(cfg)
        time.sleep(0.1)
        print(f"Sent config frame for {args.can_baud} bps: {cfg.hex(' ').upper()}")
    else:
        print(
            "Skipping config frame (--no-config) — using adapter's current saved settings."
        )

    stop_event = threading.Event()
    t = threading.Thread(
        target=reader_thread, args=(ser, stop_event, args.raw), daemon=True
    )
    t.start()

    print("Listening for CAN traffic. Commands:")
    print("  send <id_hex> <data bytes hex...> [-e]   e.g. send 123 11 22 33 -e")
    print("  quit")

    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            if line.lower() in ("quit", "exit"):
                break
            if line.lower().startswith("send"):
                parts = line.split()
                extended = "-e" in parts
                parts = [p for p in parts if p != "-e"]
                if len(parts) < 2:
                    print("Usage: send <id_hex> <data bytes hex...> [-e]")
                    continue
                try:
                    can_id = int(parts[1], 16)
                    data = bytes(int(b, 16) for b in parts[2:])
                except ValueError:
                    print("Bad hex value(s).")
                    continue
                frame = build_data_frame(can_id, data, extended=extended)
                ser.write(frame)
                print(f"Sent: {frame.hex(' ').upper()}")
            else:
                print("Unknown command.")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        stop_event.set()
        t.join(timeout=1)
        ser.close()
        print("\nClosed.")


if __name__ == "__main__":
    main()
