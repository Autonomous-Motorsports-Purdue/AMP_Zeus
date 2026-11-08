#!/usr/bin/env python3
"""Interactive USB bridge control: target field=value commands."""

import argparse
import json
import shlex
import sys
import threading

import serial


SHORTCUTS = {
    "steering": ("steering", "position_deg"),
    "throttle": ("vesc", "erpm"),
}


HELP = """Shortcuts:
  steering: 50   50 actuator protocol degrees, NOT road-wheel degrees
  throttle: 50   50 electrical RPM, NOT a percentage
Generic commands: <target> <field>=<value> [<field>=<value> ...]
  vesc erpm: 1000
  steering position_deg=50
  vesc erpm=0
  help           show commands
  quit           exit without sending a command
Use = or : between fields and values. No unit conversion is applied.
WARNING: No steering travel protection or disconnect watchdog. Use a hardware E-stop.
"""


def parse_command(line):
    lexer = shlex.shlex(line, posix=True, punctuation_chars=":=")
    lexer.whitespace_split = True
    lexer.commenters = ""
    parts = list(lexer)
    if len(parts) == 3 and parts[0] in SHORTCUTS and parts[1] == ":":
        target, field = SHORTCUTS[parts[0]]
        parts = [target, field, ":", parts[2]]
    if len(parts) < 4 or (len(parts) - 1) % 3:
        raise ValueError("Use <target> <field>=<value> [<field>=<value> ...]")
    command = {"target": parts[0]}
    for index in range(1, len(parts), 3):
        field, separator, text = parts[index:index + 3]
        if not field.isidentifier() or separator not in ("=", ":"):
            raise ValueError("Each field must use field=value or field: value")
        if field == "id" or field in command:
            raise ValueError(f"Reserved or repeated field: {field}")
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            value = text
        command[field] = value
    json.dumps(command, allow_nan=False)
    return command


def format_response(line):
    text = line.decode("utf-8", errors="replace").rstrip()
    try:
        message = json.loads(text)
    except json.JSONDecodeError:
        return f"< {text}"
    if isinstance(message, dict) and message.get("type") == "debug":
        return f"[debug] {text}"
    if isinstance(message, dict) and "debug" in message:
        debug = message.pop("debug")
        return f"< {json.dumps(message)}\n[debug] {json.dumps(debug)}"
    return f"< {text}"


def read_responses(port, stopped):
    pending = bytearray()
    while not stopped.is_set():
        try:
            line = port.read_until(b"\n", size=1024)
        except serial.SerialException as error:
            print(f"\nSerial read failed: {error}", file=sys.stderr)
            stopped.set()
            return
        if line:
            pending.extend(line)
            if pending.endswith(b"\n") or len(pending) >= 1024:
                print(f"\n{format_response(pending)}", flush=True)
                pending.clear()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Bridge USB serial port")
    args = parser.parse_args()
    try:
        port = serial.Serial(args.port, 115200, timeout=0.2, write_timeout=1)
    except serial.SerialException as error:
        print(f"Could not open bridge: {error}", file=sys.stderr)
        return 1

    stopped = threading.Event()
    reader = threading.Thread(target=read_responses, args=(port, stopped), daemon=True)
    reader.start()
    command_id = 0

    def send(command):
        nonlocal command_id
        command_id += 1
        command = {"id": command_id, **command}
        payload = (json.dumps(command, allow_nan=False) + "\n").encode("utf-8")
        if port.write(payload) != len(payload):
            raise serial.SerialException("Incomplete command write")
        print(f"> {json.dumps(command)}")

    print(HELP)
    print("Wait for the bridge to be ready before commanding motion. Nothing sent at startup.")
    result = 0
    try:
        while not stopped.is_set():
            line = input("control> ").strip()
            if not line:
                continue
            if line.lower() in ("quit", "exit"):
                break
            if line.lower() == "help":
                print(HELP)
                continue
            try:
                command = parse_command(line)
            except ValueError as error:
                print(error)
                continue
            if stopped.is_set():
                break
            send(command)
        if stopped.is_set():
            result = 1
    except (KeyboardInterrupt, EOFError):
        pass
    except serial.SerialException as error:
        print(f"Serial write failed: {error}", file=sys.stderr)
        result = 1
    finally:
        stopped.set()
        reader.join(timeout=1)
        port.close()
    return result


if __name__ == "__main__":
    sys.exit(main())
