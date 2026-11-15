# Bridge Module

## Maintained By

- Grace Yoder
  - Last Worked On: Fall 2026
  - Email: yoder177@purdue.edu
  - Discord: qyvo


> WARNING: docs are mostly slop, will update

## Hardware

Uses Standard [ESP32-S3-RS485-CAN](https://www.waveshare.com/wiki/ESP32-S3-RS485-CAN) Waveshare module.

> TODO: List Additional Hardware and Connections

## Firmware

The bridge connects a computer over USB serial to the 500 kbit/s CAN bus
(GPIO15 TX, GPIO16 RX). Use the ESP32-S3 native USB connection. The serial
monitor setting is 115200; native USB CDC does not use UART baud timing.

### USB JSON interface

Send one JSON object per line, terminated by LF or CRLF. Empty lines are ignored.
The maximum input line length is 512 bytes, excluding line endings. Oversized
lines are discarded through the next newline and produce `line_too_long`.

Commands:

```json
{"id":1,"target":"steering","position_deg":0.0}
{"id":2,"target":"vesc","erpm":1500}
{"id":3,"target":"steering","reset_origin":true}
```

- `id` is optional. If supplied, it must be an unsigned 32-bit integer and is
  echoed in the response to a valid JSON request.
- Steering directly targets the CubeMars AK80-64 KV80 at `STEERING_CAN_ID`
  (currently 104). `position_deg` is a finite number from -36000 to 36000 in
  CubeMars servo protocol degrees, not a road-wheel angle. The old `position`
  PWM field is no longer supported. See [steering setup](steering.md) before use.
- VESC targets `VESC_CAN_ID` (currently 62 in `platformio.ini`). `erpm` is a
  signed 32-bit electrical RPM setpoint, not wheel RPM or throttle percentage.
- Steering `reset_origin:true` sets the current actuator position as temporary
  zero (cleared on power loss). It must be boolean `true` and cannot be combined
  with `position_deg`; otherwise the response is `reset_origin_must_be_true` or
  `reset_origin_conflicts_with_position_deg`. Only issue this while stationary,
  with no pending motion commands. It redefines zero; it is not homing or stop.
- Unknown fields are ignored. Missing fields, wrong types, unsupported targets,
  and out-of-range values produce errors without sending a CAN command.

Responses:

```json
{"id":1,"ok":true,"status":"queued"}
{"id":2,"ok":false,"error":"erpm_must_be_int32"}
```

`queued` means the CAN driver accepted the frame, not that the actuator applied
it. CAN submission does not wait for queue space; a full queue produces
`can_transmit_failed`. Other errors include `invalid_json`, `id_must_be_uint32`,
`position_deg_must_be_number_minus36000_to_36000`, `unknown_target`, and
`can_not_ready`.
Malformed JSON and invalid IDs receive errors without an ID.

Startup emits `{"type":"ready","can_ready":true}` (or `false` if CAN failed).
The host should tolerate non-JSON bootloader output during reset.

### Interactive Python control

Run the script against the bridge's native USB serial port (not the Waveshare
USB-CAN adapter):

```sh
uv run python py/control.py --port /dev/cu.usbmodemXXXX
```

Use the shortcuts:

```text
steering: 50
throttle: 50
```

`steering: 50` sends `{"target":"steering","position_deg":50}` (actuator
protocol degrees). `throttle: 50` sends `{"target":"vesc","erpm":50}` directly:
it means **50 electrical RPM, not 50% throttle**.

Generic `<target> <field>=<value> [<field>=<value> ...]` commands also work,
with either `=` or `:` as the separator:

```text
vesc erpm: 1000
steering position_deg=50
steering reset_origin=true
vesc erpm=0
```

Generic targets and fields are forwarded unchanged; shortcuts only rename the
target/field, without unit conversion. Values are parsed as JSON numbers,
booleans, or null when possible;
otherwise they are strings. Quote strings containing spaces. Request IDs are
assigned automatically. Firmware still determines supported targets, fields,
and ranges; a generic parser does not add new firmware commands.

The script displays bridge responses and telemetry. `help` shows syntax;
`quit`, Ctrl-C, and EOF close the connection **without sending a stop command**.
To request zero VESC ERPM, explicitly send `vesc erpm=0` before exiting.
No command is sent on startup or exit. There is no disconnect watchdog, and
zero ERPM is not a hardware emergency stop. Wait for the bridge to be ready and
follow the [steering commissioning precautions](steering.md) before use.

### Serial diagnostics

Startup reports CAN bitrate, TX/RX GPIOs, and steering/VESC node IDs.
Command responses include a `debug` object with the bridge timestamp and
`command_rejected`, `can_tx_queued`, or `can_tx_failed`. Transmit attempts also
include the CAN ID, frame flags, DLC, and payload as hexadecimal bytes.

Once per second, when serial output is free, a `{"type":"debug","event":"can_status",...}`
line reports CAN driver state, pending queues, error counters, total received
frames, and the latest received frame (if any). This is a sampled diagnostic,
not a complete frame trace. Python displays these as `[debug]` lines and keeps
partial serial reads together.

`can_tx_queued` only means accepted into the local driver queue, not delivered
or applied. Increasing TX errors or `bus_off` warrant checking power, matching
bitrate, CAN-H/CAN-L wiring, grounding, and termination. A steady activity LED
alone cannot confirm successful communication.

### Telemetry and scheduling

The bridge publishes the newest received VESC status at up to 10 Hz:

```json
{"type":"vesc","node":62,"ts_ms":12540,"erpm":1500,"current_a":4.2,"duty":0.12}
```

`ts_ms` is bridge receive time in milliseconds since boot (wrapping after about
49.7 days). `current_a` is motor current in amps; `duty` is a duty-cycle fraction.
The VESC must be configured to broadcast its CAN status messages. No telemetry
is emitted before a status arrives or repeated without a new status.

One cooperative loop reads bounded chunks of USB input, dispatches commands,
polls up to eight CAN frames, and publishes telemetry. Output is buffered one
line at a time and drained only when USB has space. A pending response is sent
before another command is read. Telemetry is coalesced to the newest received
status and waits behind command traffic. A host that stops reading can delay
command acceptance, but does not stop CAN polling. The host should continuously
read output and avoid building a backlog of motion commands.

The bridge owns CAN packet interpretation for all supported modules.
`handleCanFrame()` is the shared receive entry point; it currently calls
`cacheVescStatus()`. Add INS, GPS, or steering-feedback decoding there alongside
the VESC decoder, with separately named state and publication functions such as
`publishInsTelemetry()`. JSON parsing and serial output remain shared.

Add future commands by extending the `target` dispatch in `handleLine()`.
GPS, INS, and measured steering feedback are not yet
implemented. There is currently no command timeout or automatic stop on USB
disconnect; actuator timeout behavior must be defined separately.
