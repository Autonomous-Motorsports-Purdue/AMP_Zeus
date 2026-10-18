# Bridge Module

## Maintained By

- Grace Yoder
  - Last Worked On: Fall 2026
  - Email: yoder177@purdue.edu
  - Discord: qyvo


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
{"id":1,"target":"steering","position":128}
{"id":2,"target":"vesc","erpm":1500}
```

- `id` is optional. If supplied, it must be an unsigned 32-bit integer and is
  echoed in the response to a valid JSON request.
- Steering targets CAN node 10. `position` is an integer from 0 to 255, mapped
  directly to the steering module's PWM output, not a measured steering angle.
- VESC targets `VESC_CAN_ID` (currently 62 in `platformio.ini`). `erpm` is a
  signed 32-bit electrical RPM setpoint, not wheel RPM or throttle percentage.
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
`position_must_be_integer_0_to_255`, `unknown_target`, and `can_not_ready`.
Malformed JSON and invalid IDs receive errors without an ID.

Startup emits `{"type":"ready","can_ready":true}` (or `false` if CAN failed).
The host should tolerate non-JSON bootloader output during reset.

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
