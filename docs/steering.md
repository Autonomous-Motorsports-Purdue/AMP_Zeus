# Steering Actuator

## Maintained By

- <NAME>
  - Last Worked On: Fall 2026
  - Email: <EMAIL>@purdue.edu
  - Discord: <USERNAME>

- <NAME>
  - Last Worked On: Fall 2026
  - Email: <EMAIL>@purdue.edu
  - Discord: <USERNAME>

## Hardware

Uses the [CubeMars AK80-64 KV80](https://www.cubemars.com/product/ak80-64-kv80-robotic-actuator.html)
with its integrated driver board. There is no separate steering ESP32 or PWM
output. Connect the actuator's CAN-H and CAN-L to the bridge/VESC CAN bus,
following CubeMars' wiring and grounding requirements. Terminate only the two
ends of the bus with 120 ohms. Supply the actuator from a suitable 24/48 V motor
power source, not from the bridge.

## Firmware

Build and flash only the `bridge` PlatformIO environment. Configure the actuator
using CubeMars' setup tool for **servo mode**, CAN ID **104**, and **500 kbit/s**
to match `STEERING_CAN_ID` and `CAN_BITRATE` in `platformio.ini`. Its ID must
differ from the VESC's (currently 62): the servo command identifiers overlap
with VESC commands. MIT mode is not supported by this implementation.

Send a newline-terminated bridge command:

```json
{"id":1,"target":"steering","position_deg":0.0}
```

The bridge sends one extended data frame with ID `(4 << 8) | STEERING_CAN_ID`
(`0x0468` by default), containing a big-endian signed 32-bit position scaled by
10,000 and truncated toward zero. This replaces the custom `0xF0` one-byte PWM
command. Protocol reference: CubeMars' [AK 2.0 driver manual V1.0.18](https://www.cubemars.com/data/cms/202605/ak-series-driver-manual-v1-0-18-for-ak-2-0-robotic-actuator.pdf),
sections 5.1 and 5.1.5.

### Reset temporary origin

With the motor stationary at the intended zero position and no pending motion
commands, type this in `py/control.py`:

```text
steering reset_origin=true
```

The USB JSON equivalent is `{"target":"steering","reset_origin":true}`.
This sends an extended CAN frame with ID `(5 << 8) | STEERING_CAN_ID`
(`0x0568` for node 104), DLC 1, payload `00`, per manual section 5.1.6.
It makes the current position temporary zero; it does not seek a home position,
command a stop, or write permanent calibration. Zero is lost on actuator power
loss. Do not combine it with `position_deg` in one request. A queued response
still does not confirm the motor received or applied it. Subsequent position
commands use the new origin, so verify it before commanding travel.

## Commissioning and limitations

- `position_deg` is in the actuator firmware's position coordinate system.
  Verify its relationship to the 64:1 gearbox output and steering linkage on
  the bench; no gearbox conversion, center offset, or road-wheel mapping is
  applied by the bridge.
- The accepted ±36,000 degree range is a **protocol limit, not safe steering
  travel**. Establish zero, direction, mechanical travel limits, current limits,
  and controller tuning before connecting the steering linkage.
- Position mode moves toward the target at maximum speed. This implementation
  does not provide velocity/acceleration limiting or a steering safety envelope.
  Test initially unloaded with an independent emergency stop available.
- The bridge does not automatically zero, command a starting position, or
  repeat commands. A queued response is not actuator acknowledgement. Measured
  steering feedback is not yet decoded.
- There is no bridge command watchdog or automatic stop on USB disconnect.
  Validate the actuator's communication-loss behavior before powered vehicle
  operation; do not rely on this firmware as a safety controller.
