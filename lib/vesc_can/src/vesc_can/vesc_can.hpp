#pragma once

#include <cstdint>

#include "amp_can/can_bus.hpp"

namespace amp::vesc {

enum class Packet : uint8_t {
  SetDuty = 0,
  SetCurrent = 1,
  SetCurrentBrake = 2,
  SetRpm = 3,
  SetPosition = 4,
  Status = 9,
  Status2 = 14,
  Status3 = 15,
  Status4 = 16,
  Status5 = 27,
  Status6 = 58,
};

struct Address {
  Packet packet;
  uint8_t controller_id;
};

struct Status {
  uint8_t controller_id;
  int32_t electrical_rpm;
  float motor_current_amps;
  float duty_cycle;
};

uint32_t makeId(Packet packet, uint8_t controller_id);
bool parseId(const can::Frame& frame, Address& address);
can::Frame setRpm(uint8_t controller_id, int32_t rpm);
bool setCurrent(uint8_t controller_id, float amps, can::Frame& frame);
bool setDuty(uint8_t controller_id, float duty, can::Frame& frame);
bool decodeStatus(const can::Frame& frame, Status& status);

}  // namespace amp::vesc
