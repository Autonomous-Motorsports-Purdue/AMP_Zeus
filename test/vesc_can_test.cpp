#include <cassert>
#include <cmath>
#include <cstdint>

#include "vesc_can/vesc_can.hpp"

int main() {
  using amp::can::Frame;
  using amp::vesc::Packet;

  const Frame rpm = amp::vesc::setRpm(23, -12000);
  assert(rpm.extended);
  assert(!rpm.remote);
  assert(rpm.id == 0x0317);
  assert(rpm.length == 4);
  assert(rpm.data[0] == 0xFF && rpm.data[1] == 0xFF);
  assert(rpm.data[2] == 0xD1 && rpm.data[3] == 0x20);

  Frame current{};
  assert(amp::vesc::setCurrent(23, 51.0f, current));
  assert(current.id == 0x0117);
  assert(current.data[0] == 0x00 && current.data[1] == 0x00);
  assert(current.data[2] == 0xC7 && current.data[3] == 0x38);
  assert(!amp::vesc::setCurrent(23, NAN, current));

  Frame duty{};
  assert(amp::vesc::setDuty(4, -0.5f, duty));
  assert(duty.id == 0x0004);
  assert(duty.data[0] == 0xFF && duty.data[1] == 0xFF);
  assert(duty.data[2] == 0x3C && duty.data[3] == 0xB0);
  assert(!amp::vesc::setDuty(4, 1.01f, duty));

  Frame raw_status{};
  raw_status.id = amp::vesc::makeId(Packet::Status, 7);
  raw_status.extended = true;
  raw_status.length = 8;
  const uint8_t payload[8] = {0x00, 0x00, 0x2E, 0xE0,
                              0xFF, 0x85, 0x01, 0xF4};
  for (uint8_t i = 0; i < 8; ++i) raw_status.data[i] = payload[i];

  amp::vesc::Status status{};
  assert(amp::vesc::decodeStatus(raw_status, status));
  assert(status.controller_id == 7);
  assert(status.electrical_rpm == 12000);
  assert(std::fabs(status.motor_current_amps - -12.3f) < 0.001f);
  assert(std::fabs(status.duty_cycle - 0.5f) < 0.001f);

  raw_status.remote = true;
  assert(!amp::vesc::decodeStatus(raw_status, status));
  raw_status.remote = false;
  raw_status.id |= 0x10000;
  assert(!amp::vesc::decodeStatus(raw_status, status));
}
