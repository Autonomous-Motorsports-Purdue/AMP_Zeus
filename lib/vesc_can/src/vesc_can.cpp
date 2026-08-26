#include "vesc_can/vesc_can.hpp"

#include <cmath>
#include <limits>

namespace amp::vesc {
namespace {
void putI32(uint8_t* out, int32_t value) {
  const uint32_t bits = static_cast<uint32_t>(value);
  out[0] = bits >> 24;
  out[1] = bits >> 16;
  out[2] = bits >> 8;
  out[3] = bits;
}

int32_t getI32(const uint8_t* in) {
  return static_cast<int32_t>((uint32_t(in[0]) << 24) |
                              (uint32_t(in[1]) << 16) |
                              (uint32_t(in[2]) << 8) | uint32_t(in[3]));
}

int16_t getI16(const uint8_t* in) {
  return static_cast<int16_t>((uint16_t(in[0]) << 8) | uint16_t(in[1]));
}

can::Frame scalar(Packet packet, uint8_t id, int32_t value) {
  can::Frame frame{};
  frame.id = makeId(packet, id);
  frame.extended = true;
  frame.length = 4;
  putI32(frame.data, value);
  return frame;
}

bool scaled(float value, float scale, int32_t& result) {
  if (!std::isfinite(value)) return false;
  const double encoded = double(value) * scale;
  if (encoded > std::numeric_limits<int32_t>::max() ||
      encoded < std::numeric_limits<int32_t>::min()) {
    return false;
  }
  // VESC firmware encoders truncate toward zero when converting float to int32.
  result = static_cast<int32_t>(encoded);
  return true;
}
}  // namespace

uint32_t makeId(Packet packet, uint8_t id) {
  return (uint32_t(static_cast<uint8_t>(packet)) << 8) | id;
}

bool parseId(const can::Frame& frame, Address& out) {
  // VESC reserves EID bits 28..16. Upstream decodes the command as the full
  // value of EID >> 8, so accepting nonzero upper bits would misclassify a
  // non-VESC extended frame on a shared bus.
  if (!frame.extended || frame.remote || frame.id > 0xFFFF) return false;
  out.packet = static_cast<Packet>((frame.id >> 8) & 0xFF);
  out.controller_id = frame.id & 0xFF;
  return true;
}

can::Frame setRpm(uint8_t id, int32_t rpm) {
  return scalar(Packet::SetRpm, id, rpm);
}

bool setCurrent(uint8_t id, float amps, can::Frame& frame) {
  int32_t value = 0;
  if (!scaled(amps, 1000.0f, value)) return false;
  frame = scalar(Packet::SetCurrent, id, value);
  return true;
}

bool setDuty(uint8_t id, float duty, can::Frame& frame) {
  if (duty < -1.0f || duty > 1.0f) return false;
  int32_t value = 0;
  if (!scaled(duty, 100000.0f, value)) return false;
  frame = scalar(Packet::SetDuty, id, value);
  return true;
}

bool decodeStatus(const can::Frame& frame, Status& out) {
  Address address{};
  if (!parseId(frame, address) || address.packet != Packet::Status ||
      frame.length != 8) {
    return false;
  }
  out.controller_id = address.controller_id;
  out.electrical_rpm = getI32(frame.data);
  out.motor_current_amps = getI16(frame.data + 4) / 10.0f;
  out.duty_cycle = getI16(frame.data + 6) / 1000.0f;
  return true;
}

}  // namespace amp::vesc
