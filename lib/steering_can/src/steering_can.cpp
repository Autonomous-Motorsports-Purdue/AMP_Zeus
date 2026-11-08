#include "steering_can/steering_can.hpp"

#include <cmath>

namespace amp::steering {

bool setPosition(uint8_t controller_id, double position_deg, can::Frame& frame) {
  if (controller_id == 255 || !std::isfinite(position_deg) ||
      position_deg < -36000.0 || position_deg > 36000.0) {
    return false;
  }

  const int32_t position = static_cast<int32_t>(position_deg * 10000.0);
  const uint32_t bits = static_cast<uint32_t>(position);
  frame = can::Frame{};
  frame.id = (uint32_t{4} << 8) | controller_id;
  frame.extended = true;
  frame.length = 4;
  frame.data[0] = bits >> 24;
  frame.data[1] = bits >> 16;
  frame.data[2] = bits >> 8;
  frame.data[3] = bits;
  return true;
}

}  // namespace amp::steering
