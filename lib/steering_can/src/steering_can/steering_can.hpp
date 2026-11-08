#pragma once

#include "amp_can/can_bus.hpp"

namespace amp::steering {

// CubeMars AK servo-mode position command, in the actuator's protocol degrees.
bool setPosition(uint8_t controller_id, double position_deg, can::Frame& frame);

}  // namespace amp::steering
