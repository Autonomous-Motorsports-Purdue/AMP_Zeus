#pragma once

#include "amp_can/can_bus.hpp"

namespace amp::steering {

// CubeMars AK servo-mode position command, in the actuator's protocol degrees.
bool setPosition(uint8_t controller_id, double position_deg, can::Frame& frame);

// Set the current position as temporary zero; cleared on actuator power loss.
bool resetOrigin(uint8_t controller_id, can::Frame& frame);

}  // namespace amp::steering
