#pragma once

#include <cstdint>

namespace amp::steering {

struct PositionCommand {
  static constexpr uint8_t packet = 0xF0;
  static constexpr uint8_t payload_size = 1;

  uint8_t node_id;
  uint8_t position;

  PositionCommand(uint8_t node = 10, uint8_t value = 128)
      : node_id(node), position(value) {}

  void serialize(uint8_t* data) const { data[0] = position; }

  bool deserialize(const uint8_t* data) {
    position = data[0];
    return true;
  }
};

}  // namespace amp::steering
