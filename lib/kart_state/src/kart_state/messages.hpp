#pragma once

#include "amp_can/message_codec.hpp"
#include "kart_state/state.hpp"

namespace amp::kart {

// node_id is the broadcaster; state.writer remains the original writer.
struct StateMessage {
  static constexpr uint8_t packet = 0x70;
  static constexpr uint8_t payload_size = 8;
  uint8_t node_id = 0;
  State state{};

  void serialize(uint8_t *data) const {
    data[0] = static_cast<uint8_t>(state.mode);
    data[1] = state.stop_generation;
    data[2] = state.stop_generation >> 8;
    for (uint8_t i = 0; i < 4; ++i)
      data[3 + i] = state.revision >> (8 * i);
    data[7] = state.writer;
  }

  bool deserialize(const uint8_t *data) {
    state.mode = static_cast<Mode>(data[0]);
    state.stop_generation = uint16_t(data[1]) | (uint16_t(data[2]) << 8);
    state.revision = 0;
    for (uint8_t i = 0; i < 4; ++i) {
      state.revision |= uint32_t(data[3 + i]) << (8 * i);
    }
    state.writer = data[7];
    return node_id != 255 && validState(state);
  }
};

struct StateRequest {
  static constexpr uint8_t packet = 0x71;
  static constexpr uint8_t payload_size = 0;
  uint8_t node_id = 0;

  void serialize(uint8_t *) const {}
  bool deserialize(const uint8_t *) { return node_id != 255; }
};

} // namespace amp::kart
