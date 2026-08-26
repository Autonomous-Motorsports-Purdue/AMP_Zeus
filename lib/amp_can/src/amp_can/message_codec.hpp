#pragma once

#include <cstdint>

#include "amp_can/can_bus.hpp"

namespace amp::can {

template <typename Message>
Frame encode(const Message& message) {
  // A typed AMP message is deliberately just a small struct.  It supplies a
  // one-byte packet number, a payload size, a node_id, and serialize/
  // deserialize methods.  Keeping that contract here avoids a registration
  // step or an inheritance hierarchy for every device protocol.
  static_assert(Message::packet <= 0xFF, "CAN packet must fit in 8 bits");
  static_assert(Message::payload_size <= 8, "CAN payload exceeds 8 bytes");
  Frame frame{};
  frame.id = (uint32_t(Message::packet) << 8) | message.node_id;
  frame.extended = true;
  frame.length = Message::payload_size;
  message.serialize(frame.data);
  return frame;
}

template <typename Message>
bool decode(const Frame& frame, Message& message) {
  static_assert(Message::packet <= 0xFF, "CAN packet must fit in 8 bits");
  static_assert(Message::payload_size <= 8, "CAN payload exceeds 8 bytes");
  if (!frame.extended || frame.remote || frame.id > 0xFFFF ||
      (frame.id >> 8) != Message::packet ||
      frame.length != Message::payload_size) {
    return false;
  }
  message.node_id = static_cast<uint8_t>(frame.id);
  return message.deserialize(frame.data);
}

template <typename Message>
bool decode(const Frame& frame, uint8_t node_id, Message& message) {
  return (frame.id & 0xFF) == node_id && decode(frame, message);
}

}  // namespace amp::can
