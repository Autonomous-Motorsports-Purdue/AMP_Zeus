#pragma once
#include <cstdint>

namespace amp::can {

// A transport-neutral classic CAN frame. Any bus object with begin(),
// write(Frame, timeout), and read(Frame, timeout) can be used with Node.
struct Frame {
  uint32_t id = 0;
  uint8_t length = 0;
  bool extended = false;
  bool remote = false;
  uint8_t data[8]{};
};

}  // namespace amp::can
