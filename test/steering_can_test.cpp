#include <cassert>
#include <cmath>
#include <cstdint>

#include "steering_can/steering_can.hpp"

int main() {
  amp::can::Frame frame{};
  assert(amp::steering::setPosition(104, 25.5, frame));
  assert(frame.extended);
  assert(!frame.remote);
  assert(frame.id == 0x0468);
  assert(frame.length == 4);
  assert(frame.data[0] == 0x00 && frame.data[1] == 0x03);
  assert(frame.data[2] == 0xE4 && frame.data[3] == 0x18);

  assert(amp::steering::setPosition(11, -25.5, frame));
  assert(frame.id == 0x040B);
  assert(frame.data[0] == 0xFF && frame.data[1] == 0xFC);
  assert(frame.data[2] == 0x1B && frame.data[3] == 0xE8);

  frame.remote = true;
  frame.data[7] = 0xFF;
  assert(amp::steering::setPosition(10, 0.0, frame));
  assert(!frame.remote);
  for (const uint8_t byte : frame.data) assert(byte == 0);

  assert(amp::steering::setPosition(10, 36000.0, frame));
  assert(frame.data[0] == 0x15 && frame.data[1] == 0x75);
  assert(frame.data[2] == 0x2A && frame.data[3] == 0x00);
  assert(amp::steering::setPosition(10, -36000.0, frame));
  assert(frame.data[0] == 0xEA && frame.data[1] == 0x8A);
  assert(frame.data[2] == 0xD6 && frame.data[3] == 0x00);

  assert(amp::steering::setPosition(10, 1.23456, frame));
  assert(frame.data[2] == 0x30 && frame.data[3] == 0x39);
  assert(amp::steering::setPosition(10, -1.23456, frame));
  assert(frame.data[2] == 0xCF && frame.data[3] == 0xC7);

  assert(!amp::steering::setPosition(10, 36000.1, frame));
  assert(!amp::steering::setPosition(10, -36000.1, frame));
  assert(!amp::steering::setPosition(10, NAN, frame));
  assert(!amp::steering::setPosition(10, INFINITY, frame));
  assert(!amp::steering::setPosition(10, -INFINITY, frame));
  assert(!amp::steering::setPosition(255, 0.0, frame));
}
