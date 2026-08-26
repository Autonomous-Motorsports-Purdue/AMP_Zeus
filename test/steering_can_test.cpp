#include <cassert>
#include <cstdint>

#include "amp_can/can_node.hpp"
#include "amp_can/message_codec.hpp"
#include "steering_can/steering_can.hpp"

int main() {
  amp::steering::PositionCommand command(10, 25);
  amp::can::Frame frame = amp::can::encode(command);
  assert(frame.extended);
  assert(!frame.remote);
  assert(frame.id == 0xF00A);
  assert(frame.length == 1);
  assert(frame.data[0] == 25);

  amp::steering::PositionCommand decoded(10, 0);
  assert(amp::can::decode(frame, decoded));
  assert(decoded.node_id == 10);
  assert(decoded.position == 25);

  uint32_t calls = 0;
  amp::steering::PositionCommand received;
  auto route = amp::can::on<amp::steering::PositionCommand>(
      10, [&](const amp::steering::PositionCommand& message) {
        ++calls;
        received = message;
      });
  route.dispatch(frame);
  assert(calls == 1);
  assert(received.node_id == 10);
  assert(received.position == 25);

  amp::steering::PositionCommand other_node(11, 50);
  amp::can::Frame other_frame = amp::can::encode(other_node);
  route.dispatch(other_frame);
  assert(calls == 1);

  frame.remote = true;
  assert(!amp::can::decode(frame, decoded));
  frame.remote = false;
  frame.length = 2;
  assert(!amp::can::decode(frame, decoded));
  frame.length = 1;
  frame.extended = false;
  assert(!amp::can::decode(frame, decoded));
}
