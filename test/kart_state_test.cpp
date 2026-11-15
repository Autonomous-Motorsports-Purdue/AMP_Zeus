#include <cassert>
#include <cstdint>

#include "kart_state/can_state.hpp"
#include "amp_can/can_node.hpp"

using namespace amp::kart;

namespace {

struct FakeBus {
  bool succeeds = true;
  uint32_t writes = 0;
  amp::can::Frame last{};

  bool write(const amp::can::Frame& frame, uint32_t timeout) {
    assert(timeout == 0);
    last = frame;
    ++writes;
    return succeeds;
  }
};

bool deliver(CanState& state, const amp::can::Frame& frame) {
  bool changed = false;
  auto updates = amp::can::on<StateMessage>(
      [&](const StateMessage& message) { changed = state.receive(message); });
  auto requests = amp::can::on<StateRequest>(
      [&](const StateRequest& request) { state.receive(request); });
  updates.dispatch(frame);
  requests.dispatch(frame);
  return changed;
}

void testCanState() {
  FakeBus bus;
  CanState bridge(60);
  CanState radio(61);
  assert(!bridge.is(Mode::Idle));
  assert(!bridge.is(Mode::Autonomous));
  assert(!bridge.is(Mode::Rc));
  assert(!bridge.is(Mode::Estop));
  assert(bridge.update(bus, 0));
  StateRequest request;
  assert(amp::can::decode(bus.last, request));
  assert(request.node_id == 60);
  assert(!bridge.update(bus, 99));
  // An unknown peer does not immediately answer a startup request.
  assert(!deliver(radio, bus.last));
  assert(radio.update(bus, 0));
  assert(!deliver(radio, bus.last));
  assert(!radio.update(bus, 1));
  assert(bridge.update(bus, 100));
  assert(!bridge.synchronized());
  assert(!bridge.setMode(Mode::Rc));
  assert(!bridge.update(bus, 101));

  assert(bridge.setMode(Mode::Idle));
  assert(bridge.is(Mode::Idle));
  assert(bridge.update(bus, 101));
  StateMessage message;
  assert(amp::can::decode(bus.last, message));
  assert(message.node_id == 60 && message.state.writer == 60);
  assert(deliver(radio, bus.last));
  assert(radio.synchronized());
  assert(radio.update(bus, 102));
  assert(amp::can::decode(bus.last, message));
  assert(message.node_id == 61 && message.state.writer == 60);
  assert(!deliver(bridge, bus.last));
  assert(!bridge.update(bus, 102));
  assert(!radio.update(bus, 103));

  assert(radio.setMode(Mode::Rc));
  bus.succeeds = false;
  assert(!radio.update(bus, 104));
  assert(radio.is(Mode::Rc));
  assert(!radio.update(bus, 105));
  bus.succeeds = true;
  assert(radio.update(bus, 106));
  assert(deliver(bridge, bus.last));
  assert(bridge.is(Mode::Rc));
  assert(bridge.update(bus, 107));

  request.node_id = 62;
  assert(!deliver(bridge, amp::can::encode(request)));
  assert(bridge.update(bus, 108));
  assert(amp::can::decode(bus.last, message));
  assert(message.state.writer == 61);
  assert(!bridge.update(bus, 109));

  assert(bridge.estop());
  assert(bridge.publish(bus, 110));
  assert(deliver(radio, bus.last));
  assert(!radio.is(Mode::Rc));
  assert(radio.is(Mode::Estop));
  assert(!bridge.update(bus, 110));
  assert(!bridge.setMode(Mode::Idle));
  assert(!bridge.update(bus, 111));
  assert(bridge.resetEstop());
  assert(bridge.update(bus, 112));
  assert(bridge.state().mode == Mode::Idle);

  // Invalid frames do not change state or schedule a response.
  auto invalid = bus.last;
  invalid.length = 7;
  assert(!deliver(bridge, invalid));
  assert(!bridge.update(bus, 113));
  const uint32_t revision = bridge.state().revision;
  assert(bridge.update(bus, 212));
  assert(bridge.state().revision == revision);

  // A forced failed publish is retried without waiting for the interval.
  bus.succeeds = false;
  assert(!bridge.publish(bus, 213));
  bus.succeeds = true;
  assert(bridge.update(bus, 214));

  CanState wrapping(63);
  assert(wrapping.update(bus, UINT32_MAX - 50));
  assert(!wrapping.update(bus, 48));
  assert(wrapping.update(bus, 49));
}

void sameState(const SharedState& a, const SharedState& b) {
  assert(a.state().mode == b.state().mode);
  assert(a.state().stop_generation == b.state().stop_generation);
  assert(a.state().revision == b.state().revision);
  assert(a.state().writer == b.state().writer);
}

State makeState(Mode mode, uint16_t generation, uint32_t revision,
                uint8_t writer) {
  State state;
  state.mode = mode;
  state.stop_generation = generation;
  state.revision = revision;
  state.writer = writer;
  return state;
}

}  // namespace

int main() {
  testCanState();
  for (uint8_t i = 0; i <= static_cast<uint8_t>(Mode::Estop); ++i) {
    Mode parsed = Mode::Idle;
    const Mode mode = static_cast<Mode>(i);
    assert(parseMode(modeName(mode), parsed));
    assert(parsed == mode);
  }
  Mode parsed = Mode::Rc;
  assert(!parseMode(nullptr, parsed));
  assert(!parseMode("", parsed));
  assert(!parseMode("RC", parsed));
  assert(!parseMode("reset_estop", parsed));
  assert(parsed == Mode::Rc);
  assert(strcmp(modeName(static_cast<Mode>(99)), "unknown") == 0);

  SharedState bridge(60);
  SharedState radio(61);
  assert(!bridge.synchronized());
  assert(!bridge.is(Mode::Autonomous));
  assert(!bridge.is(Mode::Idle));
  assert(!bridge.is(Mode::Rc));
  assert(!bridge.is(Mode::Estop));
  assert(!bridge.setMode(Mode::Autonomous));
  assert(!bridge.resetEstop());
  assert(bridge.setMode(Mode::Idle));
  assert(bridge.is(Mode::Idle));
  assert(radio.receive(bridge.state()));
  assert(bridge.setMode(Mode::Autonomous));
  assert(radio.receive(bridge.state()));
  assert(radio.is(Mode::Autonomous));
  assert(!radio.is(Mode::Rc));
  assert(radio.setMode(Mode::Rc));
  assert(bridge.receive(radio.state()));
  assert(bridge.is(Mode::Rc));
  assert(!bridge.is(Mode::Idle));
  assert(!bridge.is(Mode::Estop));

  // Simultaneous writes converge regardless of delivery order.
  assert(bridge.setMode(Mode::Idle));
  assert(radio.setMode(Mode::Autonomous));
  const State bridge_write = bridge.state();
  const State radio_write = radio.state();
  assert(bridge_write.revision == radio_write.revision);
  assert(bridge.receive(radio_write));
  assert(!radio.receive(bridge_write));
  sameState(bridge, radio);
  SharedState late(62);
  assert(late.receive(radio_write));
  assert(!late.receive(bridge_write));
  assert(!late.receive(radio_write));
  sameState(late, bridge);

  // A stop dominates ordinary writes, even ones with a higher counter.
  const State old_normal = makeState(Mode::Autonomous, 0, 1000, 62);
  assert(bridge.estop());
  assert(radio.receive(old_normal));
  assert(radio.receive(bridge.state()));
  assert(!bridge.receive(old_normal));
  assert(!bridge.setMode(Mode::Idle));
  assert(!bridge.setMode(Mode::Rc));
  assert(!bridge.is(Mode::Autonomous));
  assert(bridge.is(Mode::Estop));
  sameState(bridge, radio);

  // Only an explicit reset advances to an unlatched generation.
  const State old_stop = bridge.state();
  assert(radio.resetEstop());
  assert(radio.state().mode == Mode::Idle);
  assert(bridge.receive(radio.state()));
  assert(!bridge.receive(old_normal));
  assert(!bridge.receive(old_stop));
  assert(bridge.setMode(Mode::Autonomous));
  assert(radio.receive(bridge.state()));
  assert(radio.estop());
  assert(bridge.receive(radio.state()));
  assert(!bridge.is(Mode::Autonomous));

  // Startup never creates a broadcastable default or overwrites live peers.
  SharedState restarted(60);
  assert(!validState(restarted.state()));
  assert(restarted.receive(radio.state()));
  assert(!restarted.setMode(Mode::Idle));
  assert(restarted.resetEstop());
  assert(radio.receive(restarted.state()));
  assert(restarted.state().revision == 1);
  assert(restarted.setMode(Mode::Rc));
  assert(radio.receive(restarted.state()));
  SharedState joining(63);
  assert(joining.receive(radio.state()));
  assert(joining.is(Mode::Rc));
  assert(joining.setMode(Mode::Idle));
  assert(radio.receive(joining.state()));

  // A module can request Estop even before synchronization.
  SharedState emergency(64);
  assert(emergency.estop());
  assert(validState(emergency.state()));
  assert(!emergency.setMode(Mode::Idle));
  assert(emergency.resetEstop());
  assert(!emergency.setMode(static_cast<Mode>(99)));
  assert(!emergency.receive(makeState(Mode::Estop, 2, 1, 1)));
  assert(!emergency.receive(makeState(Mode::Idle, 3, 1, 1)));
  assert(!emergency.receive(makeState(Mode::Idle, 2, 0, 1)));
  assert(!emergency.receive(makeState(Mode::Idle, 2, 1, 255)));

  // Counters never wrap into an older state.
  SharedState full(65);
  assert(full.receive(makeState(Mode::Idle, 0, UINT32_MAX, 1)));
  assert(!full.setMode(Mode::Rc));
  assert(full.estop());
  assert(full.receive(makeState(Mode::Estop, UINT16_MAX, 1, 1)));
  assert(!full.resetEstop());

  StateMessage message;
  message.node_id = 60;
  message.state = makeState(Mode::Rc, 0x1234, 0x12345678, 61);
  auto frame = amp::can::encode(message);
  assert(frame.id == 0x703C && frame.length == 8);
  assert(frame.data[1] == 0x34 && frame.data[2] == 0x12);
  assert(frame.data[3] == 0x78 && frame.data[6] == 0x12);
  StateMessage decoded;
  assert(amp::can::decode(frame, decoded));
  assert(decoded.node_id == 60 && decoded.state.writer == 61);
  assert(decoded.state.stop_generation == 0x1234);
  assert(decoded.state.revision == 0x12345678);
  frame.data[0] = 99;
  assert(!amp::can::decode(frame, decoded));
  frame = amp::can::encode(message);
  frame.length = 7;
  assert(!amp::can::decode(frame, decoded));
  frame = amp::can::encode(message);
  frame.remote = true;
  assert(!amp::can::decode(frame, decoded));

  StateRequest request;
  request.node_id = 61;
  frame = amp::can::encode(request);
  assert(frame.id == 0x713D && frame.length == 0);
  StateRequest received;
  assert(amp::can::decode(frame, received));
  assert(received.node_id == 61);
  frame.length = 1;
  assert(!amp::can::decode(frame, received));
}
