#include <cassert>
#include <cstdint>
#include <memory>
#include <utility>

#include "amp_can/can_node.hpp"

namespace {

struct TestMessage {
  static constexpr uint8_t packet = 0x42;
  static constexpr uint8_t payload_size = 1;

  uint8_t node_id = 0;
  uint8_t value = 0;

  void serialize(uint8_t* data) const { data[0] = value; }
  bool deserialize(const uint8_t* data) {
    value = data[0];
    return true;
  }
};

struct OtherMessage : TestMessage {
  static constexpr uint8_t packet = 0x43;
};

class FakeBus {
 public:
  bool begin() {
    began = true;
    return true;
  }

  bool write(const amp::can::Frame& frame, uint32_t) {
    last_written = frame;
    ++writes;
    return true;
  }

  bool read(amp::can::Frame& frame, uint32_t) {
    if (!has_pending) return false;
    frame = pending;
    has_pending = false;
    return true;
  }

  bool began = false;
  bool has_pending = false;
  uint32_t writes = 0;
  amp::can::Frame pending{};
  amp::can::Frame last_written{};
};

struct MoveOnlyReceiver {
  explicit MoveOnlyReceiver(uint32_t& calls)
      : calls_(new uint32_t(0)), total_(calls) {}
  MoveOnlyReceiver(MoveOnlyReceiver&&) = default;
  MoveOnlyReceiver(const MoveOnlyReceiver&) = delete;

  void operator()(const TestMessage& message) {
    *calls_ += message.value;
    total_ += *calls_;
  }

  std::unique_ptr<uint32_t> calls_;
  uint32_t& total_;
};

}  // namespace

int main() {
  FakeBus bus;
  uint32_t total = 0;
  auto node = amp::can::makeNode(
      bus, amp::can::on<TestMessage>(7, MoveOnlyReceiver(total)));

  assert(node.begin());
  assert(bus.began);

  TestMessage sent;
  sent.node_id = 7;
  sent.value = 3;
  assert(node.send(sent));
  assert(bus.writes == 1);
  assert(bus.last_written.id == 0x4207);

  TestMessage received;
  received.node_id = 7;
  received.value = 5;
  bus.pending = amp::can::encode(received);
  bus.has_pending = true;
  assert(node.poll(4) == 1);
  assert(total == 5);

  received.node_id = 8;
  received.value = 9;
  bus.pending = amp::can::encode(received);
  bus.has_pending = true;
  assert(node.poll(1) == 1);
  assert(total == 5);

  uint32_t all_nodes = 0;
  uint32_t filtered = 0;
  uint32_t other = 0;
  uint32_t raw = 0;
  auto listeners = amp::can::makeNode(
      bus,
      amp::can::on<TestMessage>(
          [&](const TestMessage& message) { all_nodes += message.value; }),
      amp::can::on<TestMessage>(
          7, [&](const TestMessage& message) { filtered += message.value; }),
      amp::can::on<OtherMessage>(
          [&](const OtherMessage& message) { other += message.value; }),
      amp::can::onFrame([&](const amp::can::Frame&) { ++raw; }));
  auto deliver = [&](const amp::can::Frame& frame) {
    bus.pending = frame;
    bus.has_pending = true;
    assert(listeners.poll(1) == 1);
  };

  received.node_id = 7;
  received.value = 2;
  deliver(amp::can::encode(received));
  received.node_id = 8;
  deliver(amp::can::encode(received));
  assert(all_nodes == 4 && filtered == 2 && other == 0 && raw == 2);

  OtherMessage other_message;
  other_message.node_id = 8;
  other_message.value = 3;
  deliver(amp::can::encode(other_message));
  assert(all_nodes == 4 && filtered == 2 && other == 3 && raw == 3);

  auto invalid = amp::can::encode(received);
  invalid.length = 0;
  deliver(invalid);
  invalid = amp::can::encode(received);
  invalid.remote = true;
  deliver(invalid);
  invalid = amp::can::encode(received);
  invalid.extended = false;
  deliver(invalid);
  invalid = amp::can::encode(received);
  invalid.id |= 0x10000;
  deliver(invalid);
  invalid = amp::can::encode(received);
  invalid.id = 0x4407;
  deliver(invalid);
  assert(all_nodes == 4 && filtered == 2 && other == 3 && raw == 8);
}
