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
}
