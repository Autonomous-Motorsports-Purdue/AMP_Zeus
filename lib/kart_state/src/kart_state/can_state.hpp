#pragma once

#include "kart_state/messages.hpp"

namespace amp::kart {

class CanState {
public:
  explicit CanState(uint8_t node_id) : node_id_(node_id), shared_(node_id) {}

  const State &state() const { return shared_.state(); }
  bool synchronized() const { return shared_.synchronized(); }
  bool is(Mode mode) const { return shared_.is(mode); }

  bool setMode(Mode mode) {
    const bool applied = shared_.setMode(mode);
    if (applied)
      publish_pending_ = true;
    return applied;
  }

  bool estop() { return setMode(Mode::Estop); }

  bool resetEstop() {
    const bool applied = shared_.resetEstop();
    if (applied)
      publish_pending_ = true;
    return applied;
  }

  bool receive(const StateMessage &message) {
    const bool changed = shared_.receive(message.state);
    if (changed)
      publish_pending_ = true;
    return changed;
  }

  void receive(const StateRequest &) {
    if (synchronized())
      publish_pending_ = true;
  }

  can::Frame frame() const {
    if (synchronized()) {
      StateMessage message;
      message.node_id = node_id_;
      message.state = state();
      return can::encode(message);
    }
    StateRequest request;
    request.node_id = node_id_;
    return can::encode(request);
  }

  template <typename Bus> bool publish(Bus &bus, uint32_t now) {
    if (!bus.write(frame(), 0)) {
      publish_pending_ = true;
      return false;
    }
    last_sync_ms_ = now;
    publish_pending_ = false;
    return true;
  }

  template <typename Bus> bool update(Bus &bus, uint32_t now) {
    if (!publish_pending_ && now - last_sync_ms_ < kSyncIntervalMs)
      return false;
    return publish(bus, now);
  }

private:
  uint8_t node_id_;
  SharedState shared_;
  uint32_t last_sync_ms_ = 0;
  bool publish_pending_ = true;
};

} // namespace amp::kart
