#pragma once

#include <cstdint>
#include <cstring>

namespace amp::kart {

enum class Mode : uint8_t { Idle, Autonomous, Rc, Estop };

inline const char *modeName(Mode mode) {
  switch (mode) {
  case Mode::Idle:
    return "idle";
  case Mode::Autonomous:
    return "autonomous";
  case Mode::Rc:
    return "rc";
  case Mode::Estop:
    return "estop";
  }
  return "unknown";
}

inline bool parseMode(const char *name, Mode &mode) {
  if (!name)
    return false;
  for (uint8_t i = 0; i <= static_cast<uint8_t>(Mode::Estop); ++i) {
    const Mode candidate = static_cast<Mode>(i);
    if (strcmp(name, modeName(candidate)) == 0) {
      mode = candidate;
      return true;
    }
  }
  return false;
}

constexpr uint8_t kBridgeNode = 60;
constexpr uint32_t kSyncIntervalMs = 100;

inline bool validMode(uint8_t mode) {
  return mode <= static_cast<uint8_t>(Mode::Estop);
}

struct State {
  Mode mode = Mode::Estop;
  uint16_t stop_generation = 0;
  uint32_t revision = 0;
  uint8_t writer = 0;
};

inline bool validState(const State &state) {
  return validMode(static_cast<uint8_t>(state.mode)) && state.revision != 0 &&
         state.writer != 255 &&
         ((state.stop_generation & 1) != 0) == (state.mode == Mode::Estop);
}

inline bool newer(const State &incoming, const State &current) {
  if (incoming.stop_generation != current.stop_generation) {
    return incoming.stop_generation > current.stop_generation;
  }
  if (incoming.revision != current.revision) {
    return incoming.revision > current.revision;
  }
  return incoming.writer > current.writer;
}

class SharedState {
public:
  explicit SharedState(uint8_t node_id) : node_id_(node_id) {}

  const State &state() const { return state_; }
  bool synchronized() const { return synchronized_; }

  bool is(Mode mode) const { return synchronized_ && state_.mode == mode; }

  bool receive(const State &incoming) {
    if (!validState(incoming) || (synchronized_ && !newer(incoming, state_)))
      return false;
    state_ = incoming;
    synchronized_ = true;
    return true;
  }

  bool setMode(Mode mode) {
    if (!validMode(static_cast<uint8_t>(mode)) || node_id_ == 255)
      return false;
    if (mode == Mode::Estop)
      return estop();
    // An explicit Idle initializes an empty network; driving cannot.
    if (!synchronized_) {
      if (mode != Mode::Idle)
        return false;
    } else {
      if (state_.mode == Mode::Estop)
        return false;
      if (state_.mode == mode)
        return true;
    }
    if (state_.revision == UINT32_MAX)
      return false;
    state_.mode = mode;
    ++state_.revision;
    state_.writer = node_id_;
    synchronized_ = true;
    return true;
  }

  bool estop() {
    if (node_id_ == 255)
      return false;
    if (synchronized_ && state_.mode == Mode::Estop)
      return true;
    ++state_.stop_generation;
    state_.mode = Mode::Estop;
    state_.revision = 1;
    state_.writer = node_id_;
    synchronized_ = true;
    return true;
  }

  bool resetEstop() {
    if (!synchronized_ || state_.mode != Mode::Estop ||
        state_.stop_generation == UINT16_MAX || node_id_ == 255)
      return false;
    ++state_.stop_generation;
    state_.mode = Mode::Idle;
    state_.revision = 1;
    state_.writer = node_id_;
    return true;
  }

private:
  uint8_t node_id_;
  State state_{};
  bool synchronized_ = false;
};

} // namespace amp::kart
