#pragma once

#include <cstddef>
#include <cstdint>
#include <tuple>
#include <type_traits>
#include <utility>

#include "amp_can/can_bus.hpp"
#include "amp_can/message_codec.hpp"

namespace amp::can {

template <typename Message, typename Handler>
class Route {
 public:
  Route(uint8_t node_id, Handler&& handler)
      : node_id_(node_id), handler_(std::move(handler)) {}

  void dispatch(const Frame& frame) {
    Message message{};
    if (decode(frame, node_id_, message)) handler_(message);
  }

 private:
  uint8_t node_id_;
  Handler handler_;
};

template <typename Handler>
class FrameRoute {
 public:
  explicit FrameRoute(Handler&& handler) : handler_(std::move(handler)) {}

  void dispatch(const Frame& frame) { handler_(frame); }

 private:
  Handler handler_;
};

template <typename Message, typename Handler>
Route<Message, typename std::decay<Handler>::type> on(uint8_t node_id,
                                                       Handler&& handler) {
  typedef typename std::decay<Handler>::type StoredHandler;
  return Route<Message, StoredHandler>(node_id,
                                       std::forward<Handler>(handler));
}

template <typename Handler>
FrameRoute<typename std::decay<Handler>::type> onFrame(Handler&& handler) {
  typedef typename std::decay<Handler>::type StoredHandler;
  return FrameRoute<StoredHandler>(std::forward<Handler>(handler));
}

template <typename Bus, typename... Routes>
class Node {
 public:
  // Construct nodes with makeNode().  Routes and their handlers are moved into
  // the dispatcher, so handlers may safely own state such as a driver object.
  Node(Bus& bus, Routes&&... routes)
      : bus_(bus), routes_(std::forward<Routes>(routes)...) {}

  bool begin() { return bus_.begin(); }

  bool send(const Frame& frame, uint32_t timeout_ms = 10) {
    return bus_.write(frame, timeout_ms);
  }

  template <typename Message>
  bool send(const Message& message, uint32_t timeout_ms = 10) {
    return bus_.write(encode(message), timeout_ms);
  }

  uint32_t poll(uint32_t frame_budget) {
    uint32_t received = 0;
    Frame frame{};
    while (received < frame_budget && bus_.read(frame, 0)) {
      dispatch<0>(frame);
      ++received;
    }
    return received;
  }

 private:
  template <std::size_t Index>
  typename std::enable_if<Index == sizeof...(Routes)>::type dispatch(
      const Frame&) {}

  template <std::size_t Index>
  typename std::enable_if<Index < sizeof...(Routes)>::type dispatch(
      const Frame& frame) {
    std::get<Index>(routes_).dispatch(frame);
    dispatch<Index + 1>(frame);
  }

  Bus& bus_;
  std::tuple<Routes...> routes_;
};

template <typename Bus, typename... Routes>
Node<Bus, typename std::decay<Routes>::type...> makeNode(Bus& bus,
                                                          Routes&&... routes) {
  typedef Node<Bus, typename std::decay<Routes>::type...> Result;
  return Result(bus, std::forward<Routes>(routes)...);
}

}  // namespace amp::can
