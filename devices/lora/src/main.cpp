#include <Arduino.h>

#include "amp_can/can_node.hpp"
#include "amp_can/twai_can_bus.hpp"
#include "kart_state/can_state.hpp"

#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif

constexpr uint8_t kNodeId = 61;
constexpr gpio_num_t kCanTx = GPIO_NUM_15;
constexpr gpio_num_t kCanRx = GPIO_NUM_16;

amp::can::TwaiBus can_bus(kCanTx, kCanRx, CAN_BITRATE);
amp::kart::CanState kart_state(kNodeId);
bool can_ready = false;

auto can = amp::can::makeNode(
    can_bus,
    amp::can::on<amp::kart::StateMessage>(
        [](const amp::kart::StateMessage &message) {
          kart_state.receive(message);
        }),
    amp::can::on<amp::kart::StateRequest>(
        [](const amp::kart::StateRequest &request) {
          kart_state.receive(request);
        }));

void setup() { can_ready = can.begin(); }

void loop() {
  if (!can_ready)
    return;
  can.poll(8);
  kart_state.update(can_bus, millis());
}
