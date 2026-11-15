#include <Arduino.h>

#include "amp_can/twai_can_bus.hpp"
#include "amp_can/can_node.hpp"
#include "bridge_json.hpp"
#include "kart_state/can_state.hpp"
#include "vesc_can/vesc_can.hpp"

#ifndef VESC_CAN_ID
#define VESC_CAN_ID 0
#endif
#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif
#ifndef STEERING_CAN_ID
#define STEERING_CAN_ID 104
#endif

static_assert(VESC_CAN_ID >= 0 && VESC_CAN_ID <= 254,
              "VESC_CAN_ID must be in the range 0..254; 255 broadcasts to every VESC");
static_assert(STEERING_CAN_ID >= 0 && STEERING_CAN_ID <= 254,
              "STEERING_CAN_ID must be in the range 0..254");
static_assert(STEERING_CAN_ID != VESC_CAN_ID,
              "Steering and VESC must use different CAN IDs");
static_assert(STEERING_CAN_ID != amp::kart::kBridgeNode &&
                  VESC_CAN_ID != amp::kart::kBridgeNode,
              "Bridge and motor controllers must use different CAN IDs");

constexpr gpio_num_t kCanTx = GPIO_NUM_15;
constexpr gpio_num_t kCanRx = GPIO_NUM_16;
constexpr uint8_t kMaxCanFramesPerLoop = 8;

amp::can::TwaiBus can_bus(kCanTx, kCanRx, CAN_BITRATE);
bool can_ready = false;
amp::kart::CanState kart_state(amp::kart::kBridgeNode);
bridge::JsonInterface json(can_bus, kart_state, STEERING_CAN_ID, VESC_CAN_ID);

auto can = amp::can::makeNode(
    can_bus,
    amp::can::on<amp::kart::StateMessage>(
        [](const amp::kart::StateMessage &message) {
          kart_state.receive(message);
        }),
    amp::can::on<amp::kart::StateRequest>(
        [](const amp::kart::StateRequest &request) {
          kart_state.receive(request);
        }),
    amp::can::on<amp::vesc::Status>(
        VESC_CAN_ID, [](const amp::vesc::Status& status) {
          json.cacheVescStatus(status);
        }),
    amp::can::onFrame([](const amp::can::Frame &frame) {
      json.recordCanFrame(frame);
    }));

void setup() {
  Serial.begin(115200);
  can_ready = can.begin();
  json.begin(can_ready, CAN_BITRATE, kCanTx, kCanRx);
}

void loop() {
  if (can_ready) can.poll(kMaxCanFramesPerLoop);
  json.update();
  if (can_ready) kart_state.update(can_bus, millis());
}
