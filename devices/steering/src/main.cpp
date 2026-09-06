#include <Arduino.h>
#include <inttypes.h>

#include "amp_can/can_node.hpp"
#include "amp_can/twai_can_bus.hpp"
#include "steering_can/steering_can.hpp"

#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif
#ifndef STEERING_NODE_ID
#define STEERING_NODE_ID 10
#endif
#ifndef STEERING_PWM_PIN
#define STEERING_PWM_PIN 1
#endif
#ifndef STEERING_PWM_HZ
#define STEERING_PWM_HZ 1000
#endif
#ifndef STEERING_START_POSITION
#define STEERING_START_POSITION 128
#endif

static_assert(STEERING_NODE_ID >= 0 && STEERING_NODE_ID <= 254,
              "STEERING_NODE_ID must be in the range 0..254");
static_assert(STEERING_PWM_PIN != 15 && STEERING_PWM_PIN != 16,
              "Servo PWM must not use the CAN pins");

constexpr gpio_num_t kCanTx = GPIO_NUM_15;
constexpr gpio_num_t kCanRx = GPIO_NUM_16;
constexpr uint32_t kCanFramesPerLoop = 8;

amp::can::TwaiBus can_bus(kCanTx, kCanRx, CAN_BITRATE);

void writeSteeringPosition(uint8_t position) {
  analogWrite(STEERING_PWM_PIN, position);
}

void beginSteeringOutput(uint8_t initial_position) {
  analogWriteResolution(8);
  analogWriteFrequency(STEERING_PWM_HZ);
  writeSteeringPosition(initial_position);
}

void setSteering(const amp::steering::PositionCommand& message) {
  writeSteeringPosition(message.position);
  Serial.printf("RECEIVED steering=%" PRIu8 "\n", message.position);
}

auto can = amp::can::makeNode(
    can_bus, amp::can::on<amp::steering::PositionCommand>(STEERING_NODE_ID,
                                                          setSteering));
bool ready = false;

void setup() {
  Serial.begin(115200);
  delay(500);

  beginSteeringOutput(STEERING_START_POSITION);
  if (!can.begin()) {
    Serial.println("FATAL failed to start CAN (GPIO15 TX, GPIO16 RX)");
    return;
  }
  ready = true;
  Serial.printf("READY: steering node %" PRIu32 ", PWM GPIO%d, CAN %" PRIu32
                " bit/s\n",
                static_cast<uint32_t>(STEERING_NODE_ID), STEERING_PWM_PIN,
                static_cast<uint32_t>(CAN_BITRATE));
}

void loop() {
  if (!ready) {
    delay(1000);
    return;
  }
  can.poll(kCanFramesPerLoop);
}
