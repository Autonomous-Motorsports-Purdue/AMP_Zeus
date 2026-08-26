#include <Arduino.h>
#include <inttypes.h>

#include "amp_can/twai_can_bus.hpp"
#include "amp_can/can_node.hpp"
#include "steering_can/steering_can.hpp"
#include "vesc_can/vesc_can.hpp"

#ifndef VESC_CAN_ID
#define VESC_CAN_ID 0
#endif
#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif

static_assert(VESC_CAN_ID >= 0 && VESC_CAN_ID <= 254,
              "VESC_CAN_ID must be in the range 0..254; 255 broadcasts to every VESC");

constexpr gpio_num_t kCanTx = GPIO_NUM_15;
constexpr gpio_num_t kCanRx = GPIO_NUM_16;
constexpr uint8_t kMaxCanFramesPerLoop = 8;
amp::can::TwaiBus can_bus(kCanTx, kCanRx, CAN_BITRATE);

struct Command {
  const char* name;
  int32_t minimum;
  int32_t maximum;
  void (*send)(int32_t value);
};


// TEMP DATA RETURN
void printStatus(const amp::can::Frame& frame) {
  amp::vesc::Status status{};
  if (amp::vesc::decodeStatus(frame, status)) {
    Serial.printf("STATUS id=%" PRIu32 " erpm=%" PRId32
                  " current=%.1f duty=%.3f\n",
                  static_cast<uint32_t>(status.controller_id), status.electrical_rpm,
                  status.motor_current_amps, status.duty_cycle);
  }
}

auto can = amp::can::makeNode(can_bus, amp::can::onFrame(printStatus));

void sendThrottle(int32_t value) {
  if (can.send(amp::vesc::setRpm(VESC_CAN_ID, value))) {
    // twai_transmit confirms that the driver accepted/queued the frame. It does
    // not guarantee that another CAN node acknowledged or processed it.
    Serial.printf("QUEUED erpm=%" PRId32 " vesc=%" PRIu32 "\n", value,
                  static_cast<uint32_t>(VESC_CAN_ID));
  } else {
    Serial.println("ERR CAN transmit failed");
  }
}

void sendSteering(int32_t value) {
  amp::steering::PositionCommand message{10, static_cast<uint8_t>(value)};
  if (can.send(message)) {
    Serial.printf("QUEUED steering=%" PRId32 " node=10\n", value);
  } else {
    Serial.println("ERR CAN transmit failed");
  }
}

const Command commands[] = {
    {"throttle", INT32_MIN, INT32_MAX, sendThrottle},
    {"steering", 0, 255, sendSteering},
};

const Command* findCommand(const String& name) {
  for (const Command& command : commands) {
    if (name == command.name) return &command;
  }
  return nullptr;
}

bool can_ready = false;

void handleLine(String line) {
  line.trim();
  String value_text = line;
  const Command* command = &commands[0];  

  const int32_t separator = static_cast<int32_t>(line.indexOf(':'));
  if (separator >= 0) {
    command = findCommand(line.substring(0, static_cast<uint32_t>(separator)));
    value_text = line.substring(static_cast<uint32_t>(separator + 1));
  }

  // Will return 0 on invalid data. Issue for steering
  const int32_t value = static_cast<int32_t>(value_text.toInt());
  if (command == nullptr ||
      value < command->minimum || value > command->maximum) {
    Serial.println("ERR invalid command or value");
  } else {
    command->send(value);
  }
}

void readSerial() {
  while (Serial.available()) {
    handleLine(Serial.readStringUntil('\n'));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(25);
  delay(500);
  can_ready = can.begin();
  if (!can_ready) {
    Serial.println("FATAL failed to start CAN (GPIO15 TX, GPIO16 RX)");
    return;
  }
  Serial.printf("READY: use throttle:<ERPM> or steering:<0..255> (VESC ID %"
                PRIu32 ", CAN %" PRIu32 " bit/s)\n",
                static_cast<uint32_t>(VESC_CAN_ID),
                static_cast<uint32_t>(CAN_BITRATE));
}

void loop() {
  if (!can_ready) {
    delay(1000); // TEMP DELAY
    return;
  }
  readSerial();
  can.poll(kMaxCanFramesPerLoop);
}
