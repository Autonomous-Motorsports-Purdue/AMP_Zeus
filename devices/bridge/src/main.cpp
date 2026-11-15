#include <Arduino.h>
#include <ArduinoJson.h>
#include <cstdio>
#include <cstring>

#include "amp_can/twai_can_bus.hpp"
#include "amp_can/can_node.hpp"
#include "kart_state/can_state.hpp"
#include "steering_can/steering_can.hpp"
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
constexpr size_t kMaxLineLength = 512;
constexpr size_t kSerialBytesPerLoop = 128;
constexpr uint32_t kVescTelemetryIntervalMs = 100;
constexpr uint32_t kDebugIntervalMs = 1000;

amp::can::TwaiBus can_bus(kCanTx, kCanRx, CAN_BITRATE);
bool can_ready = false;
char serial_line[kMaxLineLength + 1];
size_t serial_length = 0;
bool line_overflow = false;
char output[768];
size_t output_length = 0;
size_t output_offset = 0;
amp::vesc::Status latest_vesc_status{};
bool vesc_status_pending = false;
uint32_t vesc_status_time_ms = 0;
uint32_t last_vesc_telemetry_ms = 0;
uint32_t last_debug_ms = 0;
uint32_t can_rx_count = 0;
amp::can::Frame latest_can_rx{};
amp::kart::CanState kart_state(amp::kart::kBridgeNode);

void cacheVescStatus(const amp::vesc::Status& status) {
  latest_vesc_status = status;
  vesc_status_time_ms = millis();
  vesc_status_pending = true;
}

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
    amp::can::on<amp::vesc::Status>(VESC_CAN_ID, cacheVescStatus),
    amp::can::onFrame([](const amp::can::Frame &frame) {
      latest_can_rx = frame;
      ++can_rx_count;
    }));

void queueJson(const JsonDocument &document) {
  output_length = serializeJson(document, output, sizeof(output) - 1);
  output[output_length++] = '\n';
  output_offset = 0;
}

void writeSerial() {
  if (output_offset == output_length) return;
  const int available = Serial.availableForWrite();
  if (available <= 0) return;
  const size_t remaining = output_length - output_offset;
  const size_t count = remaining < static_cast<size_t>(available)
                           ? remaining : static_cast<size_t>(available);
  output_offset += Serial.write(
      reinterpret_cast<const uint8_t*>(output + output_offset), count);
}

void describeFrame(JsonObject object, const amp::can::Frame& frame) {
  char id[9];
  char data[25]{};
  snprintf(id, sizeof(id), "%08lX", static_cast<unsigned long>(frame.id));
  if (!frame.remote) {
    for (uint8_t i = 0; i < frame.length; ++i) {
      snprintf(data + i * 3, sizeof(data) - i * 3, "%02X ", frame.data[i]);
    }
    if (frame.length) data[frame.length * 3 - 1] = '\0';
  }
  object["can_id_hex"] = id;
  object["extended"] = frame.extended;
  object["remote"] = frame.remote;
  object["dlc"] = frame.length;
  object["data_hex"] = data;
}

void sendError(const char* error) {
  StaticJsonDocument<256> response;
  response["ok"] = false;
  response["error"] = error;
  JsonObject debug = response.createNestedObject("debug");
  debug["event"] = "command_rejected";
  debug["ts_ms"] = millis();
  queueJson(response);
}

void describeKartState(JsonObject object) {
  const auto& state = kart_state.state();
  object["mode"] = static_cast<uint8_t>(state.mode);
  object["stop_generation"] = state.stop_generation;
  object["revision"] = state.revision;
  object["writer"] = state.writer;
  object["synchronized"] = kart_state.synchronized();
  object["autonomous_authorized"] =
      kart_state.is(amp::kart::Mode::Autonomous);
}

void handleLine() {
  StaticJsonDocument<768> request;
  const DeserializationError error = deserializeJson(
      request, serial_line, serial_length, DeserializationOption::NestingLimit(4));
  if (error || !request.is<JsonObject>()) {
    sendError("invalid_json");
    return;
  }

  StaticJsonDocument<768> response;
  if (request.containsKey("id")) {
    if (!request["id"].is<uint32_t>()) {
      sendError("id_must_be_uint32");
      return;
    }
    response["id"] = request["id"].as<uint32_t>();
  }

  const char* target = request["target"].is<const char*>()
                           ? request["target"].as<const char*>() : "";
  const char* failure = nullptr;
  bool sent = false;
  bool transmit_attempted = false;
  amp::can::Frame frame{};
  const bool motion_target =
      strcmp(target, "steering") == 0 || strcmp(target, "vesc") == 0;
  if (strcmp(target, "kart") == 0) {
    if (!request.containsKey("mode") && !request.containsKey("reset_estop")) {
      response["ok"] = true;
      describeKartState(response.createNestedObject("kart"));
      queueJson(response);
      return;
    }
    bool applied = false;
    if (!can_ready) {
      failure = "can_not_ready";
    } else if (request.containsKey("reset_estop")) {
      if (request.containsKey("mode") || !request["reset_estop"].is<bool>() ||
          !request["reset_estop"].as<bool>()) {
        failure = "reset_estop_requires_true_and_no_mode";
      } else {
        applied = kart_state.resetEstop();
      }
    } else {
      const char* mode = request["mode"].is<const char*>()
                             ? request["mode"].as<const char*>() : "";
      amp::kart::Mode parsed;
      if (amp::kart::parseMode(mode, parsed)) {
        applied = kart_state.setMode(parsed);
      } else {
        failure = "mode_must_be_idle_autonomous_rc_or_estop";
      }
    }
    response["applied"] = applied;
    describeKartState(response.createNestedObject("kart"));
    if (applied) {
      frame = kart_state.frame();
      transmit_attempted = true;
      sent = kart_state.publish(can_bus, millis());
    } else if (!failure) {
      failure = "kart_transition_rejected";
    }
  } else if (motion_target &&
             !kart_state.is(amp::kart::Mode::Autonomous)) {
    failure = "autonomous_not_authorized";
  } else if (strcmp(target, "steering") == 0) {
    if (!request["position_deg"].is<double>() ||
        !amp::steering::setPosition(
            STEERING_CAN_ID, request["position_deg"].as<double>(), frame)) {
      failure = "position_deg_must_be_number_minus36000_to_36000";
    } else if (!can_ready) {
      failure = "can_not_ready";
    } else {
      transmit_attempted = true;
      sent = can.send(frame, 0);
    }
  } else if (strcmp(target, "vesc") == 0) {
    if (!request["erpm"].is<int32_t>()) {
      failure = "erpm_must_be_int32";
    } else if (!can_ready) {
      failure = "can_not_ready";
    } else {
      frame = amp::vesc::setRpm(VESC_CAN_ID, request["erpm"].as<int32_t>());
      transmit_attempted = true;
      sent = can.send(frame, 0);
    }
  } else {
    failure = "unknown_target";
  }

  response["ok"] = sent;
  if (sent) response["status"] = "queued";
  else response["error"] = failure ? failure : "can_transmit_failed";
  JsonObject debug = response.createNestedObject("debug");
  debug["event"] = sent ? "can_tx_queued" :
      transmit_attempted ? "can_tx_failed" : "command_rejected";
  debug["ts_ms"] = millis();
  if (transmit_attempted) describeFrame(debug, frame);
  queueJson(response);
}

void readSerial() {
  if (output_offset != output_length) return;
  for (size_t i = 0; i < kSerialBytesPerLoop && Serial.available(); ++i) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n') {
      if (line_overflow) sendError("line_too_long");
      else if (serial_length != 0) handleLine();
      serial_length = 0;
      line_overflow = false;
      return;
    }
    if (character == '\r') continue;
    if (serial_length < kMaxLineLength && !line_overflow) {
      serial_line[serial_length++] = character;
    } else {
      line_overflow = true;
    }
  }
}

void publishVescTelemetry() {
  const uint32_t now = millis();
  if (!vesc_status_pending || output_offset != output_length || Serial.available() ||
      now - last_vesc_telemetry_ms < kVescTelemetryIntervalMs) return;

  StaticJsonDocument<256> telemetry;
  telemetry["type"] = "vesc";
  telemetry["node"] = latest_vesc_status.controller_id;
  telemetry["ts_ms"] = vesc_status_time_ms;
  telemetry["erpm"] = latest_vesc_status.electrical_rpm;
  telemetry["current_a"] = latest_vesc_status.motor_current_amps;
  telemetry["duty"] = latest_vesc_status.duty_cycle;
  queueJson(telemetry);
  vesc_status_pending = false;
  last_vesc_telemetry_ms = now;
}

void publishCanDebug() {
  const uint32_t now = millis();
  if (output_offset != output_length || Serial.available() ||
      now - last_debug_ms < kDebugIntervalMs) return;

  StaticJsonDocument<1024> event;
  event["type"] = "debug";
  event["event"] = "can_status";
  event["ts_ms"] = now;
  event["can_ready"] = can_ready;
  event["rx_total"] = can_rx_count;
  describeKartState(event.createNestedObject("kart"));
  if (can_rx_count) describeFrame(event.createNestedObject("last_rx"), latest_can_rx);
  twai_status_info_t status{};
  if (can_ready && twai_get_status_info(&status) == ESP_OK) {
    event["state"] = status.state == TWAI_STATE_RUNNING ? "running" :
        status.state == TWAI_STATE_BUS_OFF ? "bus_off" :
        status.state == TWAI_STATE_RECOVERING ? "recovering" : "stopped";
    event["tx_queued"] = status.msgs_to_tx;
    event["rx_queued"] = status.msgs_to_rx;
    event["tx_error_counter"] = status.tx_error_counter;
    event["rx_error_counter"] = status.rx_error_counter;
    event["tx_failed"] = status.tx_failed_count;
    event["bus_errors"] = status.bus_error_count;
    event["rx_missed"] = status.rx_missed_count;
  }
  queueJson(event);
  last_debug_ms = now;
}

void setup() {
  Serial.begin(115200);
  can_ready = can.begin();
  StaticJsonDocument<256> event;
  event["type"] = "ready";
  event["can_ready"] = can_ready;
  event["can_bitrate"] = CAN_BITRATE;
  event["can_tx_gpio"] = static_cast<int>(kCanTx);
  event["can_rx_gpio"] = static_cast<int>(kCanRx);
  event["steering_node"] = STEERING_CAN_ID;
  event["vesc_node"] = VESC_CAN_ID;
  queueJson(event);
}

void loop() {
  if (can_ready) can.poll(kMaxCanFramesPerLoop);
  writeSerial();
  readSerial();
  if (can_ready) kart_state.update(can_bus, millis());
  publishVescTelemetry();
  publishCanDebug();
  writeSerial();
}
