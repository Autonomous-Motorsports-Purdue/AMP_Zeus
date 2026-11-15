#pragma once

#include <ArduinoJson.h>
#include <cstddef>
#include <cstdint>

#include "amp_can/twai_can_bus.hpp"
#include "kart_state/can_state.hpp"
#include "vesc_can/vesc_can.hpp"

namespace bridge {

class JsonInterface {
 public:
  JsonInterface(amp::can::TwaiBus& bus, amp::kart::CanState& state,
                uint8_t steering_node, uint8_t vesc_node);

  void begin(bool ready, uint32_t bitrate, int tx_gpio, int rx_gpio);
  void cacheVescStatus(const amp::vesc::Status& status);
  void recordCanFrame(const amp::can::Frame& frame);
  void update();

 private:
  void queueJson(const JsonDocument& document);
  void writeSerial();
  void describeFrame(JsonObject object, const amp::can::Frame& frame);
  void sendError(const char* error);
  void describeKartState(JsonObject object);
  void handleLine();
  void readSerial();
  void publishVescTelemetry();
  void publishCanDebug();

  amp::can::TwaiBus& can_bus;
  amp::kart::CanState& kart_state;
  uint8_t steering_node;
  uint8_t vesc_node;
  bool can_ready = false;
  char serial_line[513]{};
  size_t serial_length = 0;
  bool line_overflow = false;
  char output[768]{};
  size_t output_length = 0;
  size_t output_offset = 0;
  amp::vesc::Status latest_vesc_status{};
  bool vesc_status_pending = false;
  uint32_t vesc_status_time_ms = 0;
  uint32_t last_vesc_telemetry_ms = 0;
  uint32_t last_debug_ms = 0;
  uint32_t can_rx_count = 0;
  amp::can::Frame latest_can_rx{};
};

}  // namespace bridge
