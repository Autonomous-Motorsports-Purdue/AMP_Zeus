#include "amp_can/twai_can_bus.hpp"

#include <cstring>

namespace amp::can {
namespace {
bool timingFor(uint32_t bitrate, twai_timing_config_t& timing) {
  switch (bitrate) {
    case 125000:
      timing = TWAI_TIMING_CONFIG_125KBITS();
      return true;
    case 250000:
      timing = TWAI_TIMING_CONFIG_250KBITS();
      return true;
    case 500000:
      timing = TWAI_TIMING_CONFIG_500KBITS();
      return true;
    case 1000000:
      timing = TWAI_TIMING_CONFIG_1MBITS();
      return true;
    default:
      return false;
  }
}
}  // namespace

bool TwaiBus::begin() {
  twai_timing_config_t timing{};
  if (!timingFor(bitrate_, timing)) return false;
  const twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(tx_pin_, rx_pin_, TWAI_MODE_NORMAL);
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
  if (twai_start() == ESP_OK) return true;
  twai_driver_uninstall();
  return false;
}

bool TwaiBus::write(const Frame& frame, uint32_t timeout_ms) {
  const uint32_t max_id = frame.extended ? 0x1FFFFFFF : 0x7FF;
  if (frame.length > 8 || frame.id > max_id) return false;
  twai_message_t message{};
  message.identifier = frame.id;
  message.extd = frame.extended;
  message.rtr = frame.remote;
  message.data_length_code = frame.length;
  if (!frame.remote) std::memcpy(message.data, frame.data, frame.length);
  return twai_transmit(&message, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

bool TwaiBus::read(Frame& frame, uint32_t timeout_ms) {
  twai_message_t message{};
  if (twai_receive(&message, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) return false;
  if (message.data_length_code > 8) return false;
  frame = {};
  frame.id = message.identifier;
  frame.extended = message.extd;
  frame.remote = message.rtr;
  frame.length = message.data_length_code;
  if (!frame.remote) std::memcpy(frame.data, message.data, frame.length);
  return true;
}
}  // namespace amp::can
