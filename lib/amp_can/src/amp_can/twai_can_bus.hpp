#pragma once

#include <driver/twai.h>

#include "amp_can/can_bus.hpp"

namespace amp::can {

class TwaiBus {
 public:
  TwaiBus(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t bitrate)
      : tx_pin_(tx_pin), rx_pin_(rx_pin), bitrate_(bitrate) {}
  bool begin();
  bool write(const Frame& frame, uint32_t timeout_ms = 10);
  bool read(Frame& frame, uint32_t timeout_ms = 0);
 private:
  gpio_num_t tx_pin_;
  gpio_num_t rx_pin_;
  uint32_t bitrate_;
};

}  // namespace amp::can
