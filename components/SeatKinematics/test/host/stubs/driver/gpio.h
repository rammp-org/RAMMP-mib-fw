#pragma once

// Host-test stand-in for ESP-IDF's driver/gpio.h, so the host test can include MIBconfig.hpp
// and check the real mib::config::seat_geometry. MIBconfig.hpp only names gpio_num_t and
// GPIO_NUM_NC; add any other GPIO it starts to use here.
typedef enum {
  GPIO_NUM_NC = -1,
} gpio_num_t;
