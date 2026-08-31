#pragma once
#include "bno055.h"
#include "driver/i2c_master.h"

class IMU {
public:
  // Sets up I2C + adds device + configures NDOF. Returns false on failure.
  bool init();

  bno055_t &device() { return bno055_; }

private:
  i2c_master_bus_handle_t bus_{nullptr};
  bno055_t bno055_{};
};