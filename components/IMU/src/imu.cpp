#include "imu.hpp"
#include "logger.hpp"

bool IMU::init(void) {
  espp::Logger logger({.tag = "custom_imu_component", .level = espp::Logger::Verbosity::DEBUG});

  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set("MAIN", ESP_LOG_INFO);
  esp_log_level_set(BNO055_TAG, ESP_LOG_INFO);

  bno055_t bno055{};

  i2c_master_bus_config_t i2c_master_conf{};

  i2c_master_conf.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_master_conf.flags.enable_internal_pullup = true;
  i2c_master_conf.glitch_ignore_cnt = 7;
  i2c_master_conf.i2c_port = I2C_NUM_0;
  i2c_master_conf.intr_priority = 0;
  i2c_master_conf.scl_io_num = static_cast<gpio_num_t>(CONFIG_BNO055_SCL_PIN);
  i2c_master_conf.sda_io_num = static_cast<gpio_num_t>(CONFIG_BNO055_SDA_PIN);
  i2c_master_conf.trans_queue_depth = 0;

  i2c_master_bus_handle_t i2c_master_bus = NULL;

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_conf, &i2c_master_bus));

  i2c_device_config_t bno055_conf{};

  bno055_conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  bno055_conf.device_address = CONFIG_BNO055_I2C_ADDR;
  bno055_conf.flags.disable_ack_check = false;
  bno055_conf.scl_speed_hz = CONFIG_BNO055_I2C_FREQUENCY * 1000;
  bno055_conf.scl_wait_us = 0xffff;

  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(i2c_master_bus, &bno055_conf, &bno055.config.slave_handle));

  ESP_LOGI("MAIN", "BNO055 initialized");

  ESP_ERROR_CHECK(bno055_configure(&bno055, NDOF_MODE, (ACC_MG | GY_RPS | EUL_DEG)));

  ESP_LOGI("MAIN", "BNO055 configured");

  this->bus_ = i2c_master_bus;
  this->bno055_ = bno055;
  // need to add check for failures
  return true;
}