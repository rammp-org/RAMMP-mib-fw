#include "bsp.hpp"

#include "esp_netif_sntp.h"

namespace mib::bsp {

bool MIB::init_imu() {
  auto &i2c = board_->internal_i2c();
  auto imu = std::make_unique<espp::Bno055>(espp::Bno055::Config{
      .device_address = mib::config::imu_i2c_address,
      .probe = [&i2c](uint8_t address) { return i2c.probe_device(address); },
      .write = [&i2c](uint8_t address, const uint8_t *data,
                      size_t length) { return i2c.write(address, data, length); },
      .write_then_read =
          [&i2c](uint8_t address, const uint8_t *write_data, size_t write_length,
                 uint8_t *read_data, size_t read_length) {
            return i2c.write_read(address, write_data, write_length, read_data, read_length);
          },
      .auto_init = false,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;
  if (!imu->init(ec)) {
    logger_.error("Failed to initialize BNO055 IMU: {}", ec.message());
    return false;
  }
  imu_ = std::move(imu);
  return true;
}

bool MIB::init_time_sync() {
  // Every sync steps the clock (no smooth_sync). After the first, a step is the drift over one
  // sync interval, well under 1 ms. Smooth sync slews at 1/64: following a 10 s step of the host's
  // clock would take ~10 min.
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(mib::config::ntp_server);
  config.wait_for_sync = false;
  config.sync_cb = [](struct timeval *) {
    auto &mib = MIB::instance();
    if (!mib.time_synced_.exchange(true)) {
      mib.logger_.info("System clock synced to {}", mib::config::ntp_server);
    }
  };

  esp_err_t err = esp_netif_sntp_init(&config);
  if (err != ESP_OK) {
    logger_.error("Failed to start SNTP: {}", esp_err_to_name(err));
    return false;
  }
  logger_.info("SNTP syncing to {}", mib::config::ntp_server);

  // A restart resets lwIP's retry backoff. Once synced, the regular sync interval takes over.
  time_sync_retry_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "SNTP retry",
      .period = mib::config::time_sync_retry_interval,
      .delay = mib::config::time_sync_retry_interval,
      .callback = [this]() -> bool {
        if (time_synced_) {
          return true;
        }
        esp_netif_sntp_start();
        return false;
      },
      .stack_size_bytes = 3 * 1024,
  });
  return true;
}

bool MIB::init_rtps() {
	std::string interface_address = ethernet_ip_address_;
	if (interface_address.empty() && mib::config::ethernet_dhcp_server) {
		interface_address = std::to_string(mib::config::ethernet_ip[0]) + "." +
								std::to_string(mib::config::ethernet_ip[1]) + "." +
								std::to_string(mib::config::ethernet_ip[2]) + "." +
								std::to_string(mib::config::ethernet_ip[3]);
	}
	if (interface_address.empty()) {
		interface_address = ethernet_ip_string();
	}

	rtps_participant_ = std::make_unique<espp::RtpsParticipant>(espp::RtpsParticipant::Config{
			.interface_address = interface_address,
			.log_level = espp::Logger::Verbosity::INFO,
	});

	if (!rtps_participant_->start()) {
		rtps_participant_.reset();
		return false;
	}

	return true;
}

} // namespace mib::bsp
