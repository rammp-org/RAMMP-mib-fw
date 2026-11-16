#include "bsp.hpp"

#include <cstdlib>
#include <ctime>

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
  // sync interval plus the one-way network delay, well under 1 ms on the direct link. Smooth sync
  // slews at 1/64: following a 10 s step of the host's clock would take ~10 min.
  // SNTP sets UTC; TZ only turns it into local time, for MibStatus.utc_offset_min.
  setenv("TZ", mib::config::timezone, 1);
  tzset();
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(mib::config::ntp_server);
  config.wait_for_sync = false;
  // Runs on lwIP's tcpip thread, at the bottom of its receive path on a 3 KB stack: only set the
  // flag, and let the retry timer log.
  config.sync_cb = [](struct timeval *) { MIB::instance().time_synced_ = true; };

  esp_err_t err = esp_netif_sntp_init(&config);
  if (err != ESP_OK) {
    logger_.error("Failed to start SNTP: {}", esp_err_to_name(err));
    return false;
  }
  logger_.info("SNTP syncing to {}", mib::config::ntp_server);

  // A restart resets lwIP's retry backoff. Once synced, the regular sync interval takes over. A
  // host that goes away after that gets lwIP's backoff (up to 150 s), and the clock free-runs.
  time_sync_retry_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "SNTP retry",
      .period = mib::config::time_sync_retry_interval,
      .delay = mib::config::time_sync_retry_interval,
      .callback = [this]() -> bool {
        if (time_synced_) {
          logger_.info("System clock synced to {}", mib::config::ntp_server);
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

// ------------------------------------------------------------------ actuators

bool MIB::init_twai() {
	twai_ = std::make_unique<espp::Twai>(espp::Twai::Config{
			.tx_gpio = mib::config::can_tx_gpio,
			.rx_gpio = mib::config::can_rx_gpio,
			.baudrate = mib::config::can_bitrate,
			.mode = espp::Twai::Mode::NORMAL,
			.tx_queue_depth = 8,
			// SDO exchanges are request/response, so a frame nobody acknowledged is
			// retried a few times then reported as failed rather than sent forever.
			.tx_retry_count = 3,
			.on_receive =
					[this](const espp::Twai::Message &message) {
						// Every client filters on its own node id, so each sees every frame.
						const espp::CanopenClient::CanFrame frame{
								.id = message.id,
								.extended = message.extended,
								.rtr = message.rtr,
								.dlc = message.dlc,
								.data = message.data,
						};
						for (auto &controller : controllers_) {
							controller->client->process_frame(frame);
						}
					},
			.log_level = espp::Logger::Verbosity::WARN,
	});

	std::error_code ec;
	if (!twai_->initialize(ec)) {
		logger_.error("TWAI init failed on tx={} rx={}: {}", mib::config::can_tx_gpio,
								mib::config::can_rx_gpio, ec.message());
		twai_.reset();
		return false;
	}
	logger_.info("CAN up: tx={} rx={} {} bit/s", mib::config::can_tx_gpio,
							 mib::config::can_rx_gpio, mib::config::can_bitrate);
	return true;
}

MIB::Controller &MIB::controller_for(uint8_t node_id) {
	for (auto &controller : controllers_) {
		if (controller->node_id == node_id) {
			return *controller;
		}
	}
	auto controller = std::make_unique<Controller>();
	controller->node_id = node_id;
	controller->client = std::make_unique<espp::CanopenClient>(espp::CanopenClient::Config{
			.node_id = node_id,
			.send =
					[this](const espp::CanopenClient::CanFrame &frame) {
						espp::Twai::Message message{
								.id = frame.id,
								.extended = frame.extended,
								.rtr = frame.rtr,
								.dlc = frame.dlc,
								.data = frame.data,
						};
						std::error_code ec;
						return twai_ && twai_->transmit(message, ec);
					},
			.sdo_timeout = std::chrono::milliseconds{100},
			.log_level = espp::Logger::Verbosity::WARN,
	});
	controller->mcp = std::make_unique<espp::Mcp266>(
			*controller->client, espp::Mcp266::Config{.log_level = espp::Logger::Verbosity::WARN});
	controllers_.push_back(std::move(controller));
	return *controllers_.back();
}

bool MIB::init_actuators() {
	if (twai_) {
		return true;
	}
	if (!init_twai()) {
		return false;
	}

	store_.init(); // logs itself if NVS is unavailable

	// Controllers are created on first use, so a re-pairing of legs in MIBconfig.hpp
	// needs no change here. All are created before any actuator is initialized, because
	// the TWAI receive callback iterates them.
	for (const auto &row : mib::config::actuators) {
		controller_for(row.node_id);
	}

	for (const auto &row : mib::config::actuators) {
		auto &controller = controller_for(row.node_id);
		Actuator::Config config{
				.name = row.name,
				.axis = row.channel == 0 ? Actuator::Axis::M1 : Actuator::Axis::M2,
				.range = {row.min_counts, row.max_counts},
				.profile = {row.velocity, row.acceleration, row.deceleration},
				.jog_step = row.jog_step,
				.tolerance = row.tolerance,
				.hardware_limits = row.hardware_limits,
				.homing = {.required = row.incremental_encoder,
									 .direction = row.home_direction,
									 .home_count = row.home_count},
				.store = &store_,
		};
		if (Actuator::Range saved{}; store_.load_range(row.name, saved)) {
			logger_.info("{}: using calibrated range [{}, {}] from NVS", row.name, saved.min,
									 saved.max);
			config.range = saved;
		}
		auto &slot = actuators_[static_cast<size_t>(row.leg)];
		slot = std::make_unique<Actuator>(*controller.mcp, controller.mutex,
																			config);
	}

	// Each controller is NMT-started once; then each axis is prepared. A silent
	// controller is reported here and retried on the next use of its actuators.
	for (auto &controller : controllers_) {
		std::lock_guard<std::mutex> lock(controller->mutex);
		std::error_code ec;
		if (!controller->mcp->start(ec)) {
			logger_.error("MCP266 node {} did not answer: {}", controller->node_id, ec.message());
		}
	}
	size_t ready = 0;
	for (auto &actuator : actuators_) {
		actuator->initialize();
		if (actuator->ready()) {
			++ready;
		}
	}
	logger_.info("{} of {} leg actuators ready", ready, actuators_.size());
	actuators_up_ = true;
	return true;
}

bool MIB::home_actuator(config::Leg leg) { return actuator(leg).home(); }

bool MIB::home_actuators() {
	bool all = true;
	for (auto &actuator : actuators_) {
		if (actuator && !actuator->ready() && !actuator->home()) {
			all = false;
		}
	}
	return all;
}

void MIB::save_actuator_positions() {
	for (auto &actuator : actuators_) {
		if (actuator) {
			actuator->save_position();
		}
	}
}

bool MIB::stop_all_actuators() {
	bool all = true;
	for (auto &actuator : actuators_) {
		if (actuator && !actuator->stop()) {
			all = false;
		}
	}
	return all;
}

std::array<std::optional<int32_t>, config::leg_count> MIB::read_all_positions() {
	std::array<std::optional<int32_t>, config::leg_count> positions{};
	for (size_t i = 0; i < actuators_.size(); ++i) {
		int32_t counts = 0;
		if (actuators_[i] && actuators_[i]->read_position(counts)) {
			positions[i] = counts;
		}
	}
	return positions;
}

bool MIB::set_actuator_range(config::Leg leg, const Actuator::Range &range) {
	return actuator(leg).set_range(range); // persists through the store it was given
}

} // namespace mib::bsp
