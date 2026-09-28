#include "bsp.hpp"

namespace mib::bsp {

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
				.model = nullptr, // joint models arrive with the base measurements
				.store = &store_,
		};
		if (Actuator::Range saved{}; store_.load_range(row.name, saved)) {
			logger_.info("{}: using calibrated range [{}, {}] from NVS", row.name, saved.min,
									 saved.max);
			config.range = saved;
		}
		auto &slot = actuators_[static_cast<size_t>(row.leg)];
		slot = std::make_unique<Actuator>(*controller.mcp, *controller.client, controller.mutex,
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
