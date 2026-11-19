#include "bsp.hpp"

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <thread>

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
      // Retransmit until acknowledged. On the ESP32-P4's SJA1000-style controller ESP-IDF
      // sets single-shot transmission for every value but -1 (espressif/esp-idf#19195), so
      // 0..15 all mean "one attempt": a frame that lost arbitration was reported unsent,
      // ~1.6 a second on the chair's bus, and each became a failed SDO. With -1: 2 unsent in
      // a 120 s run. The completion timeout in transmit() still gives up on a bus where
      // nothing acknowledges.
      .tx_retry_count = -1,
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
            for (const auto &controller : controllers_) {
              controller->client->process_frame(frame);
            }
            if (auto *probe = probe_client_.load()) {
              probe->process_frame(frame);
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
  logger_.info("CAN up: tx={} rx={} {} bit/s", mib::config::can_tx_gpio, mib::config::can_rx_gpio,
               mib::config::can_bitrate);
  return true;
}

void MIB::start_heartbeat() {
  // CiA 301 heartbeat: COB-ID 0x700 + node id, one byte, 0x05 = operational. The
  // commissioned controllers' heartbeat consumers fault an enabled drive ~450 ms after it
  // stops, so a crashed or reset MIB, or a cut bus, stops the legs. It runs from here on and
  // is never stopped: the consumer cannot be switched off once set.
  heartbeat_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "CAN heartbeat",
      .period = mib::config::can_heartbeat_period,
      .callback = [this]() -> bool {
        const espp::Twai::Message beat{
            .id = 0x700u + mib::config::can_heartbeat_node_id,
            .dlc = 1,
            .data = {0x05},
        };
        std::error_code ec;
        if (twai_ && !twai_->transmit(beat, ec)) {
          logger_.warn_rate_limited("CAN heartbeat not sent: {}", ec.message());
        }
        return false; // keep running
      },
      .stack_size_bytes = 3 * 1024,
      // Above the actuator and console tasks: the heartbeat should stop only when the
      // firmware does.
      .priority = 9,
  });
  logger_.info("CAN heartbeat: node {} every {} ms; consumers time out after {} ms",
               mib::config::can_heartbeat_node_id, mib::config::can_heartbeat_period.count(),
               mib::config::can_heartbeat_timeout.count());
}

std::vector<std::unique_ptr<MIB::Controller>>::iterator MIB::find_controller(uint8_t node_id) {
  return std::find_if(controllers_.begin(), controllers_.end(),
                      [node_id](const auto &controller) { return controller->node_id == node_id; });
}

MIB::Controller &MIB::controller_for(uint8_t node_id) {
  const auto existing = find_controller(node_id);
  if (existing != controllers_.end()) {
    return **existing;
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
      .sdo_timeout = mib::config::can_sdo_timeout,
      .log_level = espp::Logger::Verbosity::WARN,
  });
  // Statusword polled every 5 ms (not espp's 25) while the drive walks to OPERATION
  // ENABLED, so a move starts sooner.
  controller->mcp = std::make_unique<espp::Mcp266>(
      *controller->client, espp::Mcp266::Config{.poll_period = std::chrono::milliseconds{5},
                                                .log_level = espp::Logger::Verbosity::WARN});
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
  // Heartbeat first: a controller's consumer starts watching at the first heartbeat it
  // sees after being set.
  start_heartbeat();

  store_.init(); // logs itself if NVS is unavailable

  // Controllers are created on first use, so a re-pairing of legs in MIBconfig.hpp needs no
  // change here. All are created before any actuator is initialized, because the TWAI
  // receive callback iterates them. Creating one sends nothing.
  for (const auto &row : mib::config::actuators) {
    controller_for(row.node_id);
  }

  const uint32_t heartbeat_consumer =
      (static_cast<uint32_t>(mib::config::can_heartbeat_node_id) << 16) |
      static_cast<uint32_t>(mib::config::can_heartbeat_timeout.count());
  for (const auto &row : mib::config::actuators) {
    auto &controller = controller_for(row.node_id);
    const auto &t = row.tuning;
    Actuator::Config config{
        .name = row.name,
        .axis = row.channel == 0 ? Actuator::Axis::M1 : Actuator::Axis::M2,
        .commissioned = row.commissioned,
        .range = {row.min_counts, row.max_counts},
        .profile = {row.max_speed, row.acceleration},
        .lead = std::chrono::milliseconds{row.lead_ms},
        .controller = {.position_window = t.position_window,
                       .position_p = t.position_p,
                       .position_i = t.position_i,
                       .position_d = t.position_d,
                       .position_max_i = t.position_max_i,
                       .position_deadband = t.position_deadband,
                       .velocity_p = t.velocity_p,
                       .velocity_i = t.velocity_i,
                       .velocity_d = t.velocity_d,
                       .velocity_qpps = t.velocity_qpps,
                       .profile_velocity = t.profile_velocity,
                       // Only a commissioned leg's actuator writes it (to its node).
                       .heartbeat_consumer = heartbeat_consumer},
        .jog_step = row.jog_step,
        .tolerance = row.tolerance,
        .hardware_limits = row.hardware_limits,
        .homing = {.required = row.incremental_encoder,
                   .direction = row.home_direction,
                   .home_count = row.home_count},
        .store = &store_,
    };
    if (Actuator::Range saved{}; store_.load_range(row.name, saved)) {
      logger_.info("{}: using calibrated range [{}, {}] from NVS", row.name, saved.min, saved.max);
      config.range = saved;
    }
    auto &slot = actuators_[static_cast<size_t>(row.leg)];
    slot =
        std::make_unique<Actuator>(*controller.client, *controller.mcp, controller.mutex, config);
  }

  // Each commissioned actuator NMT-starts its controller and writes and reads back its
  // settings; one that is not commissioned sends nothing. No fault is cleared, nothing
  // moves, nothing is homed.
  size_t commissioned = 0;
  size_t ready = 0;
  for (auto &actuator : actuators_) {
    if (!actuator->commissioned()) {
      actuator->initialize(); // logs that it is not commissioned; sends nothing
      continue;
    }
    ++commissioned;
    if (actuator->initialize() && actuator->ready()) {
      ++ready;
    }
  }
  logger_.info("{} of {} commissioned leg actuators ready ({} legs configured)", ready,
               commissioned, actuators_.size());
  return true;
}

bool MIB::poll_actuators() {
  bool ok = true;
  for (auto &actuator : actuators_) {
    if (actuator && !actuator->poll()) {
      ok = false;
    }
  }
  return ok;
}

bool MIB::home_actuator(config::Leg leg) { return actuator(leg).home(); }

bool MIB::home_actuators() {
  bool all = true;
  for (auto &actuator : actuators_) {
    if (actuator && actuator->commissioned() && !actuator->ready() && !actuator->home()) {
      all = false;
    }
  }
  return all;
}

void MIB::save_actuator_positions() {
  for (auto &actuator : actuators_) {
    if (actuator && actuator->commissioned()) {
      actuator->save_position();
    }
  }
}

bool MIB::stop_all_actuators() {
  bool all = true;
  for (auto &actuator : actuators_) {
    if (actuator && actuator->commissioned() && !actuator->stop()) {
      all = false;
    }
  }
  return all;
}

std::array<std::optional<int32_t>, config::leg_count> MIB::read_all_positions() {
  std::array<std::optional<int32_t>, config::leg_count> positions{};
  for (size_t i = 0; i < actuators_.size(); ++i) {
    int32_t counts = 0;
    if (actuators_[i] && actuators_[i]->commissioned() && actuators_[i]->read_position(counts)) {
      positions[i] = counts;
    }
  }
  return positions;
}

bool MIB::set_actuator_range(config::Leg leg, const Actuator::Range &range) {
  return actuator(leg).set_range(range); // persists through the store it was given
}

std::optional<MIB::ControllerAccess> MIB::controller_access(uint8_t node_id) {
  const auto it = find_controller(node_id);
  if (it == controllers_.end()) {
    return std::nullopt;
  }
  Controller &controller = **it;
  return ControllerAccess{.client = controller.client.get(), .mutex = &controller.mutex};
}

std::vector<std::pair<uint8_t, uint32_t>> MIB::scan_bus(std::chrono::milliseconds timeout) {
  return scan_bus(1, 127, timeout);
}

namespace {
espp::CanopenClient::Config probe_config(uint8_t id, std::chrono::milliseconds timeout,
                                         std::unique_ptr<espp::Twai> &twai) {
  return {
      .node_id = id,
      .send =
          [&twai](const espp::CanopenClient::CanFrame &frame) {
            espp::Twai::Message m{.id = frame.id,
                                  .extended = frame.extended,
                                  .rtr = frame.rtr,
                                  .dlc = frame.dlc,
                                  .data = frame.data};
            std::error_code tx_ec;
            return twai && twai->transmit(m, tx_ec);
          },
      .sdo_timeout = timeout,
      .log_level = espp::Logger::Verbosity::NONE,
  };
}
} // namespace

std::vector<std::pair<uint8_t, uint32_t>>
MIB::scan_bus(uint8_t first, uint8_t last, std::chrono::milliseconds timeout) {
  std::vector<std::pair<uint8_t, uint32_t>> found;
  if (!twai_) {
    return found;
  }
  std::lock_guard<std::mutex> lock(probe_mutex_);
  for (unsigned id = std::max<unsigned>(first, 1); id <= std::min<unsigned>(last, 127); ++id) {
    espp::CanopenClient probe(probe_config(static_cast<uint8_t>(id), timeout, twai_));
    probe_client_.store(&probe);
    std::error_code ec;
    const uint32_t device_type = probe.read_u32(0x1000, 0, ec);
    probe_client_.store(nullptr);
    // The receive task may still be inside probe.process_frame(): let it leave before the
    // probe goes out of scope.
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
    if (!ec) {
      found.emplace_back(static_cast<uint8_t>(id), device_type);
    }
  }
  return found;
}

bool MIB::read_sdo(uint8_t node, uint16_t index, uint8_t sub, uint8_t bytes, int32_t &value,
                   std::error_code &ec) {
  ec.clear();
  if (!twai_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  const auto read = [&](espp::CanopenClient &client) {
    switch (bytes) {
    case 1:
      value = client.read_i8(index, sub, ec);
      break;
    case 2:
      value = client.read_i16(index, sub, ec);
      break;
    case 4:
      value = client.read_i32(index, sub, ec);
      break;
    default:
      ec = std::make_error_code(std::errc::invalid_argument);
    }
    return !ec;
  };
  if (const auto access = controller_access(node)) {
    std::lock_guard<std::mutex> lock(*access->mutex);
    return read(*access->client);
  }
  // Only nodes with a configured controller: a short-lived probe client here crashed the MIB
  // (2026-11-19, reset reason panic) and is not used until that is understood.
  ec = std::make_error_code(std::errc::no_such_device);
  return false;
}

} // namespace mib::bsp
