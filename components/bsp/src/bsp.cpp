#include "bsp.hpp"

#include <cstdlib>
#include <ctime>

#include "dhcpserver/dhcpserver.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "nvs.hpp"
#include "nvs_handle_espp.hpp"
#include "sdkconfig.h"

namespace mib::bsp {

namespace {

static_assert(mib::config::dhcp_pool_window_size >= CONFIG_LWIP_DHCPS_MAX_STATION_NUM,
              "a DHCP pool window must hold every lease the server keeps");

constexpr const char *kNetNamespace = "mib_net";
constexpr const char *kNextWindowKey = "next_pool_win";

/// espp's DHCP-server netif key (espp::Ethernet); Esp32P4Eth does not expose its netif.
constexpr const char *kDhcpServerNetif = "ETH_DHCPS";

std::uint8_t pool_window_first_host(std::uint8_t window) {
  return mib::config::dhcp_pool_first_host + window * mib::config::dhcp_pool_window_size;
}

} // namespace

void MIB::choose_dhcp_pool_window() {
  std::error_code ec;
  espp::Nvs().init(ec); // a no-op if the partition is already up
  if (ec) {
    logger_.error("NVS init failed ({}); serving the first DHCP pool window", ec.message());
    return;
  }
  espp::NvsHandle handle(kNetNamespace, ec);
  if (ec) {
    logger_.error("NVS open failed ({}); serving the first DHCP pool window", ec.message());
    return;
  }
  std::uint8_t window = 0;
  std::error_code read_ec;
  handle.get(kNextWindowKey, window, read_ec); // not found on the first boot: stays 0
  dhcp_pool_window_ = window % mib::config::dhcp_pool_window_count;
  const std::uint8_t next = (dhcp_pool_window_ + 1) % mib::config::dhcp_pool_window_count;
  handle.set(kNextWindowKey, next, ec);
  if (!ec) {
    handle.commit(ec);
  }
  if (ec) {
    logger_.warn("Could not save the next DHCP pool window ({}); the next boot reuses this one",
                 ec.message());
  }
}

void MIB::apply_dhcp_pool_window() {
  const std::uint8_t first = pool_window_first_host(dhcp_pool_window_);
  const std::uint8_t last = first + mib::config::dhcp_pool_window_size - 1;
  dhcps_lease_t pool{};
  pool.enable = true;
  IP4_ADDR(&pool.start_ip, mib::config::ethernet_ip[0], mib::config::ethernet_ip[1],
           mib::config::ethernet_ip[2], first);
  IP4_ADDR(&pool.end_ip, mib::config::ethernet_ip[0], mib::config::ethernet_ip[1],
           mib::config::ethernet_ip[2], last);

  esp_netif_t *netif = esp_netif_get_handle_from_ifkey(kDhcpServerNetif);
  esp_err_t err = netif ? esp_netif_dhcps_stop(netif) : ESP_ERR_NOT_FOUND;
  if (err == ESP_OK) {
    err = esp_netif_dhcps_option(netif, ESP_NETIF_OP_SET, ESP_NETIF_REQUESTED_IP_ADDRESS, &pool,
                                 sizeof(pool));
    // Start it again even if the option was refused: a default pool beats no server.
    const esp_err_t start_err = esp_netif_dhcps_start(netif);
    err = err != ESP_OK ? err : start_err;
  }
  if (err != ESP_OK) {
    logger_.error("Failed to move the DHCP pool: {}", esp_err_to_name(err));
    return;
  }
  logger_.info("DHCP pool window {} of {}: .{} to .{}", dhcp_pool_window_ + 1,
               mib::config::dhcp_pool_window_count, first, last);
}

DhcpStatus MIB::dhcp_status() const {
  DhcpStatus status;
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey(kDhcpServerNetif);
  esp_netif_dhcp_status_t server = ESP_NETIF_DHCP_INIT;
  status.running = netif && esp_netif_dhcps_get_status(netif, &server) == ESP_OK &&
                   server == ESP_NETIF_DHCP_STARTED;
  status.leases_assigned = dhcp_leases_assigned_.load();
  const std::int64_t last_lease_ms = last_lease_ms_.load();
  if (last_lease_ms >= 0) {
    status.last_lease_age_s = (esp_timer_get_time() / 1000 - last_lease_ms) / 1000;
  }
  status.pool_first_host = pool_window_first_host(dhcp_pool_window_);
  status.pool_last_host = status.pool_first_host + mib::config::dhcp_pool_window_size - 1;
  return status;
}

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

} // namespace mib::bsp
