#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "MIBconfig.hpp"
#include "base_component.hpp"
#include "bno055.hpp"
#include "esp32-p4-eth.hpp"
#include "esp_timer.h"
#include "rtps_participant.hpp"
#include "timer.hpp"

namespace mib::bsp {

/// @brief What the MIB's DHCP server is doing, for the panel's status.
struct DhcpStatus {
  bool running{false};               ///< ESP-IDF reports the server as started.
  std::uint32_t leases_assigned{0};  ///< ACKs since boot, renewals included.
  std::int64_t last_lease_age_s{-1}; ///< Seconds since the last ACK, -1 if there has been none.
  std::uint8_t pool_first_host{0};   ///< First host address of this boot's pool window.
  std::uint8_t pool_last_host{0};    ///< Last host address of this boot's pool window.
};

/// @brief Board Support Package for the RAMMP MIB.
///
/// Provides the MIB firmware with a singleton interface to the ESP32-P4-ETH
/// board and initializes Ethernet using the settings from MIBconfig.hpp.
/// Ethernet runs as a DHCP server by default at 192.168.4.1.
class MIB : public espp::BaseComponent {
public:
  /// @brief Access the singleton MIB board-support instance.
  /// @return Reference to the shared MIB board-support instance.
  static MIB &instance() {
    static MIB mib;
    return mib;
  }

  /// @brief Initialize the MIB board and its configured Ethernet interface.
  /// @return True if the board and Ethernet interface were initialized.
  bool init() {
    board_ = &espp::Esp32P4Eth::get();
    board_->set_log_level(espp::Logger::Verbosity::INFO);

    return init_ethernet();
  }

  /// @brief Bring up the BNO055 IMU on the board's internal I2C bus.
  /// @return True if the IMU responded and entered its fusion mode.
  /// @note Blocks for roughly 700 ms while the IMU resets.
  bool init_imu();

  /// @brief Access the IMU.
  /// @return Pointer to the IMU, or nullptr if init_imu() has not succeeded.
  espp::Bno055 *imu() { return imu_.get(); }

  /// @brief Start RTPS after Ethernet has an active link.
  /// @return True if the RTPS participant was started.
  bool init_rtps();

  /// @brief Start syncing the system clock to the ROS host over SNTP.
  /// @details Returns without waiting for the first sync; until it lands, the
  ///          system clock is time since boot, and the client is restarted
  ///          every mib::config::time_sync_retry_interval.
  /// @return True if the SNTP client was started.
  bool init_time_sync();

  /// @brief Check whether Ethernet is connected and has an IP address.
  /// @return True if Ethernet has an active link and a valid IP address.
  bool ethernet_connected() const {
    return board_ && board_->is_ethernet_connected();
  }

  /// @brief Get the current Ethernet IPv4 address.
  /// @return The IPv4 address as a dotted string, or "no-link" when unavailable.
  std::string ethernet_ip_string() const {
    if (!board_ || !board_->is_ethernet_connected()) {
      return "no-link";
    }

    auto ip = board_->ethernet_ip();
    return std::to_string(esp_ip4_addr1_16(&ip)) + "." +
           std::to_string(esp_ip4_addr2_16(&ip)) + "." +
           std::to_string(esp_ip4_addr3_16(&ip)) + "." +
           std::to_string(esp_ip4_addr4_16(&ip));
  }

  /// @brief Read the DHCP server's state.
  /// @return The server's state, this boot's pool window, and when it last gave out a lease.
  DhcpStatus dhcp_status() const;

  /// @brief Access the underlying ESP32-P4-ETH board-support object.
  /// @return Reference to the initialized ESP32-P4-ETH board object.
  espp::Esp32P4Eth &board() { return *board_; }

  /// @brief Access the started RTPS participant.
  /// @return Reference to the RTPS participant owned by the MIB BSP.
  espp::RtpsParticipant &rtps_participant() { return *rtps_participant_; }

  /// @brief Whether SNTP has set the clock since boot.
  /// @return True once the first sync has landed; until then the clock counts from 1970.
  bool time_synced() const { return time_synced_.load(); }

private:
  bool init_ethernet() {
    espp::Esp32P4Eth::EthernetConfig config{};
    config.mode = mib::config::ethernet_dhcp_server
                      ? espp::Esp32P4Eth::DhcpMode::SERVER
                      : espp::Esp32P4Eth::DhcpMode::CLIENT;
    config.server_config.ip_info.ip.addr =
        ESP_IP4TOADDR(mib::config::ethernet_ip[0], mib::config::ethernet_ip[1],
                      mib::config::ethernet_ip[2], mib::config::ethernet_ip[3]);
    config.server_config.ip_info.netmask.addr = ESP_IP4TOADDR(
        mib::config::ethernet_netmask[0], mib::config::ethernet_netmask[1],
        mib::config::ethernet_netmask[2], mib::config::ethernet_netmask[3]);
    config.server_config.ip_info.gw.addr = ESP_IP4TOADDR(
        mib::config::ethernet_gateway[0], mib::config::ethernet_gateway[1],
        mib::config::ethernet_gateway[2], mib::config::ethernet_gateway[3]);
    config.on_got_ip = [this](esp_ip4_addr_t ip) {
      ethernet_ip_address_ = std::to_string(esp_ip4_addr1_16(&ip)) + "." +
                             std::to_string(esp_ip4_addr2_16(&ip)) + "." +
                             std::to_string(esp_ip4_addr3_16(&ip)) + "." +
                             std::to_string(esp_ip4_addr4_16(&ip));
      logger_.info("Ethernet got IP: {}", ethernet_ip_address_);
    };

    if (mib::config::ethernet_dhcp_server) {
      choose_dhcp_pool_window();
      // The server is already up by the first link, and no client can have reached it.
      config.on_link_up = [this]() {
        if (!dhcp_pool_applied_.exchange(true)) {
          apply_dhcp_pool_window();
        }
      };
      config.server_config.on_client_assigned = [this](esp_ip4_addr_t, std::array<uint8_t, 6>) {
        last_lease_ms_ = esp_timer_get_time() / 1000;
        dhcp_leases_assigned_++;
      };
    }

    return board_->initialize_ethernet(config);
  }

  /// @brief Pick this boot's DHCP pool window, and store the next one in NVS.
  /// @details Runs before Ethernet, off the event-loop task's small stack. Without NVS the MIB
  ///          serves the first window on every boot.
  void choose_dhcp_pool_window();

  /// @brief Restart the DHCP server on this boot's pool window.
  /// @details Runs on the event-loop task. espp starts the server inside initialize_ethernet() and
  ///          the pool can only change while it is stopped, so this cannot happen any earlier.
  void apply_dhcp_pool_window();

  MIB() : BaseComponent("MIB", espp::Logger::Verbosity::INFO) {}
  espp::Esp32P4Eth *board_{nullptr};
  std::unique_ptr<espp::RtpsParticipant> rtps_participant_{nullptr};
  std::unique_ptr<espp::Bno055> imu_{nullptr};
  std::unique_ptr<espp::Timer> time_sync_retry_timer_{nullptr};
  std::atomic<bool> time_synced_{false};
  std::string ethernet_ip_address_;
  std::uint8_t dhcp_pool_window_{0};
  std::atomic<bool> dhcp_pool_applied_{false};
  std::atomic<std::uint32_t> dhcp_leases_assigned_{0};
  std::atomic<std::int64_t> last_lease_ms_{-1};
};

} // namespace mib::bsp
