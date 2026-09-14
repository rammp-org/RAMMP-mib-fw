#pragma once

#include <functional>
#include <memory>
#include <string>

#include "rtps_participant.hpp"
#include "MIBconfig.hpp"
#include "base_component.hpp"
#include "esp32-p4-eth.hpp"

namespace mib::bsp {

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

    return init_ethernet() && init_rtps();
  }

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

  /// @brief Access the underlying ESP32-P4-ETH board-support object.
  /// @return Reference to the initialized ESP32-P4-ETH board object.
  espp::Esp32P4Eth &board() { return *board_; }

  /// @brief Access the started RTPS participant.
  /// @return Reference to the RTPS participant owned by the MIB BSP.
  espp::RtpsParticipant &rtps_participant() { return *rtps_participant_; }

private:
  bool init_rtps();

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

    return board_->initialize_ethernet(config);
  }

  MIB() : BaseComponent("MIB", espp::Logger::Verbosity::INFO) {}
  espp::Esp32P4Eth *board_{nullptr};
  std::unique_ptr<espp::RtpsParticipant> rtps_participant_{nullptr};
  std::string ethernet_ip_address_;
};

} // namespace mib::bsp
