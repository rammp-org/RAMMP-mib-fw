#pragma once

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rtps_participant.hpp"
#include "MIBconfig.hpp"
#include "actuator.hpp"
#include "actuator_store.hpp"
#include "base_component.hpp"
#include "canopen_client.hpp"
#include "esp32-p4-eth.hpp"
#include "mcp266.hpp"
#include "twai.hpp"

namespace mib::bsp {

/// @brief Board Support Package for the RAMMP MIB.
///
/// Provides the MIB firmware with a singleton interface to the ESP32-P4-ETH
/// board and initializes Ethernet using the settings from MIBconfig.hpp.
/// Ethernet runs as a DHCP server by default at 192.168.4.1.
///
/// It also owns the CAN bus to the leg actuators: one TWAI node, one CANopen
/// client and MCP266 object per controller, and one Actuator per leg. The
/// actuator calls are synchronous (each is a few CAN round trips) and must be
/// made from a task that may block, not from an RTPS callback.
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

  /// @brief Start RTPS after Ethernet has an active link.
  /// @return True if the RTPS participant was started.
  bool init_rtps();

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

  /// @name Leg actuators
  /// @{

  /// @brief Bring up the CAN bus and the six leg actuators.
  ///
  /// Starts TWAI, creates a CANopen client and MCP266 object per controller in
  /// MIBconfig.hpp, loads any calibrated ranges from NVS over the compiled
  /// defaults, and initializes every actuator. An incremental-encoder leg
  /// restores its last saved position if one exists; otherwise it needs
  /// home_actuator() before it will move. A controller that does not answer
  /// is left offline and logged; it retries on its next use.
  /// @return True if the bus is up. Per-actuator readiness is Actuator::ready().
  bool init_actuators();

  /// @brief Home one incremental-encoder leg against its limit switch. Blocks
  /// for the approach. A no-op for an absolute-encoder leg.
  bool home_actuator(config::Leg leg);

  /// @brief Home every leg that needs it and is not yet homed.
  /// @return True if every leg is ready afterwards.
  bool home_actuators();

  /// @brief Persist the incremental-encoder legs' positions (throttled inside
  /// the actuator). Call from the periodic actuator poll.
  void save_actuator_positions();

  /// @brief Whether init_actuators() brought the bus up.
  bool actuators_ready() const { return twai_ != nullptr; }

  /// @brief The actuator of one leg. Only valid after init_actuators().
  Actuator &actuator(config::Leg leg) { return *actuators_[static_cast<size_t>(leg)]; }

  /// @brief Quick-stop every actuator. Continues past failures.
  /// @return True if every actuator accepted the stop.
  bool stop_all_actuators();

  /// @brief Read every leg's joint position, in encoder counts.
  /// @return One entry per leg in config::Leg order; empty where the read failed.
  std::array<std::optional<int32_t>, config::leg_count> read_all_positions();

  /// @brief Install and persist a calibrated range for a leg (absolute encoders,
  /// once per installation).
  bool set_actuator_range(config::Leg leg, const Actuator::Range &range);

  /// @}

private:
  /// One MCP266 on the bus and everything needed to talk to it.
  struct Controller {
    uint8_t node_id;
    std::unique_ptr<espp::CanopenClient> client;
    std::unique_ptr<espp::Mcp266> mcp;
    std::mutex mutex; ///< The controller has one SDO channel; both axes share it.
  };

  Controller &controller_for(uint8_t node_id);
  bool init_twai();

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

  std::unique_ptr<espp::Twai> twai_;
  std::vector<std::unique_ptr<Controller>> controllers_;
  std::array<std::unique_ptr<Actuator>, config::leg_count> actuators_;
  ActuatorStore store_;
};

} // namespace mib::bsp
