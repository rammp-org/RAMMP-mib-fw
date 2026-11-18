#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "MIBconfig.hpp"
#include "actuator.hpp"
#include "actuator_store.hpp"
#include "base_component.hpp"
#include "bno055.hpp"
#include "canopen_client.hpp"
#include "esp32-p4-eth.hpp"
#include "mcp266.hpp"
#include "rtps_participant.hpp"
#include "timer.hpp"
#include "twai.hpp"

namespace mib::bsp {

/// @brief Board Support Package for the RAMMP MIB.
///
/// Provides the MIB firmware with a singleton interface to the ESP32-P4-ETH
/// board and initializes Ethernet using the settings from MIBconfig.hpp.
/// Ethernet runs as a DHCP server by default at 192.168.4.1.
///
/// It also owns the CAN bus to the leg actuators: one TWAI node, the MIB's
/// CANopen heartbeat, one CANopen client and MCP266 object per controller, and
/// one Actuator per leg. The actuator calls are synchronous (each is a few CAN
/// round trips) and must be made from a task that may block, not from an RTPS
/// callback. The owner runs poll_actuators() every Actuator::kTickPeriod.
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

  /// @brief Access the underlying ESP32-P4-ETH board-support object.
  /// @return Reference to the initialized ESP32-P4-ETH board object.
  espp::Esp32P4Eth &board() { return *board_; }

  /// @brief Access the started RTPS participant.
  /// @return Reference to the RTPS participant owned by the MIB BSP.
  espp::RtpsParticipant &rtps_participant() { return *rtps_participant_; }

  /// @brief Whether SNTP has set the clock since boot.
  /// @return True once the first sync has landed; until then the clock counts from 1970.
  bool time_synced() const { return time_synced_.load(); }

  /// @name Leg actuators
  /// @{

  /// @brief Bring up the CAN bus and the leg actuators.
  ///
  /// Starts TWAI (retransmitting until acknowledged: tx_retry_count = -1), starts
  /// the MIB's heartbeat, creates a CANopen client and MCP266 object per
  /// controller in MIBconfig.hpp, loads any calibrated ranges from NVS over the
  /// compiled defaults, and initializes every actuator: a commissioned leg gets
  /// its controller settings and heartbeat consumer written and read back; a leg
  /// that is not commissioned gets nothing (no frame is addressed to it). Never
  /// moves or homes anything. A controller that does not answer is left offline
  /// and logged; it retries on its next use.
  /// @return True if the bus is up. Per-actuator readiness is Actuator::ready().
  bool init_actuators();

  /// @brief Run every actuator's move engine once. Call every
  /// Actuator::kTickPeriod (10 ms) from one task, on a fixed-period schedule.
  /// Cheap when nothing moves: no bus traffic.
  /// @return False if a tick saw a bus failure.
  bool poll_actuators();

  /// @brief Home one incremental-encoder leg against its limit switch. Blocks
  /// for the approach, while poll_actuators() keeps running in its own task. A
  /// no-op for an absolute-encoder leg; refused for a leg not commissioned.
  bool home_actuator(config::Leg leg);

  /// @brief Home every commissioned leg that needs it and is not yet homed. Not
  /// called by anything at boot: homing is always an explicit request.
  /// @return True if every commissioned leg is ready afterwards.
  bool home_actuators();

  /// @brief Persist the incremental-encoder legs' positions (throttled inside
  /// the actuator). Call from the periodic actuator poll.
  void save_actuator_positions();

  /// @brief Whether init_actuators() brought the bus up.
  bool actuators_ready() const { return twai_ != nullptr; }

  /// @brief The actuator of one leg. Only valid after init_actuators().
  Actuator &actuator(config::Leg leg) { return *actuators_[static_cast<size_t>(leg)]; }

  /// @brief Stop (disable voltage) every commissioned actuator. Continues past
  /// failures. Sends nothing to a leg that is not commissioned.
  /// @return True if every actuator accepted the stop.
  bool stop_all_actuators();

  /// @brief Read every commissioned leg's joint position, in encoder counts.
  /// @return One entry per leg in config::Leg order; empty where the read failed
  ///         or the leg is not commissioned (it is not read).
  std::array<std::optional<int32_t>, config::leg_count> read_all_positions();

  /// @brief Install and persist a calibrated range for a leg (absolute encoders,
  /// once per installation).
  bool set_actuator_range(config::Leg leg, const Actuator::Range &range);

  /// @brief Diagnostics: the CANopen client of a controller and the mutex its
  /// actuators share, for raw SDO access from a bench console. Hold the mutex
  /// around any sequence. Empty if no leg in MIBconfig.hpp is on that node.
  struct ControllerAccess {
    espp::CanopenClient *client;
    std::mutex *mutex;
  };
  std::optional<ControllerAccess> controller_access(uint8_t node_id);

  /// @brief Diagnostics: ask every node id 1..127 for its device type (0x1000,
  /// SDO upload only) and return the ones that answer. About 127 x timeout.
  std::vector<std::pair<uint8_t, uint32_t>> scan_bus(std::chrono::milliseconds timeout);

  /// @}

private:
  /// One MCP266 on the bus and everything needed to talk to it.
  struct Controller {
    uint8_t node_id{0};
    std::unique_ptr<espp::CanopenClient> client;
    std::unique_ptr<espp::Mcp266> mcp;
    std::mutex mutex; ///< The controller has one SDO channel; both axes share it.
  };

  Controller &controller_for(uint8_t node_id);
  /// The registered controller for node_id, or controllers_.end().
  std::vector<std::unique_ptr<Controller>>::iterator find_controller(uint8_t node_id);
  bool init_twai();
  /// Start the MIB's CANopen heartbeat (MIBconfig.hpp can_heartbeat_*).
  void start_heartbeat();

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
  std::unique_ptr<espp::Bno055> imu_{nullptr};
  std::unique_ptr<espp::Timer> time_sync_retry_timer_{nullptr};
  std::atomic<bool> time_synced_{false};
  std::string ethernet_ip_address_;

  std::unique_ptr<espp::Twai> twai_;
  std::unique_ptr<espp::Timer> heartbeat_timer_;
  std::vector<std::unique_ptr<Controller>> controllers_;
  std::atomic<espp::CanopenClient *> probe_client_{nullptr}; ///< scan_bus()'s, while it runs.
  std::array<std::unique_ptr<Actuator>, config::leg_count> actuators_;
  ActuatorStore store_;
};

} // namespace mib::bsp
