#pragma once

#include <functional>
#include <memory>
#include <string>
#include <optional>
#include <vector>
#include <cmath>

#include "rtps_participant.hpp"
#include "MIBconfig.hpp"
#include "base_component.hpp"
#include "esp32-p4-eth.hpp"
#include "load_cell_lchd.hpp"
#include "oneshot_adc.hpp"

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

    init_LoadCell();

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

  //=======================================================================================================
  // Load Cell Functions
  bool init_LoadCell() {
 
    constexpr auto cfg = config::LOAD_CELL;
 
    // Set up oneshot adc
    static std::vector<espp::AdcConfig> channels{
        {.unit = cfg.adc_unit, .channel = cfg.adc_channel, .attenuation = cfg.adc_atten},
    };
    static espp::OneshotAdc adc({.unit = cfg.adc_unit, .channels = channels});
 
    load_cell.emplace(espp::LoadCell::Config{
      .capacity = cfg.capacity_lbf,
      .rated_output_mv_per_v = cfg.fso_mv_per_v,
      .excitation_mv = cfg.excitation_mv,
      .amplifier_gain = cfg.gain(),
      .amp_offset_mv = cfg.ref_mv,
      .read_mv =
          []() -> std::optional<float> {
            auto maybe_mv = adc.read_mv(channels[0]);
            if (!maybe_mv.has_value()) {
              return std::nullopt;
            }
            return static_cast<float>(maybe_mv.value());
          },
      .num_samples = cfg.samples,
      .filter_alpha = cfg.filter_alpha,
      .log_level = espp::Logger::Verbosity::INFO,
    });
 
    // ---- init checks ---------------------------------------------------------
    const int gpio = 16 + static_cast<int>(cfg.adc_channel); // P4: GPIO = 16 + ADC1 channel
 
    auto maybe_mv = load_cell->get_amplifier_mv();
    if (!maybe_mv.has_value()) {
      logger.error("GPIO{}: read failed, channel not configured or ADC error", gpio);
      load_cell.reset();
      return false;
    }
    const float mv = maybe_mv.value();
    const float off = mv - cfg.ref_mv;
 
    if (mv < 100.0f || mv > 3000.0f) {
      logger.error("GPIO{}: {:.1f} mV is railed, in-amp clipping or bridge open", gpio, mv);
      load_cell.reset();
      return false;
    }
    if (std::abs(off) > 0.1f * cfg.span_mv()) {
      logger.warn("GPIO{}: {:.1f} mV sits {:+.1f} mV off REF, over 10% of span", gpio, mv, off);
    }
 
    logger.info("load cell on GPIO{}: {:.1f} mV ({:+.1f} from REF), initialized", gpio, mv, off);
    return true;
  }
 
  /// @brief Gives load cell reading in lbf, or nullopt if the read failed.
  std::optional<float> get_load_cell_lbf() {
    if (!load_cell) {
      logger.debug("get_load_cell_lbf before init");
      return std::nullopt;
    }
    return load_cell->get_lbf();
  }
 
  /// @brief Gives load cell data to terminal
  bool load_cell_log() {
    if (!load_cell) {
      return false;
    }

    auto r = load_cell->read();
    if (!r.has_value()) {
      logger.error("load cell read failed");
      return false;
    }
    logger.info("{:.1f} mV -> {:+.2f} lbf", r->amplifier_mv, r->load);
    return true;
  }
 
  /// @brief Re-zero load cell
  bool zero_load_cell() {
    if (!load_cell) {
      return false;
    }

    if (!load_cell->tare(64).has_value()) {
      logger.error("tare failed, previous zero kept");
      return false;
    }
    return true;
  }
  //=======================================================================================================

private:
  std::optional<espp::LoadCell> load_cell;

  espp::Logger logger{{.tag = "MIB", .level = espp::Logger::Verbosity::INFO}};

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
