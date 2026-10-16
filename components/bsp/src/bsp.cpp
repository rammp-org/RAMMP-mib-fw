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

bool MIB::init_load_cell() {
 
  constexpr auto cfg = config::load_cell;

  // derived values
  constexpr float gain = mib::LoadCell::gain_from_rg(cfg.rg_ohms);
 
    // Set up oneshot adc
  channels_.clear();
  channels_.push_back(
      {.unit = cfg.adc_unit, .channel = cfg.adc_channel, .attenuation = cfg.adc_atten});
 
  adc_.emplace(espp::OneshotAdc::Config{
      .unit = cfg.adc_unit,
      .channels = channels_,
  });
 
  load_cell_.emplace(mib::LoadCell::Config{
    .capacity = cfg.capacity_lbf,
    .rated_output_mv_per_v = cfg.fso_mv_per_v,
    .excitation_mv = cfg.excitation_mv,
    .amplifier_gain = gain,
    .amp_offset_mv = cfg.ref_mv,
    .read_mv =
        [this]() -> std::optional<float> {
          if (!adc_) {
            return std::nullopt;
          }
          auto maybe_mv = adc_->read_mv(channels_[0]);
          if (!maybe_mv.has_value()) {
            return std::nullopt;
          }
          return static_cast<float>(maybe_mv.value());
        },
    .num_samples = cfg.samples,
    .filter_alpha = cfg.filter_alpha,
    .log_level = espp::Logger::Verbosity::INFO,
  });
 
  // ---- init checks 
  int gpio = -1;

  esp_err_t err = adc_oneshot_channel_to_io(cfg.adc_unit, cfg.adc_channel, &gpio);
  if (err != ESP_OK) {
    logger_.error("channel to GPIO lookup failed: {}", esp_err_to_name(err));
    return false;
  }
 
  auto maybe_mv = load_cell_->get_amplifier_mv();
  if (!maybe_mv.has_value()) {
    logger_.error("GPIO{}: read failed, channel not configured or ADC error", gpio);
    load_cell_.reset();
    return false;
  }
  const float mv = maybe_mv.value();
  const float off = mv - cfg.ref_mv;
 
  if (mv < 100.0f || mv > 3000.0f) {
    logger_.error("GPIO{}: {:.1f} mV is railed, in-amp clipping or bridge open", gpio, mv);
    load_cell_.reset();
    return false;
  }
  if (std::abs(off) > 0.1f * load_cell_->nominal_span_mv()) {
    logger_.warn("GPIO{}: {:.1f} mV sits {:+.1f} mV off REF, over 10% of span", gpio, mv, off);
  }
 
  logger_.info("load cell on GPIO{}: {:.1f} mV ({:+.1f} from REF), initialized", gpio, mv, off);
    return true;
}
 
  std::optional<float> MIB::get_load_cell_lbf() {
    if (!load_cell_) {
      logger_.debug("get_load_cell_lbf before init");
      return std::nullopt;
    }
    return load_cell_->get_lbf();
  }
 
  bool MIB::zero_load_cell() {
    if (!load_cell_) {
      return false;
    }

    if (!load_cell_->tare().has_value()) {
      logger_.error("tare failed, previous zero kept");
      return false;
    }
    return true;
  }

} // namespace mib::bsp
