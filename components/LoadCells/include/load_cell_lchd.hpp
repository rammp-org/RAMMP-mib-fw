#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <mutex>

#include "base_component.hpp"

namespace espp {

/// @brief Strain-gauge load cell amplified by an instrumentation amplifier.
///
/// @details
/// Converts an amplifier output voltage into force. Written for an Omega
/// LCHD-1K (1000 lbf pancake tension/compression cell, 350 ohm bridge,
/// 3 mV/V nominal FSO at 10 Vdc excitation) feeding a TI INA118PB, but every
/// term is a config value so it works for any bridge + in-amp pair.
///
/// Like espp::Thermistor, this class does not own an ADC. It takes a
/// `read_mv` callback, so it composes with espp::OneshotAdc,
/// espp::ContinuousAdc, espp::Ads1x15, or a test stub.
///
/// Signal chain assumed:
/// @code
///   bridge diff (mV) = excitation_v * rated_output_mv_per_v * (load / capacity)
///   amp output (mV)  = amp_offset_mv + gain * bridge diff
/// @endcode
/// where `amp_offset_mv` is whatever the INA118 REF pin (pin 5) sits at, and
/// `gain` is 1 + 50k/Rg.
///
/// @section lchd_hw Hardware notes (read before wiring)
/// - INA118 gain: G = 1 + 50000/Rg. Use the static helpers
///   `gain_from_rg()` / `rg_for_gain()`.
/// - Full-scale bridge output is only +/-30 mV (10 V * 3 mV/V), so pick gain
///   to fill the ADC window, not to maximize it:
///     * Bidirectional (tension + compression): REF ~= 1.65 V, G ~= 40
///       (Rg = 1.28k), giving 1.65 V +/- 1.2 V over +/-1000 lbf.
///     * Tension-only: REF ~= 0.2 V, G ~= 92 (Rg = 549), giving ~0.2-3.0 V.
/// - Drive REF from an op-amp buffer, never a bare resistor divider. Any
///   source impedance there degrades CMRR and gain accuracy.
/// - The INA118 output cannot swing to its own rails. On a single +5 V supply
///   it is limited to roughly 0.9 V .. 4.1 V, which clips a bidirectional
///   design. Run it on +/-5 V (or +5/-5) and keep REF at mid-ADC.
/// - Protect the ESP32 GPIO: series resistor (~1-10k) plus Schottky clamps to
///   GND and 3.3 V. With bipolar supplies a fault can drive the in-amp output
///   negative, which the ADC pin will not survive.
/// - Excitation: 10 Vdc (15 V absolute max). A 350 ohm bridge draws ~29 mA at
///   10 V, so give it its own low-noise regulator, and ground the cable shield
///   at the amplifier end only.
/// - Ratiometric option: divide excitation down to <3.3 V into a second ADC
///   channel and pass `read_excitation_mv`. Excitation drift then cancels out
///   instead of showing up as span error.
/// - Use the FSO printed on the unit's 5-point NIST calibration sheet rather
///   than the nominal 3.000 mV/V; the sheet value is the real one (+/-0.25%).
/// - The ESP32's internal ADC is the accuracy floor here (~12 bits, nonlinear,
///   noisy). For real load-cell work put an ADS1115/ADS1220 in front of it and
///   point `read_mv` at that instead.
///
/// @section lchd_ex1 Example
/// @code{.cpp}
/// std::vector<espp::AdcConfig> channels{
///     {.unit = ADC_UNIT_1, .channel = ADC_CHANNEL_6, .attenuation = ADC_ATTEN_DB_12}};
/// espp::OneshotAdc adc({.unit = ADC_UNIT_1, .channels = channels});
///
/// auto read_mv = [&adc, &channels]() -> float {
///   auto maybe_mv = adc.read_mv(channels[0]);
///   return maybe_mv.has_value() ? static_cast<float>(maybe_mv.value()) : 0.0f;
/// };
///
/// espp::LoadCell load_cell({
///     .capacity = 1000.0f,                  // lbf
///     .rated_output_mv_per_v = 3.0f,        // from the calibration sheet
///     .excitation_mv = 10000.0f,            // 10 Vdc
///     .amplifier_gain = espp::LoadCell::gain_from_rg(1280.0f), // ~40.06
///     .amp_offset_mv = 1650.0f,             // INA118 REF pin
///     .read_mv = read_mv,
///     .num_samples = 16,
///     .filter_alpha = 0.1f,
///     .log_level = espp::Logger::Verbosity::INFO,
/// });
///
/// load_cell.tare();                         // unloaded fixture
/// float lbf = load_cell.get_load();
/// float newtons = load_cell.get_newtons();
/// @endcode
class LoadCell : public BaseComponent {
public:
  /// @brief Function type for reading a voltage.
  /// @return Voltage in millivolts.
  typedef std::function<float(void)> read_mv_fn;

  /// @brief Units that `capacity` is expressed in, and therefore the units
  ///        returned by get_load() / get_capacity().
  enum class Units {
    LBF,     ///< Pounds-force (Omega LCHD-1K is specified in lbf)
    KGF,     ///< Kilograms-force
    NEWTONS, ///< Newtons
  };

  /// @brief One reading, expressed at every stage of the signal chain.
  /// @details All fields come from the same set of ADC samples.
  struct Reading {
    float amplifier_mv{0.0f};  ///< Raw INA118 output in mV, as the ADC saw it
    float bridge_mv{0.0f};     ///< Differential bridge output in mV: REF offset removed and
                               ///< divided by the gain. Not tared, so at zero load this is the
                               ///< cell's zero balance (LCHD spec: +/-1% FSO, i.e. +/-0.3 mV)
    float mv_per_v{0.0f};      ///< bridge_mv normalized by excitation, to compare directly
                               ///< against the FSO on the calibration sheet
    float excitation_mv{0.0f}; ///< Excitation used for this reading, measured if available
    float load{0.0f};          ///< Tared, span-corrected, filtered load in the configured units
    bool overloaded{false};    ///< Whether |load| exceeded the overload threshold
  };

  /// @brief Configuration struct for LoadCell.
  struct Config {
    float capacity{1000.0f};           ///< Rated capacity, in `units`, e.g. 1000 for an LCHD-1K
    float rated_output_mv_per_v{3.0f}; ///< Full-scale output (mV/V). Prefer the value on the
                                       ///< calibration sheet over the nominal 3.0
    float excitation_mv{10000.0f};     ///< Nominal bridge excitation in mV, e.g. 10000
    float amplifier_gain{40.0f};       ///< In-amp gain; for an INA118, 1 + 50000/Rg
    float amp_offset_mv{0.0f};         ///< Amplifier output at zero load, i.e. the INA118
                                       ///< REF pin voltage in mV
    read_mv_fn read_mv{nullptr};       ///< Reads the amplifier output, in mV
    read_mv_fn read_excitation_mv{nullptr}; ///< Optional. Reads actual excitation (mV, already
                                            ///< scaled back up through any divider) for a
                                            ///< ratiometric measurement
    size_t num_samples{1};                  ///< Samples averaged per read (oversampling)
    float filter_alpha{1.0f};               ///< One-pole IIR coefficient in (0, 1]; 1.0 disables
                                            ///< filtering, smaller is smoother and slower
    Units units{Units::LBF};                ///< Units of `capacity` and of get_load()
    float overload_fraction{1.5f};          ///< Fraction of capacity treated as overload. The LCHD
                                            ///< safe overload is 150%, ultimate 300%
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Log level for this class
  };

  /// @brief Constructor.
  /// @param config Configuration struct.
  explicit LoadCell(const Config &config)
      : BaseComponent("LoadCell", config.log_level)
      , capacity_(config.capacity)
      , rated_output_mv_per_v_(config.rated_output_mv_per_v)
      , excitation_mv_(config.excitation_mv)
      , gain_(config.amplifier_gain)
      , amp_offset_mv_(config.amp_offset_mv)
      , read_mv_(config.read_mv)
      , read_excitation_mv_(config.read_excitation_mv)
      , num_samples_(std::max<size_t>(1, config.num_samples))
      , filter_alpha_(std::clamp(config.filter_alpha, 0.0f, 1.0f))
      , units_(config.units)
      , overload_fraction_(config.overload_fraction) {
    if (read_mv_ == nullptr) {
      logger_.error("read_mv is null; all readings will be 0");
    }
    if (capacity_ <= 0.0f) {
      logger_.error("capacity must be > 0, got {}", capacity_);
    }
    if (gain_ <= 0.0f) {
      logger_.error("amplifier_gain must be > 0, got {}", gain_);
    }
    if (rated_output_mv_per_v_ <= 0.0f) {
      logger_.error("rated_output_mv_per_v must be > 0, got {}", rated_output_mv_per_v_);
    }
    if (filter_alpha_ <= 0.0f) {
      logger_.warn("filter_alpha of 0 would freeze the output; using 1.0 (unfiltered)");
      filter_alpha_ = 1.0f;
    }
    logger_.info("full-scale span at the amplifier output: {:.1f} mV for {:.1f} at the cell",
                 nominal_span_mv(), capacity_);
  }

  /// @brief INA118 gain for a given gain-set resistor.
  /// @param rg_ohms Resistance between pins 1 and 8, in ohms.
  /// @return Gain, 1 + 50000/Rg.
  static constexpr float gain_from_rg(float rg_ohms) {
    return rg_ohms > 0.0f ? 1.0f + (INA118_RG_NUMERATOR / rg_ohms) : 1.0f;
  }

  /// @brief Gain-set resistor needed for a target INA118 gain.
  /// @param gain Desired gain, must be > 1.
  /// @return Rg in ohms, 50000/(G - 1). Returns 0 for gain <= 1 (REF pins open).
  static constexpr float rg_for_gain(float gain) {
    return gain > 1.0f ? INA118_RG_NUMERATOR / (gain - 1.0f) : 0.0f;
  }

  /// @brief Read the amplifier output voltage.
  /// @details Averages `num_samples` readings. Unfiltered and untared.
  /// @return Amplifier output in mV.
  float get_amplifier_mv() {
    if (read_mv_ == nullptr) {
      logger_.error("read_mv_ is null");
      return 0.0f;
    }
    float sum = 0.0f;
    for (size_t i = 0; i < num_samples_; i++) {
      sum += read_mv_();
    }
    float mv = sum / static_cast<float>(num_samples_);
    logger_.debug("amplifier output: {:.3f} mV ({} samples)", mv, num_samples_);
    return mv;
  }

  /// @brief Read the differential bridge output, referred back through the gain.
  /// @details Subtracts the amplifier offset (REF) but not the tare.
  /// @return Bridge differential output in mV.
  float get_bridge_mv() { return (get_amplifier_mv() - amp_offset_mv_) / gain_; }

  /// @brief Read the bridge output normalized by excitation.
  /// @details Compare against the FSO on the calibration sheet: this should
  ///          read +/-`rated_output_mv_per_v` at +/-capacity.
  /// @return Bridge output in mV/V.
  float get_mv_per_v() {
    float exc = excitation_mv();
    if (exc <= 0.0f) {
      logger_.error("excitation is 0");
      return 0.0f;
    }
    return get_bridge_mv() / (exc / 1000.0f);
  }

  /// @brief Take one reading and report every stage of the signal chain.
  /// @details One set of ADC samples, so the voltages and the load are
  ///          consistent with each other. This is what to print while bringing
  ///          the hardware up: it lets you see the amplifier output and the
  ///          implied mV/V next to the force.
  /// @return A populated Reading.
  Reading read() {
    Reading r;
    r.amplifier_mv = get_amplifier_mv();
    r.excitation_mv = excitation_mv();
    if (r.excitation_mv <= 0.0f) {
      logger_.error("excitation is 0");
      return r;
    }
    r.bridge_mv = (r.amplifier_mv - amp_offset_mv_) / gain_;
    r.mv_per_v = r.bridge_mv / (r.excitation_mv / 1000.0f);

    // mV at the amplifier output corresponding to full capacity
    float span_mv = span_mv_for_excitation(r.excitation_mv);
    if (span_mv == 0.0f) {
      logger_.error("span is 0; check gain, rated output and excitation");
      return r;
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      // NOTE: the offset comes from REF, not the bridge, so it is removed
      // before dividing by a span that may have been scaled by measured
      // excitation.
      float load =
          ((r.amplifier_mv - amp_offset_mv_ - tare_mv_) / span_mv) * capacity_ * span_correction_;

      if (!filter_primed_) {
        filtered_load_ = load;
        filter_primed_ = true;
      } else {
        filtered_load_ += filter_alpha_ * (load - filtered_load_);
      }
      r.load = filtered_load_;
      r.overloaded = std::abs(filtered_load_) > overload_fraction_ * capacity_;
      last_reading_ = r;
    }

    logger_.debug("amp: {:.3f} mV, bridge: {:.4f} mV ({:.4f} mV/V), load: {:.3f}", r.amplifier_mv,
                  r.bridge_mv, r.mv_per_v, r.load);
    if (r.overloaded) {
      logger_.warn("overload: {:.1f} exceeds {:.0f}% of the {:.1f} rating", r.load,
                   overload_fraction_ * 100.0f, capacity_);
    }
    return r;
  }

  /// @brief Get the load, in the configured units.
  /// @details Takes a reading, applies tare, span correction and the IIR
  ///          filter. Positive is tension, negative is compression (assuming
  ///          SIG+ / SIG- are wired the conventional way).
  /// @return Load in the units given in Config.
  float get_load() { return read().load; }

  /// @brief The most recent Reading, without touching the ADC.
  /// @details Useful for logging or telemetry alongside a control loop that
  ///          already called read(), so the voltage printed is the one the
  ///          loop actually acted on.
  /// @return A copy of the last Reading, zeroed if read() has not been called.
  Reading last_reading() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return last_reading_;
  }

  /// @brief Get the load in pounds-force.
  /// @return Load in lbf.
  float get_lbf() { return to_lbf(get_load()); }

  /// @brief Get the load in kilograms-force.
  /// @return Load in kgf.
  float get_kgf() { return to_lbf(get_load()) * LBF_TO_KGF; }

  /// @brief Get the load in newtons.
  /// @return Load in N.
  float get_newtons() { return to_lbf(get_load()) * LBF_TO_N; }

  /// @brief Zero the cell against the current (unloaded) reading.
  /// @details Averages `samples * num_samples` readings, ignoring the filter,
  ///          and stores the result as the tare offset. Call with the fixture
  ///          assembled but unloaded. The LCHD zero balance is +/-1% FSO, so
  ///          this is expected to be nonzero.
  /// @param samples Number of averaged reads to take.
  /// @return Tare offset in mV, relative to `amp_offset_mv`.
  float tare(size_t samples = 32) {
    samples = std::max<size_t>(1, samples);
    float sum = 0.0f;
    for (size_t i = 0; i < samples; i++) {
      sum += get_amplifier_mv();
    }
    float mean_mv = sum / static_cast<float>(samples);
    std::lock_guard<std::mutex> lk(mutex_);
    tare_mv_ = mean_mv - amp_offset_mv_;
    filter_primed_ = false;
    filtered_load_ = 0.0f;
    logger_.info("tare: {:.3f} mV offset from REF ({:.3f} mV absolute)", tare_mv_, mean_mv);
    return tare_mv_;
  }

  /// @brief Single-point span calibration against a known load.
  /// @details Applies a dead weight or reference cell of `known_load` (in the
  ///          configured units, same sign as the applied direction), then call
  ///          this. Computes and stores a multiplicative correction on top of
  ///          the datasheet span. Tare first.
  /// @param known_load The applied reference load, nonzero.
  /// @param samples Number of averaged reads to take.
  /// @return The resulting span correction factor, or 0 on failure.
  float calibrate(float known_load, size_t samples = 32) {
    if (known_load == 0.0f) {
      logger_.error("known_load must be nonzero");
      return 0.0f;
    }
    samples = std::max<size_t>(1, samples);
    // measure with the existing correction removed
    float saved = span_correction();
    set_span_correction(1.0f);
    float sum = 0.0f;
    for (size_t i = 0; i < samples; i++) {
      reset_filter();
      sum += get_load();
    }
    float measured = sum / static_cast<float>(samples);
    if (measured == 0.0f) {
      logger_.error("measured 0 under a {:.3f} load; restoring previous correction", known_load);
      set_span_correction(saved);
      return 0.0f;
    }
    float correction = known_load / measured;
    set_span_correction(correction);
    logger_.info("calibrated: measured {:.3f} against a known {:.3f}, correction {:.5f}", measured,
                 known_load, correction);
    return correction;
  }

  /// @brief Whether the last reading exceeded the overload threshold.
  /// @return True if |load| > overload_fraction * capacity.
  bool is_overloaded() {
    std::lock_guard<std::mutex> lk(mutex_);
    return filter_primed_ && std::abs(filtered_load_) > overload_fraction_ * capacity_;
  }

  /// @brief Get the tare offset, for persisting to NVS.
  /// @return Tare offset in mV.
  float tare_mv() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return tare_mv_;
  }

  /// @brief Restore a previously stored tare offset.
  /// @param mv Tare offset in mV, as returned by tare_mv().
  void set_tare_mv(float mv) {
    std::lock_guard<std::mutex> lk(mutex_);
    tare_mv_ = mv;
    filter_primed_ = false;
  }

  /// @brief Get the span correction factor, for persisting to NVS.
  /// @return Span correction, 1.0 means the datasheet span is used as-is.
  float span_correction() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return span_correction_;
  }

  /// @brief Restore a previously stored span correction factor.
  /// @param correction Multiplicative correction, as returned by calibrate().
  void set_span_correction(float correction) {
    if (correction == 0.0f) {
      logger_.error("span correction of 0 would zero all readings; ignoring");
      return;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    span_correction_ = correction;
    filter_primed_ = false;
  }

  /// @brief Set the IIR filter coefficient.
  /// @param alpha Coefficient in (0, 1]; 1.0 disables filtering.
  void set_filter_alpha(float alpha) {
    std::lock_guard<std::mutex> lk(mutex_);
    filter_alpha_ = std::clamp(alpha, 0.001f, 1.0f);
  }

  /// @brief Set the number of samples averaged per read.
  /// @param num_samples Samples per read, at least 1.
  void set_num_samples(size_t num_samples) {
    std::lock_guard<std::mutex> lk(mutex_);
    num_samples_ = std::max<size_t>(1, num_samples);
  }

  /// @brief Discard filter state so the next read is taken as-is.
  void reset_filter() {
    std::lock_guard<std::mutex> lk(mutex_);
    filter_primed_ = false;
    filtered_load_ = 0.0f;
  }

  /// @brief Get the rated capacity.
  /// @return Capacity in the configured units.
  float capacity() const { return capacity_; }

  /// @brief Get the units readings are returned in.
  /// @return The configured Units.
  Units units() const { return units_; }

  /// @brief Amplifier output span between zero load and rated capacity.
  /// @details Useful as a design check: this should comfortably fit inside the
  ///          ADC's usable input range without clipping.
  /// @return Span in mV, using the nominal excitation.
  float nominal_span_mv() const { return span_mv_for_excitation(excitation_mv_); }

  /// @brief Smallest load change one ADC LSB can resolve.
  /// @param adc_lsb_mv Size of one ADC code in mV, e.g. 3100/4096 for a 12-bit
  ///        ESP32 channel at ADC_ATTEN_DB_12.
  /// @return Resolution in the configured units, ignoring noise and averaging.
  float resolution_per_lsb(float adc_lsb_mv) const {
    float span = nominal_span_mv();
    return span > 0.0f ? adc_lsb_mv * capacity_ / span : 0.0f;
  }

protected:
  /// @brief Numerator of the INA118 gain equation, in ohms.
  static constexpr float INA118_RG_NUMERATOR = 50000.0f;
  static constexpr float LBF_TO_N = 4.4482216152605f; ///< Pounds-force to newtons
  static constexpr float LBF_TO_KGF = 0.45359237f;    ///< Pounds-force to kilograms-force
  static constexpr float KGF_TO_LBF = 1.0f / LBF_TO_KGF;
  static constexpr float N_TO_LBF = 1.0f / LBF_TO_N;

  /// @brief Amplifier output span for a given excitation.
  float span_mv_for_excitation(float exc_mv) const {
    return (exc_mv / 1000.0f) * rated_output_mv_per_v_ * gain_;
  }

  /// @brief Current excitation in mV, measured if a callback was supplied.
  float excitation_mv() {
    if (read_excitation_mv_ == nullptr) {
      return excitation_mv_;
    }
    float mv = read_excitation_mv_();
    if (mv <= 0.0f) {
      logger_.warn("measured excitation was {:.1f} mV; falling back to nominal {:.1f} mV", mv,
                   excitation_mv_);
      return excitation_mv_;
    }
    return mv;
  }

  /// @brief Convert a value in the configured units to lbf.
  float to_lbf(float value) const {
    switch (units_) {
    case Units::KGF:
      return value * KGF_TO_LBF;
    case Units::NEWTONS:
      return value * N_TO_LBF;
    case Units::LBF:
    default:
      return value;
    }
  }

  float capacity_;                         ///< Rated capacity in `units_`
  float rated_output_mv_per_v_;            ///< Full-scale bridge output, mV/V
  float excitation_mv_;                    ///< Nominal excitation, mV
  float gain_;                             ///< In-amp gain
  float amp_offset_mv_;                    ///< Amplifier output at zero load (REF), mV
  read_mv_fn read_mv_{nullptr};            ///< Reads amplifier output, mV
  read_mv_fn read_excitation_mv_{nullptr}; ///< Optional, reads excitation, mV
  size_t num_samples_;                     ///< Samples averaged per read
  float filter_alpha_;                     ///< IIR coefficient
  Units units_;                            ///< Units for capacity and get_load()
  float overload_fraction_;                ///< Overload threshold as a fraction of capacity

  mutable std::mutex mutex_;    ///< Guards the mutable state below
  float tare_mv_{0.0f};         ///< Zero offset relative to REF, mV
  float span_correction_{1.0f}; ///< Multiplicative span correction
  float filtered_load_{0.0f};   ///< Last filtered load
  bool filter_primed_{false};   ///< Whether filtered_load_ holds a real value
  Reading last_reading_{};      ///< Snapshot of the most recent read()
};

} // namespace espp
