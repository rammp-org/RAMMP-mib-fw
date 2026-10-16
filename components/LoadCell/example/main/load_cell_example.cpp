// Minimal ESP-IDF example: Omega LCHD-1K -> INA118PB -> ESP32 ADC.
//
// Wiring assumed:
//   LCHD-1K bridge  -> INA118 IN+ / IN- (confirm EXC/SIG pinout against the
//                      unit's calibration sheet, not from memory)
//   INA118 Rg       -> 1.28k between pins 1 and 8  => G = 40.06
//   INA118 REF      -> buffered 1.65 V (mid-ADC), so zero load reads ~1.65 V
//   INA118 Vout     -> 1k series + Schottky clamps -> ADC1_CH4, which is
//                      GPIO20 on the ESP32-P4 
//   Excitation      -> 10.0 Vdc from its own regulator

#include <chrono>
#include <optional>
#include <vector>

#include "load_cell_lchd.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"
#include "format.hpp"

using namespace std::chrono_literals;


namespace {

constexpr adc_channel_t signal_channel = ADC_CHANNEL_4;
constexpr adc_unit_t adc_unit = ADC_UNIT_1;

} 

extern "C" void app_main(void) {

  static espp::Logger logger({.tag = "load cell example", .level = espp::Logger::Verbosity::INFO});


  int gpio = -1;

  esp_err_t err = adc_oneshot_channel_to_io(adc_unit, signal_channel, &gpio);
  if (err != ESP_OK) {
    logger.error("channel to GPIO lookup failed: {}", esp_err_to_name(err));
    return;
  }

  static std::vector<espp::AdcConfig> channels{
    {.unit = adc_unit, .channel = signal_channel, .attenuation = ADC_ATTEN_DB_12}};
  static espp::OneshotAdc adc({.unit = adc_unit, .channels = channels});
 

  auto ch = channels[0];
  auto read_mv = [ch]() -> std::optional<float> {
    auto maybe_mv = adc.read_mv(ch);
    if (!maybe_mv.has_value()) {
      return std::nullopt;
    }
    return static_cast<float>(maybe_mv.value());
  };
 
  static mib::LoadCell load_cell({
      .name = "load_cell",
      .capacity = 1000.0f,
      .rated_output_mv_per_v = 3.0000f, // from this unit's calibration sheet
      .excitation_mv = 10000.0f,
      .amplifier_gain = mib::LoadCell::gain_from_rg(1280.0f),
      .amp_offset_mv = 1650.0f,
      .read_mv = read_mv,
      .num_samples = 16,
      .filter_alpha = 0.15f,
      .log_level = espp::Logger::Verbosity::INFO,
  });
 
  auto unloaded = load_cell.get_amplifier_mv();
  if (!unloaded.has_value()) {
    logger.error("no reading from GPIO{}; check the channel is registered with the unit",
                 gpio);
    return;
  }
  logger.info("GPIO{} unloaded: {:.1f} mV, span {:.1f} mV", gpio, unloaded.value(),
              load_cell.nominal_span_mv());
 
  if (!load_cell.tare().has_value()) {
    logger.error("tare failed, refusing to start the read loop");
    return;
  }
 
  logger.info("% time(s), amp(mV), bridge(mV), mV/V, lbf, N");
 
  auto task_fn = [](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - start)
                        .count();
 
  
    auto r = load_cell.read();
    if (!r.has_value()) {
      logger.error("{:.3f} s: read failed", elapsed);
    } else {
      fmt::print("{:.3f}, {:.1f}, {:+.4f}, {:+.4f}, {:+.2f}, {:+.1f}\n", elapsed, r->amplifier_mv,
                 r->bridge_mv, r->mv_per_v, r->load, r->load * 4.4482216f);
      if (r->overloaded) {
        logger.warn("past 150% of rating");
      }
    }
 
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 100ms);
    return false;
  };

  static auto task = espp::Task({.callback = task_fn,
                                 .task_config = {.name = "load cell", .stack_size_bytes = 6 * 1024},
                                 .log_level = espp::Logger::Verbosity::WARN});
  task.start();
}


