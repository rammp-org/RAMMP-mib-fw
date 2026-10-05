// Minimal ESP-IDF example: Omega LCHD-1K -> INA118PB -> ESP32 ADC.
//
// idf_component.yml:
//   dependencies:
//     espp/base_component: "^1.0"
//     espp/adc: "^1.0"
//     espp/task: "^1.0"
//
// Wiring assumed:
//   LCHD-1K bridge  -> INA118 IN+ / IN- (confirm EXC/SIG pinout against the
//                      unit's calibration sheet, not from memory)
//   INA118 Rg       -> 1.28k between pins 1 and 8  => G = 40.06
//   INA118 REF      -> buffered 1.65 V (mid-ADC), so zero load reads ~1.65 V
//   INA118 Vout     -> 1k series + Schottky clamps -> ADC1_CH6, which is
//                      GPIO22 on the ESP32-P4 (ADC1_CH0..CH7 = GPIO16..GPIO23)
//                      and GPIO34 on the original ESP32
//   Excitation      -> 10.0 Vdc from its own regula

#include <chrono>
#include <vector>

#include "include/load_cell_lchd.hpp"
#include "logger.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "load cell example", .level = espp::Logger::Verbosity::INFO});

  // Set up channel
  static std::vector<espp::AdcConfig> channels{
      {.unit = ADC_UNIT_1, .channel = ADC_CHANNEL_4, .attenuation = ADC_ATTEN_DB_12}};
  // Set up ADC reading
  static espp::OneshotAdc adc({.unit = ADC_UNIT_1, .channels = channels});

  auto read_mv = []() -> float {
    auto maybe_mv = adc.read_mv(channels[0]);
    return maybe_mv.has_value() ? static_cast<float>(maybe_mv.value()) : 0.0f;
  };

  // Create load cell object
  static espp::LoadCell load_cell({
      .capacity = 1000.0f,           // lbf, LCHD-1K
      .rated_output_mv_per_v = 3.0f, // replace with the calibration sheet FSO
      .excitation_mv = 10000.0f,
      .amplifier_gain = espp::LoadCell::gain_from_rg(1280.0f),
      .amp_offset_mv = 1650.0f, // INA118 REF pin
      .read_mv = read_mv,
      .num_samples = 16,
      .filter_alpha = 0.15f,
      .units = espp::LoadCell::Units::LBF,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  logger.info("full-scale span: {:.1f} mV, ~{:.2f} lbf per 12-bit LSB", load_cell.nominal_span_mv(),
              load_cell.resolution_per_lsb(3100.0f / 4096.0f));

  // bring-up check: this should sit close to the REF voltage (1650 mV here).
  // If it is pinned near 0 or near the ADC ceiling, the in-amp is clipping and
  // no amount of software will fix it.
  logger.info("unloaded amplifier output: {:.1f} mV", load_cell.get_amplifier_mv());

  // zero against the assembled but unloaded fixture
  load_cell.tare(64);

  // one-time span check against a known weight, then persist to NVS:
  //   float correction = load_cell.calibrate(500.0f);

  // print a header, then CSV rows so the output can be pasted into a plot
  logger.info("% time(s), amp(mV), bridge(mV), mV/V, lbf, N");

  // Create task for reading load cell
  auto task_fn = [](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - start).count();

    // one set of ADC samples, reported at every stage of the chain
    auto r = load_cell.read();
    fmt::print("{:.3f}, {:.1f}, {:+.4f}, {:+.4f}, {:+.2f}, {:+.1f}\n", elapsed, r.amplifier_mv,
               r.bridge_mv, r.mv_per_v, r.load, r.load * 4.4482216f);
    if (r.overloaded) {
      logger.warn("stop loading, past 150% of rating");
    }

    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 100ms);
    return false; // keep running
  };
  // Run task
  static auto task = espp::Task({.callback = task_fn,
                                 .task_config = {.name = "load cell", .stack_size_bytes = 6 * 1024},
                                 .log_level = espp::Logger::Verbosity::WARN});
  task.start();
}
