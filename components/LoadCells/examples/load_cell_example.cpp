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
//   INA118 Vout     -> 1k series + Schottky clamps -> ADC1_CH4, which is
//                      GPIO20 on the ESP32-P4 (ADC1_CH0..CH7 = GPIO16..GPIO23)
//                      and GPIO34 on the original ESP32
//   Excitation      -> 10.0 Vdc from its own regula

#include <chrono>
#include <optional>
#include <vector>

#include "load_cell_lchd.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

//====================================================================================================
namespace {

constexpr adc_channel_t SIGNAL_CHANNEL = ADC_CHANNEL_4;
constexpr int SIGNAL_GPIO = 16 + static_cast<int>(SIGNAL_CHANNEL); // GPIO20
static_assert(SIGNAL_GPIO == 20, "channel and documented GPIO disagree");
 
std::vector<espp::AdcConfig> channels{
    {.unit = ADC_UNIT_1, .channel = SIGNAL_CHANNEL, .attenuation = ADC_ATTEN_DB_12}};
espp::OneshotAdc adc({.unit = ADC_UNIT_1, .channels = channels});
 
espp::Logger logger({.tag = "load cell example", .level = espp::Logger::Verbosity::INFO});
} 
//====================================================================================================
extern "C" void app_main(void) {
  // FIX 1 and 2: capture the AdcConfig BY VALUE, and propagate a failed read
  // as std::nullopt instead of 0 mV. read_mv() resolves the channel by unit
  // and channel number, so a copy is equivalent to the vector's element.
  auto ch = channels[0];
  auto read_mv = [ch]() -> std::optional<float> {
    auto maybe_mv = adc.read_mv(ch);
    if (!maybe_mv.has_value()) {
      return std::nullopt;
    }
    return static_cast<float>(maybe_mv.value());
  };
 
  static espp::LoadCell load_cell({
      .name = "load_cell",
      .capacity = 1000.0f,
      .rated_output_mv_per_v = 3.0000f, // from this unit's calibration sheet
      .excitation_mv = 10000.0f,
      .amplifier_gain = espp::LoadCell::gain_from_rg(1280.0f),
      .amp_offset_mv = 1650.0f,
      .read_mv = read_mv,
      .num_samples = 16,
      .filter_alpha = 0.15f,
      .log_level = espp::Logger::Verbosity::INFO,
  });
 
  auto unloaded = load_cell.get_amplifier_mv();
  if (!unloaded.has_value()) {
    logger.error("no reading from GPIO{}; check the channel is registered with the unit",
                 SIGNAL_GPIO);
    return;
  }
  logger.info("GPIO{} unloaded: {:.1f} mV, span {:.1f} mV", SIGNAL_GPIO, unloaded.value(),
              load_cell.nominal_span_mv());
 
  if (!load_cell.tare(64).has_value()) {
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
  //Run task
  static auto task = espp::Task({.callback = task_fn,
                                 .task_config = {.name = "load cell", .stack_size_bytes = 6 * 1024},
                                 .log_level = espp::Logger::Verbosity::WARN});
  task.start();
}
//====================================================================================================

