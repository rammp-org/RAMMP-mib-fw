#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "butterworth_filter.hpp"
#include "i2c.hpp"
#include "mt6701.hpp"
#include "spi.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  fmt::print("Starting mt6701 example, rotate to -720 degrees to quit!\n");

  {
    fmt::print("Running SPI example\n");
    //! [mt6701 ssi example]
    std::atomic<bool> quit_test = false;

    // make the SPI bus and SSI device that we'll use to communicate
    espp::Spi spi({
        .host = SPI2_HOST,
        .sclk_io_num = GPIO_NUM_4,
        .mosi_io_num = GPIO_NUM_NC,
        .miso_io_num = GPIO_NUM_5,
        .max_transfer_sz = 32,
        .log_level = espp::Logger::Verbosity::WARN,
    });
    std::error_code ec;

    auto encoder_spi_device = spi.add_device(
        {
            .mode = 0,
            .clock_speed_hz = 1000000,
            .input_delay_ns = 0,
            .cs_io_num = GPIO_NUM_48, // FOR TESTING CHANGE THIS PIN ONLY
            .queue_size = 1,
        },
        ec);

    if (ec || !encoder_spi_device) {
      fmt::print("Failed to initialize SPI device: {}\n", ec.message());
      return;
    }

    // make the velocity filter
    static constexpr float filter_cutoff_hz = 10.0f;
    static constexpr float encoder_update_period = 0.001f; // seconds
    espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period});
    auto filter_fn = [&filter](float raw) -> float { return filter.update(raw); };

    // now make the mt6701 which decodes the data
    using Mt6701 = espp::Mt6701<espp::Mt6701Interface::SSI>;
    Mt6701 mt6701({.read = [&](uint8_t *data, size_t len) -> bool {
                     std::error_code read_ec;
                     return encoder_spi_device->read(std::span<uint8_t>(data, len), {}, read_ec);
                   },
                   .velocity_filter = filter_fn,
                   .update_period = std::chrono::duration<float>(encoder_update_period),
                   .run_task = true, // run a task which calls the update function at the update
                                     // period
                   .log_level = espp::Logger::Verbosity::WARN});

    // get the initial state
    auto field_strength = mt6701.get_magnetic_field_strength();
    auto tracking_status = mt6701.get_tracking_status();
    bool push_button = mt6701.get_push_button();
    fmt::print("Initial state: field_strength: {}, tracking_status: {}, push_button: {}\n",
               field_strength, tracking_status, push_button);

    // and finally, make the task to periodically poll the mt6701 and print the
    // state. NOTE: the Mt6701 runs its own task to maintain state, so we're
    // just polling the current state.
    auto task_fn = [&quit_test, &mt6701](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      auto count = mt6701.get_count();
      auto radians = mt6701.get_radians();
      auto degrees = mt6701.get_degrees();
      auto rpm = mt6701.get_rpm();
      fmt::print("{:.3f}, {}, {:.3f}, {:.3f}, {:.3f}\n", seconds, count, radians, degrees, rpm);
      quit_test = degrees <= -720.0f;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 10ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Mt6701 Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    fmt::print("% time(s), count, radians, degrees, rpm\n");
    task.start();
    //! [mt6701 ssi example]

    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Mt6701 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}