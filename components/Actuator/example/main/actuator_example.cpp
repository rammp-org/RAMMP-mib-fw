/// Actuator example and bench console.
///
/// Brings up one mib::Actuator on one channel of one MCP266 and hands you a
/// console over the serial monitor to exercise its API:
///
///   status                position, drive state, target reached, online, range
///   pos                   read the position
///   abs <counts>          move to an absolute count
///   rel <delta>           move by a signed number of counts
///   inc / dec             move one jog step
///   wait                  block until the drive reports target reached
///   stop                  quick stop
///   cal on|off            lift or restore the range clamp for calibration
///   range <min> <max>     install a calibrated range and save it to NVS
///   forget                erase the saved range; the compiled default applies
///   selftest              inc, wait, dec, wait; passes if it returns to start
///
/// The settings below would come from a configuration file in the real
/// firmware; they are written out here so the example stands on its own.

#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "actuator.hpp"
#include "actuator_range_store.hpp"
#include "canopen_client.hpp"
#include "cli.hpp"
#include "format.hpp"
#include "logger.hpp"
#include "mcp266.hpp"
#include "twai.hpp"

using namespace std::chrono_literals;

namespace {

// ---- settings ---------------------------------------------------------------
// CAN transceiver on the TWAI peripheral.
constexpr int kTxGpio = 17;
constexpr int kRxGpio = 16;
constexpr uint32_t kBitrate = 1'000'000;
// Which controller and channel the actuator is on.
constexpr uint8_t kNodeId = 10;
constexpr auto kAxis = mib::Actuator::Axis::M1;
// How the actuator behaves. Positions are the joint encoder's counts.
constexpr mib::Actuator::Config kActuatorConfig{
    .name = "example",              // also the NVS key prefix for the saved range
    .axis = kAxis,
    .range = {0, 4095},             // calibrated travel; a range saved to NVS overrides it
    .profile = {500, 500, 500},     // counts/s, counts/s^2, counts/s^2
    .jog_step = 50,                 // counts per inc / dec
    .tolerance = 10,                // counts within which selftest counts as returned
    .hardware_limits = false,       // no limit switches wired to this channel
};
// -----------------------------------------------------------------------------

espp::Logger logger({.tag = "Actuator Example", .level = espp::Logger::Verbosity::INFO});

// The transport and controller objects the actuator is layered on. The client
// filters frames by node id and the controller has one SDO channel, so an
// actuator on M2 of the same controller would share `mcp` and `mcp_mutex`.
std::unique_ptr<espp::Twai> twai;
std::unique_ptr<espp::CanopenClient> client;
std::unique_ptr<espp::Mcp266> mcp;
std::mutex mcp_mutex;
std::unique_ptr<mib::Actuator> actuator;
mib::ActuatorRangeStore range_store;

bool bring_up() {
  //! [actuator example bring-up]
  client = std::make_unique<espp::CanopenClient>(espp::CanopenClient::Config{
      .node_id = kNodeId,
      .send =
          [](const espp::CanopenClient::CanFrame &frame) {
            espp::Twai::Message message{.id = frame.id,
                                        .extended = frame.extended,
                                        .rtr = frame.rtr,
                                        .dlc = frame.dlc,
                                        .data = frame.data};
            std::error_code ec;
            return twai && twai->transmit(message, ec);
          },
      .sdo_timeout = 100ms,
      .log_level = espp::Logger::Verbosity::WARN,
  });
  twai = std::make_unique<espp::Twai>(espp::Twai::Config{
      .tx_gpio = kTxGpio,
      .rx_gpio = kRxGpio,
      .baudrate = kBitrate,
      .mode = espp::Twai::Mode::NORMAL,
      .tx_queue_depth = 8,
      .tx_retry_count = 3,
      .on_receive =
          [](const espp::Twai::Message &message) {
            client->process_frame({.id = message.id,
                                   .extended = message.extended,
                                   .rtr = message.rtr,
                                   .dlc = message.dlc,
                                   .data = message.data});
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  std::error_code ec;
  if (!twai->initialize(ec)) {
    logger.error("TWAI init failed on tx={} rx={}: {}", kTxGpio, kRxGpio, ec.message());
    return false;
  }
  logger.info("CAN up on tx={} rx={} at {} bit/s", kTxGpio, kRxGpio, kBitrate);

  mcp = std::make_unique<espp::Mcp266>(*client,
                                       espp::Mcp266::Config{.log_level = espp::Logger::Verbosity::INFO});
  if (!mcp->start(ec)) {
    logger.error("MCP266 node {} did not answer: {}", kNodeId, ec.message());
    // Carry on: the actuator marks itself offline and retries on its next use.
  }

  // A range saved by an earlier `range` command wins over the compiled default.
  auto config = kActuatorConfig;
  range_store.init();
  if (mib::Actuator::Range saved{}; range_store.load(config.name, saved)) {
    logger.info("using calibrated range [{}, {}] from NVS", saved.min, saved.max);
    config.range = saved;
  }

  actuator = std::make_unique<mib::Actuator>(*mcp, mcp_mutex, config);
  logger.info("initialize: {}", actuator->initialize() ? "ready" : "NOT ready (see log)");
  //! [actuator example bring-up]
  return true;
}

const char *state_name(mib::Actuator::DriveState state) {
  using S = mib::Actuator::DriveState;
  switch (state) {
  case S::NotReadyToSwitchOn: return "not ready to switch on";
  case S::SwitchOnDisabled: return "switch on disabled";
  case S::ReadyToSwitchOn: return "ready to switch on";
  case S::SwitchedOn: return "switched on";
  case S::OperationEnabled: return "operation enabled";
  case S::QuickStopActive: return "quick stop active";
  case S::FaultReactionActive: return "fault reaction active";
  case S::Fault: return "FAULT";
  default: return "unknown";
  }
}

void print_position(std::ostream &out) {
  int32_t counts = 0;
  if (actuator->get_position(counts)) {
    const auto r = actuator->range();
    out << fmt::format("{} counts (range [{}, {}])\n", counts, r.min, r.max);
  } else {
    out << "read failed (see log)\n";
  }
}

/// Poll until the drive reports target reached. Prints progress once a second.
bool wait_for_target(std::ostream &out, std::chrono::seconds timeout = 30s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto next_print = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline) {
    bool reached = false;
    if (!actuator->is_target_reached(reached)) {
      out << "status read failed (see log)\n";
      return false;
    }
    int32_t counts = 0;
    actuator->get_position(counts);
    if (reached) {
      out << fmt::format("target reached at {} counts\n", counts);
      return true;
    }
    if (std::chrono::steady_clock::now() >= next_print) {
      out << fmt::format("moving, at {} counts\n", counts);
      next_print += 1s;
    }
    std::this_thread::sleep_for(100ms);
  }
  out << "timed out waiting for target\n";
  return false;
}

void report(std::ostream &out, bool ok, const char *what) {
  out << fmt::format("{} {}\n", what, ok ? "accepted" : "FAILED (see log)");
}

std::unique_ptr<cli::Menu> build_menu() {
  auto menu = std::make_unique<cli::Menu>("actuator");

  menu->Insert(
      "status",
      [](std::ostream &out) {
        mib::Actuator::DriveState state{};
        bool reached = false;
        const bool have_state = actuator->get_drive_state(state);
        const bool have_reached = have_state && actuator->is_target_reached(reached);
        const auto r = actuator->range();
        out << fmt::format("{}: {}, drive {}, target {}, range [{}, {}], jog {}{}\n",
                           actuator->name(), actuator->online() ? "online" : "OFFLINE",
                           have_state ? state_name(state) : "unreadable",
                           have_reached ? (reached ? "reached" : "not reached") : "?", r.min,
                           r.max, actuator->config().jog_step,
                           actuator->calibration_mode() ? ", CALIBRATING (clamp lifted)" : "");
        print_position(out);
      },
      "Position, drive state, target reached, online, range");

  menu->Insert("pos", [](std::ostream &out) { print_position(out); }, "Read the position");
  menu->Insert(
      "abs", [](std::ostream &out, int counts) { report(out, actuator->move_absolute(counts), "move"); },
      "Move to an absolute count: abs <counts>");
  menu->Insert(
      "rel", [](std::ostream &out, int delta) { report(out, actuator->move_relative(delta), "move"); },
      "Move by a signed count: rel <delta>");
  menu->Insert("inc", [](std::ostream &out) { report(out, actuator->increment(), "increment"); },
               "Move one jog step up");
  menu->Insert("dec", [](std::ostream &out) { report(out, actuator->decrement(), "decrement"); },
               "Move one jog step down");
  menu->Insert("wait", [](std::ostream &out) { wait_for_target(out); },
               "Block until the drive reports target reached");
  menu->Insert("stop", [](std::ostream &out) { report(out, actuator->stop(), "stop"); },
               "Quick stop");

  menu->Insert(
      "cal",
      [](std::ostream &out, std::string mode) {
        const bool on = mode == "on" || mode == "1";
        report(out, actuator->set_calibration_mode(on),
               on ? "calibration mode ON (clamp lifted, mind the mechanical ends)"
                  : "calibration mode off");
      },
      "Lift or restore the range clamp: cal on|off");
  menu->Insert(
      "range",
      [](std::ostream &out, int min, int max) {
        if (!range_store.save(actuator->name(), {min, max})) {
          out << "NVS save failed (see log)\n";
        }
        report(out, actuator->set_range({min, max}), "range install");
      },
      "Install a calibrated range and save it to NVS: range <min> <max>");
  menu->Insert(
      "forget",
      [](std::ostream &out) {
        out << (range_store.erase(actuator->name())
                    ? "saved range erased; the compiled default applies after reboot\n"
                    : "erase failed (see log)\n");
      },
      "Erase the saved range");

  menu->Insert(
      "selftest",
      [](std::ostream &out) {
        int32_t start = 0;
        if (!actuator->get_position(start)) {
          out << "FAIL: cannot read position (see log)\n";
          return;
        }
        out << fmt::format("start at {} counts, jog step {}\n", start, actuator->config().jog_step);
        if (!actuator->increment() || !wait_for_target(out)) {
          out << "FAIL: increment (see log)\n";
          return;
        }
        int32_t up = 0;
        actuator->get_position(up);
        if (!actuator->decrement() || !wait_for_target(out)) {
          out << "FAIL: decrement (see log)\n";
          return;
        }
        int32_t back = 0;
        actuator->get_position(back);
        const int32_t tolerance = actuator->config().tolerance;
        const bool moved_up = (up - start) > tolerance;
        const bool returned = std::abs(back - start) <= tolerance;
        out << fmt::format("{}: start {} -> up {} -> back {} (tolerance {})\n",
                           moved_up && returned ? "PASS" : "FAIL", start, up, back, tolerance);
        if (!moved_up) {
          out << "  increment did not raise the count: check direction and the position loop gains\n";
        }
        if (!returned) {
          out << "  did not return to start: check the position loop gains and deadzone\n";
        }
      },
      "Move one jog step up and back; passes if the joint returns");

  return menu;
}

} // namespace

extern "C" void app_main(void) {
  logger.info("Actuator example: node {} {}", kNodeId, kAxis == mib::Actuator::Axis::M1 ? "M1" : "M2");
  if (!bring_up()) {
    return;
  }

  cli::Cli cli(build_menu());
  cli.ExitAction([](auto &out) { out << "bye\n"; });
  espp::Cli input(cli);
  input.SetInputHistorySize(20);
  fmt::print("Type 'help' for the commands.\n");
  input.Start();

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
