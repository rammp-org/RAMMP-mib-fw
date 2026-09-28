/// Actuator example and bench console.
///
/// Brings up the CAN bus to the MCP266s exactly the way the MIB BSP does (one
/// TWAI node, one CANopen client and Mcp266 per controller in MIBconfig.hpp),
/// creates an Actuator per leg, and then hands you a console over the serial
/// monitor to exercise them one command at a time:
///
///   legs                  list every leg with its controller, range and state
///   init                  (re)initialize every actuator
///   pos [leg]             read the joint position, or all of them
///   abs <leg> <counts>    move to an absolute count
///   rel <leg> <delta>     move by a signed number of counts
///   inc <leg> / dec <leg> move one jog step
///   wait <leg>            block until the drive reports target reached
///   stop [leg]            quick-stop one leg, or all
///   state <leg>           drive state, target reached, online, range
///   cal <leg> on|off      lift or restore the range clamp for calibration
///   range <leg> <min> <max>   install a calibrated range and save it to NVS
///   forget <leg>          erase the saved range; the compiled default applies
///   selftest <leg>        inc, wait, dec, wait; passes if it returns to start
///
/// Legs are named as in MIBconfig.hpp (FC, RC, ML, MR, LCarr, RCarr, any case)
/// or given as their index 0..5.
///
/// This is the first thing to run on new hardware: `legs` proves which
/// controllers answer at all, `pos` proves the encoder wiring, `inc` and `dec`
/// prove the direction, and `selftest` proves the position loop.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "MIBconfig.hpp"
#include "actuator.hpp"
#include "actuator_range_store.hpp"
#include "canopen_client.hpp"
#include "cli.hpp"
#include "format.hpp"
#include "mcp266.hpp"
#include "twai.hpp"

using namespace std::chrono_literals;

namespace {

/// One MCP266 on the bus, as the BSP keeps it: client, controller and the SDO mutex
/// its two axes share.
struct Controller {
  uint8_t node_id;
  std::unique_ptr<espp::CanopenClient> client;
  std::unique_ptr<espp::Mcp266> mcp;
  std::mutex mutex;
};

std::unique_ptr<espp::Twai> twai;
std::vector<std::unique_ptr<Controller>> controllers;
std::vector<std::unique_ptr<mib::Actuator>> actuators; // in mib::config::Leg order
mib::ActuatorRangeStore range_store;

Controller &controller_for(uint8_t node_id) {
  for (auto &c : controllers) {
    if (c->node_id == node_id) {
      return *c;
    }
  }
  auto c = std::make_unique<Controller>();
  c->node_id = node_id;
  c->client = std::make_unique<espp::CanopenClient>(espp::CanopenClient::Config{
      .node_id = node_id,
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
  c->mcp = std::make_unique<espp::Mcp266>(*c->client,
                                          espp::Mcp266::Config{.log_level = espp::Logger::Verbosity::INFO});
  controllers.push_back(std::move(c));
  return *controllers.back();
}

bool bring_up_bus() {
  twai = std::make_unique<espp::Twai>(espp::Twai::Config{
      .tx_gpio = mib::config::can_tx_gpio,
      .rx_gpio = mib::config::can_rx_gpio,
      .baudrate = mib::config::can_bitrate,
      .mode = espp::Twai::Mode::NORMAL,
      .tx_queue_depth = 8,
      .tx_retry_count = 3,
      .on_receive =
          [](const espp::Twai::Message &message) {
            const espp::CanopenClient::CanFrame frame{.id = message.id,
                                                      .extended = message.extended,
                                                      .rtr = message.rtr,
                                                      .dlc = message.dlc,
                                                      .data = message.data};
            for (auto &c : controllers) {
              c->client->process_frame(frame);
            }
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  std::error_code ec;
  if (!twai->initialize(ec)) {
    fmt::print("TWAI init failed on tx={} rx={}: {}\n", mib::config::can_tx_gpio,
               mib::config::can_rx_gpio, ec.message());
    return false;
  }
  fmt::print("CAN up on tx={} rx={} at {} bit/s\n", mib::config::can_tx_gpio,
             mib::config::can_rx_gpio, mib::config::can_bitrate);
  return true;
}

void build_actuators() {
  for (const auto &row : mib::config::actuators) {
    controller_for(row.node_id);
  }
  for (const auto &row : mib::config::actuators) {
    auto &c = controller_for(row.node_id);
    mib::Actuator::Config config{
        .name = row.name,
        .axis = row.channel == 0 ? mib::Actuator::Axis::M1 : mib::Actuator::Axis::M2,
        .range = {row.min_counts, row.max_counts},
        .profile = {row.velocity, row.acceleration, row.deceleration},
        .jog_step = row.jog_step,
        .tolerance = row.tolerance,
        .hardware_limits = row.hardware_limits,
    };
    if (mib::Actuator::Range saved{}; range_store.load(row.name, saved)) {
      fmt::print("{}: calibrated range [{}, {}] from NVS\n", row.name, saved.min, saved.max);
      config.range = saved;
    }
    actuators.push_back(std::make_unique<mib::Actuator>(*c.mcp, c.mutex, config));
  }
}

void initialize_all(std::ostream &out) {
  std::error_code ec;
  for (auto &c : controllers) {
    std::lock_guard<std::mutex> lock(c->mutex);
    if (c->mcp->start(ec)) {
      out << fmt::format("node {}: answering\n", c->node_id);
    } else {
      out << fmt::format("node {}: no answer ({})\n", c->node_id, ec.message());
    }
  }
  for (auto &a : actuators) {
    out << fmt::format("{}: {}\n", a->name(), a->initialize() ? "ready" : "NOT ready (see log)");
  }
}

/// Find a leg by name (any case) or index. Returns nullptr and prints on failure.
mib::Actuator *leg(std::ostream &out, const std::string &which) {
  std::string wanted = which;
  std::transform(wanted.begin(), wanted.end(), wanted.begin(),
                 [](unsigned char ch) { return std::tolower(ch); });
  for (size_t i = 0; i < actuators.size(); ++i) {
    std::string name = actuators[i]->name();
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char ch) { return std::tolower(ch); });
    if (name == wanted || std::to_string(i) == wanted) {
      return actuators[i].get();
    }
  }
  out << "no such leg '" << which << "'; use one of";
  for (auto &a : actuators) {
    out << ' ' << a->name();
  }
  out << '\n';
  return nullptr;
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

void print_position(std::ostream &out, mib::Actuator &a) {
  int32_t counts = 0;
  if (a.get_position(counts)) {
    const auto r = a.range();
    out << fmt::format("{}: {} counts (range [{}, {}])\n", a.name(), counts, r.min, r.max);
  } else {
    out << fmt::format("{}: read failed (see log)\n", a.name());
  }
}

/// Poll until the drive reports target reached. Prints progress once a second.
bool wait_for_target(std::ostream &out, mib::Actuator &a, std::chrono::seconds timeout = 30s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto next_print = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline) {
    bool reached = false;
    if (!a.is_target_reached(reached)) {
      out << fmt::format("{}: status read failed (see log)\n", a.name());
      return false;
    }
    int32_t counts = 0;
    a.get_position(counts);
    if (reached) {
      out << fmt::format("{}: target reached at {} counts\n", a.name(), counts);
      return true;
    }
    if (std::chrono::steady_clock::now() >= next_print) {
      out << fmt::format("{}: moving, at {} counts\n", a.name(), counts);
      next_print += 1s;
    }
    std::this_thread::sleep_for(100ms);
  }
  out << fmt::format("{}: timed out waiting for target\n", a.name());
  return false;
}

void report(std::ostream &out, mib::Actuator &a, bool ok, const char *what) {
  out << fmt::format("{}: {} {}\n", a.name(), what, ok ? "accepted" : "FAILED (see log)");
}

std::unique_ptr<cli::Menu> build_menu() {
  auto menu = std::make_unique<cli::Menu>("actuator");

  menu->Insert(
      "legs",
      [](std::ostream &out) {
        for (size_t i = 0; i < actuators.size(); ++i) {
          auto &a = *actuators[i];
          const auto &row = mib::config::actuators[i];
          const auto r = a.range();
          out << fmt::format("{} {:<6} node {:>3} {}  range [{}, {}]  jog {}  {}{}{}",
                             i, a.name(), row.node_id, row.channel == 0 ? "M1" : "M2", r.min,
                             r.max, row.jog_step, a.online() ? "online" : "OFFLINE",
                             a.initialized() ? ", ready" : ", not initialized",
                             a.calibration_mode() ? ", CALIBRATING" : "");
          if (auto p = a.last_position()) {
            out << fmt::format("  last {}", *p);
          }
          out << (row.hardware_limits ? "  limit switches\n" : "\n");
        }
      },
      "List every leg with its controller, range and state");

  menu->Insert("init", [](std::ostream &out) { initialize_all(out); },
               "(Re)initialize every controller and actuator");

  menu->Insert(
      "pos",
      [](std::ostream &out) {
        for (auto &a : actuators) {
          print_position(out, *a);
        }
      },
      "Read every joint position");
  menu->Insert(
      "pos",
      [](std::ostream &out, std::string which) {
        if (auto *a = leg(out, which)) {
          print_position(out, *a);
        }
      },
      "Read one joint position: pos <leg>");

  menu->Insert(
      "abs",
      [](std::ostream &out, std::string which, int counts) {
        if (auto *a = leg(out, which)) {
          report(out, *a, a->move_absolute(counts), "move");
        }
      },
      "Move to an absolute count: abs <leg> <counts>");
  menu->Insert(
      "rel",
      [](std::ostream &out, std::string which, int delta) {
        if (auto *a = leg(out, which)) {
          report(out, *a, a->move_relative(delta), "move");
        }
      },
      "Move by a signed count: rel <leg> <delta>");
  menu->Insert(
      "inc",
      [](std::ostream &out, std::string which) {
        if (auto *a = leg(out, which)) {
          report(out, *a, a->increment(), "increment");
        }
      },
      "Move one jog step up: inc <leg>");
  menu->Insert(
      "dec",
      [](std::ostream &out, std::string which) {
        if (auto *a = leg(out, which)) {
          report(out, *a, a->decrement(), "decrement");
        }
      },
      "Move one jog step down: dec <leg>");
  menu->Insert(
      "wait",
      [](std::ostream &out, std::string which) {
        if (auto *a = leg(out, which)) {
          wait_for_target(out, *a);
        }
      },
      "Block until the drive reports target reached: wait <leg>");

  menu->Insert(
      "stop",
      [](std::ostream &out) {
        for (auto &a : actuators) {
          report(out, *a, a->stop(), "stop");
        }
      },
      "Quick-stop every leg");
  menu->Insert(
      "stop",
      [](std::ostream &out, std::string which) {
        if (auto *a = leg(out, which)) {
          report(out, *a, a->stop(), "stop");
        }
      },
      "Quick-stop one leg: stop <leg>");

  menu->Insert(
      "state",
      [](std::ostream &out, std::string which) {
        auto *a = leg(out, which);
        if (!a) {
          return;
        }
        mib::Actuator::DriveState state{};
        bool reached = false;
        const bool have_state = a->get_drive_state(state);
        const bool have_reached = have_state && a->is_target_reached(reached);
        const auto r = a->range();
        out << fmt::format("{}: {}, drive {}, target {}, range [{}, {}]{}\n", a->name(),
                           a->online() ? "online" : "OFFLINE",
                           have_state ? state_name(state) : "unreadable",
                           have_reached ? (reached ? "reached" : "not reached") : "?", r.min,
                           r.max, a->calibration_mode() ? ", CALIBRATING (clamp lifted)" : "");
      },
      "Drive state and range of one leg: state <leg>");

  menu->Insert(
      "cal",
      [](std::ostream &out, std::string which, std::string mode) {
        auto *a = leg(out, which);
        if (!a) {
          return;
        }
        const bool on = mode == "on" || mode == "1";
        report(out, *a, a->set_calibration_mode(on),
               on ? "calibration mode ON (clamp lifted, mind the mechanical ends)"
                  : "calibration mode off");
      },
      "Lift or restore the range clamp: cal <leg> on|off");
  menu->Insert(
      "range",
      [](std::ostream &out, std::string which, int min, int max) {
        auto *a = leg(out, which);
        if (!a) {
          return;
        }
        if (!range_store.save(a->name(), {min, max})) {
          out << fmt::format("{}: NVS save failed (see log)\n", a->name());
        }
        report(out, *a, a->set_range({min, max}), "range install");
      },
      "Install a calibrated range and save it to NVS: range <leg> <min> <max>");
  menu->Insert(
      "forget",
      [](std::ostream &out, std::string which) {
        auto *a = leg(out, which);
        if (!a) {
          return;
        }
        if (range_store.erase(a->name())) {
          out << fmt::format("{}: saved range erased; compiled default applies after reboot\n",
                             a->name());
        } else {
          out << fmt::format("{}: erase failed (see log)\n", a->name());
        }
      },
      "Erase the saved range for a leg: forget <leg>");

  menu->Insert(
      "selftest",
      [](std::ostream &out, std::string which) {
        auto *a = leg(out, which);
        if (!a) {
          return;
        }
        int32_t start = 0;
        if (!a->get_position(start)) {
          out << fmt::format("FAIL {}: cannot read position (see log)\n", a->name());
          return;
        }
        out << fmt::format("{}: start at {} counts, jog step {}\n", a->name(), start,
                           a->config().jog_step);
        if (!a->increment() || !wait_for_target(out, *a)) {
          out << fmt::format("FAIL {}: increment (see log)\n", a->name());
          return;
        }
        int32_t up = 0;
        a->get_position(up);
        if (!a->decrement() || !wait_for_target(out, *a)) {
          out << fmt::format("FAIL {}: decrement (see log)\n", a->name());
          return;
        }
        int32_t back = 0;
        a->get_position(back);
        const int32_t tolerance = a->config().tolerance;
        const bool moved_up = (up - start) > tolerance;
        const bool returned = std::abs(back - start) <= tolerance;
        out << fmt::format("{} {}: start {} -> up {} -> back {} (tolerance {})\n",
                           moved_up && returned ? "PASS" : "FAIL", a->name(), start, up, back,
                           tolerance);
        if (!moved_up) {
          out << "  increment did not raise the count: check direction and the position loop gains\n";
        }
        if (!returned) {
          out << "  did not return to start: check the position loop gains and deadzone\n";
        }
      },
      "Move one jog step up and back; passes if the joint returns: selftest <leg>");

  return menu;
}

} // namespace

extern "C" void app_main(void) {
  fmt::print("Actuator example: {} legs on {} controllers\n", mib::config::leg_count,
             std::size(mib::config::actuators));

  range_store.init(); // logs itself if NVS is unavailable
  if (!bring_up_bus()) {
    return;
  }
  build_actuators();
  initialize_all(std::cout);

  cli::Cli cli(build_menu());
  cli.ExitAction([](auto &out) { out << "bye\n"; });
  espp::Cli input(cli);
  input.SetInputHistorySize(20);
  fmt::print("Type 'help' for the commands. Legs: FC RC ML MR LCarr RCarr.\n");
  input.Start();

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
