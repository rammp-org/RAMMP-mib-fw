/// Actuator example: the bench console for the leg actuators.
///
/// Brings the legs up exactly as the firmware does, through the BSP: TWAI retransmitting until
/// acknowledged, the MIB's CANopen heartbeat, and one mib::Actuator per leg in MIBconfig.hpp,
/// each commissioned leg's controller settings written and read back. A leg that is not
/// commissioned (the left main leg, the carriages) gets nothing written and refuses moves.
/// Nothing moves or homes until a command asks. The selected leg is the right main leg (MR)
/// until `leg` changes it; every command acts on the selected leg.
///
///   legs                     the legs: node, channel, commissioned, travel, stroke sign
///   leg <name>               select a leg (MR, ML, LCarr, RCarr)
///   settings                 configured vs read-back controller settings
///   status                   count, PWM, current, statusword, drive state, fault, last move
///   result                   the last move's result
///   abs <counts> [speed]     move to a count (speed in counts/s; default the leg's)
///   rel <delta> [speed]      move by a signed count
///   inc / dec                one jog step
///   stop                     disable voltage now, then PWM and current
///   clear                    CiA 402 fault reset of the selected channel
///   wait                     print the count until the move ends (any key ends the wait,
///                            not the move: type `stop` for that)
///   init                     rerun initialize() (settings written and read back)
///
///   scan                     node ids that answer an SDO (reads only)
///   sdor <node> <index> <sub> <bytes>          raw SDO read (index decimal or 0x hex)
///   sdow <node> <index> <sub> <bytes> <value>  raw SDO write; refused on a node with no
///                                              commissioned leg (node 2)
///   flags                    the controller's status flags (Basicmicro command 90)
///
///   Calibration (absolute encoder, once per installation):
///   limits / cal on|off / range <min> <max> / forget
///   Homing (incremental encoder; never automatic): home / restore / save / setenc <counts>
///
/// Moves return at once: the poll task streams them, and the Actuator logs how each ended
/// (arrived or FAILED, final count, peak speed, then the PWM and current after the release).
/// So `stop` can be typed at any time during a move; no keystroke is needed to end one.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "MIBconfig.hpp"
#include "actuator.hpp"
#include "bsp.hpp"
#include "cli.hpp"
#include "format.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;
using mib::Actuator;
using mib::config::Leg;

namespace {

espp::Logger logger({.tag = "Actuator Example", .level = espp::Logger::Verbosity::INFO});

std::atomic<Leg> selected{Leg::MAIN_RIGHT};

mib::bsp::MIB &board() { return mib::bsp::MIB::instance(); }
const mib::config::ActuatorConfig &row() { return mib::config::actuator_config(selected.load()); }
Actuator &actuator() { return board().actuator(selected.load()); }

/// True if any byte is waiting on the console, consuming it. Ends `wait`.
bool key_pressed() {
  const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  char discard[16];
  const bool pressed = read(STDIN_FILENO, discard, sizeof(discard)) > 0;
  fcntl(STDIN_FILENO, F_SETFL, flags);
  return pressed;
}

float percent(int16_t pwm) { return pwm * 100.0f / 32767.0f; }

const char *sign_text(int8_t sign) {
  return sign > 0 ? "+counts = stroke extends"
                  : (sign < 0 ? "+counts = stroke retracts" : "stroke sign not established");
}

void print_legs(std::ostream &out) {
  for (const auto &r : mib::config::actuators) {
    out << fmt::format("  {:<6} node {} M{}  {:<17} travel [{}, {}]  {}{}\n", r.name, r.node_id,
                       r.channel + 1, r.commissioned ? "commissioned" : "NOT commissioned",
                       r.min_counts, r.max_counts, sign_text(r.stroke_sign),
                       r.leg == selected.load() ? "   <- selected" : "");
  }
}

/// A move command on a leg that is not commissioned: say so before the Actuator refuses it.
bool refuse_uncommissioned(std::ostream &out) {
  if (row().commissioned) {
    return false;
  }
  out << fmt::format("REFUSED: {} (node {} M{}) is not commissioned (MIBconfig.hpp "
                     "commissioned = false). The MIB writes nothing to it and will not move it.\n",
                     row().name, row().node_id, row().channel + 1);
  return true;
}

void print_settings(std::ostream &out) {
  Actuator &a = actuator();
  Actuator::SettingsReadback rb{};
  if (!a.read_controller_settings(rb)) {
    out << "settings read failed (see log)\n";
    return;
  }
  const auto &c = a.controller_settings();
  const auto &r = rb.settings;
  const auto range = a.range();
  int mismatches = 0;
  out << fmt::format("{} (node {} M{}, {}): configured / read back\n", row().name, row().node_id,
                     row().channel + 1, row().commissioned ? "commissioned" : "NOT commissioned");
  const auto line = [&](const char *what, int64_t configured, int64_t read) {
    const bool same = configured == read;
    mismatches += same ? 0 : 1;
    out << fmt::format("  {:<28} {:>10} / {:<10}{}\n", what, configured, read,
                       same ? "" : "  <-- DIFFERS");
  };
  line("position window 0x6067", c.position_window, r.position_window);
  line("position P (x1024)", c.position_p, r.position_p);
  line("position I (x1024)", c.position_i, r.position_i);
  line("position D (x1024)", c.position_d, r.position_d);
  line("position MaxI", c.position_max_i, r.position_max_i);
  line("position deadband", c.position_deadband, r.position_deadband);
  line("velocity P (16.16)", c.velocity_p, r.velocity_p);
  line("velocity I (16.16)", c.velocity_i, r.velocity_i);
  line("velocity D (16.16)", c.velocity_d, r.velocity_d);
  line("velocity QPPS", c.velocity_qpps, r.velocity_qpps);
  line("clamp min", range.min, rb.clamp.min);
  line("clamp max", range.max, rb.clamp.max);
  line("software limit 0x607D min", range.min, rb.software_limit.min);
  line("software limit 0x607D max", range.max, rb.software_limit.max);
  line("profile velocity 0x6081", c.profile_velocity, r.profile_velocity);
  line("heartbeat consumer 0x1016:1", c.heartbeat_consumer, r.heartbeat_consumer);
  const auto &cfg = a.config();
  out << fmt::format(
      "  moves: {} counts/s, {} counts/s^2, lead {} ms, tolerance {}, settle {} ms\n",
      cfg.profile.velocity, cfg.profile.acceleration, cfg.lead.count(), cfg.tolerance,
      cfg.settle_time.count());
  if (!row().commissioned) {
    out << "  (not commissioned: these were never written by the MIB; the read-back is the "
           "controller's own)\n";
  } else if (a.calibration_mode()) {
    out << "  (calibration mode: the clamp is lifted on purpose)\n";
  }
  out << (mismatches == 0 ? "  all match\n"
                          : fmt::format("  {} field(s) differ\n", mismatches).c_str());
}

void print_result(std::ostream &out) {
  const auto r = actuator().move_result();
  if (r.outcome == Actuator::Outcome::NONE) {
    out << "no move yet\n";
    return;
  }
  out << fmt::format("move #{}: {}  {} -> {} (target), ", r.serial, mib::to_string(r.outcome),
                     r.start, r.target);
  if (r.outcome == Actuator::Outcome::MOVING) {
    out << fmt::format("{:.2f} s profile, peak so far {:.0f} counts/s\n", r.profile_s,
                       r.peak_speed);
    return;
  }
  out << fmt::format("ended at {} ({:+} counts from target), {:.2f} s, profile {:.2f} s, peak "
                     "{:.0f} counts/s, {} SDO failure(s) tolerated\n",
                     r.final_count, r.final_count - r.target, r.elapsed_s, r.profile_s,
                     r.peak_speed, r.sdo_failures);
  if (r.outcome == Actuator::Outcome::ARRIVED) {
    out << fmt::format("  released {:.2f} s after the profile ended", r.release_s);
  } else {
    out << fmt::format("  reason: {}", r.reason);
  }
  out << (r.release_readback ? fmt::format("; after release: PWM {:.1f} %, {} mA\n",
                                           percent(r.release_pwm), r.release_current_ma)
                             : std::string("; PWM/current not read back yet\n"));
}

void print_status(std::ostream &out) {
  Actuator &a = actuator();
  Actuator::Status s{};
  if (!a.read_status(s)) {
    out << "status read failed (see log)\n";
    return;
  }
  out << fmt::format("{}: count {}, PWM {:.1f} %, {} mA, statusword 0x{:04X} ({}){}{}\n", a.name(),
                     s.count, percent(s.pwm), s.current_ma, s.statusword, mib::to_string(s.state),
                     s.fault ? " FAULT (`clear` to reset)" : "",
                     (s.statusword & (1u << 11)) ? " [internal limit active]" : "");
  out << fmt::format(
      "  {}, {}, travel [{}, {}]{}\n", a.online() ? "online" : "OFFLINE",
      a.ready() ? "ready" : (a.commissioned() ? "NOT ready (see log)" : "not commissioned"),
      a.range().min, a.range().max, a.calibration_mode() ? ", CALIBRATING (clamp lifted)" : "");
  print_result(out);
}

void report(std::ostream &out, bool ok, const char *what) {
  out << fmt::format("{} {}\n", what, ok ? "accepted" : "FAILED (see log)");
}

Actuator::Profile profile_at(int speed) {
  const auto &cfg = actuator().config();
  return {static_cast<uint32_t>(speed), cfg.profile.acceleration};
}

bool valid_speed(std::ostream &out, int speed) {
  if (speed <= 0 || static_cast<uint32_t>(speed) > row().max_speed) {
    out << fmt::format("speed must be 1..{} counts/s (the leg's max_speed)\n", row().max_speed);
    return false;
  }
  return true;
}

/// Print the count every half second until the move ends; any key ends the wait.
void wait_for_move(std::ostream &out) {
  Actuator &a = actuator();
  auto next = std::chrono::steady_clock::now();
  while (a.moving()) {
    if (key_pressed()) {
      out << "wait ended (the move continues; `stop` to halt it)\n";
      return;
    }
    if (std::chrono::steady_clock::now() >= next) {
      if (const auto here = a.current_position()) {
        out << fmt::format("  at {}\n", *here);
      }
      next += 500ms;
    }
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(200ms); // let the release readback land
  print_result(out);
}

/// Raw SDO access to a node. Index may be given as 0x6086 or decimal.
int parse_index(const std::string &text) {
  return static_cast<int>(std::strtol(text.c_str(), nullptr, 0));
}

bool node_has_commissioned_leg(int node) {
  return std::any_of(std::begin(mib::config::actuators), std::end(mib::config::actuators),
                     [node](const auto &r) { return r.node_id == node && r.commissioned; });
}

std::unique_ptr<cli::Menu> build_menu() {
  auto menu = std::make_unique<cli::Menu>("actuator");

  menu->Insert(
      "legs", [](std::ostream &out) { print_legs(out); },
      "The legs: node, channel, commissioned, travel, stroke sign");
  menu->Insert(
      "leg",
      [](std::ostream &out, const std::string &name) {
        const auto it =
            std::find_if(std::begin(mib::config::actuators), std::end(mib::config::actuators),
                         [&name](const auto &r) { return name == r.name; });
        if (it == std::end(mib::config::actuators)) {
          out << "no such leg; `legs` lists them\n";
          return;
        }
        selected = it->leg;
        out << fmt::format("selected {} (node {} M{}, {})\n", it->name, it->node_id,
                           it->channel + 1,
                           it->commissioned ? "commissioned" : "NOT commissioned: moves refused");
      },
      "Select a leg: leg <MR|ML|LCarr|RCarr>");
  menu->Insert(
      "settings", [](std::ostream &out) { print_settings(out); },
      "Configured vs read-back controller settings");
  menu->Insert(
      "status", [](std::ostream &out) { print_status(out); },
      "Count, PWM, current, statusword, fault, last move");
  menu->Insert(
      "result", [](std::ostream &out) { print_result(out); }, "The last move's result");

  menu->Insert(
      "abs",
      [](std::ostream &out, int counts) {
        if (!refuse_uncommissioned(out)) {
          report(out, actuator().move_absolute(counts), "move");
        }
      },
      "Move to a count at the leg's speed: abs <counts>");
  menu->Insert(
      "abs",
      [](std::ostream &out, int counts, int speed) {
        if (!refuse_uncommissioned(out) && valid_speed(out, speed)) {
          report(out, actuator().move_absolute(counts, profile_at(speed)), "move");
        }
      },
      "Move to a count at a speed: abs <counts> <counts/s>");
  menu->Insert(
      "rel",
      [](std::ostream &out, int delta) {
        if (!refuse_uncommissioned(out)) {
          report(out, actuator().move_relative(delta), "move");
        }
      },
      "Move by a signed count: rel <delta>");
  menu->Insert(
      "rel",
      [](std::ostream &out, int delta, int speed) {
        if (!refuse_uncommissioned(out) && valid_speed(out, speed)) {
          report(out, actuator().move_relative(delta, profile_at(speed)), "move");
        }
      },
      "Move by a signed count at a speed: rel <delta> <counts/s>");
  menu->Insert(
      "inc",
      [](std::ostream &out) {
        if (!refuse_uncommissioned(out)) {
          report(out, actuator().increment(), "increment");
        }
      },
      "Move one jog step up");
  menu->Insert(
      "dec",
      [](std::ostream &out) {
        if (!refuse_uncommissioned(out)) {
          report(out, actuator().decrement(), "decrement");
        }
      },
      "Move one jog step down");
  menu->Insert(
      "stop",
      [](std::ostream &out) {
        if (!row().commissioned) {
          out << fmt::format("{} is not commissioned: the MIB never enabled it, nothing sent\n",
                             row().name);
          return;
        }
        const bool ok = actuator().stop();
        out << (ok ? "stop: disable voltage acknowledged\n"
                   : "stop: disable voltage NOT acknowledged (see log)\n");
        std::this_thread::sleep_for(150ms); // PWM falls to 0 within ~100 ms
        Actuator::Status s{};
        if (actuator().read_status(s)) {
          out << fmt::format(
              "after stop: count {}, PWM {:.1f} %, {} mA, statusword 0x{:04X} ({})\n", s.count,
              percent(s.pwm), s.current_ma, s.statusword, mib::to_string(s.state));
        }
      },
      "Disable voltage now; then PWM and current");
  menu->Insert(
      "clear", [](std::ostream &out) { report(out, actuator().clear_faults(), "clear"); },
      "CiA 402 fault reset of the selected channel");
  menu->Insert(
      "wait", [](std::ostream &out) { wait_for_move(out); },
      "Print the count until the move ends (any key ends the wait, not the move)");
  menu->Insert(
      "init", [](std::ostream &out) { report(out, actuator().initialize(), "initialize"); },
      "Rerun initialize(): settings written and read back");

  menu->Insert(
      "scan",
      [](std::ostream &out) {
        out << "asking node ids 1..127 for 0x1000 (reads only, about 8 s)\n";
        const auto found = board().scan_bus(60ms);
        for (const auto &[id, type] : found) {
          out << fmt::format("  node {:3}: device type 0x{:08X}\n", id, type);
        }
        out << fmt::format("{} node(s) answered\n", found.size());
      },
      "Find which node ids answer an SDO (reads only)");
  menu->Insert(
      "sdor",
      [](std::ostream &out, int node, const std::string &index_text, int sub, int bytes) {
        const auto access = board().controller_access(static_cast<uint8_t>(node));
        if (!access) {
          out << "no leg in MIBconfig.hpp is on that node\n";
          return;
        }
        const int index = parse_index(index_text);
        std::error_code ec;
        int64_t value = 0;
        {
          std::lock_guard<std::mutex> lock(*access->mutex);
          switch (bytes) {
          case 1:
            value = access->client->read_i8(index, sub, ec);
            break;
          case 2:
            value = access->client->read_i16(index, sub, ec);
            break;
          case 4:
            value = access->client->read_i32(index, sub, ec);
            break;
          default:
            out << "bytes must be 1, 2 or 4\n";
            return;
          }
        }
        if (ec) {
          out << fmt::format("read n{} 0x{:04X}:{} failed: {}\n", node, index, sub, ec.message());
        } else {
          out << fmt::format("n{} 0x{:04X}:{} = {} (0x{:X})\n", node, index, sub, value,
                             static_cast<uint32_t>(value));
        }
      },
      "Raw SDO read: sdor <node> <index> <sub> <bytes>");
  menu->Insert(
      "sdow",
      [](std::ostream &out, int node, const std::string &index_text, int sub, int bytes,
         int value) {
        if (!node_has_commissioned_leg(node)) {
          out << fmt::format("REFUSED: node {} has no commissioned leg; the MIB does not write "
                             "to it\n",
                             node);
          return;
        }
        const auto access = board().controller_access(static_cast<uint8_t>(node));
        if (!access) {
          out << "no leg in MIBconfig.hpp is on that node\n";
          return;
        }
        const int index = parse_index(index_text);
        std::error_code ec;
        bool ok = false;
        {
          std::lock_guard<std::mutex> lock(*access->mutex);
          switch (bytes) {
          case 1:
            ok = access->client->write_i8(index, sub, static_cast<int8_t>(value), ec);
            break;
          case 2:
            ok = access->client->write_i16(index, sub, static_cast<int16_t>(value), ec);
            break;
          case 4:
            ok = access->client->write_i32(index, sub, value, ec);
            break;
          default:
            out << "bytes must be 1, 2 or 4\n";
            return;
          }
        }
        out << fmt::format("write n{} 0x{:04X}:{} = {} {}\n", node, index, sub, value,
                           ok ? "accepted" : "FAILED: " + ec.message());
      },
      "Raw SDO write: sdow <node> <index> <sub> <bytes> <value> (commissioned nodes only)");
  menu->Insert(
      "flags",
      [](std::ostream &out) {
        // Basicmicro "read status" (command 90) mirrored at 0x205A. Bit map from the
        // RoboClaw/MCP manual; the bench saw it read 0 throughout.
        static constexpr struct {
          uint32_t bit;
          const char *name;
        } kBits[] = {
            {0x00000001, "E-stop"},
            {0x00000002, "temperature error"},
            {0x00000004, "temperature 2 error"},
            {0x00000008, "main voltage high error"},
            {0x00000010, "logic voltage high error"},
            {0x00000020, "logic voltage low error"},
            {0x00000040, "M1 driver fault"},
            {0x00000080, "M2 driver fault"},
            {0x00000100, "M1 speed error"},
            {0x00000200, "M2 speed error"},
            {0x00000400, "M1 position error"},
            {0x00000800, "M2 position error"},
            {0x00001000, "M1 current error"},
            {0x00002000, "M2 current error"},
            {0x00010000, "M1 over-current warning"},
            {0x00020000, "M2 over-current warning"},
            {0x00040000, "main voltage high warning"},
            {0x00080000, "main voltage low warning"},
            {0x00100000, "temperature warning"},
            {0x00200000, "temperature 2 warning"},
            {0x00400000, "S4 signal triggered"},
            {0x00800000, "S5 signal triggered"},
            {0x01000000, "speed error limit warning"},
            {0x02000000, "position error limit warning"},
        };
        const auto access = board().controller_access(row().node_id);
        if (!access) {
          return;
        }
        std::error_code ec;
        uint32_t flags = 0;
        {
          std::lock_guard<std::mutex> lock(*access->mutex);
          flags = access->client->read_u32(0x205A, 0, ec);
        }
        if (ec) {
          out << fmt::format("status flags read failed: {}\n", ec.message());
          return;
        }
        out << fmt::format("node {} status flags 0x{:08X}:", row().node_id, flags);
        bool any = false;
        for (const auto &b : kBits) {
          if (flags & b.bit) {
            out << " [" << b.name << "]";
            any = true;
          }
        }
        out << (any ? "\n" : " normal\n");
      },
      "The selected leg's controller status flags (command 90)");

  // Calibration: absolute encoders, once per installation.
  menu->Insert(
      "limits",
      [](std::ostream &out) {
        int32_t min = 0, max = 0;
        if (actuator().read_controller_limits(min, max)) {
          const auto r = actuator().range();
          out << fmt::format("controller clamp [{}, {}]; actuator range [{}, {}]{}\n", min, max,
                             r.min, r.max,
                             actuator().calibration_mode() ? " (calibrating: clamp lifted)" : "");
        } else {
          out << "read failed (see log)\n";
        }
      },
      "Read the MinPos/MaxPos clamp the controller currently holds");
  menu->Insert(
      "cal",
      [](std::ostream &out, const std::string &mode) {
        const bool on = mode == "on" || mode == "1";
        report(out, actuator().set_calibration_mode(on),
               on ? "calibration mode ON (clamp lifted, jogs only, mind the mechanical ends)"
                  : "calibration mode off");
      },
      "Lift or restore the range clamp: cal on|off");
  menu->Insert(
      "range",
      [](std::ostream &out, int min, int max) {
        report(out, board().set_actuator_range(selected.load(), {min, max}),
               "range install and save");
      },
      "Install a range and save it to NVS: range <min> <max>");
  menu->Insert(
      "forget",
      [](std::ostream &out) {
        mib::ActuatorStore store;
        out << (store.init() && store.erase_range(actuator().name())
                    ? "saved range erased; the compiled default applies after reboot\n"
                    : "erase failed (see log)\n");
      },
      "Erase the saved range");

  // Homing: incremental encoders. Runs in its own task so `stop` stays available.
  menu->Insert(
      "home",
      [](std::ostream &out) {
        if (refuse_uncommissioned(out)) {
          return;
        }
        static std::atomic<bool> homing{false};
        if (homing.exchange(true)) {
          out << "already homing\n";
          return;
        }
        xTaskCreate(
            [](void *) {
              const bool ok = actuator().home();
              logger.info("homing {}", ok ? "done" : "FAILED (see log)");
              homing = false;
              vTaskDelete(nullptr);
            },
            "homing", 6 * 1024, nullptr, 4, nullptr);
        out << "homing started; `stop` aborts it\n";
      },
      "Stream toward the limit switch and install the home count");
  menu->Insert(
      "restore", [](std::ostream &out) { report(out, actuator().restore_position(), "restore"); },
      "Install the position saved on a previous run");
  menu->Insert(
      "save", [](std::ostream &out) { report(out, actuator().save_position(true), "save"); },
      "Save the current position now");
  menu->Insert(
      "setenc",
      [](std::ostream &out, int counts) {
        report(out, actuator().set_encoder(counts), "set encoder");
      },
      "Write the controller's encoder count: setenc <counts>");

  return menu;
}

/// The move engine: every Actuator::kTickPeriod, on a fixed-period schedule, above the
/// console so typing never delays a tick.
void poll_task(void *) {
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(Actuator::kTickPeriod.count());
  for (;;) {
    board().poll_actuators();
    if (xTaskDelayUntil(&last_wake, period) == pdFALSE) {
      last_wake = xTaskGetTickCount(); // overran: restart the schedule, do not burst
    }
  }
}

} // namespace

extern "C" void app_main(void) {
  logger.info("Actuator example: leg actuators through the BSP; selected {}", row().name);
  if (!board().init_actuators()) {
    logger.error("CAN bring-up failed; nothing to do");
    return;
  }
  xTaskCreate(poll_task, "actuator poll", 6 * 1024, nullptr, 6, nullptr);

  // The selected leg's settings as the boot check: configured and read back must match.
  print_settings(std::cout);
  print_status(std::cout);

  cli::Cli cli(build_menu());
  cli.ExitAction([](auto &out) { out << "bye\n"; });
  espp::Cli input(cli);
  input.SetInputHistorySize(20);
  // Whatever the reset or the monitor left in the receive buffer would otherwise become
  // the first keystroke ("wrong command: <garbage>"). Drain it before the prompt opens.
  std::this_thread::sleep_for(200ms);
  {
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    char discard[16];
    while (read(STDIN_FILENO, discard, sizeof(discard)) > 0) {
    }
    fcntl(STDIN_FILENO, F_SETFL, flags);
  }
  fmt::print("Type 'help' for the commands. Selected leg: {} (`leg` to change).\n", row().name);
  input.Start();

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
