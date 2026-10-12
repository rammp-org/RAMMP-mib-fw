/// Actuator example and bench console.
///
/// Brings up one mib::Actuator on one channel of one MCP266 and hands you a
/// console over the serial monitor to exercise its API:
///
///   scan                  find which node ids answer an SDO on the bus (1..127)
///   init                  re-run initialize() (faults, clamp, saved position)
///   status                position, state, range
///   pos                   read the position from the encoder
///   profile [v a d]       show or set the motion profile used by moves (counts/s, /s^2)
///   abs <counts>          move to an absolute count
///   rel <delta>           move by a signed number of counts
///   inc / dec             move one jog step
///   wait                  block until the drive reports target reached
///   stop                  quick stop
///   clear                 clear a latched fault / limit / E-stop on the controller
///   flags                 the controller's own status flags (Basicmicro command 90)
///   sdor <index> <sub> <bytes>          raw SDO read (index in decimal or 0x hex)
///   sdow <index> <sub> <bytes> <value>  raw SDO write (signed value)
///
///   Calibration (absolute encoder, once per installation):
///   limits                read the clamp the controller currently holds
///   cal on|off            lift or restore the range clamp
///   range <min> <max>     install a calibrated range and save it to NVS
///   forget                erase the saved range; the compiled default applies
///
///   Homing (incremental encoder, every boot; only with kHomingRequired):
///   home                  drive into the limit switch and install the home count
///   restore               install the position saved on a previous run
///   save                  save the current position now
///   setenc <counts>       write the controller's encoder count directly
///
///   selftest              inc, wait, dec, wait; passes if it returns to start
///
/// The settings below would come from a configuration file in the real
/// firmware; they are written out here so the example stands on its own.

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "actuator.hpp"
#include "actuator_store.hpp"
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
constexpr uint8_t kNodeId = 3;
constexpr auto kAxis = mib::Actuator::Axis::M1;
// Set true for an incremental (AB) encoder, which must be homed every boot.
constexpr bool kHomingRequired = true;
// "Forward" (positive count, `inc`) is toward the FRONT of the rack. The carriage homes at
// the front switch, which becomes count kTravel; the back switch is the 0 end. On the
// controller: front switch = M1FLimit, back switch = M1RLimit, encoder counting up
// toward the front.
constexpr int kHomeDirection = +1;
constexpr int32_t kTravel = 12600;
// How the actuator behaves. Positions are the joint encoder's counts.
constexpr mib::Actuator::Config kActuatorConfig{
    .name = "example",              // also the NVS key prefix
    .axis = kAxis,
    .range = {0, kTravel},          // calibrated travel; a range saved to NVS overrides it
    .profile = {500, 500, 500},     // counts/s, counts/s^2, counts/s^2
    .jog_step = 500,                // counts per inc / dec
    .tolerance = 10,                // counts within which a position counts as at target
    .hardware_limits = true,        // limit switches at both ends, configured on the controller
    .homing = {.required = kHomingRequired, .direction = kHomeDirection, .home_count = kTravel},
    .store = nullptr,               // filled in at bring-up
};
// -----------------------------------------------------------------------------

espp::Logger logger({.tag = "Actuator Example", .level = espp::Logger::Verbosity::INFO});

// The transport and controller objects the actuator is layered on. The client
// filters frames by node id and the controller has one SDO channel, so an
// actuator on M2 of the same controller would share `mcp` and `mcp_mutex`.
std::unique_ptr<espp::Twai> twai;
std::unique_ptr<espp::CanopenClient> client;
// Received frames go to this client; `scan` points it at a temporary one per node id.
std::atomic<espp::CanopenClient *> rx_client{nullptr};
std::unique_ptr<espp::Mcp266> mcp;
std::mutex mcp_mutex;
mib::ActuatorStore store;
std::unique_ptr<mib::Actuator> actuator;
// The profile moves use; `profile` changes it at the prompt to try ramps without reflashing.
mib::Actuator::Profile profile = kActuatorConfig.profile;

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
            auto *target = rx_client.load();
            if (!target) {
              return;
            }
            target->process_frame({.id = message.id,
                                   .extended = message.extended,
                                   .rtr = message.rtr,
                                   .dlc = message.dlc,
                                   .data = message.data});
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  rx_client.store(client.get());
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

  // The store keeps the calibrated range (and, for an incremental encoder, the
  // last position) across boots. A saved range wins over the compiled default.
  store.init();
  auto config = kActuatorConfig;
  config.store = &store;
  if (mib::Actuator::Range saved{}; store.load_range(config.name, saved)) {
    logger.info("using calibrated range [{}, {}] from NVS", saved.min, saved.max);
    config.range = saved;
  }

  actuator = std::make_unique<mib::Actuator>(*mcp, mcp_mutex, config);
  actuator->initialize();
  logger.info("actuator {}", actuator->ready() ? "ready" : "NOT ready (see log)");
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
  if (!actuator->read_position(counts)) {
    out << "read failed (see log)\n";
    return;
  }
  const auto r = actuator->range();
  out << fmt::format("{} counts  (range [{}, {}])\n", counts, r.min, r.max);
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
    actuator->read_position(counts);
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
        uint16_t sw = 0;
        const bool have_state = actuator->get_drive_state(state);
        const bool have_reached = have_state && actuator->is_target_reached(reached);
        const bool have_sw = actuator->read_statusword(sw);
        const auto r = actuator->range();
        out << fmt::format("{}: {}, {}, drive {}, target {}, range [{}, {}], jog {}{}{}\n",
                           actuator->name(), actuator->online() ? "online" : "OFFLINE",
                           actuator->ready() ? "ready" : "NOT ready",
                           have_state ? state_name(state) : "unreadable",
                           have_reached ? (reached ? "reached" : "not reached") : "?", r.min,
                           r.max, actuator->config().jog_step,
                           actuator->calibration_mode() ? ", CALIBRATING (clamp lifted)" : "",
                           kHomingRequired ? (actuator->homed() ? ", homed" : ", NOT homed") : "");
        if (have_sw) {
          out << fmt::format("statusword 0x{:04X}{}{}{}\n", sw,
                             (sw & (1 << 11)) ? " [internal limit active]" : "",
                             (sw & (1 << 3)) ? " [FAULT]" : "", (sw & (1 << 7)) ? " [warning]" : "");
        }
        print_position(out);
      },
      "Position, drive state, target reached, online, range");

  menu->Insert(
      "scan",
      [](std::ostream &out) {
        // Ask every node id for its device type (0x1000). A node in CANopen mode answers;
        // a node in raw-CAN mode ACKs the frame but stays silent, so a bus that ACKs with
        // no answers points at the controller's CAN mode rather than its wiring.
        out << "scanning node ids 1..127 (about 10 s)\n";
        int found = 0;
        for (uint8_t id = 1; id <= 127; ++id) {
          espp::CanopenClient probe({
              .node_id = id,
              .send =
                  [](const espp::CanopenClient::CanFrame &frame) {
                    espp::Twai::Message m{.id = frame.id,
                                          .extended = frame.extended,
                                          .rtr = frame.rtr,
                                          .dlc = frame.dlc,
                                          .data = frame.data};
                    std::error_code tx_ec;
                    return twai && twai->transmit(m, tx_ec);
                  },
              .sdo_timeout = 60ms,
              .log_level = espp::Logger::Verbosity::NONE,
          });
          rx_client.store(&probe);
          std::error_code ec;
          const uint32_t device_type = probe.read_u32(0x1000, 0, ec);
          rx_client.store(client.get());
          if (!ec) {
            ++found;
            std::string name;
            std::error_code name_ec;
            rx_client.store(&probe);
            name = probe.read_string(0x1008, 0, name_ec);
            rx_client.store(client.get());
            out << fmt::format("  node {:3}: device type 0x{:08X} \"{}\"\n", id, device_type,
                               name_ec ? "?" : name);
          }
        }
        out << fmt::format("{} node(s) answered; this example talks to node {}\n", found, kNodeId);
      },
      "Find which node ids answer an SDO on the bus");
  menu->Insert("init", [](std::ostream &out) { report(out, actuator->initialize(), "initialize"); },
               "Re-run initialize(): clear faults, install the clamp, restore a saved position");
  menu->Insert("pos", [](std::ostream &out) { print_position(out); }, "Read the position");
  menu->Insert(
      "profile",
      [](std::ostream &out) {
        out << fmt::format("profile: velocity {} counts/s, accel {} counts/s^2, decel {} counts/s^2\n",
                           profile.velocity, profile.acceleration, profile.deceleration);
      },
      "Show the motion profile used by abs / rel");
  menu->Insert(
      "profile",
      [](std::ostream &out, int velocity, int accel, int decel) {
        if (velocity <= 0 || accel <= 0 || decel <= 0) {
          out << "all three must be positive\n";
          return;
        }
        profile = {static_cast<uint32_t>(velocity), static_cast<uint32_t>(accel),
                   static_cast<uint32_t>(decel)};
        out << fmt::format("profile set: ramp up {:.1f} s, ramp down {:.1f} s at full speed\n",
                           static_cast<float>(velocity) / accel, static_cast<float>(velocity) / decel);
      },
      "Set the motion profile: profile <velocity> <accel> <decel>");
  menu->Insert(
      "abs",
      [](std::ostream &out, int counts) { report(out, actuator->move_absolute(counts, profile), "move"); },
      "Move to an absolute count: abs <counts>");
  menu->Insert(
      "rel",
      [](std::ostream &out, int delta) { report(out, actuator->move_relative(delta, profile), "move"); },
      "Move by a signed count: rel <delta>");
  menu->Insert("inc", [](std::ostream &out) { report(out, actuator->increment(), "increment"); },
               "Move one jog step up");
  menu->Insert("dec", [](std::ostream &out) { report(out, actuator->decrement(), "decrement"); },
               "Move one jog step down");
  menu->Insert("wait", [](std::ostream &out) { wait_for_target(out); },
               "Block until the drive reports target reached");
  menu->Insert("stop", [](std::ostream &out) { report(out, actuator->stop(), "stop"); },
               "Quick stop");
  menu->Insert("clear", [](std::ostream &out) { report(out, actuator->clear_faults(), "clear"); },
               "Clear a latched fault, limit or E-stop on the controller");
  // Raw object access for experiments the Actuator API does not cover (profile type,
  // manufacturer mirrors). Index may be given as 0x6086 or decimal.
  const auto parse_index = [](const std::string &text) -> int {
    return static_cast<int>(std::strtol(text.c_str(), nullptr, 0));
  };
  menu->Insert(
      "sdor",
      [parse_index](std::ostream &out, std::string index_text, int sub, int bytes) {
        const int index = parse_index(index_text);
        std::error_code ec;
        int64_t value = 0;
        switch (bytes) {
        case 1: value = client->read_i8(index, sub, ec); break;
        case 2: value = client->read_i16(index, sub, ec); break;
        case 4: value = client->read_i32(index, sub, ec); break;
        default: out << "bytes must be 1, 2 or 4\n"; return;
        }
        if (ec) {
          out << fmt::format("read 0x{:04X}:{} failed: {}\n", index, sub, ec.message());
        } else {
          out << fmt::format("0x{:04X}:{} = {} (0x{:X})\n", index, sub, value,
                             static_cast<uint32_t>(value));
        }
      },
      "Raw SDO read: sdor <index> <sub> <bytes>");
  menu->Insert(
      "sdow",
      [parse_index](std::ostream &out, std::string index_text, int sub, int bytes, int value) {
        const int index = parse_index(index_text);
        std::error_code ec;
        bool ok = false;
        switch (bytes) {
        case 1: ok = client->write_i8(index, sub, static_cast<int8_t>(value), ec); break;
        case 2: ok = client->write_i16(index, sub, static_cast<int16_t>(value), ec); break;
        case 4: ok = client->write_i32(index, sub, value, ec); break;
        default: out << "bytes must be 1, 2 or 4\n"; return;
        }
        out << fmt::format("write 0x{:04X}:{} = {} {}\n", index, sub, value,
                           ok ? "accepted" : "FAILED: " + ec.message());
      },
      "Raw SDO write: sdow <index> <sub> <bytes> <value>");
  menu->Insert(
      "flags",
      [](std::ostream &out) {
        // Basicmicro "read status" (command 90) mirrored at 0x205A. espp::Mcp266 does not
        // wrap it yet, so this reads it through the client directly. The bit map is the
        // RoboClaw/MCP 32-bit status word from the Basicmicro manual; unverified on this
        // firmware, which is what this command is for.
        static constexpr struct { uint32_t bit; const char *name; } kBits[] = {
            {0x00000001, "E-stop"}, {0x00000002, "temperature error"},
            {0x00000004, "temperature 2 error"}, {0x00000008, "main voltage high error"},
            {0x00000010, "logic voltage high error"}, {0x00000020, "logic voltage low error"},
            {0x00000040, "M1 driver fault"}, {0x00000080, "M2 driver fault"},
            {0x00000100, "M1 speed error"}, {0x00000200, "M2 speed error"},
            {0x00000400, "M1 position error"}, {0x00000800, "M2 position error"},
            {0x00001000, "M1 current error"}, {0x00002000, "M2 current error"},
            {0x00010000, "M1 over-current warning"}, {0x00020000, "M2 over-current warning"},
            {0x00040000, "main voltage high warning"}, {0x00080000, "main voltage low warning"},
            {0x00100000, "temperature warning"}, {0x00200000, "temperature 2 warning"},
            {0x00400000, "S4 signal triggered"}, {0x00800000, "S5 signal triggered"},
            {0x01000000, "speed error limit warning"}, {0x02000000, "position error limit warning"},
        };
        std::error_code ec;
        uint32_t flags = client->read_u32(0x205A, 0, ec);
        int width = 32;
        if (ec) {
          ec.clear();
          flags = client->read_u16(0x205A, 0, ec);
          width = 16;
        }
        if (ec) {
          out << fmt::format("status flags read failed: {}\n", ec.message());
          return;
        }
        out << fmt::format("status flags 0x{:08X} ({}-bit):", flags, width);
        bool any = false;
        for (const auto &b : kBits) {
          if (flags & b.bit) {
            out << " [" << b.name << "]";
            any = true;
          }
        }
        out << (any ? "\n" : " normal\n");
        uint16_t sw = 0;
        if (actuator->read_statusword(sw)) {
          out << fmt::format("statusword 0x{:04X}\n", sw);
        }
      },
      "Read the controller's own status flags (command 90) and decode them");

  // Calibration: absolute encoders, once per installation.
  menu->Insert(
      "limits",
      [](std::ostream &out) {
        int32_t min = 0, max = 0;
        if (actuator->read_controller_limits(min, max)) {
          const auto r = actuator->range();
          out << fmt::format("controller clamp [{}, {}]; actuator range [{}, {}]{}\n", min, max,
                             r.min, r.max,
                             actuator->calibration_mode() ? " (calibrating: clamp lifted)" : "");
        } else {
          out << "read failed (see log)\n";
        }
      },
      "Read the MinPos/MaxPos clamp the controller currently holds");
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
        report(out, actuator->set_range({min, max}), "range install and save");
      },
      "Install a calibrated range and save it to NVS: range <min> <max>");
  menu->Insert(
      "forget",
      [](std::ostream &out) {
        out << (store.erase_range(actuator->name())
                    ? "saved range erased; the compiled default applies after reboot\n"
                    : "erase failed (see log)\n");
      },
      "Erase the saved range");

  // Homing: incremental encoders, every boot.
  menu->Insert("home", [](std::ostream &out) { report(out, actuator->home(), "homing"); },
               "Drive into the limit switch and install the home count");
  menu->Insert("restore", [](std::ostream &out) { report(out, actuator->restore_position(), "restore"); },
               "Install the position saved on a previous run");
  menu->Insert("save", [](std::ostream &out) { report(out, actuator->save_position(true), "save"); },
               "Save the current position now");
  menu->Insert(
      "setenc", [](std::ostream &out, int counts) { report(out, actuator->set_encoder(counts), "set encoder"); },
      "Write the controller's encoder count: setenc <counts>");

  menu->Insert(
      "selftest",
      [](std::ostream &out) {
        int32_t start = 0;
        if (!actuator->read_position(start)) {
          out << "FAIL: cannot read position (see log)\n";
          return;
        }
        out << fmt::format("start at {} counts, jog step {}\n", start, actuator->config().jog_step);
        if (!actuator->increment() || !wait_for_target(out)) {
          out << "FAIL: increment (see log)\n";
          return;
        }
        int32_t up = 0;
        actuator->read_position(up);
        if (!actuator->decrement() || !wait_for_target(out)) {
          out << "FAIL: decrement (see log)\n";
          return;
        }
        int32_t back = 0;
        actuator->read_position(back);
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
