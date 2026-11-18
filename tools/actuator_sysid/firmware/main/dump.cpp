#include "dump.hpp"

#include <algorithm>
#include <array>
#include <string>

#include "format.hpp"
#include "wire.hpp"

namespace sysid {

namespace {

constexpr uint16_t kM2Offset = 0x800; // M2's CiA 402 objects mirror M1's at +0x800

void emit_cfg(uint8_t node_id, const std::string &name, uint16_t index, uint8_t sub,
              const std::string &value) {
  emit_line(fmt::format("#cfg,{},{},0x{:04X},{},{}", node_id, name, index, sub, value));
}

/// Upload one object; returns the printed value and, if it worked, the number.
std::string upload(McpNode &node, uint16_t index, uint8_t sub, uint32_t *number = nullptr) {
  std::array<uint8_t, 4> buffer{};
  std::error_code ec;
  const size_t n = node.client().sdo_upload(index, sub, buffer, ec);
  if (n == 0) {
    const uint32_t abort_code = node.client().last_abort_code();
    if (ec == std::errc::protocol_error && abort_code != 0) {
      return fmt::format("abort:0x{:08X}", abort_code);
    }
    return "error:" + ec.message();
  }
  uint32_t raw = 0;
  for (size_t i = 0; i < n; ++i) {
    raw |= static_cast<uint32_t>(buffer[i]) << (8 * i);
  }
  if (number != nullptr) {
    *number = raw;
  }
  // Signed reading of the same n bytes, for counts and limits.
  int32_t as_signed = static_cast<int32_t>(raw);
  if (n < 4 && (raw & (1u << (8 * n - 1)))) {
    as_signed = static_cast<int32_t>(raw | ~((1u << (8 * n)) - 1));
  }
  return fmt::format("0x{:0{}X}={}", raw, static_cast<int>(2 * n), as_signed);
}

void one(McpNode &node, const std::string &name, uint16_t index, uint8_t sub) {
  emit_cfg(node.id(), name, index, sub, upload(node, index, sub));
}

void text(McpNode &node, const std::string &name, uint16_t index) {
  std::error_code ec;
  std::string value = node.client().read_string(index, 0, ec);
  if (ec) {
    const uint32_t abort_code = node.client().last_abort_code();
    value = (ec == std::errc::protocol_error && abort_code != 0)
                ? fmt::format("abort:0x{:08X}", abort_code)
                : "error:" + ec.message();
  } else {
    std::replace_if(
        value.begin(), value.end(), [](char c) { return c == ',' || c == '\n' || c == '\r'; }, ';');
  }
  emit_cfg(node.id(), name, index, 0, value);
}

/// An object with a sub-0 count: sub 0, then subs 1..min(count, max_subs).
void record(McpNode &node, const std::string &name, uint16_t index, uint8_t max_subs) {
  uint32_t count = 0;
  const std::string first = upload(node, index, 0, &count);
  emit_cfg(node.id(), name + "_count", index, 0, first);
  if (first.rfind("0x", 0) != 0) {
    return; // refused or no answer: the record is not there
  }
  for (uint8_t sub = 1; sub <= count && sub <= max_subs; ++sub) {
    one(node, fmt::format("{}_{}", name, sub), index, sub);
  }
}

} // namespace

void dump_node(McpNode &node) {
  emit_event(fmt::format("dump n{} start (read-only)", node.id()));

  // First, what the operator decides from: each channel's present count and the
  // two windows the controller limits a position target to (the position PID
  // clamp, command 63 / 64 subs 6 and 7, and the CiA 402 software limits 0x607D).
  for (int ch = 0; ch < 2; ++ch) {
    const uint16_t off = ch == 0 ? 0 : kM2Offset;
    const std::string m = ch == 0 ? "m1_" : "m2_";
    one(node, m + "present_count", 0x6064 + off, 0);
    // The clamp through the one proven path: the whole record from sub 1, read
    // twice (McpNode::read_clamp_checked). Reading sub 6 / 7 alone returns 0.
    int32_t clamp_min = 0, clamp_max = 0;
    std::string why;
    if (node.read_clamp_checked(ch, clamp_min, clamp_max, why)) {
      emit_cfg(node.id(), m + "clamp_min", 0x203F + ch, 6,
               fmt::format("0x{:08X}={}", static_cast<uint32_t>(clamp_min), clamp_min));
      emit_cfg(node.id(), m + "clamp_max", 0x203F + ch, 7,
               fmt::format("0x{:08X}={}", static_cast<uint32_t>(clamp_max), clamp_max));
    } else {
      emit_cfg(node.id(), m + "clamp_min", 0x203F + ch, 6, "error:" + why);
      emit_cfg(node.id(), m + "clamp_max", 0x203F + ch, 7, "error:" + why);
    }
    one(node, m + "software_limit_min", 0x607D + off, 1);
    one(node, m + "software_limit_max", 0x607D + off, 2);
  }

  // Identity and communication (CiA 301).
  one(node, "device_type", 0x1000, 0);
  one(node, "error_register", 0x1001, 0);
  text(node, "device_name", 0x1008);
  text(node, "hardware_version", 0x1009);
  text(node, "software_version", 0x100A);
  record(node, "heartbeat_consumer", 0x1016, 4);
  one(node, "heartbeat_producer_ms", 0x1017, 0);
  record(node, "identity", 0x1018, 4);
  // Basicmicro's own firmware string (command 21 mirror). espp does not wrap it.
  text(node, "basicmicro_firmware", 0x2015);

  // Per channel: CiA 402 configuration and state.
  for (int ch = 0; ch < 2; ++ch) {
    const uint16_t off = ch == 0 ? 0 : kM2Offset;
    const std::string m = ch == 0 ? "m1_" : "m2_";
    one(node, m + "supported_modes", 0x6502 + off, 0);
    one(node, m + "mode_display", 0x6061 + off, 0);
    one(node, m + "mode", 0x6060 + off, 0);
    one(node, m + "abort_connection_option", 0x6007 + off, 0);
    one(node, m + "quick_stop_option", 0x605A + off, 0);
    one(node, m + "controlword", 0x6040 + off, 0);
    one(node, m + "statusword", 0x6041 + off, 0);
    one(node, m + "position_actual", 0x6064 + off, 0);
    one(node, m + "software_limit_min", 0x607D + off, 1);
    one(node, m + "software_limit_max", 0x607D + off, 2);
    one(node, m + "profile_velocity", 0x6081 + off, 0);
    one(node, m + "profile_acceleration", 0x6083 + off, 0);
    one(node, m + "profile_deceleration", 0x6084 + off, 0);
    one(node, m + "motion_profile_type", 0x6086 + off, 0);
  }

  // Basicmicro records mirrored at 0x2000 + command (M1 command, M2 command + 1).
  static constexpr const char *kPositionPid[] = {"p_x1024", "i_x1024", "d_x1024", "max_i",
                                                 "deadband", "min_pos_clamp", "max_pos_clamp"};
  static constexpr const char *kVelocityPid[] = {"p_16_16", "i_16_16", "d_16_16", "qpps"};
  for (int ch = 0; ch < 2; ++ch) {
    const std::string m = ch == 0 ? "m1_" : "m2_";
    for (uint8_t sub = 1; sub <= 7; ++sub) { // command 63 / 64, readback order P, I, D, ...
      one(node, m + "position_pid_" + kPositionPid[sub - 1], 0x203F + ch, sub);
    }
    for (uint8_t sub = 1; sub <= 4; ++sub) { // command 55 / 56
      one(node, m + "velocity_pid_" + kVelocityPid[sub - 1], 0x2037 + ch, sub);
    }
    // Stored defaults, per the teammate's bench notes: speed command 70 / 71 sub 0
    // (u16); accel command 68 / 69 sub 1 (u32), sub 2 decel (inert).
    one(node, m + "default_speed", 0x2046 + ch, 0);
    one(node, m + "default_accel", 0x2044 + ch, 1);
    one(node, m + "default_decel", 0x2044 + ch, 2);
  }
  one(node, "status_flags", 0x205A, 0);              // command 90
  one(node, "main_battery_min_x10", 0x203B, 1);      // command 59
  one(node, "main_battery_max_x10", 0x203B, 2);
  one(node, "encoder_mode_m1", 0x205B, 1);           // command 91
  one(node, "encoder_mode_m2", 0x205B, 2);

  // PDO communication and mapping parameters.
  for (uint16_t i = 0; i < 4; ++i) {
    record(node, fmt::format("rpdo{}_comm", i + 1), 0x1400 + i, 6);
    record(node, fmt::format("rpdo{}_map", i + 1), 0x1600 + i, 8);
    record(node, fmt::format("tpdo{}_comm", i + 1), 0x1800 + i, 6);
    record(node, fmt::format("tpdo{}_map", i + 1), 0x1A00 + i, 8);
  }

  emit_event(fmt::format("dump n{} done", node.id()));
}

} // namespace sysid
