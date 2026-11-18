#pragma once

/// What this firmware may put on the CAN bus, decided frame by frame.
///
/// Pure functions on plain data (no ESP-IDF, no espp), so they read on their own
/// and can be compiled on a host.
///
/// check_frame() is the LAST line of defence, not the first. Read-only nodes (the
/// carriages, node 2 by default) must never be moved by this firmware. The motion
/// code refuses a read-only node before it builds any frame, and `stop` sends
/// nothing to one (it only reports its readbacks). This filter sits in the one
/// function that hands frames to the TWAI driver, so that a bug above it still
/// cannot write to a read-only node or broadcast to it.
///
/// It is an allow-list. A frame goes out only if it is one of:
///   * an SDO request (COB-ID 0x600 + node, 8 data bytes). To a read-only node, only
///     an upload initiate (byte 0 = 0x40) or an upload segment request (byte 0 =
///     0x60 or 0x70), i.e. a read.
///   * an NMT command (COB-ID 0x000) addressed to one node that is not read-only.
///     Target 0 is a broadcast and would reach the read-only nodes.
///   * an RPDO (COB-ID 0x200 / 0x300 / 0x400 / 0x500 + node) to a node that is not
///     read-only. Nothing here sends one today.
///   * the MIB's own heartbeat (COB-ID 0x700 + heartbeat node id, 1 byte). It is
///     not a write, and only a controller configured to consume it acts on it.
/// Everything else is dropped: SYNC (0x080), TIME (0x100), LSS, any remote or
/// extended frame, anything unrecognised.

#include <bitset>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace sysid {

/// A set of CANopen node ids (1..127).
using NodeSet = std::bitset<128>;

/// Parse "1,2,3" into ids, skipping anything outside 1..127. At most `max_count`.
inline std::vector<uint8_t> parse_node_list(const char *text, size_t max_count = 127) {
  std::vector<uint8_t> ids;
  std::string token;
  for (const char *p = text;; ++p) {
    if (*p == ',' || *p == '\0') {
      const int id = std::atoi(token.c_str());
      if (id >= 1 && id <= 127 && ids.size() < max_count) {
        ids.push_back(static_cast<uint8_t>(id));
      }
      token.clear();
      if (*p == '\0') {
        break;
      }
    } else {
      token += *p;
    }
  }
  return ids;
}

inline NodeSet to_node_set(const std::vector<uint8_t> &ids) {
  NodeSet set;
  for (uint8_t id : ids) {
    set.set(id);
  }
  return set;
}

/// The parts of a CAN frame the policy looks at.
struct FrameView {
  uint32_t id;
  bool extended;
  bool rtr;
  uint8_t dlc;
  const uint8_t *data; ///< dlc bytes
};

/// The decision on one frame.
struct Verdict {
  bool allow;
  uint8_t node;   ///< the node the frame addresses; 0 for a broadcast or unknown
  const char *why; ///< for the event when blocked
};

/// Decide whether a frame may go on the bus.
/// \param frame The frame.
/// \param read_only Nodes that may only be read.
/// \param heartbeat_node_id The MIB's own heartbeat id, or 0 if it sends none.
inline Verdict check_frame(const FrameView &frame, const NodeSet &read_only,
                           uint8_t heartbeat_node_id) {
  if (frame.extended || frame.rtr) {
    return {false, 0, "extended or remote frame"};
  }
  const uint32_t function = frame.id & 0x780; // CANopen function code
  const uint8_t node = static_cast<uint8_t>(frame.id & 0x7F);

  // SDO request to a server.
  if (function == 0x600 && node != 0) {
    if (!read_only.test(node)) {
      return {true, node, "sdo"};
    }
    if (frame.dlc < 1) {
      return {false, node, "empty SDO frame"};
    }
    const uint8_t command = frame.data[0];
    if (command == 0x40) {
      return {true, node, "sdo upload initiate"};
    }
    if (command == 0x60 || command == 0x70) {
      return {true, node, "sdo upload segment"};
    }
    return {false, node, "SDO request that is not a read"};
  }

  // NMT command: [command, target node].
  if (frame.id == 0x000) {
    if (frame.dlc < 2) {
      return {false, 0, "malformed NMT"};
    }
    const uint8_t target = frame.data[1];
    if (target == 0) {
      return {false, 0, "NMT broadcast"};
    }
    if (target > 127 || read_only.test(target)) {
      return {false, target, "NMT to a read-only node"};
    }
    return {true, target, "nmt"};
  }

  // RPDO 1..4 to a node.
  if ((function == 0x200 || function == 0x300 || function == 0x400 || function == 0x500) &&
      node != 0) {
    if (read_only.test(node)) {
      return {false, node, "RPDO to a read-only node"};
    }
    return {true, node, "rpdo"};
  }

  // Our own heartbeat.
  if (heartbeat_node_id != 0 && frame.id == 0x700u + heartbeat_node_id && frame.dlc == 1) {
    return {true, 0, "heartbeat"};
  }

  return {false, 0, "not on the allow-list (SYNC, TIME, LSS or unknown)"};
}

} // namespace sysid
