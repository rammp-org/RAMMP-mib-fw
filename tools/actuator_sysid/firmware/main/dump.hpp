#pragma once

#include "mcp_node.hpp"

namespace sysid {

/// `dump <node>`: read everything that describes a controller's configuration and
/// print one `#cfg,<node>,<name>,<index>,<sub>,<value>` line per object.
///
/// Read-only: SDO uploads only (and upload-segment requests for strings), so it
/// works in the passive build and on read-only nodes. Runs in the calling (console)
/// task; rows keep coming from the sample task meanwhile.
///
/// <index> is 0xXXXX and <sub> decimal. <value> is one of:
///   0x..=d       the bytes as uploaded, little-endian assembled, two hex digits
///                per byte (so the width shows: 0x07 is 1 byte, 0x00000700 is 4),
///                then the same bytes read as a signed integer (0x00002710=10000)
///   text         a visible-string object, commas replaced by ';'
///   abort:0x...  the controller refused with that SDO abort code (a valid result:
///                0x06020000 = no such object, 0x06090011 = no such subindex)
///   error:...    no answer, or another failure
void dump_node(McpNode &node);

} // namespace sysid
