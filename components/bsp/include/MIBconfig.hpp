#pragma once

#include <cstdint>

namespace mib::config {

/// @brief Enable DHCP server mode for the MIB Ethernet interface.
inline constexpr bool ethernet_dhcp_server = true;

/// @brief IPv4 address assigned to the MIB Ethernet interface.
/// @details Stored as four octets in network order.
inline constexpr std::uint8_t ethernet_ip[4] = {192, 168, 4, 1};

/// @brief IPv4 netmask used by the MIB Ethernet interface.
/// @details Stored as four octets in network order.
inline constexpr std::uint8_t ethernet_netmask[4] = {255, 255, 255, 0};

/// @brief IPv4 gateway address advertised by the MIB DHCP server.
/// @details Stored as four octets in network order.
inline constexpr std::uint8_t ethernet_gateway[4] = {192, 168, 4, 1};

} // namespace mib::config