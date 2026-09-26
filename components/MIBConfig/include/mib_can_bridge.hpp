/**
 * @file mib_can_bridge.hpp
 * @brief Raw CAN bridge over RTPS, so a bench tool can talk CANopen to the MCP266 motor
 *        controllers through the MIB's TWAI peripheral.
 *
 * The MIB does not interpret the frames. Every CanFrame received on the tx topic is
 * transmitted on the bus as-is, and every frame the bus delivers is republished on the
 * rx topic as-is. All CANopen knowledge (SDO, NMT, the MCP266 object dictionary) lives in
 * tools/mib_debugger, so it can change without a reflash. A CanStatus sample is published
 * with the other periodic status so the tool can tell a dead transceiver or a bit-rate
 * mismatch from a quiet bus.
 *
 * This is a bench aid on the debugger branch. When main grows a real MCP266 driver
 * (espp/mcp266 over espp/canopen) it will own the bus, and this bridge must not transmit
 * alongside it.
 *
 * Wire rules match the other RAMMP messages: each struct IS the wire layout, XCDR1,
 * fields in declaration order. tools/mib_debugger mirrors these in mib_messages.py.
 */
#pragma once

#include <array>
#include <cstdint>

#define RAMMP_TOPIC_JOYSTICK_CAN_TX "rammp/joystick/can_tx"
#define RAMMP_TOPIC_MIB_CAN_RX "rammp/mib/can_rx"
#define RAMMP_TYPE_CAN_FRAME "rammp/msg/CanFrame"
#define RAMMP_TOPIC_MIB_CAN_STATUS "rammp/mib/can_status"
#define RAMMP_TYPE_CAN_STATUS "rammp/msg/CanStatus"

namespace mib {

namespace can_config {
/// TWAI pins. The espp examples use TX on 17 and RX on 16; swap here if the transceiver
/// is wired the other way round.
inline constexpr int tx_gpio = 17;
inline constexpr int rx_gpio = 16;
/// Basicmicro MCP266 default CAN bit rate.
inline constexpr uint32_t bitrate = 1000000;
} // namespace can_config

/// Flag bits in CanFrame::flags.
inline constexpr uint8_t kCanFlagExtended = 0x01; ///< 29-bit identifier.
inline constexpr uint8_t kCanFlagRtr = 0x02;      ///< Remote transmission request, no data.

/** One classic CAN 2.0 frame, in either direction. 16 bytes on the wire. */
struct CanFrame {
  uint8_t seq;                 /**< +1 per frame from each sender, wraps */
  uint8_t flags;               /**< kCanFlag* bits */
  uint8_t dlc;                 /**< number of valid data bytes, 0..8 */
  uint8_t reserved;            /**< keeps id 4-byte aligned; always 0 */
  uint32_t id;                 /**< arbitration id, 11 or 29 bits */
  std::array<uint8_t, 8> data; /**< payload; bytes past dlc are 0 */
};

/** Bridge health. MIB -> tool, resent periodically, best-effort. 36 bytes on the wire. */
struct CanStatus {
  uint8_t initialized;     /**< 1 once the TWAI node was created and enabled */
  uint8_t enabled;         /**< 1 while the node is on the bus */
  uint8_t bus_state;       /**< twai_error_state_t: 0 active, 1 warning, 2 passive, 3 bus-off */
  uint8_t reserved;        /**< always 0 */
  uint32_t bitrate;        /**< configured bit rate */
  uint32_t tx_ok;          /**< frames acknowledged on the bus */
  uint32_t tx_failed;      /**< frames the controller gave up on (no ack, bit error) */
  uint32_t rx_frames;      /**< frames received from the bus and republished */
  uint32_t rx_dropped;     /**< received frames that could not be republished */
  uint32_t bus_errors;     /**< bus error callbacks since boot */
  uint16_t tx_error_count; /**< controller TX error counter */
  uint16_t rx_error_count; /**< controller RX error counter */
  int16_t tx_gpio;         /**< configured pins, so the tool can show them */
  int16_t rx_gpio;
};

} // namespace mib
