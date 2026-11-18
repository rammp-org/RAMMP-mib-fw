#pragma once

/// One MCP266 on the CAN bus, polled over SDO by its own task.
///
/// Reads only: every call below is an SDO upload (expedited, or segmented for the
/// device name). No NMT command, no controlword, no fault reset, no encoder write;
/// the controller's CiA 402 state is never touched. (The bench found a drive left
/// in "switch on disabled" by firmware blocks Motion Studio's PWM slider until a
/// power cycle, so a logger must not leave a trace.) Each poll pass reads the fast fields of both channels (encoder,
/// speed, current, duty) and one slow field from a ring (statusword M1,
/// statusword M2, status flags, battery, temperature). The sample task copies the
/// latest values with snapshot(); every value carries the time it was read, so a
/// slow or silent node never holds up a row: its values just get older, and once
/// it is marked offline they are printed empty.
///
/// Where each value comes from (espp 1.4.3):
///   enc     Mcp266::read_encoder      CiA 402 position actual   0x6064 / 0x6864
///   spd     Mcp266::read_speed        CiA 402 velocity actual   0x606C / 0x686C
///   status  Mcp266::read_statusword   CiA 402 statusword        0x6041 / 0x6841
///   cur     CanopenClient::sdo_upload  0x2031:1 / :2  (Basicmicro command 49 mirror)
///   duty    CanopenClient::sdo_upload  0x2030:1 / :2  (Basicmicro command 48 mirror)
///   flags   CanopenClient::sdo_upload  0x205A:0       (Basicmicro command 90 mirror)
///   vbat    Mcp266::read_main_battery_voltage  0x2018 (command 24)
///   temp    Mcp266::read_temperature           0x2052 (command 82)
/// Mcp266 has no reader for current, duty or status flags, hence the raw SDOs.
/// The subindex layout of 0x2030 / 0x2031 (one per motor, in reply order) follows
/// the position PID record's (0x203F:1..7) and is unverified on the bench.

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "canopen_client.hpp"
#include "mcp266.hpp"

namespace sysid {

/// One value read from a controller.
struct Field {
  bool valid{false};
  int32_t value{0};
  int64_t t_us{0}; ///< Midpoint of the SDO round trip that read it.
};

/// One motor channel's values.
struct ChannelData {
  Field enc;    ///< counts
  Field spd;    ///< counts/s
  Field cur_ma; ///< mA
  Field duty;   ///< -32767..32767 = -100..100 %
  Field status; ///< CiA 402 statusword
};

/// Whether a node is answering.
enum class Link : uint8_t { UNKNOWN, ONLINE, OFFLINE };

/// Everything known about one node at one instant.
struct NodeData {
  Link link{Link::UNKNOWN};
  ChannelData ch[2]; ///< [0] = M1 (c1), [1] = M2 (c2)
  Field flags;       ///< Basicmicro status flags (command 90), node-wide
  Field vbat_mv;     ///< main battery, mV
  Field temp_dc;     ///< board temperature, 0.1 degC
};

/// Bits of the `_lim` column. Bit 0 is per channel; bits 1 and 2 are the node's S4 /
/// S5 inputs, copied to both channels because which channel's limit a pin is depends
/// on its Motion Studio configuration.
inline constexpr int32_t kLimInternalLimit = 1 << 0; ///< statusword bit 11
inline constexpr int32_t kLimS4 = 1 << 1;            ///< flags 0x00400000
inline constexpr int32_t kLimS5 = 1 << 2;            ///< flags 0x00800000
inline constexpr uint32_t kFlagS4Triggered = 0x00400000;
inline constexpr uint32_t kFlagS5Triggered = 0x00800000;
inline constexpr uint16_t kStatuswordFault = 1 << 3;
inline constexpr uint16_t kStatuswordInternalLimit = 1 << 11;

/// The `_lim` value of channel `ch` (0 or 1): needs the statusword; the S4/S5 bits
/// are added when the flags were readable. Returns false if there is no value.
bool limit_bits(const NodeData &node, int ch, int32_t &bits);

/// "n3c1 duty=0 cur_ma=0 status=0x0250 (12 ms old)": a channel's output as the
/// poller last read it, for the event after a stop. "?" where there is no value.
std::string output_readback(uint8_t node_id, const NodeData &node, int ch, int64_t t_us);

/// Polled encoder counts kept per channel for settled_count(): 9 passes, about
/// 90 ms at 100 Hz.
inline constexpr size_t kEncoderWindow = 9;

/// SDO request/reply accounting, reported as #meta:n<id>_sdo.
struct SdoAccounting {
  uint32_t requests{0};   ///< SDO frames sent to the node (initiate and segment)
  uint32_t replies{0};    ///< SDO frames received from it
  uint32_t mismatched{0}; ///< replies not for the request in flight (espp drops them)
};

/// Poll statistics, reported as #meta:n<id>_poll.
struct PollStats {
  uint32_t passes{0};        ///< passes that completed
  uint32_t failed_passes{0}; ///< passes cut short by a timeout or transmit failure
  uint32_t timeouts{0};      ///< SDO reads that got no answer (or could not be sent)
  uint32_t aborts{0};        ///< SDO reads the controller refused
  int64_t pass_us_sum{0};    ///< over completed passes
  int64_t pass_us_max{0};
  int64_t sdo_us_max{0};     ///< slowest single SDO round trip that succeeded
};

class McpNode {
public:
  using Axis = espp::Mcp266::Axis;

  /// \param node_id CANopen node id of the controller.
  /// \param send Puts a frame on the bus.
  /// \param period_ms Shared pass period: a pass starts at most this often.
  McpNode(uint8_t node_id, espp::CanopenClient::send_fn send, const std::atomic<int> &period_ms);

  uint8_t id() const { return node_id_; }

  /// Feed a received CAN frame (called from the TWAI receive task).
  void process_frame(const espp::CanopenClient::CanFrame &frame) {
    note_reply(frame);
    client_.process_frame(frame);
  }

  /// Start the poll task.
  void start();

  /// Copy of the latest values.
  NodeData snapshot() const;

  /// Copy of the poll statistics.
  PollStats stats() const;

  /// The controller's device name (0x1008), read once when it first answers.
  std::string device_name() const;

  /// Total SDO reads that failed for lack of an answer; the motion guard watches it.
  uint32_t timeouts() const { return timeouts_.load(); }

  /// The position PID record of a channel, read in subindex order 1..7 (readback
  /// order P, I, D, MaxI, deadband, MinPos, MaxPos). The ONLY way this firmware
  /// reads the clamp. On the bench (MCP266 1.4.9), reading sub 6 or 7 on its own
  /// returned 0, while the same subs read after subs 1..5 returned the real clamp
  /// (README, "The clamp read bug").
  bool read_position_record(int ch, std::array<int32_t, 7> &record, std::error_code &ec);

  /// The clamp from two full record reads; false with `why` if either read fails
  /// or the two disagree.
  bool read_clamp_checked(int ch, int32_t &clamp_min, int32_t &clamp_max, std::string &why);

  /// Median and spread (max - min) of the channel's last kEncoderWindow polled
  /// counts. False if there are not that many, or the oldest is older than
  /// 3 x kEncoderWindow sample periods, so a single outlier never decides.
  bool settled_count(int ch, int64_t now, int32_t &median, int32_t &spread) const;

  /// Poll this node or not (`nodes <list>`). A node that is not polled has no
  /// columns, sends nothing, and its values are cleared.
  void set_enabled(bool enabled) { enabled_ = enabled; }
  bool enabled() const { return enabled_.load(); }

  SdoAccounting sdo_accounting() const {
    return {sdo_requests_.load(), sdo_replies_.load(), sdo_mismatched_.load()};
  }

  /// For commands (stop, motion): the driver. Its SDOs share the poll task's
  /// channel to this node; the client serializes them.
  espp::Mcp266 &mcp() { return mcp_; }
  espp::CanopenClient &client() { return client_; }

private:
  /// What one SDO read fetches.
  enum class Item : uint8_t {
    ENC_M1, SPD_M1, CUR_M1, DUTY_M1,
    ENC_M2, SPD_M2, CUR_M2, DUTY_M2,
    STATUS_M1, STATUS_M2, FLAGS, VBAT, TEMP,
    COUNT
  };
  /// How a read ended.
  enum class Result : uint8_t {
    OK,       ///< value stored
    REFUSED,  ///< the controller answered with an abort (or a bad size): the field is
              ///< invalid, the node is alive
    NO_ANSWER ///< timeout or transmit failure: the pass ends
  };

  static constexpr Item kFastItems[] = {Item::ENC_M1, Item::SPD_M1, Item::CUR_M1, Item::DUTY_M1,
                                        Item::ENC_M2, Item::SPD_M2, Item::CUR_M2, Item::DUTY_M2};
  static constexpr Item kSlowItems[] = {Item::STATUS_M1, Item::STATUS_M2, Item::FLAGS, Item::VBAT,
                                        Item::TEMP};

  void task();
  bool run_pass(size_t &slow_index);
  Result read_item(Item item);
  /// Read a manufacturer mirror object whose width is not documented: 1, 2 or 4
  /// bytes, sign-extended from the width the controller reports.
  bool read_mirror(uint16_t index, uint8_t subindex, bool is_signed, int32_t &value,
                   std::error_code &ec);
  Field &field_for(Item item);
  void store(Item item, int32_t value, int64_t t_us);
  void invalidate(Item item);
  void set_link(Link link);
  void on_heartbeat(uint8_t node_id, espp::CanopenClient::NmtState state);
  void report_changes(Item item, int32_t old_value, bool old_valid, int32_t new_value);
  static const char *item_name(Item item);
  void note_request(const espp::CanopenClient::CanFrame &frame);
  void note_reply(const espp::CanopenClient::CanFrame &frame);
  void clear_all();

  const uint8_t node_id_;
  const std::atomic<int> &period_ms_;
  espp::CanopenClient client_;
  espp::Mcp266 mcp_;

  mutable std::mutex mutex_; ///< guards data_, stats_, device_name_
  NodeData data_;
  PollStats stats_;
  std::string device_name_;
  std::atomic<uint32_t> timeouts_{0};
  std::atomic<bool> enabled_{true};

  // Encoder history per channel (guarded by mutex_).
  struct Count {
    int32_t value;
    int64_t t_us;
  };
  std::array<std::array<Count, kEncoderWindow>, 2> enc_history_{};
  std::array<size_t, 2> enc_history_len_{};
  std::array<size_t, 2> enc_history_next_{};

  // SDO accounting: the address of the request in flight, set by the send path,
  // checked against each reply (espp performs the same check and drops a
  // mismatch; this one counts them).
  std::atomic<uint32_t> sdo_requests_{0};
  std::atomic<uint32_t> sdo_replies_{0};
  std::atomic<uint32_t> sdo_mismatched_{0};
  std::atomic<uint32_t> expected_address_{0}; ///< index << 8 | sub
  std::atomic<bool> expect_segment_{false};

  // Poll task only.
  bool unsupported_[static_cast<size_t>(Item::COUNT)]{};
  int failed_passes_in_row_{0};
  bool have_device_name_{false};
  espp::CanopenClient::NmtState last_nmt_state_{espp::CanopenClient::NmtState::Unknown};
};

} // namespace sysid
