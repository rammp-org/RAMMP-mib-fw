#pragma once

#include "sdkconfig.h"

#if CONFIG_SYSID_ENABLE_MOTION

/// Bounded closed-loop excitation over CAN: `arm <node> <ch>`, then
/// `pstep <node> <ch> <delta_counts>` (one CiA 402 profile-position move to the
/// present count + delta); `clearfaults <node>`; and the active `stop`.
///
/// Only nodes in CONFIG_SYSID_MOTION_NODE_IDS that are not read-only can be
/// addressed. Every command checks that before it builds a frame. The transmit
/// filter (bus_policy.hpp) is a second, independent barrier.
///
/// Never used: open-loop duty (0x2020 mirror, inert on this controller), 0x2041
/// (the buffered position mirror, which moved a motor outside the CiA 402 state
/// machine and could not be stopped), cyclic-synchronous modes, PDOs.
///
/// Threads: arm/pstep/clearfaults/stop/console_byte from the console task; check()
/// from the sample task. The bus work of a move or a stop runs in the motion
/// worker task, so the sample task keeps logging (and checking guards) meanwhile.
/// One mutex covers the state; no SDO is sent while holding it, except the
/// heartbeat-consumer writes of arm/disarm.

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "bus_policy.hpp"
#include "mcp_node.hpp"

namespace sysid {

class Motion {
public:
  /// What the sample task saw this period, for the guards.
  struct Inputs {
    int64_t t_us;
    const std::vector<NodeData> &nodes; ///< same order as the nodes vector
    bool imu_valid;
    float tilt_rate_dps; ///< angular rate perpendicular to gravity
    bool can_bus_ok;     ///< TWAI controller in the error-active state
    uint32_t can_errors; ///< bus errors since boot, arbitration lost excluded (it is normal)
  };

  /// \param set_heartbeat Starts / stops the MIB's CANopen heartbeat (sent only
  ///        while a channel is armed).
  /// \param node_count How many entries of `nodes` exist (it only grows; see
  ///        `nodes <list>` in sysid_main.cpp).
  Motion(std::vector<std::unique_ptr<McpNode>> &nodes, const std::atomic<size_t> &node_count,
         const std::atomic<int> &period_ms,
         const NodeSet &motion_nodes, const NodeSet &read_only_nodes,
         std::function<void(bool)> set_heartbeat);

  /// `arm <node> <ch>`: allow that one channel to be moved for the next
  /// CONFIG_SYSID_ARM_TIMEOUT_S.
  void arm(int node_id, int ch_number);

  /// `pstep <node> <ch> <delta_counts>`.
  /// \param velocity, acceleration CiA 402 profile velocity (0x6081) and
  ///        acceleration = deceleration (0x6083 / 0x6084) written for this move;
  ///        0 = the Kconfig defaults.
  /// \param stream_rate 0 = one step to the target. > 0 = `pstream`: enable the
  ///        drive at the present count, then write a moving target (0x607A) every
  ///        stream_period_ms, advancing at stream_rate counts/s until it reaches
  ///        present + delta. Tests whether the controller follows live target
  ///        updates. \param stream_handshake true = each update also does the CiA 402
  ///        new-set-point handshake; false = the object write alone.
  void pstep(int node_id, int ch_number, int delta, int velocity = 0, int acceleration = 0,
             int stream_rate = 0, int stream_period_ms = 20, int stream_handshake = 0,
             int stream_accel = 0, int stream_lead_ms = 0);

  /// `hb <0|1>`: stop / start the MIB heartbeat by hand, to test the controller's
  /// heartbeat consumer (watchdog) during a move.
  void set_heartbeat_for_test(bool on) { set_heartbeat_(on); }

  /// `vgains <node> <ch> <P> <I> <D> <QPPS>`: rewrite the velocity PID record
  /// (controller RAM only). P, I, D are 16.16 fixed point; QPPS is counts/s at
  /// full output.
  void set_velocity_gains(int node_id, int ch_number, int32_t kp, int32_t ki, int32_t kd,
                          int32_t qpps);

  /// `clamp <node> <ch> <min> <max>`: install the travel window on the controller.
  void set_clamp(int node_id, int ch_number, int32_t new_min, int32_t new_max);

  /// `gains <node> <ch> <P> <I> <D> <MaxI> <deadband>`: rewrite the position PID
  /// record's gains (controller RAM only), keeping the installed clamp.
  void set_gains(int node_id, int ch_number, int32_t kp, int32_t ki, int32_t kd, int32_t max_i,
                 int32_t deadband);

  /// `sdow`: write one of a short allow-list of CiA 402 window objects (see .cpp).
  void write_object(int node_id, uint16_t index, uint8_t sub, int size, int32_t value);

  /// `clearfaults <node>`.
  void clear_faults(int node_id);

  /// Every sample period: run the guards on a running move, expire the arming.
  void check(const Inputs &in);

  /// Any byte arrived on the console.
  void console_byte();

  /// `stop`: end any move, disarm, and run the stop sequence on every channel of
  /// every motion node. Read-only nodes are only reported.
  void stop();

  /// The `_tgt` column: the target of the move running on a channel.
  bool target(size_t node_index, int ch, int32_t &counts) const;

private:
  /// Work for the worker task.
  struct Request {
    enum class Kind : uint8_t { MOVE, STOP_ONE, STOP_ALL } kind;
    uint8_t node_index;
    uint8_t ch; ///< 0 = M1, 1 = M2
    int32_t target;
    char reason[80];
  };

  /// Whether commands may write to this node: a motion node and not read-only.
  bool writable(uint8_t node_id) const {
    return motion_nodes_.test(node_id) && !read_only_.test(node_id);
  }
  /// Index of a node, or node_count_ if there is none.
  size_t index_of(int node_id) const;

  bool armed_locked(int64_t t_us) const { return armed_ && t_us < armed_until_us_; }
  void refuse(const char *what, const std::string &why);
  void trip(const std::string &guard, const std::string &detail);
  /// One line on what the running move did: start, target, final count (median),
  /// peak |duty|, peak current, whether target reached was ever reported.
  std::string move_summary() const;
  void disarm_locked(const std::string &why);
  void set_heartbeat_consumer(McpNode &node, bool enable);
  void post(const Request &request);
  void worker_task();
  void run_move(const Request &request);
  void run_stop(const Request &request);
  bool profile_move(McpNode &node, int ch, int32_t goal, bool nmt_start, std::string &error);

  /// The two windows the controller limits a position target to, and the count.
  struct Windows {
    int32_t present;   ///< median of the last kEncoderWindow polled counts (0x6064 / 0x6864)
    int32_t spread;    ///< max - min of those counts
    int32_t clamp_min; ///< position PID record MinPos (0x203F:6 / 0x2040:6)
    int32_t clamp_max; ///< position PID record MaxPos (0x203F:7 / 0x2040:7)
    int32_t sw_min;    ///< CiA 402 software position limit (0x607D:1 / 0x687D:1)
    int32_t sw_max;    ///< (0x607D:2 / 0x687D:2)
  };
  /// The present count is the median of the last kEncoderWindow polled counts and
  /// must be settled (spread <= SYSID_ENCODER_SETTLED_SPREAD_COUNTS), so a single
  /// outlier or a glitch never becomes a target. The clamp is read twice through
  /// the whole PID record (McpNode::read_clamp_checked); 0x607D by two uploads.
  /// False with `error` set if anything is missing, unsettled or inconsistent.
  static bool read_windows(McpNode &node, int ch, Windows &w, std::string &error);
  /// "clamp [a, b], 0x607D [c, d], present e".
  static std::string describe(const Windows &w);

  std::vector<std::unique_ptr<McpNode>> &nodes_;
  const std::atomic<size_t> &node_count_;
  const std::atomic<int> &period_ms_;
  const NodeSet motion_nodes_;
  const NodeSet read_only_;
  const std::function<void(bool)> set_heartbeat_;
  QueueHandle_t queue_{nullptr};

  mutable std::mutex mutex_;
  bool armed_{false};
  int64_t armed_until_us_{0};
  size_t armed_node_index_{0};
  int armed_ch_{0};

  // The running move, if any. Set when pstep is accepted, before the worker sends
  // the first frame, so the guards watch the whole sequence.
  bool running_{false};
  size_t node_index_{0};
  int ch_{0}; ///< 0 = M1, 1 = M2
  int32_t target_{0};
  int32_t move_velocity_{0};     ///< profile velocity of the accepted move
  int32_t final_target_{0};      ///< where the move ends (== target_ unless streaming)
  int32_t stream_rate_{0};       ///< counts/s; 0 = a single step
  int32_t stream_period_ms_{20};
  int32_t stream_accel_{0};      ///< counts/s^2 for a trapezoid; 0 = constant-speed ramp
  int stream_handshake_{0};      ///< 0 = bare 0x607A write, 1 = espp handshake (polls), 2 = lean handshake
  int32_t stream_lead_ms_{0};    ///< target leads the profile by profile speed x this (cancels the P-loop lag)
  int32_t move_acceleration_{0}; ///< profile acceleration = deceleration of the accepted move
  int32_t start_enc_{0};
  int64_t end_us_{0};
  uint32_t start_can_errors_{0};
  uint32_t start_timeouts_{0};
  // What the move did, for its end event.
  int32_t peak_duty_{0};        ///< largest |duty| seen
  int32_t peak_current_ma_{0};  ///< largest |current| seen
  bool saw_target_reached_{false}; ///< statusword bit 10 seen set during the move

  // Latest guard inputs, so pstep can refuse on them before starting.
  bool have_inputs_{false};
  bool imu_valid_{false};
  float tilt_rate_dps_{0.0f};
  bool can_bus_ok_{false};
};

} // namespace sysid

#endif // CONFIG_SYSID_ENABLE_MOTION
