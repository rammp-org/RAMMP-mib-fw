#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <system_error>

#include "actuator_store.hpp"
#include "base_component.hpp"
#include "canopen_client.hpp"
#include "mcp266.hpp"
#include "motion_profile.hpp"

namespace mib {

/// @brief One positioned axis of a Basicmicro MCP266: a leg's linear actuator.
///
/// How the controller behaves, as measured on the chair, and why this class drives it the
/// way it does, is in DESIGN.md (evidence: docs/actuator-revamp/bench-findings.md). In short:
/// - The MCP266 runs a cascade: position error x position P is a speed command, and a
///   velocity PI turns speed error into PWM. It applies no ramp to a target, so the MIB
///   streams one: a trapezoid generated here (MotionProfile), written as the position target
///   every other tick with a new-set-point handshake, led ahead of the profile by
///   speed x (1 / position P) so the leg tracks it instead of lagging.
/// - Inside the CiA 402 position window (0x6067) the controller stops commanding speed.
///   Its stored 100 counts made legs stop ~90 short; initialize() sets the leg's window.
/// - initialize() writes the controller settings from the leg's config to controller RAM
///   (never to its flash) and reads every one back; a mismatch leaves the axis not ready.
/// - A move ends when the profile has finished and the median count has stayed within
///   `tolerance` of the target for `settle_time`. Then the drive is released (disable
///   voltage): the EHAs cannot be back-driven, and a held loop pushes ~5 % PWM / ~5 A
///   against the load for nothing. A leg that has not arrived by the profile time plus
///   `arrival_margin` is stopped and released, and the move is reported FAILED with how
///   far short it is.
/// - Faults are never cleared as a side effect: a faulted drive refuses moves until
///   clear_faults() is called.
///
/// Positions are encoder counts, always. Engineering units and kinematics belong above the
/// BSP; this class does not know which way a leg's stroke goes for positive counts (that is
/// recorded in MIBconfig.hpp).
///
/// **Ticks.** poll() is the move engine. The owner calls it every kTickPeriod (10 ms) from
/// one task that may block on CAN, with a fixed-period schedule (deadline += period, not
/// sleep(period) after the work), the same way MibSystem runs its IMU task. It is cheap when
/// idle (no bus traffic) and a few SDOs while moving. Two actuators on one controller may be
/// polled from the same task. The other methods may be called from any task that may block.
///
/// **Commissioned.** A leg whose `commissioned` is false is never written to: initialize()
/// sends nothing, every move, jog, homing, calibration and stop() is refused or a no-op, and
/// only the explicit reads (read_position, read_status, read_controller_settings) reach its
/// controller. Commissioning a leg is a config change, not a runtime one.
///
/// Two actuators can share one Mcp266 (its M1 and M2 channels), and must then be given the
/// same CanopenClient and the same mutex: the client serialises single SDOs, but sequences
/// (a move start, the settings, a tick) must not interleave on one controller.
class Actuator : public espp::BaseComponent {
public:
  using Axis = espp::Mcp266::Axis;
  using DriveState = espp::Ds402Drive::State;
  using Range = ActuatorStore::Range;

  /// The period poll() is designed for. Each new-set-point bit level is held one tick
  /// (a 1 ms pulse was missed for a whole move on the bench), so the target is updated at
  /// half this rate, 50 Hz.
  static constexpr std::chrono::milliseconds kTickPeriod{10};

  /// A move's trapezoid. One acceleration, used to speed up and to slow down.
  struct Profile {
    uint32_t velocity;     ///< Cruise speed, counts per second.
    uint32_t acceleration; ///< Counts per second squared.
  };

  /// What initialize() writes to the controller's RAM and reads back. Raw controller units,
  /// as Motion Studio and the bench logs show them.
  struct ControllerSettings {
    uint32_t position_window;   ///< CiA 402 0x6067, counts: no speed command inside it.
    uint32_t position_p;        ///< Position PID, x1024 (4096 = 4.0: counts/s per count).
    uint32_t position_i;        ///< x1024.
    uint32_t position_d;        ///< x1024.
    uint32_t position_max_i;    ///< Integral windup limit.
    uint32_t position_deadband; ///< Counts.
    uint32_t velocity_p;        ///< Velocity PID, 16.16 (65536 = 1.0).
    uint32_t velocity_i;        ///< 16.16.
    uint32_t velocity_d;        ///< 16.16.
    uint32_t velocity_qpps;     ///< Counts/s the velocity loop takes as full output.
    /// CiA 402 0x6081. It caps the controller's speed command, so it is set well above any
    /// streamed speed (the bench used 20000); the stream sets the speed.
    uint32_t profile_velocity;
    /// CiA 301 0x1016:1, the controller's heartbeat consumer: (producer node id << 16) |
    /// timeout ms. 0 leaves it alone. Node-wide: both channels of the controller share it.
    uint32_t heartbeat_consumer;
  };

  /// What read_controller_settings() finds on the controller.
  struct SettingsReadback {
    ControllerSettings settings;
    Range clamp;          ///< Position PID record MinPos / MaxPos.
    Range software_limit; ///< CiA 402 0x607D.
  };

  /// How to home an incremental-encoder joint against its limit switch.
  struct Homing {
    bool required{false};       ///< True for an incremental (AB) encoder.
    int direction{-1};          ///< +1 or -1: which way the home switch lies.
    int32_t home_count{0};      ///< Count installed once the switch has stopped the motor.
    Profile profile{500, 1000}; ///< Approach speed and acceleration (streamed).
    int32_t stall_counts{20};   ///< Median movement below this over stall_time = stopped.
    std::chrono::milliseconds stall_time{500};
    std::chrono::seconds timeout{60};
    int32_t segment{50000}; ///< Approach length per streamed move.
    /// Polled during the approach; returning true aborts homing with a stop.
    std::function<bool()> cancel{};
  };

  struct Config {
    const char *name{}; ///< Short name for logs and NVS keys (11 characters max).
    Axis axis{};        ///< Which channel of the shared controller.
    /// False: this firmware never writes to the channel (see the class comment).
    bool commissioned{};
    Range range{};     ///< Travel in counts: the controller's clamp; targets clamped.
    Profile profile{}; ///< Default speed and acceleration of a move.
    std::chrono::milliseconds lead{}; ///< Target lead, 1 / position P (250 ms for P 4.0).
    ControllerSettings controller{};  ///< Written at initialize().
    int32_t jog_step{};               ///< Counts moved by increment() and decrement().
    /// Counts from the target within which the median count must stay for settle_time
    /// for the leg to have arrived. Encoder noise is about +-30 counts.
    int32_t tolerance{};
    /// Zero-vibration input shaper, ms; 0 = off. The streamed target is the mean of the
    /// profile now and the profile this long ago, which cancels a resonance whose half
    /// period is this delay (the left main leg's ~3.7 Hz chassis mode: 135 ms). Moves
    /// take this much longer.
    std::chrono::milliseconds shaper_delay{0};
    std::chrono::milliseconds settle_time{200};
    /// A move not arrived this long after its profile finished is stopped and FAILED.
    std::chrono::milliseconds arrival_margin{2000};
    /// A fresh move starts from the median of kMedianSamples reads, which must spread less
    /// than this (a glitching encoder is refused, not followed).
    int32_t max_start_spread{200};
    bool hardware_limits{false};   ///< Limit switches are wired to the controller.
    Homing homing{};               ///< Leave required=false for an absolute encoder.
    ActuatorStore *store{nullptr}; ///< Where ranges and positions persist; may be null.
    std::chrono::milliseconds offline_retry{2000}; ///< How long to skip a silent controller.
    std::chrono::milliseconds position_save_interval{5000}; ///< Throttle for save_position().
    int32_t position_save_threshold{20};                    ///< Counts moved before saving.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  /// What one move did. move_result() returns the latest; `serial` tells moves apart.
  enum class Outcome : uint8_t {
    NONE,    ///< No move yet.
    MOVING,  ///< Starting, streaming or settling.
    ARRIVED, ///< Settled within tolerance and released.
    FAILED,  ///< Stopped and released short of the target; `reason` says why.
    STOPPED, ///< Ended by stop().
  };
  struct MoveResult {
    Outcome outcome{Outcome::NONE};
    uint32_t serial{0};
    int32_t start{0};       ///< Median count the move started from.
    int32_t target{0};      ///< Where it was going (after clamping).
    int32_t final_count{0}; ///< Median count when it ended (ARRIVED / FAILED / STOPPED).
    /// Counts the median of the last 200 ms of reads moved per second, the largest seen.
    float peak_speed{0};
    float profile_s{0};     ///< Planned profile duration, seconds (of the last plan).
    float elapsed_s{0};     ///< From the first frame of the move to its end.
    float release_s{0};     ///< From the end of the profile to the release.
    int16_t release_pwm{0}; ///< PWM read after the release (+-32767 = +-100 %).
    int32_t release_current_ma{0};
    bool release_readback{false}; ///< release_pwm / _current_ma were read.
    uint32_t sdo_failures{0};     ///< Failed exchanges tolerated during the move.
    char reason[96]{};            ///< Why it FAILED.
  };

  /// One read of the drive's output and state (read_status()).
  struct Status {
    int32_t count{};      ///< One encoder read (not a median).
    int16_t pwm{};        ///< +-32767 = +-100 %.
    int32_t current_ma{}; ///< Motor current.
    uint16_t statusword{};
    DriveState state{};
    bool fault{}; ///< Statusword bit 3.
  };

  /// Reads per median, and so the window of encoder history: 9 x 10 ms.
  static constexpr size_t kMedianSamples = 9;

  /// @param client The controller's CANopen client (shared by both channels).
  /// @param mcp The controller this axis lives on. Must outlive the actuator.
  /// @param mcp_mutex Shared by every actuator on the same controller.
  Actuator(espp::CanopenClient &client, espp::Mcp266 &mcp, std::mutex &mcp_mutex,
           const Config &config);

  /// @name Lifecycle
  /// @{

  /// NMT-start the controller, write the controller settings, the travel clamp and
  /// software limits, and read them all back. Controller RAM only: the MCP266 reverts to
  /// its flash at power-up, so this is once per boot (and again if a move finds the
  /// settings gone). Does not clear a fault (it logs one). An incremental-encoder joint
  /// also tries restore_position(). Sends nothing for a leg that is not commissioned.
  bool initialize();

  /// Commissioned, initialised with every setting read back as written, and homed where
  /// homing is required. Moves are refused otherwise.
  bool ready() const;
  bool initialized() const;
  bool commissioned() const { return config_.commissioned; }
  bool online() const;
  /// @}

  /// @name Position
  /// @{

  /// One encoder read. Refreshes current_position(). Noisy (+-30 counts): decisions use
  /// read_position_median().
  bool read_position(int32_t &counts);

  /// kMedianSamples reads back to back: the median and the spread (max - min).
  bool read_position_median(int32_t &median, int32_t &spread);

  /// The latest reading from any bus exchange, without one, and its age.
  std::optional<int32_t> current_position() const;
  std::chrono::milliseconds position_age() const;

  /// The last count this actuator was sent to, for layers that queue moves.
  std::optional<int32_t> last_target() const;
  /// @}

  /// @name Moves. All clamp to the range (unless calibrating) and use the configured
  ///       profile unless one is given. A move asked for while one is in progress continues
  ///       smoothly from the profile's position and speed. Each returns once the move is
  ///       accepted; poll() runs it, and move_result() reports how it ended.
  /// @{
  bool move_absolute(int32_t counts);
  bool move_absolute(int32_t counts, const Profile &profile);
  /// Relative to where the joint is now (median read), or, mid-move, to the move's target.
  bool move_relative(int32_t delta);
  bool move_relative(int32_t delta, const Profile &profile);
  bool increment(); ///< One jog step, in counts.
  bool decrement();

  /// Disable voltage, now: ends any move, power stage off (the leg stays put; the EHAs
  /// cannot be back-driven). Retried, since a stop sent once and lost is not a stop. Then
  /// read_status() shows PWM and current. A no-op on a leg that is not commissioned (this
  /// firmware never enabled it). A drive in FAULT REACTION ACTIVE ignores it (DESIGN.md).
  bool stop();

  /// The move engine, every kTickPeriod (see the class comment). False on a bus failure.
  bool poll();

  /// True from an accepted move until it has ended (arrived, failed or stopped).
  bool moving() const;
  MoveResult move_result() const;
  /// @}

  /// @name Drive state and faults
  /// @{

  /// Encoder count, PWM, current and statusword, one read each.
  bool read_status(Status &status);
  bool get_drive_state(DriveState &state);
  bool read_statusword(uint16_t &statusword);
  /// Statusword bit 11; set on this controller while a target or count crosses the clamp.
  bool limit_active(bool &active);

  /// CiA 402 fault reset of this channel (controlword 0, then 0x80). The only way a fault
  /// is cleared: nothing else in this class does it. A drive still in FAULT REACTION
  /// ACTIVE (a heartbeat-watchdog trip holds it there ~11 s) may not take it yet.
  bool clear_faults();
  /// @}

  /// @name Controller settings
  /// @{
  const ControllerSettings &controller_settings() const { return config_.controller; }
  /// Read every setting initialize() writes, from the controller (about 20 SDOs).
  bool read_controller_settings(SettingsReadback &readback);
  /// The MinPos/MaxPos clamp the controller currently holds (the position PID record).
  bool read_controller_limits(int32_t &min_counts, int32_t &max_counts);
  /// @}

  /// @name Calibration: absolute encoders, once per installation.
  /// @{

  /// Lift or restore the range clamp on the controller so the axis can be jogged to its
  /// mechanical ends, and let an unhomed incremental axis be jogged. Only jogs are accepted
  /// in this mode. Off at every boot; nothing enters it but an explicit call.
  bool set_calibration_mode(bool enabled);
  bool calibration_mode() const;

  /// Change the input shaper delay (Config::shaper_delay) until the next boot; 0 = off.
  void set_shaper_delay(std::chrono::milliseconds delay);
  std::chrono::milliseconds shaper_delay() const;

  /// Install a calibrated range: on the controller if initialised, and in the store.
  bool set_range(const Range &range);
  Range range() const;

  /// Clamp a count to the range (identity in calibration mode or while homing).
  int32_t clamp(int32_t counts) const;
  /// @}

  /// @name Homing: incremental encoders, every boot (never automatic).
  /// @{

  /// Stream toward the home switch at the homing profile until the count stops (the switch
  /// stopped the motor), stop, install homing.home_count as the encoder count, restore the
  /// clamp and save the position. A drive that faulted is not the switch: homing fails and
  /// the fault is left for clear_faults(). Blocks for the approach (up to homing.timeout);
  /// the owner's poll() must keep running meanwhile.
  bool home();
  bool homed() const;

  /// Install the position saved by save_position() as the encoder count, so the joint is
  /// homed without moving. Valid only if the joint did not move while powered off.
  bool restore_position();

  /// Persist the current position for restore_position(). Throttled unless forced.
  bool save_position(bool force = false);

  /// Write the controller's encoder count (Basicmicro command 22/23). Disables the drive
  /// first, so the loop cannot chase its old target.
  bool set_encoder(int32_t counts);
  /// @}

  const Config &config() const { return config_; }
  const char *name() const { return config_.name; }

private:
  enum class Phase : uint8_t {
    IDLE,
    ACQUIRE,  ///< Reading the encoder kMedianSamples times for the start count.
    STREAM,   ///< Streaming the profile; then waiting for the median to settle.
    RELEASED, ///< Disabled; PWM and current are read into the result shortly after.
  };
  enum class Handshake : uint8_t { SET, CLEAR };

  bool check_online();
  void note_result(bool ok, const std::error_code &ec, const char *what);
  /// Refuse with a log line if the leg is not commissioned.
  bool require_commissioned(const char *what) const;
  /// Write the settings, clamp and limits and read them back. Caller holds mcp_mutex_.
  bool write_settings_locked(const Range &clamp, std::error_code &ec, bool &matched);
  bool read_settings_locked(SettingsReadback &readback, std::error_code &ec);
  /// Rewrite the position record with a new clamp (and 0x607D), read back. Holds the mutex.
  bool apply_clamp(const Range &clamp);
  /// Compare the controller's position PID record with the config, to catch a controller
  /// that reset (it comes back on its flash settings). Caller holds mcp_mutex_.
  bool settings_intact_locked(std::error_code &ec);
  Range installed_limits() const;
  bool do_move(int32_t counts, const Profile &profile, bool jog);
  /// Start a move: validate, then hand it to poll() (fresh) or replan in flight.
  bool start_move(int32_t target, const Profile &profile);
  /// Disable voltage, retried. Caller holds mcp_mutex_.
  bool disable_locked(std::error_code &ec);
  /// Mode, target pre-write, enable (refuses a faulted drive), profile velocity.
  bool enable_locked(int32_t start, std::error_code &ec);
  /// End the move in progress with `outcome` and release the drive; a later tick reads
  /// PWM and current back into the result. Caller holds mcp_mutex_, not state_mutex_.
  void finish_locked(Outcome outcome, const char *reason);
  /// One poll() tick of a move. Caller holds mcp_mutex_.
  bool tick_locked();
  void push_count(int32_t counts, std::chrono::steady_clock::time_point at);
  std::optional<int32_t> history_median() const; ///< Caller holds state_mutex_.
  bool read_status_locked(Status &status, std::error_code &ec);
  bool read_position_locked(int32_t &counts, std::error_code &ec);
  uint16_t object(uint16_t m1_index) const; ///< 0x6xxx object of this channel.

  espp::CanopenClient &client_;
  espp::Mcp266 &mcp_;
  std::mutex &mcp_mutex_;
  Config config_;

  mutable std::mutex state_mutex_; ///< Guards everything below. Taken after mcp_mutex_.
  Range range_;
  std::optional<int32_t> current_position_;
  std::chrono::steady_clock::time_point position_time_{};
  std::optional<int32_t> last_target_;

  // The move in progress.
  Phase phase_{Phase::IDLE};
  int32_t target_{0};
  Profile move_profile_{};
  MotionProfile profile_;
  std::chrono::steady_clock::time_point profile_t0_{};
  std::chrono::steady_clock::time_point move_t0_{};
  std::chrono::steady_clock::time_point last_handshake_{};
  Handshake handshake_{Handshake::SET};
  bool final_sent_{false};    ///< The last SET carried the final target.
  bool final_latched_{false}; ///< ...and its CLEAR went through: nothing left to stream.
  std::optional<std::chrono::steady_clock::time_point> settled_since_;
  std::optional<std::chrono::steady_clock::time_point> profile_done_at_;
  std::chrono::steady_clock::time_point last_good_read_{};
  std::chrono::steady_clock::time_point released_at_{};
  uint32_t consecutive_failures_{0};
  uint32_t ticks_{0};
  MoveResult result_{};

  // Encoder history (every tick of a move): for the median and the speed estimate.
  struct Count {
    int32_t value{};
    std::chrono::steady_clock::time_point at{};
  };
  std::array<Count, kMedianSamples> history_{};
  size_t history_len_{0};
  size_t history_next_{0};
  // Medians, every tick, for the speed estimate over the last 200 ms.
  std::array<Count, 21> medians_{};
  size_t medians_len_{0};
  size_t medians_next_{0};

  int32_t last_saved_position_{0};
  std::chrono::steady_clock::time_point last_save_time_{};
  bool initialized_{false};
  bool settings_ok_{false};
  bool homed_{false};
  bool homing_{false};
  bool calibration_mode_{false};
  std::chrono::milliseconds shaper_delay_{0}; ///< Config::shaper_delay, changeable at run time.
  bool online_{true};
  std::chrono::steady_clock::time_point offline_since_{};
};

/// The name of a drive state, for logs and consoles.
const char *to_string(Actuator::DriveState state);
/// The name of a move outcome.
const char *to_string(Actuator::Outcome outcome);

} // namespace mib
