#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <random>
#include <string>
#include <utility>

#include "base_component.hpp"

namespace mib {

/// @brief A capability token: 128 random bits minted by the Arbiter on every grant.
///
/// Plain data, so it copies straight into a message field and compares byte for byte.
/// It is the same shape as `uint8[16] token` in rammp_common_interfaces and
/// `kinova::interface::Token` in the arm driver, so one client can hold a base token
/// and an arm token side by side.
using Token = std::array<uint8_t, 16>;

/// @brief Whether commands must carry the live token.
enum class ArbitrationMode : uint8_t {
  ENFORCED, ///< A command is admitted only if it carries the live token.
  DISABLED, ///< Every command is admitted. For bring-up without an orchestrator.
};

/// @brief Why the base must stop. The Arbiter says why; the halt handler decides how.
enum class HaltReason : uint8_t {
  OWNERSHIP_REVOKED, ///< Nobody owns the base now: the owner released, or was revoked.
  OWNERSHIP_SEIZED,  ///< A new grant took the base from its owner; the new owner acts next.
  EMERGENCY_STOP,    ///< The software e-stop was engaged.
};

/// @brief Result of a grant request. Mirrors the AcquireControl service response.
struct GrantResult {
  bool accepted{false};
  Token token{};          ///< The capability; put it on every command to the base.
  uint64_t generation{0}; ///< Bumps on every grant, so a rejection is diagnosable.
  std::string message{};  ///< Why the grant was refused; empty on success.
};

/// @brief Who may command the base right now. Mirrors the ControlStatus message.
/// @note Deliberately carries no token. Publishing a capability on a status topic
///       would defeat it.
struct ArbitrationStatus {
  ArbitrationMode mode{ArbitrationMode::ENFORCED};
  bool estopped{false};       ///< Latched; nothing is admitted until estop_clear().
  bool owned{false};          ///< Does anyone hold the base?
  std::string owner_id{};     ///< Empty if nobody does.
  uint64_t generation{0};     ///< Bumps on every grant.
  uint64_t rejected_count{0}; ///< Commands refused since start.
};

/// @brief Decides WHO may command the base, and nothing else.
///
/// A port of the arm driver's `kinova::interface::Arbiter` to the MIB. The protocol
/// is the one in rammp_common_interfaces, shared by every arbitrated resource on
/// the robot:
///
/// - grant() mints a fresh random token and hands it to one owner. A grant SEIZES:
///   if someone else holds the base, they are halted first and then dispossessed.
/// - A command is admitted only if it carries the live token (or the mode is
///   DISABLED). A stale token from an earlier grant simply does not match.
/// - release(), revoke(), and a new grant() all halt the base before ownership
///   changes hands, so the next owner always starts from a stopped base.
/// - estop() drops the grant, halts, and LATCHES. Nothing is admitted, DISABLED
///   mode included, until estop_clear(), which exits to "no owner" and never
///   straight back to owned.
///
/// The Arbiter knows nothing about driving. It does not know what a command is,
/// what state the base is in, or what a halt does. The caller wraps each command
/// handler in run_if_admitted(), and supplies an `on_halt` callback that stops
/// the base however the system state machine sees fit. That is what keeps it
/// separate from MibSystem's state machine, and testable in its example with
/// nothing but a counter standing in for the drive.
///
/// Lock discipline, carried over from the arm driver:
///
/// - The mutex IS held across the command inside run_if_admitted(), so
///   admit-and-deliver is atomic against revoke() and estop(). Otherwise a
///   command admitted a moment before a revoke could land AFTER the halt and
///   restart a stopped base.
/// - The mutex is NOT held across on_halt() on the ownership paths, so a halt
///   handler may take its own locks freely.
/// - estop() never waits for the mutex. It latches an atomic flag first, so
///   admission is refused from that instant whoever holds the lock, then
///   delivers the halt, and only then takes the mutex for bookkeeping. It then
///   delivers the halt a second time UNDER the mutex, which is what stops a
///   command admitted just before the latch from landing after the first halt.
///   So `on_halt` must be idempotent, and it must never call back into the
///   Arbiter.
///
/// What is NOT here, on purpose: the RTPS services and status topic (the
/// "server" face), the e-stop clear age check, and the in-firmware joystick
/// client. Those build on this class; see components/Arbiter/README.md.
class Arbiter : public espp::BaseComponent {
public:
  /// Called whenever the base must stop. See the class notes for the rules.
  using halt_fn = std::function<void(HaltReason)>;

  /// @brief Arbiter configuration.
  struct Config {
    ArbitrationMode mode{ArbitrationMode::ENFORCED}; ///< Fixed for the Arbiter's lifetime.
    halt_fn on_halt{};                               ///< Stops the base. May be empty.
    /// 0 mints tokens from the hardware RNG. Any other value seeds a deterministic
    /// generator instead, for a bench test that wants repeatable tokens.
    uint64_t seed{0};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO}; ///< Log verbosity.
  };

  /// @brief Construct an Arbiter with no owner and the e-stop clear.
  /// @param config Mode, halt callback, and token seed.
  explicit Arbiter(const Config &config);

  /// @brief Grant control of the base to one owner, minting a fresh token.
  /// @param owner_id Human-readable label, reported in status().
  /// @return The token and generation on success; accepted=false with a message
  ///         if the e-stop is latched.
  /// @note SEIZES. A current owner is halted (on_halt with OWNERSHIP_SEIZED) and
  ///       dispossessed first. A re-grant is never a silent swap under a moving base.
  GrantResult grant(const std::string &owner_id);

  /// @brief Release control you hold.
  /// @param token The token from your grant.
  /// @return True if the token matched the current owner and control was released,
  ///         halting the base. False, with the reason logged, otherwise.
  bool release(const Token &token);

  /// @brief Take the base from whoever holds it. Operator recovery; needs no token.
  /// @note Halts the base if it was owned. A no-op if it was not.
  void revoke();

  /// @brief Engage the software e-stop: drop the grant, halt, and latch.
  /// @note Never waits for the mutex. Delivers on_halt twice; see the class notes.
  void estop();

  /// @brief Clear the software e-stop.
  /// @note Exits to "no owner". Someone must deliberately grant() again.
  void estop_clear();

  /// @brief Snapshot who may command the base.
  /// @return Mode, e-stop latch, owner, generation, and rejection count.
  ArbitrationStatus status() const;

  /// @brief Run a command if its token is admitted, holding the mutex across it.
  /// @param token The token the command carries.
  /// @param command The command to run. Must not call back into the Arbiter.
  /// @return True if the command ran. False if it was refused, in which case the
  ///         rejection is counted in status() and logged at debug level.
  /// @note Admitted iff the e-stop is clear AND (mode is DISABLED OR the token
  ///       matches the live grant).
  template <typename Command> bool run_if_admitted(const Token &token, Command &&command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!admit(token)) {
      ++rejected_;
      logger_.debug("Command rejected: estopped={} owned={} generation={} rejected={}",
                    estopped_.load(std::memory_order_acquire), owned_, generation_, rejected_);
      return false;
    }
    std::forward<Command>(command)();
    return true;
  }

private:
  bool admit(const Token &token) const; ///< Caller holds mutex_.
  Token mint();                         ///< Caller holds mutex_.
  void halt(HaltReason reason);         ///< Delivers on_halt if one was configured.

  const ArbitrationMode mode_;
  const halt_fn on_halt_;
  mutable std::mutex mutex_;
  const bool use_hardware_rng_;
  std::mt19937_64 rng_;
  /// Atomic and deliberately readable WITHOUT mutex_. estop() latches this before
  /// it contends for the mutex, so admission is refused from that instant no
  /// matter who holds the lock.
  std::atomic<bool> estopped_{false};
  bool owned_{false};
  Token token_{};
  std::string owner_id_{};
  uint64_t generation_{0};
  uint64_t rejected_{0};
};

} // namespace mib
