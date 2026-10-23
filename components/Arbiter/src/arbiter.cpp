#include "arbiter.hpp"

#include <cstring>

#include "esp_random.h"

namespace mib {

Arbiter::Arbiter(const Config &config)
    : BaseComponent("Arbiter", config.log_level)
    , mode_(config.mode)
    , on_halt_(config.on_halt)
    , use_hardware_rng_(config.seed == 0)
    , rng_(config.seed) {
  logger_.info("Arbiter ready: mode={}, no owner",
               mode_ == ArbitrationMode::ENFORCED ? "ENFORCED" : "DISABLED");
}

Token Arbiter::mint() {
  Token token{};
  if (use_hardware_rng_) {
    esp_fill_random(token.data(), token.size());
    return token;
  }
  const uint64_t a = rng_();
  const uint64_t b = rng_();
  std::memcpy(token.data(), &a, sizeof(a));
  std::memcpy(token.data() + sizeof(a), &b, sizeof(b));
  return token;
}

// Admitted iff the e-stop is clear, and either the bypass is on or the command
// carries the live token. The e-stop latch is the one thing DISABLED does not bypass.
bool Arbiter::admit(const Token &token) const {
  if (estopped_.load(std::memory_order_acquire)) {
    return false;
  }
  if (mode_ == ArbitrationMode::DISABLED) {
    return true;
  }
  return owned_ && token == token_;
}

void Arbiter::halt(HaltReason reason) {
  if (on_halt_) {
    on_halt_(reason);
  }
}

GrantResult Arbiter::grant(const std::string &owner_id) {
  bool need_halt = false;
  std::string previous_owner;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (estopped_.load(std::memory_order_acquire)) {
      logger_.warn("Grant to '{}' refused: e-stopped", owner_id);
      return {false, Token{}, generation_, "e-stopped"};
    }
    // A re-grant is revoke-then-grant, never a silent swap under a moving base.
    if (owned_) {
      owned_ = false;
      token_ = Token{};
      previous_owner = owner_id_;
      owner_id_.clear();
      need_halt = true;
    }
  }

  if (need_halt) {
    // Loud on purpose. In a high-trust system an unexpected seizure is exactly the
    // mistake arbitration exists to catch.
    logger_.warn("Control SEIZED: '{}' is taking the base from '{}'; halting", owner_id,
                 previous_owner);
    halt(HaltReason::OWNERSHIP_SEIZED); // mutex NOT held
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (estopped_.load(std::memory_order_acquire)) {
    // Raced an estop() during the halt window above.
    logger_.warn("Grant to '{}' refused: e-stopped during handover", owner_id);
    return {false, Token{}, generation_, "e-stopped"};
  }
  token_ = mint();
  owned_ = true;
  owner_id_ = owner_id;
  ++generation_;
  logger_.info("Control granted to '{}' (generation {})", owner_id, generation_);
  return {true, token_, generation_, ""};
}

bool Arbiter::release(const Token &token) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!owned_ || token != token_) {
      logger_.warn("Release refused: token does not match the current owner");
      return false;
    }
    logger_.info("Control released by '{}'", owner_id_);
    owned_ = false;
    token_ = Token{};
    owner_id_.clear();
  }
  halt(HaltReason::OWNERSHIP_REVOKED); // mutex NOT held
  return true;
}

void Arbiter::revoke() {
  bool need_halt = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (owned_) {
      logger_.warn("Control REVOKED from '{}'", owner_id_);
      owned_ = false;
      token_ = Token{};
      owner_id_.clear();
      need_halt = true;
    }
  }
  if (need_halt) {
    halt(HaltReason::OWNERSHIP_REVOKED); // mutex NOT held
  }
}

// The ONE path that never waits for the mutex. A command inside run_if_admitted()
// holds it, and an e-stop that queues behind a command is not an e-stop. So:
//   1. latch estopped_ (atomic, release): admit() refuses from this instant, no
//      matter who holds the mutex;
//   2. deliver the halt immediately, still without the mutex;
//   3. only then take the mutex for the ownership bookkeeping, which is not
//      safety-critical because step 1 already refuses every command;
//   4. re-latch and re-deliver the halt under the mutex. See the two comments inside.
void Arbiter::estop() {
  estopped_.store(true, std::memory_order_release);
  logger_.error("E-STOP engaged");
  halt(HaltReason::EMERGENCY_STOP); // unconditional: an e-stop always halts

  std::lock_guard<std::mutex> lock(mutex_);
  // Re-latch UNDER the mutex. The early store races an estop_clear() that already
  // holds the mutex, which would otherwise clear the latch AFTER estop() set it and
  // leave estop() returning to a non-estopped arbiter. The last writer under the
  // mutex must be the stop.
  estopped_.store(true, std::memory_order_release);
  owned_ = false;
  token_ = Token{};
  owner_id_.clear();
  // Re-deliver the halt now that the mutex is held. The first delivery could not
  // see a command that had ALREADY passed admit() and was mid-delivery under the
  // mutex: that command could land after the first halt and move a stopped base.
  // Holding the mutex makes this delivery strictly later than every such command.
  // on_halt must therefore be idempotent.
  halt(HaltReason::EMERGENCY_STOP);
}

void Arbiter::estop_clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Cleared UNDER the mutex, unlike the latch: re-enabling the base may wait its turn.
  estopped_.store(false, std::memory_order_release);
  logger_.warn("E-stop cleared; the base has no owner until someone grants again");
}

ArbitrationStatus Arbiter::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {mode_,    estopped_.load(std::memory_order_acquire), owned_, owner_id_, generation_,
          rejected_};
}

} // namespace mib
