// Host checks for the Arbiter. The cases follow the arm driver's
// tests/interface/arbiter_test.cpp one for one where the MIB has the same surface,
// plus the races the arm's suite does not cover.

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "arbiter.hpp"

using namespace std::chrono_literals;
using mib::Arbiter;
using mib::ArbitrationMode;
using mib::HaltReason;
using mib::Token;

namespace {

int failures = 0;

#define CHECK(condition)                                                                           \
  do {                                                                                             \
    if (!(condition)) {                                                                            \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                           \
      ++failures;                                                                                  \
    }                                                                                              \
  } while (0)

// Records the halts the Arbiter delivers. Thread-safe, as on_halt must be.
struct Halts {
  std::mutex mutex;
  std::vector<HaltReason> reasons;
  void operator()(HaltReason reason) {
    std::lock_guard<std::mutex> lock(mutex);
    reasons.push_back(reason);
  }
  size_t size() {
    std::lock_guard<std::mutex> lock(mutex);
    return reasons.size();
  }
};

// A halt handler whose FIRST delivery blocks until the test opens the gate, so a
// test can act inside the window where the Arbiter has released its mutex.
struct GatedHalts {
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<HaltReason> reasons;
  bool in_halt = false;
  bool gate_open = false;
  void operator()(HaltReason reason) {
    std::unique_lock<std::mutex> lock(mutex);
    reasons.push_back(reason);
    if (gate_open) {
      return;
    }
    in_halt = true;
    cv.notify_all();
    cv.wait(lock, [this] { return gate_open; });
  }
  bool wait_until_in_halt() {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, 2s, [this] { return in_halt; });
  }
  void open() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      gate_open = true;
    }
    cv.notify_all();
  }
};

Arbiter::Config config(ArbitrationMode mode, Arbiter::halt_fn on_halt = {}) {
  return {.mode = mode,
          .on_halt = std::move(on_halt),
          .seed = 1234,
          .log_level = espp::Logger::Verbosity::NONE};
}

bool run(Arbiter &arbiter, const Token &token, int &delivered) {
  return arbiter.run_if_admitted(token, [&] { ++delivered; });
}

void enforced_rejects_a_command_without_a_grant() {
  Arbiter arbiter(config(ArbitrationMode::ENFORCED));
  int delivered = 0;
  CHECK(!run(arbiter, Token{}, delivered));
  CHECK(delivered == 0);
}

void grant_mints_unique_tokens_and_increments_generation() {
  Arbiter arbiter(config(ArbitrationMode::ENFORCED));
  const auto a = arbiter.grant("a");
  const auto b = arbiter.grant("b");
  CHECK(a.accepted && b.accepted);
  CHECK(a.token != b.token);
  CHECK(a.token != Token{});
  CHECK(a.generation == 1 && b.generation == 2);
}

void the_correct_token_runs_the_command_exactly_once() {
  Arbiter arbiter(config(ArbitrationMode::ENFORCED));
  const auto grant = arbiter.grant("planner");
  int delivered = 0;
  CHECK(run(arbiter, grant.token, delivered));
  CHECK(delivered == 1);
}

void a_wrong_token_is_rejected_and_counted() {
  Arbiter arbiter(config(ArbitrationMode::ENFORCED));
  auto token = arbiter.grant("planner").token;
  token[0] ^= 0xff;
  int delivered = 0;
  CHECK(!run(arbiter, token, delivered));
  CHECK(delivered == 0);
  CHECK(arbiter.status().rejected_count == 1);
}

void a_stale_token_is_rejected_after_a_regrant() {
  Arbiter arbiter(config(ArbitrationMode::ENFORCED));
  const auto old_token = arbiter.grant("zombie").token;
  arbiter.grant("planner");
  int delivered = 0;
  CHECK(!run(arbiter, old_token, delivered));
  CHECK(delivered == 0);
}

void disabled_mode_admits_any_token() {
  Arbiter arbiter(config(ArbitrationMode::DISABLED));
  int delivered = 0;
  CHECK(run(arbiter, Token{}, delivered));
  CHECK(delivered == 1);
}

void revoke_halts_and_rejects_the_old_token() {
  Halts halts;
  Arbiter arbiter(config(ArbitrationMode::ENFORCED, std::ref(halts)));
  const auto token = arbiter.grant("planner").token;
  int delivered = 0;
  CHECK(run(arbiter, token, delivered));
  arbiter.revoke();
  CHECK(halts.reasons == std::vector{HaltReason::OWNERSHIP_REVOKED});
  CHECK(!run(arbiter, token, delivered));
  CHECK(delivered == 1); // the ex-owner cannot keep driving
}

void revoke_with_no_owner_does_not_halt() {
  Halts halts;
  Arbiter arbiter(config(ArbitrationMode::ENFORCED, std::ref(halts)));
  arbiter.revoke();
  CHECK(halts.size() == 0);
}

void a_regrant_halts_with_seized_before_the_new_grant_is_live() {
  Halts halts;
  Arbiter arbiter(config(ArbitrationMode::ENFORCED, std::ref(halts)));
  arbiter.grant("planner");
  CHECK(halts.size() == 0);
  arbiter.grant("joystick");
  CHECK(halts.reasons == std::vector{HaltReason::OWNERSHIP_SEIZED});
  CHECK(arbiter.status().owner_id == "joystick");
}

void release_requires_the_owner_token() {
  Halts halts;
  Arbiter arbiter(config(ArbitrationMode::ENFORCED, std::ref(halts)));
  const auto token = arbiter.grant("planner").token;
  CHECK(!arbiter.release(Token{}));
  CHECK(arbiter.status().owned);
  CHECK(halts.size() == 0);
  CHECK(arbiter.release(token));
  CHECK(!arbiter.status().owned);
  CHECK(halts.reasons == std::vector{HaltReason::OWNERSHIP_REVOKED});
  CHECK(!arbiter.release(token)); // already released
}

void estop_halts_twice_drops_the_grant_and_rejects_everything() {
  Halts halts;
  Arbiter arbiter(config(ArbitrationMode::ENFORCED, std::ref(halts)));
  const auto token = arbiter.grant("planner").token;
  arbiter.estop();
  CHECK(halts.reasons == (std::vector{HaltReason::EMERGENCY_STOP, HaltReason::EMERGENCY_STOP}));
  int delivered = 0;
  CHECK(!run(arbiter, token, delivered));
  CHECK(delivered == 0);
  CHECK(arbiter.status().estopped);
  CHECK(!arbiter.status().owned);
}

void an_estop_clear_inside_the_estop_window_cannot_unlatch_the_stop() {
  GatedHalts halts;
  Arbiter arbiter(config(ArbitrationMode::DISABLED, std::ref(halts)));
  std::thread stopper([&] { arbiter.estop(); });
  CHECK(halts.wait_until_in_halt()); // latched, not yet holding the mutex
  arbiter.estop_clear();             // the mutex is free: this wins the store race
  halts.open();
  stopper.join();
  CHECK(arbiter.status().estopped);
  int delivered = 0;
  CHECK(!run(arbiter, Token{}, delivered)); // a real refusal: DISABLED bypasses tokens
  CHECK(delivered == 0);
}

void estop_latches_and_refuses_a_fresh_grant() {
  Arbiter arbiter(config(ArbitrationMode::ENFORCED));
  arbiter.estop();
  const auto grant = arbiter.grant("planner");
  CHECK(!grant.accepted);
  CHECK(grant.message == "e-stopped");
}

void estop_rejects_even_in_disabled_mode() {
  Arbiter arbiter(config(ArbitrationMode::DISABLED));
  int delivered = 0;
  CHECK(run(arbiter, Token{}, delivered));
  arbiter.estop();
  CHECK(!run(arbiter, Token{}, delivered));
  CHECK(delivered == 1);
}

void estop_clear_exits_to_no_owner() {
  Arbiter arbiter(config(ArbitrationMode::ENFORCED));
  const auto token = arbiter.grant("planner").token;
  arbiter.estop();
  arbiter.estop_clear();
  CHECK(!arbiter.status().estopped);
  CHECK(!arbiter.status().owned);
  int delivered = 0;
  CHECK(!run(arbiter, token, delivered));
  const auto grant = arbiter.grant("planner");
  CHECK(grant.accepted);
  CHECK(run(arbiter, grant.token, delivered));
}

void estop_clear_works_in_disabled_mode() {
  Arbiter arbiter(config(ArbitrationMode::DISABLED));
  arbiter.estop();
  arbiter.estop_clear();
  int delivered = 0;
  CHECK(run(arbiter, Token{}, delivered));
}

void status_reflects_the_owner_and_rejection_count() {
  Arbiter arbiter(config(ArbitrationMode::ENFORCED));
  arbiter.grant("planner");
  int delivered = 0;
  run(arbiter, Token{}, delivered);
  run(arbiter, Token{}, delivered);
  const auto status = arbiter.status();
  CHECK(status.owned);
  CHECK(status.owner_id == "planner");
  CHECK(status.generation == 1);
  CHECK(status.rejected_count == 2);
}

// An orchestrator that shares one key across the arm and the base supplies it.
void grant_adopts_a_presupplied_token() {
  Halts halts;
  Arbiter arbiter(config(ArbitrationMode::ENFORCED, std::ref(halts)));
  Token key{};
  key[3] = 0x5a;
  const auto grant = arbiter.grant("orchestrator", key);
  CHECK(grant.accepted);
  CHECK(grant.token == key);
  CHECK(grant.generation == 1);
  int delivered = 0;
  CHECK(run(arbiter, key, delivered));
  // Re-granting the SAME key is still revoke-then-grant: the base halts.
  const auto again = arbiter.grant("orchestrator", key);
  CHECK(again.accepted);
  CHECK(again.generation == 2);
  CHECK(halts.reasons == std::vector{HaltReason::OWNERSHIP_SEIZED});
}

void grant_refuses_an_all_zero_presupplied_token() {
  Halts halts;
  Arbiter arbiter(config(ArbitrationMode::ENFORCED, std::ref(halts)));
  const auto owner = arbiter.grant("planner").token;
  const auto grant = arbiter.grant("orchestrator", Token{});
  CHECK(!grant.accepted);
  CHECK(grant.token == Token{});
  CHECK(halts.size() == 0);
  CHECK(arbiter.status().owner_id == "planner");
  CHECK(arbiter.status().generation == 1);
  int delivered = 0;
  CHECK(run(arbiter, owner, delivered));
}

// Not in the arm's suite. Grant A seizes from X and halts with the mutex released;
// grant B lands in that window and finds no owner. A must then seize from B, with a
// halt, rather than silently overwrite B's grant.
void two_racing_grants_each_seize_with_a_halt() {
  GatedHalts halts;
  Arbiter arbiter(config(ArbitrationMode::ENFORCED, std::ref(halts)));
  arbiter.grant("x");
  mib::GrantResult a;
  std::thread granter([&] { a = arbiter.grant("a"); });
  CHECK(halts.wait_until_in_halt()); // A is halting X, mutex released
  const auto b = arbiter.grant("b");
  CHECK(b.accepted);
  halts.open();
  granter.join();
  CHECK(a.accepted);
  CHECK(halts.reasons == (std::vector{HaltReason::OWNERSHIP_SEIZED, HaltReason::OWNERSHIP_SEIZED}));
  CHECK(arbiter.status().owner_id == "a");
  int delivered = 0;
  CHECK(!run(arbiter, b.token, delivered)); // B was dispossessed, not left driving
  CHECK(run(arbiter, a.token, delivered));
}

// Not in the arm's suite. estop() must not wait for a command in flight, and its
// second halt must land after that command, so the command cannot restart the base.
void estop_does_not_wait_for_a_command_and_halts_again_after_it() {
  std::mutex mutex;
  std::condition_variable cv;
  bool command_started = false;
  bool release_command = false;
  std::vector<std::string> events;
  auto record = [&](const std::string &event) {
    std::lock_guard<std::mutex> lock(mutex);
    events.push_back(event);
    cv.notify_all();
  };
  Arbiter arbiter(config(ArbitrationMode::DISABLED, [&](HaltReason) { record("halt"); }));
  std::thread commander([&] {
    arbiter.run_if_admitted(Token{}, [&] {
      record("command start");
      std::unique_lock<std::mutex> lock(mutex);
      command_started = true;
      cv.notify_all();
      cv.wait(lock, [&] { return release_command; });
      events.push_back("command end");
    });
  });
  {
    std::unique_lock<std::mutex> lock(mutex);
    CHECK(cv.wait_for(lock, 2s, [&] { return command_started; }));
  }
  std::thread stopper([&] { arbiter.estop(); });
  {
    std::unique_lock<std::mutex> lock(mutex);
    // The first halt arrives while the command still holds the Arbiter's mutex.
    CHECK(cv.wait_for(lock, 2s, [&] { return events.size() == 2; }));
    release_command = true;
    cv.notify_all();
  }
  commander.join();
  stopper.join();
  CHECK(events == (std::vector<std::string>{"command start", "halt", "command end", "halt"}));
}

void seeded_arbiters_mint_repeatable_tokens() {
  Arbiter a(config(ArbitrationMode::ENFORCED));
  Arbiter b(config(ArbitrationMode::ENFORCED));
  CHECK(a.grant("a").token == b.grant("b").token);
}

void unseeded_arbiters_mint_different_tokens() {
  Arbiter a({.seed = 0, .log_level = espp::Logger::Verbosity::NONE});
  Arbiter b({.seed = 0, .log_level = espp::Logger::Verbosity::NONE});
  CHECK(a.grant("a").token != b.grant("b").token);
}

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  const std::pair<const char *, void (*)()> tests[] = {
      {"enforced rejects a command without a grant", enforced_rejects_a_command_without_a_grant},
      {"grant mints unique tokens", grant_mints_unique_tokens_and_increments_generation},
      {"the correct token runs once", the_correct_token_runs_the_command_exactly_once},
      {"a wrong token is rejected and counted", a_wrong_token_is_rejected_and_counted},
      {"a stale token is rejected", a_stale_token_is_rejected_after_a_regrant},
      {"disabled mode admits any token", disabled_mode_admits_any_token},
      {"revoke halts and rejects", revoke_halts_and_rejects_the_old_token},
      {"revoke with no owner does not halt", revoke_with_no_owner_does_not_halt},
      {"a regrant halts with SEIZED", a_regrant_halts_with_seized_before_the_new_grant_is_live},
      {"release requires the owner token", release_requires_the_owner_token},
      {"estop halts twice", estop_halts_twice_drops_the_grant_and_rejects_everything},
      {"estop_clear inside the estop window",
       an_estop_clear_inside_the_estop_window_cannot_unlatch_the_stop},
      {"estop refuses a fresh grant", estop_latches_and_refuses_a_fresh_grant},
      {"estop rejects in disabled mode", estop_rejects_even_in_disabled_mode},
      {"estop_clear exits to no owner", estop_clear_exits_to_no_owner},
      {"estop_clear works in disabled mode", estop_clear_works_in_disabled_mode},
      {"status reflects owner and rejections", status_reflects_the_owner_and_rejection_count},
      {"grant adopts a presupplied token", grant_adopts_a_presupplied_token},
      {"grant refuses an all-zero token", grant_refuses_an_all_zero_presupplied_token},
      {"two racing grants each seize", two_racing_grants_each_seize_with_a_halt},
      {"estop does not wait for a command",
       estop_does_not_wait_for_a_command_and_halts_again_after_it},
      {"seeded tokens repeat", seeded_arbiters_mint_repeatable_tokens},
      {"unseeded tokens differ", unseeded_arbiters_mint_different_tokens},
  };
  for (const auto &[name, test] : tests) {
    const int before = failures;
    test();
    // cppcheck can't see test() bump `failures` through the function pointer.
    // cppcheck-suppress knownConditionTrueFalse
    std::printf("%s %s\n", failures == before ? "PASS" : "FAIL", name);
  }
  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
