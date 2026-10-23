#include <string>

#include "arbiter.hpp"
#include "logger.hpp"

// Walks the token lifecycle against a stand-in for the drive: a counter that
// records admitted commands, and a counter that records halts. The Arbiter never
// learns what either one is, which is the point.

namespace {

std::string hex(const mib::Token &token) {
  static constexpr char digits[] = "0123456789abcdef";
  std::string out;
  for (auto byte : token) {
    out += digits[byte >> 4];
    out += digits[byte & 0x0f];
  }
  return out;
}

const char *name(mib::HaltReason reason) {
  switch (reason) {
  case mib::HaltReason::OWNERSHIP_REVOKED:
    return "OWNERSHIP_REVOKED";
  case mib::HaltReason::OWNERSHIP_SEIZED:
    return "OWNERSHIP_SEIZED";
  case mib::HaltReason::EMERGENCY_STOP:
    return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

} // namespace

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Arbiter Example", .level = espp::Logger::Verbosity::INFO});

  //! [arbiter example]
  int commands_delivered = 0;
  int halts = 0;

  mib::Arbiter arbiter({
      .mode = mib::ArbitrationMode::ENFORCED,
      .on_halt =
          [&](mib::HaltReason reason) {
            ++halts;
            logger.warn("  on_halt({}) -> the drive would stop here", name(reason));
          },
  });

  // A command carrying a token. The lambda is whatever the handler would do.
  auto drive = [&](const char *who, const mib::Token &token) {
    const bool ran = arbiter.run_if_admitted(token, [&] { ++commands_delivered; });
    logger.info("{} command: {}", who, ran ? "ADMITTED" : "rejected");
  };

  auto report = [&] {
    const auto s = arbiter.status();
    logger.info(
        "status: owned={} owner='{}' generation={} estopped={} rejected={} | delivered={} halts={}",
        s.owned, s.owner_id, s.generation, s.estopped, s.rejected_count, commands_delivered, halts);
  };

  logger.info("1. No owner yet: a command with an empty token is refused");
  drive("autonomy", mib::Token{});
  report();

  logger.info("2. Grant to 'autonomy': its commands are admitted");
  auto autonomy = arbiter.grant("autonomy");
  logger.info("   accepted={} generation={} token={}", autonomy.accepted, autonomy.generation,
              hex(autonomy.token));
  drive("autonomy", autonomy.token);
  drive("autonomy", autonomy.token);
  report();

  logger.info("3. Grant to 'joystick' SEIZES: autonomy is halted, its token goes stale");
  auto joystick = arbiter.grant("joystick");
  drive("autonomy", autonomy.token);
  drive("joystick", joystick.token);
  report();

  logger.info("4. Release with the wrong token is refused; with the right one it halts");
  logger.info("   release(autonomy token) -> {}", arbiter.release(autonomy.token));
  logger.info("   release(joystick token) -> {}", arbiter.release(joystick.token));
  drive("joystick", joystick.token);
  report();

  logger.info("5. Revoke needs no token: operator recovery for a crashed owner");
  auto crashed = arbiter.grant("crashed_node");
  arbiter.revoke();
  drive("crashed_node", crashed.token);
  report();

  logger.info("6. E-stop halts twice, latches, and refuses even a fresh grant");
  auto owner = arbiter.grant("autonomy");
  arbiter.estop();
  drive("autonomy", owner.token);
  auto refused = arbiter.grant("autonomy");
  logger.info("   grant during e-stop: accepted={} message='{}'", refused.accepted,
              refused.message);
  report();

  logger.info("7. Clearing the e-stop exits to NO owner; a new grant is needed to drive");
  arbiter.estop_clear();
  drive("autonomy", owner.token);
  owner = arbiter.grant("autonomy");
  drive("autonomy", owner.token);
  report();
  //! [arbiter example]

  logger.info("8. DISABLED mode admits anything, but the e-stop still latches over it");
  mib::Arbiter bypass({.mode = mib::ArbitrationMode::DISABLED});
  logger.info("   empty token: {}",
              bypass.run_if_admitted(mib::Token{}, [] {}) ? "ADMITTED" : "rejected");
  bypass.estop();
  logger.info("   empty token after e-stop: {}",
              bypass.run_if_admitted(mib::Token{}, [] {}) ? "ADMITTED" : "rejected");

  logger.info("9. A seeded Arbiter mints repeatable tokens, for a bench test");
  mib::Arbiter seeded_a({.seed = 42});
  mib::Arbiter seeded_b({.seed = 42});
  logger.info("   same seed, same first token: {}",
              seeded_a.grant("a").token == seeded_b.grant("b").token);
}
