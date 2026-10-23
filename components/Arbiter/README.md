# Arbiter

Decides **who** may command the base, and nothing else.

A port of the arm driver's `kinova::interface::Arbiter`
([kinova-gen3-driver](https://github.com/rammp-org/kinova-gen3-driver/blob/dev/include/kinova_lowlevel/interface/arbiter.h))
to the MIB. The arm and the base run separate arbiters speaking one protocol,
the one in `rammp_common_interfaces`, so an orchestrator or operator tool on
the Jetson uses one client for both resources.

This component is the admission core only. It has no RTPS endpoints and no
knowledge of the system state machine, driving, or the joystick. Those layers
build on it; see [docs/arbitration.md](../../docs/arbitration.md) for the
system design and what comes next.

## The idea in one paragraph

A client that wants to command the base asks for a **grant**. The Arbiter mints
a fresh random 128-bit **token** and hands it back. The client puts that token
on every command it sends. The Arbiter admits a command only if its token
matches the live grant. Nobody else's commands get through, a token from an
earlier grant no longer matches, and the **e-stop** is the special case where
nobody's token matches. Every change of ownership halts the base first, so the
next owner always starts from a stopped base.

## States

| State | Meaning |
|---|---|
| no owner | no live grant; every command is refused in `ENFORCED` mode |
| owned | exactly one owner: token, `owner_id`, `generation` |
| e-stopped | latched over both; every command is refused, `DISABLED` mode included |

A command is admitted iff the e-stop is clear **and** (the mode is `DISABLED`
**or** the token matches the live grant).

## API

| Method | What it does | Halts? |
|---|---|---|
| `grant(owner_id)` | Mints a token for one owner. **Seizes**: a current owner is halted and dispossessed first. Refused while e-stopped. | if someone was owner |
| `release(token)` | Gives control back. Refused unless the token matches the current owner. | yes |
| `revoke()` | Takes control from whoever holds it. Operator recovery; needs no token. | if someone was owner |
| `estop()` | Drops the grant, halts, latches. | yes, twice |
| `estop_clear()` | Clears the latch. Exits to *no owner*, never straight back to owned. | no |
| `status()` | Mode, latch, owner, generation, rejection count. Never gated. | no |
| `run_if_admitted(token, fn)` | Runs `fn` under the mutex if the token is admitted. Counts and debug-logs a rejection. | no |

`generation` bumps on every grant. It is not used for the check, a stale token
already fails to match, but it lets a client notice it was dispossessed and
makes a rejection diagnosable.

## How the rest of the firmware will use it

Each RTPS command handler wraps its work in `run_if_admitted`, and the system
state machine supplies `on_halt`:

```cpp
mib::Arbiter arbiter_({
    .mode = mib::config::arbitration_mode,
    .on_halt = [this](mib::HaltReason reason) {
      drive_controller_.stop();          // every halt stops the chair
      post_event(HaltEvent{reason});     // the state machine decides what follows:
                                         // REVOKED -> IDLE, SEIZED -> stay put
    },
});

void MibSystem::handle_twist(const TwistSetpoint &setpoint) {
  arbiter_.run_if_admitted(setpoint.token, [&] {
    // what the state machine does with an admitted twist
  });
}
```

The Arbiter never sees the state machine. The state machine never sees a token.
That is the boundary: arbitration says who may ask, the state machine says what
an admitted command means right now.

## Lock discipline

Carried over from the arm driver, where it was reviewed and tested. The
reasoning is in the header; the rules are:

- **The mutex is held across the command** inside `run_if_admitted`, so
  admit-and-deliver is atomic against `revoke()` and `estop()`. Without this a
  command admitted a moment before a revoke could land after the halt and
  restart a stopped base.
- **The mutex is not held across `on_halt`** on the ownership paths
  (`grant`, `release`, `revoke`), so a halt handler may take its own locks.
- **`estop()` never waits for the mutex.** It latches an atomic flag first, so
  admission is refused from that instant whoever holds the lock, delivers the
  halt, and only then takes the mutex for bookkeeping. It then delivers the
  halt a second time *under* the mutex, which is what stops a command that had
  already passed `admit()` from landing after the first halt.

Consequences for the `on_halt` callback:

- It **must be idempotent**. An e-stop delivers it twice.
- It **must not call back into the Arbiter**. The second e-stop delivery runs
  under the mutex and would deadlock.
- It should be quick. On the MIB a halt is a planner stop and a state change,
  so this is not a constraint in practice.

## Deviations from the arm's Arbiter

Small and deliberate. Everything else is one-to-one.

- **Admission is a wrapper, not a decorator.** The arm's Arbiter implements
  the driver's sink interfaces and forwards each method. The MIB has three
  commands and its vocabulary is still changing (`TwistSetpoint` is moving to a
  new `rammp_control_interfaces` package), so this port offers
  `run_if_admitted` and lets the handler supply the command. Same atomicity,
  fewer moving parts. The cost is that a handler has to remember to call it.
- **A seize has its own halt reason.** The arm halts with `kOwnershipRevoked` both
  when an owner is revoked and when a new grant seizes the base. Here a seize halts
  with `OWNERSHIP_SEIZED` and a release or revoke with `OWNERSHIP_REVOKED`, because
  the system state machine treats them differently: after a revoke nobody owns the
  base and the stick returns to the GUI; after a seize the chair has stopped and the
  new owner's next command decides what happens.
- **`release(token)` lives in the core.** On the arm the ROS server retains the
  minted token and checks a release against it. The core already holds the
  token, so checking it here avoids a second copy.
- **Tokens come from the hardware RNG** (`esp_fill_random`) unless a seed is
  given, in which case a `std::mt19937_64` makes them repeatable for a bench
  test. The arm seeds its generator from `std::random_device`.
- **Enum values are `UPPER_CASE`** (`ENFORCED`, `OWNERSHIP_REVOKED`) to match
  this repo, where the arm uses `kEnforced`, `kOwnershipRevoked`.
- **Rejections are logged** at debug level as well as counted. The arm's core
  has no logger and leaves logging to its transport.

Like the arm's, this is a **mistake boundary, not a security boundary**.
Anything on the bus can observe a token. It keeps two well-behaved systems
from fighting over the base; it does not defend against a hostile one. It is
also a software e-stop, not the hardware chain.

## Example

`example/` is a standalone ESP-IDF project that walks the lifecycle with a
counter standing in for the drive: refusal with no owner, grant, seize, release
with the wrong and right token, revoke, e-stop and its double halt, clear, the
`DISABLED` bypass, and seeded tokens.

```console
cd components/Arbiter/example
idf.py build flash monitor
```
