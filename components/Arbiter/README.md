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
| `grant(owner_id[, token])` | Mints a token for one owner, or adopts a non-zero `token`. **Seizes**: a current owner is halted and dispossessed first. Refused while e-stopped. | if someone was owner |
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

Carried over from the arm driver. The rules for `on_halt` are in the class
comment in `arbiter.hpp`; the reasoning is next to the code in `arbiter.cpp`.
In short: the mutex is held across a command in `run_if_admitted`, it is not
held across `on_halt` on the ownership paths, and `estop()` never waits for it.
So `on_halt` must be idempotent (an e-stop delivers it twice), thread-safe, and
must never call back into the Arbiter.

## Deviations from the arm's Arbiter

Small and deliberate. Everything else is one-to-one; the host tests are the
arm's tests where the surface is the same.

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
- **Racing grants each seize.** The arm's `grant` halts the old owner with the
  mutex released and then installs the new one, so a second grant landing in
  that window could take the base with no halt and then be silently
  overwritten. Here `grant` loops: after the halt it looks again, and seizes
  (and halts) once more if someone took the base meanwhile. The arm has the
  same race; `test/host` has a case for it that fails on the arm's logic.
- **`release(token)` lives in the core.** On the arm the ROS server retains the
  minted token and checks a release against it. The core already holds the
  token, so checking it here avoids a second copy.
- **Tokens come from `esp_fill_random`** unless a seed is given, in which case
  a `std::mt19937_64` makes them repeatable for a test. The arm seeds its
  generator from `std::random_device`. On the ESP32-P4 (no RF) the RNG is
  seeded with entropy by the bootloader and is not a continuous true RNG after
  that, which is enough for a mistake boundary: tokens only have to differ
  across grants and reboots. Never seed on the robot; the Arbiter warns if you do.
- **Enum values are `UPPER_CASE`** (`ENFORCED`, `OWNERSHIP_REVOKED`) to match
  this repo, where the arm uses `kEnforced`, `kOwnershipRevoked`.
- **The core logs.** Grants, seizes, releases, revokes, the e-stop and refused
  grants are logged, and rejections at debug level. The arm's core has no
  logger and leaves logging to its transport. The e-stop and seize paths log
  after the halt, never before: a log line can block on the UART.
- **A seize clears the owner first.** During the seize halt `status()` shows no
  owner; the arm keeps showing the old one until the new grant lands.
- **A pre-supplied token** (`grant(owner_id, token)`) follows the arm's
  kinova-gen3-driver#90 and rammp-interfaces-ros2 dev: adopted if non-zero,
  refused before anything changes if all-zero.

Like the arm's, this is a **mistake boundary, not a security boundary**.
Anything on the bus can observe a token. It keeps two well-behaved systems
from fighting over the base; it does not defend against a hostile one. It is
also a software e-stop, not the hardware chain.

## Tests

`test/host` builds the Arbiter with the host compiler and runs 23 checks: the
arm's `arbiter_test.cpp` cases where the MIB has the same surface, the
pre-supplied token, and two races the arm's suite does not cover (racing grants,
and an e-stop during a command in flight). UBSan everywhere, ASan on Linux
(Apple's ASan deadlocks at startup on recent macOS).

```console
idf.py reconfigure   # once, in the repo root: fetches the espp headers
components/Arbiter/test/host/run.sh
```

The RTPS face is tested on hardware from ROS 2: see
[components/ArbitrationServer](../ArbitrationServer/README.md).

## Example

`example/` is a standalone ESP-IDF project that walks the lifecycle with a
counter standing in for the drive: refusal with no owner, grant, seize, release
with the wrong and right token, revoke, e-stop and its double halt, clear, the
`DISABLED` bypass, and seeded tokens.

```console
cd components/Arbiter/example
idf.py build flash monitor
```
