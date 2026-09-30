# From a joint position to a moving MCP266

How a command in engineering units becomes motion, what each layer owns, where
the limits live, and what is still to come (marked *planned*).

The rule behind the split: the **BSP is the electronics interface** (Ethernet,
CAN, the MCP266s, their encoders, the calibration of those encoders) and
nothing in it knows what a joint means. Kinematics, units and any move that
spans joints live in an **application layer above the BSP**, which is not
designed here yet; it will hold references to the BSP's actuators and speak
to them in fractions of travel.

## The flow

```
                 state machine / seat command / calibration script
                                     │
                                     │  set_position(joint, value)      metres or degrees
                                     ▼
        ┌────────────────────────────────────────────────────────────┐
        │  Application layer                        (planned)        │
        │  per joint: units ⇄ fraction of travel (0..1)              │
        │  one kinematics class per family: main legs, casters,      │
        │  carriages; written once the base is measured              │
        │  pure arithmetic, host-testable, never sees counts         │
        │  holds references to the BSP's actuators                   │
        └────────────────────────────────────────────────────────────┘
                                     │
                                     │  move_fraction(f)      f in 0..1
                                     ▼
        ┌────────────────────────────────────────────────────────────┐
        │  mib::Actuator            (this component, owned by BSP)   │
        │  • fraction ⇄ counts using its calibrated range            │
        │  • clamps to the range (unless calibrating), logs the clip │
        │  • holds the range, jog step, default profile              │
        │  • initialize(): writes the range to the controller        │
        │  • verifies the clamp before each move; re-initialises a   │
        │    controller that reset                                   │
        │  • calibration (absolute) and homing (incremental) paths   │
        │  • offline marking                                         │
        └────────────────────────────────────────────────────────────┘
                                     │
                                     │  move_to_position(axis, counts,
                                     │                   velocity, accel, decel)
                                     ▼
        ┌────────────────────────────────────────────────────────────┐
        │  espp::Mcp266  +  espp::CanopenClient  +  espp::Twai        │
        │  CiA 402 enable sequence, profile objects, target,          │
        │  new-set-point handshake — each one an SDO frame pair      │
        └────────────────────────────────────────────────────────────┘
                                     │
                                     │  CAN bus, 1 Mbit/s
                                     ▼
        ┌────────────────────────────────────────────────────────────┐
        │  MCP266 (one channel)                                       │
        │  • runs the motion profile and the position PID itself     │
        │  • reads the joint's absolute encoder                      │
        │  • position clamp  MinPos/MaxPos      ← written by Actuator │
        │  • software limits 0x607D             ← written by Actuator │
        │  • limit switches on S-pins (carriages) ← its own config    │
        └────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
                          linear actuator moves the joint
```

Reads go the other way along the same path: `read_position()` is one SDO for
the counts, the range turns them into a fraction (`read_fraction`,
`current_fraction`), and the application layer turns the fraction into units.

## Who owns what

| Concern | Where | Notes |
|---|---|---|
| Units (m, deg) and kinematics | Application layer, constants from `MIBconfig.hpp` | Planned. Not in the BSP. |
| Counts and the fraction ⇄ counts mapping | Actuator | The only layer that knows what a count is. |
| Range in counts | Actuator (source of truth), persisted in MIB NVS | Compiled default in `MIBconfig.hpp`, overridden by the calibrated value from NVS. |
| Enforcing the range | Both | Actuator clamps before sending; the controller clamps again (see below). |
| Current position | The encoder, read on demand | `read_position()` asks; `current_position()` is the last reading plus its age, fresh as often as the owner polls. |
| Motion profile | Actuator config, per-move override | Executed by the controller. |
| Position loop (PID) | MCP266 | Gains tuned in Motion Studio or on the debugger's CAN page. |
| Limit switches | MCP266 | Its S-pin functions; the Actuator only knows they exist. |
| Fault reset, NMT start | Actuator / BSP | Once per boot, and again when a move finds the controller's clamp gone (it reset). |
| Last known position (incremental) | Actuator via `ActuatorStore` | Saved from the periodic poll, throttled; restored at boot instead of homing. |

## Fractions, absolute and relative

```cpp
bool move_fraction(float f);            // 0..1 of the calibrated travel, clamped
bool read_fraction(float &f);           // read the encoder, convert with the range
bool move_relative_fraction(float df);  // read, add, command absolute
std::optional<int32_t> last_target();   // last count commanded, for queued moves
```

`f = (counts - min) / (max - min)` and back. Because the encoders are
absolute, the fraction is well defined as soon as the range is calibrated.

**Absolute** moves are the natural fit: the converter turns units into a
fraction from constants, and the Actuator does the rest.

**Relative** moves need the current position, and it comes from the encoder,
not from stored state. `move_relative` reads the position from the controller
(one SDO), adds the delta, clamps, and commands the absolute result.
`move_relative_fraction` does the same in fraction space. For a linear joint a
delta in units is a fixed delta in fraction and can be passed straight down;
for a non-linear joint the caller goes through the absolute path (read the
fraction, convert to units, add, convert back, `move_fraction`).

`current_position()` is not a second source of truth: it is the last reading,
with `position_age()`, and the owner's periodic poll (which also publishes the
positions) keeps it fresh. Decisions such as a relative move use
`read_position()`; status uses the cache.

"Relative" means relative to where the joint *is* at the moment of the call.
A second relative command issued mid-move starts from the mid-move position,
not from the previous target. That is the safer meaning when a move was
clipped or stopped; a layer that wants "relative to the last target" can use
`last_target()`.

**`increment` and `decrement` stay in counts.** They serve calibration and
manual jogging, where the range may not exist yet or is deliberately lifted,
so a fraction of the travel is not defined. A jog step in raw counts is the
right unit there.

## Where the limits actually live

**Position clamp vs 0x607D.** Two objects on the controller with the same
intent. The *position clamp* is Basicmicro's own: the MinPos/MaxPos fields of
the position PID record (command 61/62). It ships as `[0, 0]` and the
controller forces any target outside it to the clamp. *0x607D* is the standard
CiA 402 software position limit that any CANopen drive exposes; the profile
generator limits targets to it. Whether the MCP266 implements them as one
mechanism or two is undocumented; espp writes both, so we do. Writing one and
reading the other on the bench will show if they are aliases.

**Three limits act on a target, in order:**

1. `Actuator::clamp()` clips the requested count to `[min, max]` before
   anything goes on the bus, and logs the clip. Visible, deterministic, and
   independent of what the controller does with an over-range target (clamp,
   abort or ignore: not documented). Also the only limit that exists before
   the controller's are written or after they are lost.
2. The controller's position clamp.
3. The controller's 0x607D limits.

Limits 2 and 3 catch anything that bypasses the Actuator, such as a raw SDO
from the debugger. The carriages' **limit switches** are a fourth, physical
limit the controller handles alone; the count it reports when stopped on a
switch is what calibration records as that end.

**The controller reset hazard.** The MCP266 reverts to its EEPROM at
power-up. If one controller browns out or resets while the MIB is running, it
comes back with the factory `[0, 0]` clamp and the MIB still believes it is
initialized; the next move would be forced to count 0 and the joint would run
to one end. So before every move the Actuator reads the clamp back (two SDOs,
a few milliseconds) and compares it with what it installed; a mismatch means
the controller reset, and `initialize()` is re-run first. For an incremental
encoder that also means the count is arbitrary again, so the saved position is
restored with a warning, or the joint waits for `home()`.

**The MCP266's EEPROM.** Not written today: WRITENVM over CAN is unverified on
this controller, the MIB as single source of truth means a swapped controller
is configured at boot with nothing to remember, and Motion Studio writes the
same EEPROM. *Planned:* once the bench confirms the save works over CAN, the
calibration script also saves to EEPROM, so the controller keeps its limits
through its own resets; the MIB still re-writes every boot. If the CAN save
aborts, Motion Studio's "save settings" after the MIB has written the values
does the same job.

## Calibration and homing are different things

| | Calibration | Homing |
|---|---|---|
| Encoder | absolute (FC, RC, ML, MR) | incremental AB (LCarr, RCarr) |
| When | once per installation | every boot, unless the saved position is restored |
| Finds | the counts at the two mechanical ends | a known count at the limit switch |
| API | `set_calibration_mode`, `set_range` | `home`, `restore_position`, `save_position` |
| Persists | the range, in NVS | the last position, in NVS, written while running |
| Script | calibration script, separate | part of boot / the actuator task |

**Homing sequence** (`home()`): lift the clamp; move toward the switch at the
homing profile; poll the count every 100 ms until it stops changing for
`stall_time` (the controller stopped the motor on the switch); quick stop;
write `home_count` into the controller's encoder (set-encoder command 22/23,
mirrored, unverified) and read it back; restore the clamp; save the position.
Fails loudly and refuses moves if any step does not take.

**Position memory**: the owner calls `save_position()` from its periodic poll.
It writes to NVS only when the joint moved more than a threshold and at least
an interval has passed (defaults 20 counts, 5 s), so a machine running for
hours does not wear the flash. At boot `initialize()` restores the saved count
into the controller, which makes the joint ready without moving, on the
assumption that it did not move while off. `home()` when in doubt.

## Calibration, end to end

```
  cal on ──▶ Actuator writes a ±2e9 clamp to the controller   (limits lifted)
     │
     ├── inc / dec ──▶ small position moves; carriages stop on their switch
     │                 read pos at each mechanical end
     │
  range <min> <max> ──▶ ActuatorRangeStore.save() → NVS
                        Actuator.set_range()      → clamp + 0x607D on the controller
                        (planned) save to the controller's EEPROM
     │
  cal off ──▶ Actuator restores the range clamp on the controller

  next boot: BSP loads NVS range → Actuator::initialize() writes it again
```

No zeroing step: with absolute encoders the counts at the two ends are the
calibration. What calibration does not give you is the meaning of those ends
in units; that comes from the mechanical design and belongs in
`MIBconfig.hpp` next to the leg geometry.

## Keeping the calibration outside the ESP32

NVS is a key-value store in the ESP32's flash. It survives a reflash, but it
is not readable and a dead board takes it with it. *Planned:* a text export.

```json
{
  "FC": {"node_id": 10, "channel": "M1", "min_counts": 412, "max_counts": 3710,
         "min_units": 0.0, "max_units": 0.120, "units": "m"},
  "RC": {...}
}
```

- The console (example and, later, the firmware) gets `export`, which prints
  this block, and `import`, which accepts a paste and writes NVS.
- The exported block is committed under `config/` and its numbers copied into
  the compiled defaults in `MIBconfig.hpp`. A replacement ESP32 then boots
  with the right values even with empty NVS; NVS only carries a re-calibration
  not yet promoted to the repo.

## The converter, sketched

For a linear joint:

```
f     = (value - value_at_min) / (value_at_max - value_at_min)
value = value_at_min + f * (value_at_max - value_at_min)
```

`value_at_min` and `value_at_max` are per-joint constants in `MIBconfig.hpp`
(what the joint measures at each calibrated end). For a joint where actuator
stroke and joint angle are not proportional, the same interface holds and the
body becomes the inverse kinematics from the leg geometry, still ending in a
fraction of stroke. The Actuator underneath is unchanged either way.

Where it sits: between the state machine and the BSP, one instance per joint,
owning nothing but the mapping. The BSP hands out actuators; the state
machine talks units.

## Planned changes, collected

1. The application layer above the BSP: one kinematics class per family
   (main legs, casters, carriages) once the base is measured, with host unit
   tests; the periodic poll that publishes positions and saves the carriages'
   counts; homing at boot.
2. The actuator task in MibSystem that drives that layer.
3. The calibration script for the absolute encoders, separate from boot.
4. Calibration: save to the controller's EEPROM as well, once WRITENVM over
   CAN is confirmed on the bench.
5. Calibration export/import as text; committed copy under `config/`;
   promote into `MIBconfig.hpp` defaults.
6. Bench checks: the set-encoder mirror (homing depends on it); whether
   MinPos/MaxPos and 0x607D are one mechanism or two.
