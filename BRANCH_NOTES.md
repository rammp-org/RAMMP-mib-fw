# Branch notes: feature/18-mib-debugger

This branch carries the bench debugger in `tools/mib_debugger` and the firmware
support it needs. **It is not intended to merge into `main`.** Everything the
debugger turned up that `main` should act on is tracked here, so nothing is lost
when the branch is eventually dropped. Keep this file current: when the debugger
reveals something new, add it under "Findings for main".

## Findings for main

Ordered by importance. Each entry says where the problem is on `main`, how it
was found, and how to confirm a fix with the debugger.

### 1. Joystick right turns the chassis left (sign error)

**Status: open on main.** This is the important one.

- **Where:** `components/MibSystem/src/mib_system.cpp`, `handle_joystick_message`,
  on `main` at line 170:
  ```cpp
  drive_controller_.set_target(sample.y, sample.x);
  ```
- **Why it is wrong:** `XYTwist.x` is defined in `joystick_message.hpp` as
  "+1 = right". The MIB passes it straight through as the angular velocity.
  `DifferentialDrive::compute` uses `left = linear - angular * half` and
  `right = linear + angular * half`, the standard convention where positive
  angular is counter-clockwise, a left turn. So a rightward stick produces a left
  turn.
- **Fix on main (one character):**
  ```cpp
  drive_controller_.set_target(sample.y, -sample.x);
  ```
  If the team would rather define positive angular as clockwise, flip the sign in
  `DifferentialDrive::compute` instead and leave the handler alone. Pick one and
  document it in `MIBconfig.hpp`.
- **How it was found:** the simulator reproduces the firmware math exactly; with
  a rightward stick the debugger's chassis-frame readout reports "turning left
  (CCW)".
- **How to verify a fix:** on this branch, set the `JOYSTICK_X_SIGN` parameter to
  -1 on the Parameters page and push the stick right; the debugger reports
  "turning right (CW)". On a fixed `main`, the same result must appear with no
  parameter change (and the parameter will not exist).
- **On this branch** the handler multiplies `sample.x` by the runtime parameter
  `JOYSTICK_X_SIGN`, default +1, purely so the bug can be demonstrated live.
  That parameter is a bench aid, not the fix.

### 2. Build breaks under ESP-IDF v6.1 (LVGL memory setting)

**Status: fixed on this branch in commit `cf16ccb`; safe to cherry-pick to main.**

- **Where:** `sdkconfig.defaults` sets `CONFIG_LV_MEM_SIZE_KILOBYTES=0`.
- **Why it is wrong:** LVGL 9.5, which the component manager resolves under
  ESP-IDF v6.1, derives its pool size from that symbol and fails the build with
  `LV_MEM_SIZE >= 2kB is required`. The adjacent `CONFIG_LV_MEM_SIZE=65536` is
  ignored by LVGL 9.x.
- **Fix:** `CONFIG_LV_MEM_SIZE_KILOBYTES=64`, matching the intended 65536 bytes.
- **Note:** CI pins ESP-IDF v6.0.1 and may resolve a different LVGL, which is
  why CI does not show this. Anyone building locally on v6.1 hits it.

### 3. CALIBRATE is a dead state

**Status: informational.**

- `SystemState::CALIBRATE` exists in `MibSystem` with a case in
  `run_state_step`, but nothing ever assigns it. No command, RTPS message or
  internal path enters it. Either add an entry path or remove the state so the
  state machine reflects what the firmware can do.

### 4. Joystick values are treated as metres per second

**Status: informational, worth a deliberate decision.**

- `handle_joystick_message` passes the normalised stick value (−1..+1) directly
  to `set_target` as linear m/s and angular rad/s. The trajectory planner then
  clamps to the active profile's maximum. The effect is that full stick reaches
  the profile limit only because every profile's limit is ≥ 1 in magnitude for
  linear; for the LOW profile's 0.35 m/s cap the stick saturates at 35 % travel.
  If the intent is "full stick = profile maximum", scale by the profile limits
  before clamping.

### 5. App partition headroom is 13 %

**Status: informational.**

- The binary is about 900 KB in a 1 MB app partition (`--flash-size 2MB`
  default). Adding features will overflow it. Decide the real flash size for the
  PCB and update the partition table before that happens.

## Firmware changes on this branch (not for main)

These exist only to support the debugger's Parameters page. They add about 320
lines across eight files. If a future decision is to bring runtime parameters to
`main`, use this as the starting point, but it was written for bench use and is
not persisted, not authenticated, and not reviewed for a production wheelchair.

| File | Change |
|---|---|
| `components/MIBConfig/include/mib_params.hpp` | New. Parameter table (X-macro), `mib::Params` mutex-guarded store seeded from `MIBconfig.hpp`, and the `ParamSet` / `ParamState` messages and topic names. |
| `components/DifferentialDrive/{include,src}` | Added `set_config()` so geometry and inversion can change at runtime. |
| `components/DriveController/{include,src}` | `profile_config()` reads limits from `mib::Params` instead of the constexpr table; new `apply_params()`; a mutex around the differential drive and active profile. |
| `components/MibSystem/{include,src}` | Reliable subscriber for `rammp/joystick/param_set`, best-effort publisher for `rammp/mib/params` at the publication rate, `handle_param_set` / `publish_param_state`, and the `JOYSTICK_X_SIGN` multiply in the joystick handler. |
| `sdkconfig.defaults` | Finding 2 above. This one *should* go to main. |

The message definitions for parameters live in this repository rather than in
the shared `external/rammp-rtps` submodule, deliberately, so the submodule is
untouched by this branch.

## Cherry-picking to main

Only the LVGL fix is meant for main:

```
git checkout main
git cherry-pick cf16ccb
```

The joystick sign fix (finding 1) is a hand edit on main, not a cherry-pick,
because this branch demonstrates the bug through a parameter rather than fixing
it.
