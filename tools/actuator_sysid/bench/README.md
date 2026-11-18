# Bench scripts

The scripts used on the chair on 2026-11-17 with the `actuator_sysid` motion firmware (`../firmware`).
They are what produced `docs/actuator-revamp/bench-findings.md` and its figures. Small and single-purpose;
the analysis package in `../host` was written alongside them and was not used on the chair.

## Setup

```sh
export SYSID_LOG=$PWD/../.bench/2026-11-18_session.log   # any new file; session.py appends to it
export SYSID_PORT=/dev/cu.usbmodem...                     # optional when only one usbmodem port exists
python3 session.py &                                      # holds the port open, logs everything, runs queued commands
```

Opening the port resets the MIB (CH343 auto-reset), so keep one `session.py` running for the whole session.
Every other script queues console commands into `$SYSID_CMDS` (default `../.bench/cmds.txt`) and reads the log.
Writing `QUIT` to that file makes `session.py` send `stop` and exit.

## Scripts

| Script | Does |
|---|---|
| `session.py` | Long-lived serial session: appends the console to `$SYSID_LOG`, sends each new line of `$SYSID_CMDS` |
| `say.py <wait_s> cmd ...` | Queue commands, wait, print the events they produced |
| `walk.py "delta rate accel" ...` | Streamed moves on node 3 M1, one after another; narrows the clamp to each move, prints a one-line result per move, stops at the first guard trip or miss. Env: `HS` handshake (2 = lean), `PERIOD` ms, `LEAD` ms |
| `runstream.py delta rate` | One streamed move with optional `GAINS` / `VG` / `CLAMP` env, then `streamsum.py` |
| `runstep.py delta` | One single-target step (`pstep`), then `stepsum.py` |
| `single.py delta speed_cap` | One target with the controller's speed cap, no streaming |
| `stepsum.py`, `streamsum.py` | Summarise the last move in the log: target, encoder, PWM, current, gyro in time bins |
| `cap.py out seconds [t:cmd ...]` | Fixed-length capture with timed commands (passive runs) |
| `reflash.sh` | Stop the session, flash `../firmware/build_motion`, leave the board running |
| `benchlog.py log` | Parse a log into boots and moves (used by `make_figures.py`) |
| `make_figures.py` | The PR figures and CSVs, Alex style (needs the raw logs: `SYSID_RUNS`, and `ALEX_STYLE_DIR`) |

Everything here goes through the firmware's guards (arming, clamp window, node 2 read-only). The scripts add none of their own beyond stopping on a guard trip or a missed target.
