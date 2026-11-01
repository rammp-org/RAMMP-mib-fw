# DriveProfileStore

Keeps the limits of the LOW, NORMAL, and HIGH drive profiles in flash, so a
clinician can tune a chair for its user without a rebuild and reflash
([#33](https://github.com/rammp-org/RAMMP-mib-fw/issues/33)).

Each profile is one file on the `littlefs` partition (`partitions.csv`),
written with espp's
[`PersistentData`](https://esp-cpp.github.io/espp/storage/persistent_data.html):
`drive_profile_low.bin`, `drive_profile_normal.bin`, `drive_profile_high.bin`.
A file is a 20-byte header (magic, schema version, layout fingerprint, length,
CRC32) followed by the `DriveProfileConfig` fields. Saves are atomic: a temp
file is written and synced, then renamed over the old one, so a reset mid-save
keeps the previous profile.

The partition sits after the two OTA slots. A firmware update over the control
panel leaves it, and the profiles saved there, alone, but it can't create it
either: a chair whose partition table predates it needs one USB flash
(`idf.py flash`). Until then every profile runs on its default and a
persistent change reports `SAVE_FAILED`.

## What the MIB runs on

At boot, before driving can be enabled, each profile loads its file:

| File | Profile runs on | Source reported |
|---|---|---|
| none | the compiled-in `MIBconfig.hpp` default | `DEFAULT` |
| loads, and every value passes `invalid_field()` | the saved values | `SAVED` |
| corrupt, truncated, from newer firmware, or holding a bad value | the compiled-in default, with the reason logged | `DEFAULT` |

A file that can't be used is left in place, not overwritten, until a
persistent change replaces it. Defaults are never written to flash, so a
firmware update that changes a default reaches every profile nobody has tuned.

`invalid_field()` (`DriveController/include/drive_profile.hpp`) applies the
checks `espp::TrajectoryPlanner::set_config()` makes, plus the ones it lacks:
every field finite, jerk and centripetal limits not negative. There are
deliberately no upper limits: the clinician tuning the chair sets the speeds.

## Changing a profile over RTPS

Send a `rammp::DriveProfileCommand` on `rammp/joystick/drive_profile_command`.
The MIB reports every profile's limits, where they came from, and the outcome
of the last command in `MIB::DriveProfileState` on `rammp/mib/drive_profiles`,
twice a second. Both types are in `external/rammp-rtps`.

| Command | `persist = 0` | `persist = 1` |
|---|---|---|
| `SET` | use `values` until reboot (`UNSAVED`) | use and save them (`SAVED`) |
| `RESET` | use the default until reboot | use the default and delete the file (`DEFAULT`) |

The MIB acts on a new `req_id` only, so a sender repeats the command until
`command_ack` shows its `req_id`, then reads `result`: `OK`, `INVALID` (nothing
changed), `SAVE_FAILED` (in use until reboot), or `BAD_REQUEST`. A change to the
profile being driven takes effect at once and keeps the chair's current speed.

## Changing `DriveProfileConfig`

The files hold `DriveProfileConfig` as is, so once a layout has been on a
chair, adding, removing, reordering, or changing the meaning of a field needs a
new schema version, or every saved profile falls back to its default (the
header's layout fingerprint catches an unversioned change as
`layout_mismatch`):

1. Copy the current struct to a frozen `DriveProfileConfigV1` (same fields,
   `VERSION = 1`) next to it in `MIBconfig.hpp`.
2. Change `DriveProfileConfig`, set `VERSION = 2`, and add
   `static DriveProfileConfig migrate(const DriveProfileConfigV1 &v1)` that
   fills the new layout from the old one.
3. List both in `DriveProfileStore::File`, newest first:
   `espp::PersistentData<config::DriveProfileConfig, config::DriveProfileConfigV1>`.
4. Mirror the change in `MIB::DriveProfileValues` (rammp-rtps) and
   `tools/drive_profile/drive_profile.py`.

An old file then loads as V1, migrates to V2, is validated, and is rewritten as
V2. A file from newer firmware (a downgrade) is refused and the profile runs on
its default.

## Host test

`test/drive_profile_store_host_test.cpp` runs the store against espp's host
builds of `PersistentData` and `FileSystem` in a temp directory: fallback for
missing, corrupt, truncated, newer, and invalid files, and set/reset with and
without persist. Run it with `components/DriveProfileStore/test/run.sh` on
macOS or Linux; CI's `host-tests` job runs it on every pull request. It takes
espp from `managed_components/` after an `idf.py build`, or clones espp
(`ESPP_REF`, default `v1.3.11`) when there is none. `test/stubs/` stands in for
the one ESP-IDF header `MIBconfig.hpp` includes.
