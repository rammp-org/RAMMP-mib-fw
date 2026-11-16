# Chair configuration

Version-controlled values that describe this chair, kept here so every board,
laptop and tool starts from the same numbers.

| file | holds |
| --- | --- |
| `poses.json` | MIB poses: one entry per leg (MIBconfig.hpp leg names). A number is an absolute count to move to; `{"delta": N}` moves the leg by N counts from where it is, and `{"delta": 0}` means it stays. `home` is the rest pose. |

To change a pose: on the control panel's Seat actuators tab hold **Hold to
edit poses**, set the pose, lock it again, **Export poses.json**, replace
`poses.json` here with the download, and commit it. A new pose is a new entry
in the `poses` list.
