# Encoder scope (old chair, USB console)

Live plot of MCP266 encoder channels through the Actuator example's bench console on the MIB's
USB-C (921600 baud): `python3 encoder_scope.py --nodes 3,10` then http://localhost:8790/.
`POST /cmd` runs a console command between reads and returns its output; during `wait` /
`trace` the scope sends `stop` if the traced speed passes 4,000 counts/s (`POST /guard` sets it).
`bench_move.py <delta> <speed> [label]` runs one traced move through it (count every 20 ms plus
the IMU tilt rate from `trace`) and saves it to `moves/`.

Used on the old chair on 2026-11-17/18 (left-leg encoder fault, its ~3.9 Hz surge, the input
shaper A/B). The new chair is Ethernet-only: see `docs/new-chair-bringup/HANDOFF.md` on
`alexchunlin/new-chair-bringup`.
