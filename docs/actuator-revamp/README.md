# Actuator revamp: evidence

The measurements behind `components/Actuator` (right main leg on the MCP266, 2026-11-17).

- [`bench-findings.md`](bench-findings.md): everything measured, in the order it was found. Later sections correct earlier ones.
- [`figures/`](figures): the PR figures; [`data/`](data): the numbers plotted in each, as CSV.
- Raw logs (`runs/`, about 250 MB) and the MIB flash backup (`backups/`) are kept out of git.
- The bench firmware and scripts that took the data: `tools/actuator_sysid/` (stacked PR); `tools/actuator_sysid/bench/make_figures.py` redraws the figures from the raw logs.

| Figure | Shows |
|---|---|
| 01 | Stock settings 400-count step vs tuned streamed 1,000-count move |
| 02 | Peak chassis rotation rate by kind of move |
| 03 | The CiA 402 position window: same ramp at window 100 and 10 |
| 04 | End error and peak current across the right leg's travel |
| 05 | Chassis tilt against right-leg position |
| 06 | The heartbeat watchdog stopping a move |
| 07 | Leading the target removes the tracking lag |
| 09 | The main firmware driving MR: moves, a held jog, an e-stop mid-move |
