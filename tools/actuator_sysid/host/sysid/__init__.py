"""Host-side library for the MIB actuator system-identification logger.

wire       line classification and the incremental parser capture.py uses live
runlog     raw.log -> arrays keyed by column name, events, parse statistics
timebase   jitter, gaps, dropped rows
attitude   pitch and roll from the gravity vector and the quaternion, mounting, level reference
signals    fresh CAN samples, smoothed derivatives, command plateaus
fopdt      first-order-plus-dead-time fit in the position domain
analyses   one module per analyze.py subcommand
plotstyle  the shared plot style (alex-style)
plant      the simulated chair simulate.py and the tests use
"""
