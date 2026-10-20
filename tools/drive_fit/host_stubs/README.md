Minimal stand-ins for the espp headers that `espp::TrajectoryPlanner` includes,
so `replay_drive_profile.cpp` can compile the real planner source on a laptop.
The logger drops every message, and the timer and task never run: the replay
calls `update()` directly at the firmware's planning period.
