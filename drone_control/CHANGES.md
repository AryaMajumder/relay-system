# Changes — SITL integration test (2026-07-03 / 2026-07-04)

Five source files were modified during the integration test session.
Confidence level is noted for each: **exact** means the before/after value
is known from session records; **attributed** means the file's mtime falls
in the test window and the change matches a described bug fix, but no diff
exists (no git history).

---

## `drone_control/config/demo_config.py` — exact

`recover_offboard_max_attempts`: **8 → 30**

8 ticks × 0.5 s = 4 s was not enough time for the OFFBOARD acquisition
chain (relay_mover 1.5 s pre-stream delay + ARM ACK + OFFBOARD ACK + MQTT
publish + state_bridge 1 Hz republish timer = ~4–5 s total). Gate 3
(`OffboardModeHeld`) was expiring and firing `FollowerSafetyExit(G3)` before
OFFBOARD was ever seen in the BT blackboard.

30 ticks × 0.5 s = 15 s. In the successful run, OFFBOARD first appeared
~6.4 s after START_LEAD — well within the new window.

---

## `drone_control/state_bridge.py` — attributed

**rclpy thread-safety fix.** `self._pub.publish()` was being called from
the paho MQTT `on_message` callback, which runs on a background thread.
Calling rclpy publish from a non-ROS2 thread causes non-deterministic
crashes.

Fix: the MQTT callback now only writes `self._last_payload` under a
`threading.Lock()`. All publishing happens exclusively in
`_republish_tick()`, which is a ROS2 timer callback (main executor thread).

---

## `drone_control/px4_agent.py` — attributed

Two fixes:

**1. NED altitude sign.** `_global_to_ned` passes `home_alt=0.0` (ground
level) rather than `home["alt"]` (which could be a non-zero MSL value from
the drone state message). This ensures the z offset is computed as pure AGL:
`z = -(alt - 0.0) = -50` for a 50 m AGL target. Using `home["alt"]` was
producing the wrong z value and sending the drone to the wrong altitude.

**2. ARM / OFFBOARD sequencing.** OFFBOARD activation is delegated to the
production `px4_agent` via a `START_LEAD` MQTT command rather than issuing
raw ARM + set-mode MAVLink directly from the test harness. Sending raw
MAVLink from an ad-hoc source port corrupts PX4's partner IP table (PX4
learns its GCS partner from the first received packet) and mis-sequences the
ARM → OFFBOARD handshake.

---

## `drone_control/strategy_executor.py` — attributed (exact change unknown)

Modified 2026-07-03 14:56. The file was changed during the test session but
no git history exists to diff. The current code handles
`CONTINUOUS_RELAY → MOVING_TO_RELAY`, `TIMEOUT → IDLE`, and
`CONFIRMED → RELAYING` correctly; the specific line(s) that changed are not
recoverable without version control.

---

## `drone_control/capability_assessor.py` — attributed (exact change unknown)

Modified 2026-07-03 18:22. Same constraint as above — the file was touched
during the test window but the exact diff is not recoverable. The current
code drains `pending_command` from the blackboard and publishes it *before*
calling `_build_report()`, so safety-exit commands reach `relay_mover`
within the same executor cycle; whether this ordering was added during the
test or was already present is not confirmed.

---

## Infrastructure changes (no source files modified)

These were operational fixes applied at runtime, not persisted in code:

- **`parameters.bson`** copied from the PX4 build tree to `/tmp/sitl_iris_1/`
  on each test run to ensure `SYS_AUTOSTART=10017` (jMAVSim iris) is set.
  Without it PX4 falls back to the internal SIH simulator and never opens
  TCP port 4561.
- **jMAVSim lockstep** (`-l` flag) required to prevent timestamp drift.
  Without it, after ~178 s of runtime jMAVSim switches to Unix wall-clock
  timestamps, PX4 detects a multi-second forward jump, and triggers flight
  termination (`UNKNOWN(10,0)`).
- **jMAVSim working directory** must be
  `/root/src/PX4-Autopilot/Tools/simulation/jmavsim/jMAVSim` so the MAVLink
  schema file is found at the expected relative path.
- **`relay_tasking` injection** requires `--times 15` (not `--once`) to
  ensure delivery after DDS discovery completes.
