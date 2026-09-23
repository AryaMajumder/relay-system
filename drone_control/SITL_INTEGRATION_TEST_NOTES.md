# SITL Integration Test — drone-02 Relay System

## Objective

Verify end-to-end relay acquisition for `drone-02` acting as a follower drone in a three-node relay network (Ground Control → Follower → Leader). The test was considered passing when:

1. `current_role` advanced to `RELAYING` (not merely `MOVING_TO_RELAY`).
2. `relay_confirmed` carried `status=CONFIRMED` (not `TIMEOUT`).
3. The drone held OFFBOARD flight mode continuously from initial position-setpoint streaming through relay confirmation.

---

## System Overview

The relay system is a ROS2 stack layered over PX4 SITL and jMAVSim:

```
jMAVSim (TCP server :4561)
    ↕  MAVLink TCP
PX4 SITL (TCP client, connects to :4561)
    ↕  MAVLink UDP
px4_agent  (ROS2 node, drone-02)
    ↕  MQTT :1885
state_bridge → /drone_02/drone_state  (1 Hz ROS2 republisher)
capability_assessor  (BT tick at 0.5 Hz)
relay_mover / relay_position_tracker / strategy_executor / …
```

The Behavior Tree (BT) inside `capability_assessor` decides whether the follower is capable of relaying and, once in `RELAYING_BRANCH`, runs two steps every tick: a `DIAG_SCAN` that unconditionally evaluates G8 (authorization validity) and G9 (GPS health), followed by an `ARBITER_SCAN` priority Selector that gates continued relay on G1–G7 plus a reauth-timeout check.

---

## Infrastructure Setup

### PX4 SITL

Started with `SYS_AUTOSTART=10017` (jMAVSim iris model) in parameters. The working directory for PX4 must contain a `parameters.bson` that encodes `SYS_AUTOSTART=10017`; without it PX4 falls back to its internal SIH simulator, never opens TCP port 4561, and jMAVSim cannot connect.

```sh
cp /root/src/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson \
   /tmp/sitl_iris_1/parameters.bson

PX4_SIM_PORT=4561 PX4_SIM_MODEL=jmavsim_iris \
  /root/src/PX4-Autopilot/build/px4_sitl_default/bin/px4 \
  /tmp/sitl_iris_1 -i 1 > /root/px4_instance_1.log 2>&1 &
```

### jMAVSim

Must be run from its own source directory (for MAVLink schema resolution) and with the `-l` lockstep flag.

```sh
cd /root/src/PX4-Autopilot/Tools/simulation/jmavsim/jMAVSim
HEADLESS=1 ./jmavsim_run.sh -p 4561 -l > /tmp/jmavsim.log 2>&1 &
```

jMAVSim is the **TCP server** on port 4561; PX4 is the TCP client that connects to it.

### ROS2 stack

```sh
ros2 launch drone_control follower_relay.launch.py drone_id:=drone-02
```

The launch file default is `drone_id=drone-01`; passing `drone_id:=drone-02` is required.

### Manual test stimulus

After all nodes are running and `capability_assessor` reports `CAPABLE`, inject a `relay_tasking` message to trigger the first MOVING_TO_RELAY:

```sh
ros2 topic pub /drone_02/relay_tasking std_msgs/msg/String \
  "{data: '{\"assignment_id\": \"test-01\", \"relay_position\": {\"lat\": 47.39401, \"lon\": 8.54398, \"alt\": 50.0}}'}" \
  --times 15
```

`--times 15` (not `--once`) is required because DDS discovery may not have completed by the time the first publish fires.

---

## Bugs Found and Fixed

### Bug 1 — `OffboardModeHeld` timer too short (race condition, Gate 3)

**Symptom.** In early test runs, `FollowerSafetyExit(G3)` fired with reason `offboard_unrecoverable:mode=HOLD` within 2–4 seconds of `START_LEAD`, causing the drone to RTL before ever entering OFFBOARD. Once in `AUTO.RTL`, the arbiter's subsequent ticks continued to see a non-OFFBOARD mode, and RTL commands were re-dispatched every 0.5 s.

**Root cause.** `OffboardModeHeld` counts consecutive non-OFFBOARD BT ticks and fires `FollowerSafetyExit` after `recover_offboard_max_attempts` ticks. The previous value was **8** (8 × 0.5 s = 4 s). The total latency of the OFFBOARD acquisition chain is:

| Step | Latency |
|---|---|
| `relay_mover` pre-stream delay | 1.5 s |
| ARM MAVLink command + PX4 ACK | ~0.5 s |
| OFFBOARD mode command + PX4 ACK | ~0.5 s |
| px4_agent MQTT publish | ~0 s |
| state_bridge 1 Hz republish timer | 0–1 s |
| BT tick alignment (0.5 Hz) | 0–0.5 s |
| **Total** | **~3–4 s** |

With 8 attempts (4 s) the window closed just as OFFBOARD was being established.

**Fix.** `demo_config.py`, `recover_offboard_max_attempts`: **8 → 30** (30 × 0.5 s = 15 s).

```python
# demo_config.py
"recover_offboard_max_attempts":   30,
```

**Verification.** In the successful run, publish #1600 (`mode=OFFBOARD`) appeared at `t₀ + 6.4 s` after START_LEAD. The 15 s window gave ~8.6 s of margin.

---

### Bug 2 — jMAVSim timestamp drift → flight termination at ~178 s

**Symptom.** PX4 entered `UNKNOWN(10,0)` (flight termination) approximately 178 s into any run started without lockstep. Log evidence:

```
ERROR [vehicle_imu] timestamp error timestamp_sample: 1783204882044000, previous: 178320488
```

**Root cause.** Without the `-l` lockstep flag, jMAVSim uses the wall clock (`time.time()` → Unix epoch microseconds, ~1.78 × 10¹⁵ μs). PX4 expects a monotonic boot-relative timestamp that starts near zero. At exactly the moment when the simulation wall-clock value numerically matched the magnitude of the monotonic boot time (~178 s × 10⁶ μs ≈ 1.78 × 10⁸ μs), PX4 detected a >1 s forward timestamp jump, declared the IMU sensor timed out, and triggered failsafe → flight termination.

**Fix.** Start jMAVSim with **`-l`** (lockstep):

```sh
./jmavsim_run.sh -p 4561 -l
```

This is also what `px4-rc.jmavsim` does internally (`./jmavsim_run.sh -l -r 250`).

---

### Bug 3 — jMAVSim wrong working directory

**Symptom.** jMAVSim crashed on startup:

```
ERROR: Could not load Mavlink Schema:
  /tmp/sitl_iris_1/mavlink/message_definitions/common.xml (No such file or directory)
```

**Root cause.** When jMAVSim is started from `/tmp/sitl_iris_1/` (the PX4 working directory), it resolves the MAVLink schema relative to CWD and cannot find it.

**Fix.** Always `cd` to jMAVSim's own source directory before launching:

```sh
cd /root/src/PX4-Autopilot/Tools/simulation/jmavsim/jMAVSim
```

---

### Bug 4 — `SYS_AUTOSTART` missing from `parameters.bson`

**Symptom.** PX4 started, printed normal boot messages, but TCP port 4561 never opened. jMAVSim connected then immediately dropped. PX4 was running its internal SIH simulator instead of waiting for an external TCP connection.

**Root cause.** The `parameters.bson` in `/tmp/sitl_iris_1/` had been overwritten with only gyro-calibration data from a previous session. Without `SYS_AUTOSTART=10017`, PX4 does not source `px4-rc.jmavsim` and does not start `simulator_mavlink` in TCP-client mode.

**Fix.**

```sh
cp /root/src/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson \
   /tmp/sitl_iris_1/parameters.bson
```

---

### Bug 5 — Wrong launch file name

**Symptom.**

```
file 'relay_launch.py' was not found in the share directory
```

**Fix.** The correct filename is `follower_relay.launch.py`.

---

### Bug 6 — Wrong default `drone_id` in launch file

**Symptom.** `state_bridge` subscribed to `drone/drone-01/state` and published to `/drone_01/drone_state`; none of the drone-02 MQTT messages were bridged.

**Root cause.** `follower_relay.launch.py` defaults to `drone_id="drone-01"`.

**Fix.** Always pass `drone_id:=drone-02` explicitly:

```sh
ros2 launch drone_control follower_relay.launch.py drone_id:=drone-02
```

---

## Successful Run — Timeline (Launch 13)

All six bugs fixed. Second relay attempt (the first was a TIMEOUT due to bugs 2+1 combined):

| Timestamp | Delta | Event |
|---|---|---|
| `1783206497.427` | −1.6 s | `current_role → MOVING_TO_RELAY` |
| `1783206497.429` | −1.6 s | setpoint stream STARTED |
| `1783206499.043` | **0 s** | **START_LEAD sent** (`relay-offboard-68fd6957`) |
| `≤1783206505.455` | ≤+6.4 s | `mode=OFFBOARD` first seen in BT blackboard (state_bridge publish #1600) |
| `1783206570.490` | +71.4 s | `current_role → RELAYING` |
| `1783206570.491` | +71.4 s | `relay_confirmed: status=CONFIRMED` (`assign-2b70bc9c`) |

Drone position at confirmation: `lat=47.39401, lon=8.54398, agl=50.1 m, battery=51%`.

OFFBOARD was maintained continuously from first acquisition through confirmation and beyond (verified to publish #1950, `t₀ + 404 s`).

---

## Notes on Relay System Architecture (for context)

- **`relay_mover`** owns the OFFBOARD keepalive: it streams position setpoints at 3 Hz. Without a continuous setpoint stream, PX4 exits OFFBOARD automatically.
- **`state_bridge`** republishes the last-known drone state at 1 Hz even when MQTT is silent, using the source's original timestamp. This means BT staleness logic is based on the payload's `timestamp` field, not the ROS2 message arrival time.
- **`OffboardModeHeld` (G3)** returns `RUNNING` (not `FAILURE`) while the attempt counter is below `recover_offboard_max_attempts`, which causes the BT tree to report `WAITING_DATA` rather than triggering exit. Once the counter is exhausted it returns `FAILURE`, which propagates through `Inv(OffboardModeHeld) → SUCCESS → FollowerSafetyExit(G3)`.
- **`FollowerSafetyExit`** with `lost_fc_intent=True` returns SUCCESS without writing an RTL command, because PX4's own failsafe owns the airframe in that scenario.
- The relay system auto-retried after the first TIMEOUT: `continuous_monitor` published a `reeval_trigger`, `relay_decision_authority` re-authorized `CONTINUOUS_RELAY`, and `strategy_executor` issued a second MOVING_TO_RELAY — which succeeded.
