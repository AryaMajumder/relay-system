# Multi-Drone Radio Relay Pipeline

A follower drone that autonomously positions itself between a leader drone and ground control (GC) when their direct radio link degrades, computes the optimal relay position from live FSPL-derived signal geometry, and gracefully exits when the geometry becomes infeasible.

## Status

- **310 / 310 unit tests passing** (`pytest tests/`, ~50 s), across 18 test files.
- **Two-drone SITL end-to-end validated** over a **25-minute continuous run** across 5 leader positions (see the influence table below). No RTL flare, no OFFBOARD-lost cascades, no manual intervention.
- **Band-infeasibility exit path validated** by pushing the leader to 2,913 m from GC and observing the follower's `OFFBOARD → HOLD` transition 2:34 after the feasibility crossover.
- ~7,640 LoC across 28 Python modules. Layered along dependency order (waves 0–9).

## Architecture

Three lanes: **Leader** (early-warning detection only), **GC / Cloud** (primary detection + authorization), **Follower** (sense → assess → propose → execute → maintain via a behaviour tree).

See [`docs/architecture.md`](docs/architecture.md) for the full flow diagram (Mermaid, renders natively on GitHub) and the design invariants.

## Exit gate design — the philosophy in one table

The follower's behaviour tree has two families of exit gates, and the difference matters:

| Gate | Selector | Failure mode | Action | Command result |
|---|---|---|---|---|
| **G1** `FcuTelemetryFresh` | ARBITER_SCAN | telemetry lost | `FollowerSafetyExit` | **RTL** |
| **G2** `BatteryStillSufficientToRelay` | ARBITER_SCAN | low battery | `FollowerSafetyExit` | **RTL** |
| **G3** `OffboardModeHeld` | ARBITER_SCAN | OFFBOARD unrecoverable | `FollowerSafetyExit` | **RTL** |
| **G4** `PositionServiceable` | ARBITER_SCAN | band no longer feasible | `ProposeExitRelay` | **HOLD in place** |
| **G5** `RfLinkTelemetryFresh` | ARBITER_SCAN | RF telemetry stale | `ProposeExitRelay` | **HOLD in place** |
| **G6** `RelayStillNeeded` | ARBITER_SCAN | direct link recovered | `ProposeExitRelay` | **HOLD in place** |
| **G7** `RelayLinkAdequate` | ARBITER_SCAN | relay hops degraded | `ProposeReposition` / `ProposeExitRelay` | **reposition or HOLD** |
| **G8** `RelayActuallyImproved` | DIAG_SCAN | auth stale / relay not improving | writes `reauth_requested_at` | **advisory — never exits** |
| **G9** `GpsHealthy` | DIAG_SCAN | GPS degraded | GPS alert published | **advisory — never exits** |

G1–G3 are **safety gates** — the drone is physically compromised, get it home. G4–G7 are **viability gates** — the drone is healthy but the *relay task* is no longer useful or reachable; hold in place or reposition, let the pipeline re-engage if geometry recovers. G8–G9 are **diagnostic gates** — they run unconditionally every tick in a separate `DIAG_SCAN` step before `ARBITER_SCAN`, so a G1–G7 alarm never suppresses them. Neither G8 nor G9 can exit the relay; their outputs are advisory signals to the GC.

## Demonstrated behaviour

### Leader tracking under changing geometry (25-min continuous run)

Follower re-authorization triggered by leader position changes across a diamond of 5 waypoints (240 s hold at each), holding altitude 50 m AGL:

| # | Leader position | Leader d_GC | Follower position | Follower d_GC | F↔L distance | Re-auth fired |
|---|---|---|---|---|---|---|
| P1 | 47.40100, 8.54900 (NE) | **1,398 m** | 47.39590, 8.54455 | 740 m | ~660 m | Yes |
| P2 | 47.39774, 8.55100 (E) | 1,194 m | *same as P1* | 740 m | ~530 m | No — bucketed R_target unchanged |
| P3 | 47.39450, 8.54900 (SE) | 842 m | 47.39459, 8.54687 | 727 m | ~160 m | Yes |
| P4 | 47.39450, 8.54300 (SW) | 549 m | 47.39333, 8.54507 | 532 m | ~200 m | Yes |
| P5 | 47.39774, 8.54559 (base) | 958 m | 47.39523, 8.54378 | 647 m | ~304 m | Yes — clean return to origin |

The `follower d_GC / leader d_GC` ratio ranged 0.53–0.97. As the leader moves closer to GC the relay geometry collapses; the follower's optimal position converges toward the leader. **The P1→P2 non-repositioning is intentional** — the bucketed R_target absorbed a ~200 m leader shift as being within tolerance, preventing thrash.

### Band-infeasibility handling

Leader pushed to (47.41000, 8.56500), **2,913 m from GC**. At severity 0.7 with `radio_range_m=1500`, the follower's per-hop reach `r_G + r_L ≈ 1,740 m` — any greater and no relay position satisfies both hops.

- **T+0:00** — leader crossed feasibility limit at d_GC = 1,753 m. Follower still OFFBOARD, unchanged.
- **T+2:34** — follower flight_mode: `OFFBOARD → HOLD`. Setpoint stream stopped. Drone held in place at its last valid relay position.
- **T+9:58** — leader arrived at target 2,913 m; follower still in HOLD. No thrashing, no re-authorization attempts.
- **Re-engagement** — when leader returned to baseline geometry, follower re-authorized and returned to its original R_target. The round-trip is clean; exact re-engage timing was not captured (see `KNOWN_LIMITATIONS.md`).

The 2:34 delay from crossing to mode change is the sum of `BandSensorNode` re-tick cadence (~1 Hz), the strategy_evaluator round-trip waiting for the next `relay_tasking` window, chain_assigner processing, and the ~500 ms PX4 mode drop after setpoint stream stops.

### Setpoint-stall rate after broker-bridge disable

Mosquitto is single-threaded; a bridge to a dead upstream endpoint was retrying every 5–30 s and starving the message pump. Setpoint stream stalls dropped from **~18 per 10 min → 1 per 15 min**, a ~27× improvement. Each remaining stall self-heals via the mover's `HOLD → START_LEAD` retry loop in ~10 s.

## Architecture Decision Records

### ADR-1 — State-typed ROS 2 topics use `TRANSIENT_LOCAL`, not the default VOLATILE

**Decision:** `/current_role` (and by extension every "current value of X" topic) uses `TRANSIENT_LOCAL` durability, depth=1, on both publishers and subscribers.

**Alternatives rejected:**
- Default VOLATILE + a heartbeat retransmit. Cheaper on the wire, but every subscriber that restarts between transitions gets nothing until the next transition. Silent failure mode.
- One-shot readiness request/response (a "who has the current role?" service). More correct but adds RPC surface area and race conditions on multi-publisher topics.

**Why:** VOLATILE + transition-only publishing is a latent split-brain waiting for a node restart at the wrong moment — one instance of it left the follower stuck for 7+ hours before diagnosis. QoS mismatch is a *silent* contract violation: publisher and subscriber both look correct in isolation; messages just never arrive. `TRANSIENT_LOCAL` costs one message-buffer on the publisher and eliminates the class of bug entirely. The rule: topics carrying "state" get `TRANSIENT_LOCAL`; topics carrying "event" stay VOLATILE.

### ADR-2 — Two exit families: G1–G3 command RTL, G4–G6 hold in place

**Decision:** The follower's BT has two structurally distinct exit paths. Safety failures (`G1 FcuTelemetryFresh`, `G2 Battery`, `G3 OffboardModeHeld`) route through `FollowerSafetyExit` and command RTL. Viability failures (`G4 PositionServiceable`, `G5 RfLinkTelemetryFresh`, `G6 RelayStillNeeded`) route through `ProposeExitRelay` and leave the drone hovering in place.

**Alternatives rejected:**
- Unified exit: all failures RTL. Simpler tree, one exit action to reason about. Would burn battery on flights back home for conditions that will recover on their own.
- Unified exit: all failures HOLD. Symmetric but strands drones with dying batteries or stale telemetry.

**Why:** These are qualitatively different states of the world. G1–G3 mean the drone is compromised — it should get home while it can. G4–G6 mean the relay job is no longer useful right now, but the drone is fine; hold in place at the last valid position and let the pipeline re-engage if geometry recovers. Verified in test: after G4 fired and the follower entered HOLD, when the leader eventually returned to feasible geometry the follower re-authorized and returned to its original R_target with no operator intervention. RTL would have wasted battery on a round-trip home.

### ADR-3 — Bucketed R_target with a tolerance radius, not continuous re-optimization

**Decision:** Chain_assigner writes the authorized R_target verbatim. Strategy_evaluator buckets R_target to a fixed grid; the `RelayActuallyImproved` gate fires only when the leader drifts outside a tolerance radius. Small leader movements produce no follower reposition.

**Alternatives rejected:**
- Continuous re-optimization on every leader position update. Optimal at every instant but produces follower thrash — each ~1 Hz drone_state message could recompute R_target.
- Time-based re-authorization only (fixed 5-min timer). Simple but decoupled from geometry — updates when nothing has changed and misses genuine geometry changes.

**Why:** Follower motion has real cost (battery, sim time, jitter risk). The P1→P2 transition in the 25-min run confirms this: leader moved ~200 m; bucketed R_target was unchanged; follower stayed put; follower ↔ leader distance shifted from 660 m to 530 m and the relay stayed functional. Continuous re-optimization would have flown the follower for no measurable improvement. The `bucket + tolerance-radius + authorization-timer` triple is what the design converged on after two rewrites within one design pass.

### ADR-4 — PX4 params persisted via `PX4_PARAM_*` env vars, not MAVLink writes

**Decision:** `COM_OBL_RC_ACT=5` (fall to `AUTO.LOITER` on OFFBOARD loss, not `POSCTL`) is set at boot via `PX4_PARAM_COM_OBL_RC_ACT=5` in the launch script. PX4's `rcS` auto-applies any `PX4_PARAM_<NAME>=<VAL>` env var.

**Alternatives rejected:**
- ROMFS edit. Proper upstream mechanism but requires editing files inside the PX4 source and rebuilding. Persistent across everything; also easy to lose to a `git pull`.
- Live MAVLink write via `PARAM_SET` + `MAV_CMD_PREFLIGHT_STORAGE`. Worked once, then PX4's GCS MAVLink instance latched onto a stale ephemeral port and stopped acknowledging clients. Not repeatable.

**Why:** Env-var persistence is repo-visible (grep for `PX4_PARAM_`), survives PX4 rebuilds unchanged, requires no MAVLink handshake, and reads well next to the other launch config. `COM_OBL_RC_ACT=0` (`POSCTL`, pilot takes over) is the correct default for a manned aircraft with a physical RC; in SITL it's the wrong default because the simulated RC is always reported available and there is no pilot. Value 5 (`AUTO.LOITER`) is the mode the mover's retry loop can recover from.

### ADR-5 — Local broker cloud bridge disabled, not throttled

**Decision:** Commented out the bridge blocks in the per-drone mosquitto configs.

**Alternatives rejected:**
- Extend `restart_timeout 5 30` to `300 3600`. Reduces bridge-retry noise but doesn't eliminate it.
- Manage the SSH tunnel under systemd so the bridge stays connected. Correct long-term fix but requires knowing who owns the far endpoint.

**Why:** The local control loop (mover → daemon → PX4) is entirely on 127.0.0.1 and does not depend on the bridge. With the bridge target dead, mosquitto's single-threaded pump was being starved by 5–30 s reconnect attempts, enough to delay MQTT setpoint delivery past PX4's 3.0 s stale threshold and cause OFFBOARD dropouts every 30–90 s. Reversible: uncomment and restart when remote-ops needs to come back.

### ADR-6 — SITL kept in lockstep despite the multi-instance sim-rate cost

**Decision:** `boards/px4/sitl/sitl.cmake` retains `ENABLE_LOCKSTEP_SCHEDULER=yes`; jMAVSim invocations retain `-lockstep`.

**Alternatives rejected:**
- Disable lockstep in cmake, drop `-lockstep` from jMAVSim. Attempted, clean-rebuilt, and reverted the same day: PX4's IMU pipeline validates monotonic sensor timestamps *independent of* the scheduler flag. Without lockstep, jMAVSim delivers timestamps that occasionally regress (multiple Java threads writing sensor values), and PX4 rejects the whole IMU stream with `vehicle_imu ... timestamp error`, cascading to `Preflight Fail: No valid data from Accel/Gyro/Baro/Compass`.

**Why:** Lockstep is fighting the multi-instance load on the host (drones move at ~3 m/s instead of 5 m/s cruise; console emits `simulator_mavlink poll timeout` errors), but the alternative doesn't actually work — the timestamp validator lives elsewhere in the sensor drivers. Correctly reverted. The proper long-term fix is switching from jMAVSim to Gazebo, which handles multi-instance lockstep cleanly; not done yet.

## Known limitations

See [`KNOWN_LIMITATIONS.md`](KNOWN_LIMITATIONS.md) for the full inventory. Short version:

- No GC-side software. The "GC" that authorizes proposals is a local stub that auto-authorizes everything.
- No end-to-end pytest. The 25-minute run and the infeasibility test were executed by hand against SITL; all 308 passing tests are unit-level.
- Multi-drone chain relay is a stub. `CHAIN_RELAY` flows through the pipeline; `chain_assigner` assigns a single target. Peer discovery, slot assignment, handoff sequencing between multiple followers do not exist.
- Geofence polygon absent from config. `relay_decision_authority` calls `_point_in_polygon()` if `geofence_polygon` is set. It isn't. Spatial constraint validation silently passes.
- Symmetric re-engagement was observed but not measured. Exact timing for feasible-again → OFFBOARD was not captured because logging was not running during that transition.
- All flight is in simulation (PX4 SITL + jMAVSim). No hardware-in-the-loop, no physical drones.

## Stack

- ROS 2 Humble, `rclpy`, Cyclone DDS
- `py_trees` for the behaviour tree
- PX4 Autopilot SITL (built from source, lockstep enabled) + jMAVSim
- MQTT (mosquitto), `paho-mqtt` for the intra-drone control plane and the state bridge
- MAVLink via `pymavlink` for PX4 command / telemetry
- Python 3.10, pytest, standard-library `http.server` for an operator dashboard, `foxglove-websocket` for a Foxglove Studio bridge
- Ubuntu 22.04

## Repository layout

```
drone_control/                  # 22 top-level modules
  relay_bt/                     # behaviour tree — blackboard, geometry, condition/action nodes, tree_builder
  config/                       # demo_config.py — the entire config surface
tests/                          # 24 test files, 310 test cases (all passing)
launch/                         # ROS 2 launch files
docs/
  architecture.md               # full Mermaid flow diagram + design invariants
KNOWN_LIMITATIONS.md            # honest inventory of what is not built and not tested
```
