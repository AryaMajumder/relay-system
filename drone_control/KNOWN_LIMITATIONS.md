# Known Limitations

Honest inventory of what is not built, what is a simplification of a real system, and what has not been tested end-to-end.

## Not built

### GC-side software

`relay_decision_authority` publishes proposals to MQTT and expects a response. The "GC" that responds is a local stub (`proposal_handler.py`, marked `CLOUD_STUB`) that auto-authorizes every proposal without any fleet-level policy, geofence, priority, or human-in-the-loop check. In a real deployment this would be a cloud service; here it is a local shim.

### Cloud authorization path

`relay_decision_authority._cloud_authorize()` publishes to MQTT and waits 10 s for a response. There is no cloud service on the far end. The code falls back to the local path after timeout.

### Multi-drone chain relay

`ProposeChainRelay` and the `CHAIN_RELAY` strategy exist and flow through the pipeline. `chain_assigner` assigns a single target. Peer discovery, slot assignment, and handoff sequencing between multiple followers are not implemented. Two-drone continuous relay is the only shape actually validated.

### Geofence enforcement

`relay_decision_authority` calls `_point_in_polygon()` if a `geofence_polygon` config key is set. `demo_config.py` does not define this key, so all positions pass the check silently.

## Not tested end-to-end

### No integration test in pytest

All 308 passing unit tests are graph/message-shape/BT-logic tests. There is no automated test that injects `relay_tasking`, asserts `current_role` transitions, and verifies setpoints flow. The 25-minute continuous run and the band-infeasibility test were both executed by hand against SITL.

### Symmetric re-engagement timing not captured

The forward direction (feasible → infeasible) is measured: 2:34 delay from crossing to `OFFBOARD → HOLD`. The reverse direction (infeasible → feasible → resume) has been observed to work — the follower did return to its original R_target — but the exact timing of re-engagement was not captured because monitoring was not running during that transition window.

## Simulation only

- PX4 SITL built from source with lockstep scheduler enabled.
- jMAVSim as the physics simulator.
- Both PX4 instances and both simulators run on the same host under WSL2 Ubuntu 22.04.
- No hardware-in-the-loop, no physical drones, no physical radios.
- Signal quality is FSPL-modelled by `signal_faker` from configured hop severities, not measured.

## Instrumentation left in place

- `relay_mover.py` contains a `rate_probe` warning that fires every ~5 s during operation, logging tick-window statistics. Useful diagnostic; can be silenced in one commit if the log volume becomes a problem.

## Clock synchronization assumption

### Leader position age check requires synchronized clocks

`BandSensorNode` computes leader position age as `self._clock() - leader_state["timestamp"]`, where `leader_state["timestamp"]` is `time.time()` stamped by `px4_agent` on the leader airframe and `self._clock()` is `time.time()` on the follower airframe. The subtraction is only meaningful if the two clocks are synchronized (e.g. GPS time or NTP). Unsynchronized clocks would make the age check unreliable: a follower whose clock runs ahead would see artificially old positions; a follower whose clock runs behind would see artificially fresh ones and fail to detect stale data.

## Not addressed

### Physical drift not detected during RELAYING

`within_acceptance_radius` in `relay_position_tracker` latches `True` at arrival and is never re-evaluated during `RELAYING`. `distance_to_target_decreasing` is locked `False` throughout `RELAYING` — the computation is gated on `not self._arrived`. On a real airframe, physical displacement from the relay position (wind, GPS error) would degrade SNR until G7 fires, at which point the G7 handler exits with `ProposeExitRelay` rather than letting the mover correct. The fix is a live on-station signal in `relay_position_tracker` computed every tick independent of the arrival latch.


- Ground station UI (drone position map with GC and follower markers is provided as a local Leaflet dashboard and a Foxglove Studio bridge, but these are operator-side visualizations, not a real GC).
- Encrypted MQTT (`telemetry_enc` topics carry base64-wrapped payloads but do not integrate a real key management path).
- Battery / cruise-speed airframe constants (`consumption_rate_pct_per_s`, `cruise_speed_mps`) carry arbitrary placeholder values in `config/demo_config.py`'s `DRONE_MODELS["generic"]` section — 0.05 %/s and 12.0 m/s. The `_Unresolved` sentinel machinery (`config/demo_config.py`) remains available: replace either value with `_Unresolved("...")` and any arithmetic on it raises `RuntimeError` at the point of use, preventing silent bad answers. Deliberately arbitrary; tune to the actual airframe before flying.

## Issue A — Silent cross-enclave DDS match loss (observed 2026-10-01)

**One-time observation (2026-10-01 15:11:32 PDT):** both drone-02 subscribers (`capability_assessor_drone_02`, `relay_strategy_evaluator_drone_02`) stopped receiving `/relay_tasking` from the gc-enclave `relay_decision_authority` publisher simultaneously. Publisher continued publishing without pause; subscribers were still alive and ticking. The match never auto-recovered until the subscribers were restarted ~6 h later.

**What was ruled out from log evidence:** no RDA restart, no SROS2/security errors, no application-level log on either side, no ghost publisher (the ~100 received-only round_ids were journal-rotation artifacts, not a second publisher — the test harness was ruled out separately). No test-harness injection either.

**Cause unconfirmed** — the symptom is consistent with CycloneDDS participant liveliness-lease expiry under a transient GC pause or scheduling stall, but the default Cyclone config emits no logs that would distinguish that from a loopback-socket drop.

**Mitigation now active:** CycloneDDS tracing enabled at `fine` verbosity, writing to `/var/log/cyclonedds/cyclonedds.log` with 200MB rotation (see `/etc/ros/cyclonedds.xml` and `/etc/logrotate.d/cyclonedds`). Any recurrence will be captured. No code-level mitigation added yet — a decision between application-layer heartbeats vs. transport-level liveliness tuning is pending repro + trace evidence.

## current_role has two publishers (restart race)

`/{drone}/current_role` is TRANSIENT_LOCAL and has two publishers:

- `strategy_executor` — writes `MOVING_TO_RELAY` on CONTINUOUS/CHAIN authorization and `OPEN_TO_RELAY` on EXIT_RELAY authorization.
- `relay_position_tracker` — writes `RELAYING` when the follower arrives at the authorized `current_relay_target`.

Each publisher keeps its own TRANSIENT_LOCAL cache. A subscriber that restarts gets replays from both caches, in a DDS-implementation-defined order. Observed 2026-10-06: restarting only `capability_assessor` after a prior session had reached RELAYING caused the tracker's latched `RELAYING` to arrive after the executor's latched `OPEN_TO_RELAY`. The subscriber's final view was `RELAYING`, `IsAlreadyRelaying` returned SUCCESS, the BT entered RELAYING_BRANCH against an actually-HOLD follower, and G3 (`OffboardModeHeld`) spam-fired `FollowerSafetyExit` on every tick.

**Operational workaround today:** restart `capability_assessor` and `strategy_executor` together. The executor's startup reconcile (`SESSION_LOG 2026-10-01`) will publish a fresh value that wins the ordering against the tracker's stale latch, provided the restart is close in time.

**Proper fix (not implemented):** `current_role` should have a single owner. Candidates:
- Have the tracker publish `RELAYING` back into the executor (new topic or a service call) and let the executor be the sole publisher of `current_role`.
- Add a session epoch field to each `current_role` message; subscribers reject values from unknown epochs. (See `SESSION_LOG 2026-10-01` option 2 for the general pattern.)
- Collapse the two writers into one node that owns both the authorization-mapping and the arrival-latching logic.
