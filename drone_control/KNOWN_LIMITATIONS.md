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

## Not addressed

- Ground station UI (drone position map with GC and follower markers is provided as a local Leaflet dashboard and a Foxglove Studio bridge, but these are operator-side visualizations, not a real GC).
- Encrypted MQTT (`telemetry_enc` topics carry base64-wrapped payloads but do not integrate a real key management path).
- Battery / cruise-speed airframe constants (`consumption_rate_pct_per_s`, `cruise_speed_mps`) carry arbitrary placeholder values in `config/demo_config.py`'s `DRONE_MODELS["generic"]` section — 0.05 %/s and 12.0 m/s. The `_Unresolved` sentinel machinery (`config/demo_config.py`) remains available: replace either value with `_Unresolved("...")` and any arithmetic on it raises `RuntimeError` at the point of use, preventing silent bad answers. Deliberately arbitrary; tune to the actual airframe before flying.
