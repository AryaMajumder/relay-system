# CURRENT_DESIGN.md

**Written from the code, not from other docs.** All references cite file:line in
`drone_control/`. When this document and any other document disagree, this
document is correct and the other is stale. Updated 2026-10-08.

---

## Behaviour tree

Constructed in `relay_bt/tree_builder.py:build_relay_decision_tree`.

```
RelayRoot [Selector]                                              tree_builder.py:270
├── RELAYING_BRANCH [Sequence]                                    tree_builder.py:221
│   ├── IsAlreadyRelaying                                         condition_nodes.py:370
│   ├── BandSensorNode(relay)                                     condition_nodes.py:472
│   ├── DIAG_SCAN [Selector] (unconditional diagnostics)          tree_builder.py:159
│   │   ├── Seq(RelayActuallyImproved,  AlwaysFail)   writes reauth_requested_at on FAIL
│   │   ├── Seq(GpsHealthy,             AlwaysFail)   writes gps_health_advisory
│   │   └── AlwaysSucceed  (terminal → DIAG_SCAN always SUCCESS)
│   └── ARBITER_SCAN [Selector]                                   tree_builder.py:171
│       ├── G1 Seq(¬FcuTelemetryFresh,             FollowerSafetyExit)   tree_builder.py:173-176
│       ├── G2 Seq(¬BatteryStillSufficientToRelay, FollowerSafetyExit)   tree_builder.py:178-181
│       ├── G3 Seq(¬OffboardModeHeld,              FollowerSafetyExit)   tree_builder.py:183-186
│       ├── G4 Seq(¬PositionServiceable,           ProposeExitRelay)     tree_builder.py:188-191
│       ├── G5 Seq(¬RfLinkTelemetryFresh,          ProposeExitRelay)     tree_builder.py:193-196
│       ├── G6 Seq(¬RelayStillNeeded,              ProposeExitRelay)     tree_builder.py:198-201
│       ├── G7 Seq(¬RelayLinkAdequate,             ProposeExitRelay)     tree_builder.py:203-206
│       ├── REAUTH_TIMEOUT  Seq(¬ReauthResponseTimedOut, ProposeExitRelay) tree_builder.py:208-212
│       └── ProposeIncumbentContinuousRelay (incumbent bid)       tree_builder.py:218
└── IDLE_BRANCH [Sequence]                                        tree_builder.py:263
    ├── NotAlreadyRelaying                                        condition_nodes.py:191
    ├── RelayRequestReceived                                      condition_nodes.py:384
    └── TASKING_RESPONSE [Selector]                               tree_builder.py:258
        ├── FULL_ENTRY [Sequence]                                 tree_builder.py:242
        │   ├── TaskingIsValid
        │   ├── BandSensorNode(entry)
        │   ├── LeaderReachabilityFresh
        │   ├── GeometryFeasible                                  condition_nodes.py:129
        │   ├── DataFreshness                                     condition_nodes.py:155
        │   ├── GPSFixAdequate
        │   ├── FlightModeAcceptable
        │   ├── BatteryAboveFloor
        │   ├── GeofenceContainsRelayPos
        │   ├── BatterySufficientForReturn
        │   └── STRATEGY_SELECTION [Selector]                     tree_builder.py:230
        │       ├── Seq(SingleFollowerSufficient, ProposeContinuousRelay)
        │       ├── Seq(ChainFeasible,            ProposeChainRelay)
        │       └── ProposeLetLeaderIsolate(NoStrategy)
        └── ProposeLetLeaderIsolate(CapFail)
```

### Nine maintenance gates (`relay_bt/condition_nodes.py:862`)

| Gate | Class | Selector | On FAIL action | Command result |
|---|---|---|---|---|
| G1 | `FcuTelemetryFresh` (`condition_nodes.py:570`) | ARBITER_SCAN | `FollowerSafetyExit`: `alert_intent=FOLLOWER_SAFETY_EXIT` is written UNCONDITIONALLY; `pending_command=RTL` is written only when `bb["lost_fc_intent"]` is **not** latched. When latched, PX4's own failsafe owns the airframe and RTL is suppressed with an ERROR log (`action_nodes.py:266-293`). | **RTL** (suppressed when `lost_fc_intent` latched) |
| G2 | `BatteryStillSufficientToRelay` | ARBITER_SCAN | `FollowerSafetyExit` (same alert/RTL rules as G1) | **RTL** (suppressed when `lost_fc_intent` latched) |
| G3 | `OffboardModeHeld` | ARBITER_SCAN | `FollowerSafetyExit` (same alert/RTL rules as G1) | **RTL** (suppressed when `lost_fc_intent` latched) |
| G4 | `PositionServiceable` (`condition_nodes.py:673`) | ARBITER_SCAN | `ProposeExitRelay` | **HOLD** (viability) |
| G5 | `RfLinkTelemetryFresh` (`condition_nodes.py:699`) | ARBITER_SCAN | `ProposeExitRelay` | **HOLD** |
| G6 | `RelayStillNeeded` | ARBITER_SCAN | `ProposeExitRelay` | **HOLD** |
| G7 | `RelayLinkAdequate` (N=3 debounce) | ARBITER_SCAN | `ProposeExitRelay` | **HOLD** |
| G8 | `RelayActuallyImproved` (`condition_nodes.py:775`) | DIAG_SCAN | writes `reauth_requested_at`; capability_assessor publishes `reauth_request` | **re-authorization requested**. Tolerance is `min(tolerance_radius_m, band_width_m/2)` (`condition_nodes.py:797-805`), capped by half the band width so a geometrically narrow band can't be "almost right". |
| G9 | `GpsHealthy` | DIAG_SCAN | writes `gps_health_advisory` | **advisory only** |

G1–G3 safety; G4–G7 viability; G8–G9 diagnostic (run every tick in DIAG_SCAN, cannot be short-circuited by G1–G7). REAUTH_TIMEOUT in ARBITER_SCAN wraps `ReauthResponseTimedOut` (`condition_nodes.py:866`) and fires `ProposeExitRelay` only if a reauth remained outstanding past `reauth_response_timeout_s` (120 s).

### G4 check (`condition_nodes.py:679-696`)

G4 checks exactly: (a) `band_fillable` from BandSensorNode and (b) `current_relay_target` inside `geofence_polygon` via `point_in_polygon`. It does **not** check whether the authorized point is still inside the live band — that is gate 8's job.

### GeometryFeasible delegates to BandSensorNode (`condition_nodes.py:129-153`)

`GeometryFeasible` reads `bb["band_fillable"]`, `bb["band_D"]`, `bb["band_r_G"]`, `bb["band_r_L"]`. One shared geometry computation; no independent formula.

### G6 exit threshold (`relay_exit_quality_threshold`)

G6 (`RelayStillNeeded`, `condition_nodes.py:720-737`) FAILS when the direct GC↔leader quality rises **above** `config["relay_exit_quality_threshold"]`. Current config value is **0.7** (`demo_config.py:198`). Prior value was 0.99 (effectively "never let go"); lowered 2026-10-07 after audit — no deliberate rationale was recorded for 0.99. Node code uses the same default (0.7) if the config key is missing.

### Incumbent bid (`action_nodes.py:305-376`)

`ProposeIncumbentContinuousRelay` is the terminal child of ARBITER_SCAN. When all G1–G7 pass:
- Reads `bb["R_target"]` (the live band center written each tick by BandSensorNode, **not** the authorized `current_relay_target`).
- If `R_target` is unset, returns SUCCESS with no proposal (so RELAYING_BRANCH's Sequence completes).
- Else writes `bb["pending_proposal"]` with `strategy="CONTINUOUS_RELAY"`, `reason="incumbent_bid"`, `trigger="incumbent"`, and a fresh `proposal_id = "prop-" + uuid.uuid4().hex[:12]` **per bid** (`action_nodes.py:351`).

### Decline reason attribution

`ProposeLetLeaderIsolate` has two tree-wiring modes identified by parsing `self.name`:
- `(CapFail)` — reason = `bb["last_capability_check"]` (set by every `ConditionNodeBase._set`, `condition_nodes.py:76-85`).
- `(NoStrategy)` — reason = literal `"no_strategy"`.

`ProposeExitRelay` carries `trigger = <gate>` parsed from `self.name` (one of `G4|G5|G6|G7|ReauthTimeout`), independent of blackboard-inferred `reason`.

---

## Topics

### Shared / per-drone (prefix `/{drone_id_underscored}` for per-drone)

| Topic | Publisher(s) | Subscriber(s) | QoS |
|---|---|---|---|
| `/relay_tasking` | `relay_decision_authority.py:556` | `capability_assessor.py`, `relay_strategy_evaluator.py` | default |
| `/{d}/relay_request` | `leader_link_detector.py:141` | `relay_decision_authority.py:600` | default |
| `/{d}/strategy_proposal` | `relay_strategy_evaluator.py:236` | `relay_decision_authority.py:603` | default |
| `/{d}/authorization` | `relay_decision_authority.py:564` (per drone) | `strategy_executor.py:180` | default |
| `/{d}/current_role` | **`strategy_executor.py:163`** AND **`relay_position_tracker.py:229`** (two publishers) | `capability_assessor.py`, `relay_mover.py`, `continuous_monitor.py`, `relay_position_tracker.py` | **TRANSIENT_LOCAL** |
| `/{d}/relay_assignment` | `chain_assigner.py:149` | `capability_assessor.py`, `relay_mover.py`, `relay_position_tracker.py` | default |
| `/{d}/relay_confirmed` | `relay_position_tracker.py:227` | `continuous_monitor.py:107` | default |
| `/{d}/position_reached` | `relay_position_tracker.py:226` | — | default |
| `/{d}/movement_status` | `relay_position_tracker.py:228` | — | default |
| `/{d}/capability_report` | `capability_assessor.py:190` | `relay_strategy_evaluator.py:240`, `continuous_monitor.py:110` | default |
| `/{d}/pending_command` | `capability_assessor.py:191` | — (MQTT bridge consumes) | default |
| `/{d}/alert_intent` | `capability_assessor.py:198` | `relay_decision_authority.py:612` | default |
| `/{d}/reauth_request` | `capability_assessor.py:201` | `relay_decision_authority.py:606` | default |
| `/{d}/reeval_trigger` | `continuous_monitor.py:98` | `capability_assessor.py:255` | default |
| `/{d}/drone_state` | `state_bridge.py:67` | `relay_mover.py`, `relay_position_tracker.py`, `capability_assessor.py` | default |
| `/gc/gc_link_quality` | `gc_link_observer.py:79` | `relay_decision_authority.py:591-593` (`_on_gc_link_quality`) | default |
| `/{d}/gc_leader_direct_quality` | `gc_link_observer.py` | `capability_assessor.py:461` (dynamically created on relay_assignment) | default |
| `/*/radio_health` (per-role) | the three `*_radio_health_reader.py` nodes publish their own hop's health via the shared `_radio_health_core.ReaderBase` after consuming `/signal/*` | consumed by `BandSensorNode` via blackboard writes | default |
| `/signal/*` (per-hop) | `signal_faker.py` (SITL only — single node publishes all five hop topics) | the three `*_radio_health_reader.py` nodes | default |

**`current_role` has two publishers** — see KNOWN_LIMITATIONS "two publishers (restart race)".

---

## RDA round logic (`relay_decision_authority.py:72-493`)

### State (`_DecisionCore.__init__`, lines 83-126)
- `_current_round_id`, `_collected` (dict `drone_id→proposal`), `_window_start`, `_window_closed`, `_rebroadcast_at`, `_round_start_at`
- `_armed` (bool, line 102) — ready-to-fire on next quality drop
- `_last_gc_quality` (float|None, line 108) — updated on every `on_gc_link_quality`
- `_active_auths` (dict `drone_id→valid_until`, line 109)
- `_rr_dedup` (`relay_request` dedup table 1), `_decline_dedup` (dedup table 2)
- `_alert_intents` (ring buffer per drone)

### Triggers (`on_*` handlers)
- `on_gc_link_quality` (lines 136-158) — **PRIMARY**. Fires `_start_round("initial")` when quality drops below `_GC_QUALITY_TRIGGER` (0.5, line 55) AND `_armed=True`. On recovery, sets `_armed=True` (does **not** cancel `_rebroadcast_at`).
- `on_relay_request` (lines 160-187) — **enrichment / dedup logging only; does NOT start a round.**
- `on_reauth_request` (lines 189-209) — fires `_start_round("reauth")` when no round active.
- `on_alert_intent` (lines 211-243) — ring-buffers. **If `type == "FOLLOWER_SAFETY_EXIT"`, pops drone from `_active_auths`** (lines 237-243; SESSION_LOG 2026-10-05 fix).
- `on_strategy_proposal` (lines 245-314) — insert-or-replace in `_collected` keyed by `drone_id` (BUILDSPEC §4.10 step 3b). Logs `Collected inserted/replaced: ... gate=<trigger_context.gate_fired>` (line 306-312, "gate=" field added 2026-10-06).

### `check_timers` (lines 316-348)
Window-close and rebroadcast gating:
- If `_rebroadcast_at` elapsed → prune `_active_auths` of expired entries → **fire only if** `_last_gc_quality < _GC_QUALITY_TRIGGER` **AND** `_active_auths` is empty (lines 331-347). Otherwise clear `_rebroadcast_at` and log suppression reason. (SESSION_LOG 2026-10-04 fix.)

### `_grant_authorization` (lines 481-521)
- Publishes `/{drone}/authorization` with `valid_until = now + authorization_validity_s`.
- Records `_active_auths[drone_id] = valid_until` for CONTINUOUS/CHAIN; pops the entry for EXIT_RELAY.
- Calls `_maybe_schedule_rebroadcast_on_empty_auths` to re-poll the fleet after an EXIT_RELAY if the link is still degraded. No unconditional post-auth rebroadcast — the old scheduled fire was always suppressed by gating and has been removed (SESSION_LOG 2026-10-07).

### `_maybe_schedule_rebroadcast_on_empty_auths`
Called from three paths — EXIT_RELAY authorization, FOLLOWER_SAFETY_EXIT in `on_alert_intent`, and auth-expiry pruning in `check_timers`. If `_active_auths` is empty AND `_last_gc_quality < _GC_QUALITY_TRIGGER` AND no rebroadcast is already pending, schedules `_rebroadcast_at = now + rebroadcast_pause_s`. Idempotent. This is the only path that re-polls the fleet while jamming persists.

### `_handle_decline` (lines 472-493)
LET_LEADER_ISOLATE always removes the drone from `_collected` (hard rule); dedup is logging-only.

---

## capability_assessor (`capability_assessor.py`)

### `_on_reeval_trigger` (lines 381-403)
- `reason == "relay_completed"` → **no reauth_request published** (SESSION_LOG 2026-10-06 fix). Event is still logged.
- Other reasons (currently only `snr_degraded`) → publish `reauth_request` with `reason="reeval:<reason>"`.

### Gate-8 reauth path (lines 475-486)
Each tick: if `bb["reauth_requested_at"]` is set and `_reauth_request_sent` is False → publish `reauth_request` once; flag set. Flag reset on new `relay_assignment` (`_apply_relay_assignment`, line 134-146).

---

## strategy_executor (`strategy_executor.py`)

### Mapping (§4.8)
- `CONTINUOUS_RELAY` / `CHAIN_RELAY` → publish `current_role=MOVING_TO_RELAY`.
- `EXIT_RELAY` → publish `current_role=OPEN_TO_RELAY`.
- `REPOSITION_RELAY` and `LET_LEADER_ISOLATE` → no role change.

### Startup reconcile (`strategy_executor.py:_load_persisted_auth` + `__init__` reconcile)
- Reads `/var/lib/drone-control/active_auth_{drone_id}.json` (overridable via `STRATEGY_EXECUTOR_STATE_DIR`).
- Checks the payload's `valid_until` against `now` (not file age). If `now < valid_until` and strategy is CONTINUOUS/CHAIN → publish `MOVING_TO_RELAY`.
- Else (missing / expired / malformed / EXIT_RELAY) → publish `OPEN_TO_RELAY` (clears stale TRANSIENT_LOCAL latch).
- Rationale: the authorization is the authority on its own validity. Previous implementation used a 300 s file-age cap that could evict a still-valid authorization and tear down an active relay (SESSION_LOG 2026-10-07).

---

## continuous_monitor (`continuous_monitor.py`)

Two `_fire(reason, …)` call sites — these are the **only** reasons it publishes:

| Reason | Fired by | Coverage by BT gates |
|---|---|---|
| `snr_degraded` | `_on_signal_report`, line 210 | **Not covered** — faster than G7's 3-tick debounce; complementary |
| `relay_completed` | `_on_relay_confirmed`, line 227 | **Covered by G8** (runs every tick in DIAG_SCAN). Event delivered but suppressed in capability_assessor — see above. |

---

## Blackboard keys (writers / readers)

Keys written by `BandSensorNode` (`condition_nodes.py:547-559`):
`band_D`, `band_r_G`, `band_r_L`, `band_t_lo`, `band_t_hi`, `band_fillable`, `R_target`, `stale_radio_health`, `predicted_inside_band`.

Keys written by condition nodes via `_set` (`condition_nodes.py:84`):
`last_capability_check` ← every ConditionNode's `self.name` on every tick.

Keys written by action nodes:
- `pending_proposal` ← `ProposeContinuousRelay`, `ProposeChainRelay`, `ProposeLetLeaderIsolate`, `ProposeExitRelay`, `ProposeIncumbentContinuousRelay`
- `pending_command` ← `FollowerSafetyExit` (RTL)
- `alert_intent` ← `FollowerSafetyExit`

Keys written by capability_assessor on `_apply_relay_assignment` (`capability_assessor.py:134-146`):
`current_relay_target`, `authorization_valid_until`, `tolerance_radius_m`, `reauth_requested_at=None` (reset).

Keys written by gate 8 `RelayActuallyImproved` (`condition_nodes.py:775-826`):
`reauth_requested_at` on FAIL.

Keys read by capability_assessor each tick (`_build_report`, `_drain`, lines ~164-549):
`pending_proposal`, `pending_command`, `alert_intent`, `drone_state`, `current_role`, `gc_leader_direct_quality`, `follower_severity`, `gc_severity`, `leader_severity`, `band_fillable`, `band_D`, `band_r_G`, `band_r_L`, `band_t_lo`, `band_t_hi`, `R_target`, `current_relay_target`, `authorization_valid_until`, `relay_tasking_received`, `reauth_requested_at`, etc.

---

## Config keys (values at `demo_config.py`)

| Key | Value | Line |
|---|---|---|
| `gc_pos` | `{lat:47.39, lon:8.54, alt:0}` | 95 |
| `leader_pos` | `{lat:47.3980, lon:8.5476, alt:50}` | 99 |
| `leader_id` | `drone-01` | 104 |
| `gc_radio_range_m` / `leader_radio_range_m` / `follower_radio_range_m` / `radio_range_m` | **1500** | 121-124 |
| `geofence_polygon` | 4-vertex box 47.38–47.43, 8.53–8.58 | 128-133 |
| `baseline_noise_dbm` | -95 | 147 |
| `noise_range_db` | 40.0 | 152 |
| `tx_power_dbm` | 20.0 | 156 |
| `frequency_mhz` | 915 | 161 |
| `jamming_severity_factor` | 0.6 | 173 |
| `battery_reserve_pct` | 30 | 177 |
| `fcu_telemetry_max_age_s` | 6.0 | 198 |
| `rf_link_max_age_s` | 5.0 | 199 |
| `radio_health_max_age_s` | 10.0 | 203 |
| `stale_severity_floor` | 0.5 | 205 |
| `collection_window_s` | 45 | 230 |
| `rebroadcast_pause_s` | 120 | 234 |
| `relay_request_dedup_expiry_s` | 600 | 239 |
| `decline_dedup_expiry_s` | 180 | 243 |
| `boot_grace_window_s` | 600 | 248 |
| `tolerance_radius_m` | 10.0 | 253 |
| `relay_exit_quality_threshold` | 0.7 | 198 |
| `leader_position_max_age_s` | 10.0 | 211 (added 2026-10-07) |
| `authorization_validity_s` | 1800 | 257 |
| `reauth_response_timeout_s` | 120 | 262 |
| `return_margin_buffer_pct` | 10.0 | 267 |
| `position_bucket_m` | 5.0 | 281 |
| `bt_tick_period_s` | 0.5 | 295 |
| `hop_severities` | `gc_to_leader=0.70, leader_to_gc=0.70, gc_to_follower=0.35, follower_to_gc=0.35, leader_to_follower=0.35` | 315-321 |
| `_GC_QUALITY_TRIGGER` (env var `GC_QUALITY_TRIGGER`) | 0.5 | `relay_decision_authority.py:55` |

## Freshness constants

See the config-key table above for the live values and `demo_config.py:198-211` for the source. All subscribers that write to the blackboard timestamp their inputs so freshness is checked against origin time, not local-arrival time.

## Band math at `leader_radio_range_m=1500`

With `gc_severity=leader_severity=0.70`, `follower_severity=0.35`, `factor=0.6`:
- `cap_gc = cap_ldr = 1500 × (1 − 0.70 × 0.6) = 870 m`
- `r_G = r_L = min(1500, 870) × (1 − 0.35 × 0.6) = 687 m`
- `r_G + r_L = 1375 m`

The band is fillable iff `D ≤ 1375 m`. Diamond waypoints (see `demo_config.py:107-120`): P5-base (958m) ✓, P4-SW (549m) ✓, P3-SE (842m) ✓, P2-E (1194m) ✓, **P1-NE (1398m) ✗** (margin −24m).

---

## Freshness rules

| Signal | Freshness source | Max age | Reader |
|---|---|---|---|
| `drone_state` | `data["timestamp"]` from `px4_agent` on each airframe | `fcu_telemetry_max_age_s=6` and `staleness_windows_s.drone_state=10` | G1 and `DataFreshness` |
| `leader_state.timestamp` | FCU timestamp, preserved by `state_bridge` | `leader_position_max_age_s=10` | `BandSensorNode` (fail-closed → `band_fillable=False`, SESSION_LOG 2026-09-30) |
| `follower_severity` | blackboard-write timestamp | `rf_link_max_age_s=5` | G5 |
| `gc_severity` / `leader_severity` / `follower_severity` (for band compute) | blackboard-write timestamps via `get_with_freshness` | `radio_health_max_age_s=10` | BandSensorNode |
| severity stale fallback value | — | — | `stale_severity_floor=0.5` |
| `authorization_valid_until` | GC-stamped `valid_until` field | `authorization_validity_s=1800` | G8 |

---

## Dead / removed items

- `REPOSITION_RELAY` strategy — removed from the design 2026-09-29 (commit `3d5ee8e`). No node writes it; `strategy_executor` would ignore it via the "unrecognized strategies are silently ignored" catch-all.
- `link_thresholds` config block, `down_threshold_s`, `up_threshold_s` — deleted 2026-10-07 as unreferenced.
- `signal_reader.py`, `follower_signal_faker.py` — deleted 2026-10-07. The single `signal_faker.py` publishes all five hop topics.
- `gc_radio_health_publisher.py`, `leader_radio_health_publisher.py` — deleted 2026-10-07 as unreferenced. See KNOWN_LIMITATIONS for the hardware-radio-health gap.

## Two drone_state freshness windows (not a bug)

- G1 `FcuTelemetryFresh`: uses `fcu_telemetry_max_age_s = 6.0` (`demo_config.py:198`, `condition_nodes.py:593`).
- `DataFreshness` entry gate: uses `staleness_windows_s["drone_state"]` (also 6.0 as of 2026-10-07; was 10.0 before).
Both read the same `drone_state["timestamp"]`. The values were aligned 2026-10-07 so entry and maintenance don't disagree on "fresh".

## Historical pointers

Behavioural changes listed with the commit that landed them (`git log --oneline`):
- `(2026-10-07)` — rebroadcast triggered on auth-set-empty transitions; dead post-auth schedule removed; reconcile uses `valid_until`; `relay_exit_quality_threshold=0.7`; `staleness_windows_s["drone_state"]=6.0`; `link_thresholds` dropped; `leader_position_max_age_s` added to config; publisher scripts and dead faker/reader deleted.
- `8eeebf2` — `relay_completed` no longer triggers reauth.
- `15c57ae` — incumbent bid gets a fresh `proposal_id` per bid.
- `125f6be` — incumbent bid reads `R_target`, not `current_relay_target`; RDA log line carries `gate=`.
- `5922a63` — `GeometryFeasible` delegates to `BandSensorNode`.
- `7e42588` — decline reason attribution (CapFail / NoStrategy).
- `85c8e2f` — `ProposeIncumbentContinuousRelay` terminal added.
- `48f007b` — evaluator stopped synthesising LET_LEADER_ISOLATE.
- `1d677e2` — RDA drops `_active_auths` on FOLLOWER_SAFETY_EXIT.
- `817e22e` — rebroadcast gating in `check_timers`.
- `2f681c5` — `strategy_executor` persists & reconciles on startup.
- `7f16cb1` — `ProposeExitRelay` gate attribution.
