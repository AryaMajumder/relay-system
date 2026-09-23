# Inline Annotation Reference

Every non-obvious line in the build has an explanatory comment above it in the source file.
This document catalogues those annotations by wave, so a reader can find the WHY behind any decision without session context.

## How to read this document

Land here first if you're a new session. The rest of the document is file-by-file
reference; these front-matter sections give the big picture:

1. **System Overview** — three layers, ownership rules, what talks to what.
2. **Data Flow** — trigger → decision → execution → setpoint, end-to-end walkthrough.
3. **Live vs Legacy** — which .py files are deployed, which are dead code on disk.
4. **Known Quirks & Design Decisions** — the non-obvious surprises, ranked by likelihood of biting you.
5. **SESSION_LOG Index** — pointers to `/root/SESSION_LOG.md` entries by topic.
6. **Audit Discipline** — the review rules that closed 6+ classes of silent-failure bugs.

Then the file-by-file annotations organized by BUILDSPEC wave (§1). If you're
looking for "what does this code do" the file rows have it. If you're looking
for "why this design" the SESSION_LOG date entries do.

Two companion documents in the same tree:
- `/root/BUILDSPEC.md` — normative spec (§2 message schemas, §4.x per-file).
- `/root/SESSION_LOG.md` — chronological decision record (60+ dated entries).
- `deploy/systemd/README.md` — deployment recipe.
- `deploy/smoke/README.md` — topology smoke test.
- `deploy/dev-scaffolding/README.md` — SITL-only utilities.

---

## System Overview

### Three-layer architecture (BUILDSPEC §5.5)

```
LAYER 1 — Blackboard-only (relay_bt/)
  condition_nodes.py, action_nodes.py, tree_builder.py, blackboard.py, geometry.py
  Hard rule: read/write BB only, NO I/O, NO ROS2 publish/subscribe.
  Owns: decision logic, gate evaluations, cost math.

LAYER 2 — BT hosts + sole publishers (capability_assessor.py)
  Ticks the BT at 0.5 Hz, drains BB outputs, publishes messages.
  Hard rule: sole publisher for BT-owned topics (capability_report,
             reauth_request, alert_intent, current_role subscription).

LAYER 3 — Network-facing decision layer (BUILDSPEC §5.5)
  relay_strategy_evaluator, relay_decision_authority, chain_assigner,
  strategy_executor, relay_position_tracker, relay_mover, continuous_monitor.
  Handles proposal enrichment, authorization, execution, monitoring.
```

### Topic ownership (invariants enforced by `deploy/smoke/topology_smoke_test.py`)

**Shared broadcast (1 publisher, N subscribers):**
- `/relay_tasking` — RDA broadcasts (§2.4). Every follower listens.

**Sole-publisher per drone (§4.x-exclusive):**
- `/{drone}/strategy_proposal` — `relay_strategy_evaluator` (§4.9)
  — capability_assessor MUST NOT publish here. Enforced by smoke test.
- `/{drone}/authorization` — `relay_decision_authority` (§2.6)
- `/{drone}/relay_assignment` — `chain_assigner_{drone}` (§2.7)
- `/{drone}/relay_confirmed` — `relay_position_tracker_{drone}` (§2.10)
- `/{drone}/reauth_request` — `capability_assessor_{drone}` (§2.11)
- `/{drone}/reeval_trigger` — `continuous_monitor_{drone}` (§2.12)
- `/{drone}/capability_report` — `capability_assessor_{drone}` (§4.11)
- `/{drone}/alert_intent` — `capability_assessor_{drone}` (§4.11 item 11), subscribed by RDA
- `/{drone}/radio_health` — `follower_radio_health_reader_{drone}` (or leader variant, §4.2)
- `/gc/radio_health` — `gc_radio_health_reader` (§4.2)
- `/gc/gc_link_quality` — `gc_link_observer` (§4.3)
- `/{leader}/relay_request` — `leader_link_detector_{leader}` (§2.3)

**Dual publisher by design (2 publishers, split by transition source):**
- `/{drone}/current_role` — `strategy_executor` (MOVING_TO_RELAY / OPEN_TO_RELAY on
  authorization) + `relay_position_tracker` (RELAYING on physical arrival). §4.8.

### Cross-cutting hard rules (never violate without a DEVIATION entry)

- `signal_report` was removed from design (§8 verification checklist).
- `loss_report` orphaned since v6.3 — subscription kept for diagnostics only (§4.13).
- Gate 8 fails on radius OR timer, not AND (§5.4).
- Position `r_target` echoed VERBATIM from proposal → authorization → assignment,
  never recomputed (§5.3 Decision 5). Snapping happens in `relay_strategy_evaluator`
  once and only once.
- `§7.1` hard stops: `consumption_rate_pct_per_s` and `cruise_speed_mps` — do not
  assign values, do not default, raise `_Unresolved` sentinel.

---

## Data Flow (end-to-end)

The full chain that fires when the GC-leader link degrades. Every arrow is a
ROS 2 topic or in-process BB write.

```
signal_faker (SITL only) OR real telemetry
    │  publishes /signal/gc_to_leader, /signal/leader_to_follower/{drone}, etc.
    ▼
[gc_radio_health_reader]   [leader_radio_health_reader]   [follower_radio_health_reader]
    │                            │                             │
    │ /gc/radio_health           │ /{leader}/radio_health      │ /{drone}/radio_health
    ▼                            ▼                             ▼
[gc_link_observer]         (leader BT input)             (follower BT input)
    │
    │ /gc/gc_link_quality  (0.0-1.0)
    ▼
[relay_decision_authority] ← triggers broadcast round when quality < 0.5
    │
    │ /relay_tasking  {round_id, timestamp, trigger}
    ▼
[capability_assessor] (on follower drone)
    │  BT ticks at 0.5 Hz:
    │    IDLE_BRANCH → RelayRequestReceived, TaskingIsValid, GeometryFeasible,
    │                  BandSensorNode(entry), F_cap gates (DataFreshness, GPSFixAdequate,
    │                  FlightModeAcceptable, BatteryAboveFloor, GeofenceContainsRelayPos,
    │                  BatterySufficientForReturn), strategy_selection
    │    → writes bb["pending_proposal"] {strategy, relay_position, cost{...}}
    │    → capability_report includes pending_proposal
    │
    │ /{drone}/capability_report  (includes pending_proposal)
    ▼
[relay_strategy_evaluator]
    │  Content-hash dedup, snap relay_position → r_target on position_bucket_m grid,
    │  enrich with drone_id, round_id (echoed from tasking), capability_snapshot,
    │  trigger_context, eta_s. Convert internal "alt" to §2.7 "alt_m".
    │
    │ /{drone}/strategy_proposal  (§2.5 full shape)
    ▼
[relay_decision_authority]
    │  Collection window (45 s from first proposal). Winner rank:
    │    battery DESC, eta ASC, band DESC, gps DESC. Grant authorization.
    │
    │ /{drone}/authorization  {proposal_id, r_target verbatim, valid_until, ...}
    ▼
[strategy_executor]  ─┬─ /{drone}/current_role → MOVING_TO_RELAY
                      │  (or OPEN_TO_RELAY on EXIT_RELAY)
[chain_assigner_{drone}]
    │  Echoes r_target verbatim to relay_assignment. No recomputation.
    │
    │ /{drone}/relay_assignment  {r_target, tolerance_radius_m, valid_until, ...}
    ▼
[relay_position_tracker_{drone}]  [relay_mover_{drone}]
    │  tracks arrival                │  streams setpoints
    │                                 │
    │ on CONFIRMED arrival:           │ MQTT drone/{drone}/setpoint at 3 Hz
    │   /{drone}/relay_confirmed      │ {x, y, z, yaw} in NED
    │   /{drone}/current_role         │
    │      → RELAYING                 ▼
    ▼                          [production px4_agent daemon]
[continuous_monitor]                (in /opt/drone-command/px4_agent.py)
    │  monitors SNR baseline drift    forwards to PX4 via
    │  during RELAYING                SET_POSITION_TARGET_LOCAL_NED
    │
    │ on SNR degradation:
    │   /{drone}/reeval_trigger
    ▼
[capability_assessor._on_reeval_trigger]
    │  Publishes reauth_request immediately (SNR fast-path,
    │  parallel to G8 geometric/timer path).
    ▼
    /{drone}/reauth_request → RDA starts new round
    (loop back to top)
```

Also flowing continuously:
- `state_bridge_{drone}` bridges MQTT `drone/{drone}/state` → ROS `/{drone}/drone_state`
  (position, battery, flight_mode, home_pos, GPS fix). One instance per drone,
  runs on GC host.
- `capability_assessor` publishes `alert_intent` when `FollowerSafetyExit` fires
  (Layer 1 writes BB, Layer 2 enriches with drone_id + publishes). RDA subscribes,
  ring-buffers per drone.

Approximate end-to-end latency observed under SITL: signal degradation →
first MQTT setpoint ≈ 55 s (dominated by RDA's 45 s collection window).

---

## Live vs Legacy Files

15 .py files are LIVE (in `setup.py` entry points AND `deploy/systemd/` unit files).
4 more sit on disk as dead code — BUILDSPEC §1 lists them as deletions but they
haven't been physically removed.

### LIVE (in production topology)

| File | Wave | Role |
|---|---|---|
| `config/demo_config.py` | 0 | Config surface (§3). |
| `signal_faker.py` | 1 | SITL-only FSPL simulator. Real fleet uses telemetry. |
| `state_bridge.py` | 1 | MQTT drone_state → ROS. Per-drone, runs on GC. |
| `_radio_health_core.py` | 2 | Shared FSPL-inverse helper for the 3 readers. |
| `follower_radio_health_reader.py` | 2 | Own-hop severities → `/{drone}/radio_health`. |
| `leader_radio_health_reader.py` | 2 | Own gc_to_leader hop → `/{leader}/radio_health`. |
| `gc_radio_health_reader.py` | 2 | Per-follower gc_to_follower_{drone} → `/gc/radio_health`. |
| `gc_link_observer.py` | 3 | SNR → quality [0,1] → `/gc/gc_link_quality`. RDA trigger. |
| `leader_link_detector.py` | 3 | SNR < LINK_MARGINAL → `/{leader}/relay_request`. |
| `relay_bt/blackboard.py` | 4 | Timestamped BB with `get_with_freshness`. |
| `relay_bt/geometry.py` | 4 | haversine, FSPL, band, snap, cost math. |
| `relay_bt/condition_nodes.py` | 5 | BT condition nodes (F_cap, G1-G9, band, freshness). |
| `relay_bt/action_nodes.py` | 5 | BT action nodes (ProposeContinuousRelay, FollowerSafetyExit, etc.). |
| `relay_bt/tree_builder.py` | 5 | Assembles the BT (Root Selector → RELAYING_BRANCH / IDLE_BRANCH). |
| `capability_assessor.py` | 6 | Layer 2 — hosts BT, drains BB, publishes. |
| `relay_strategy_evaluator.py` | 7 | §4.9 sole publisher of strategy_proposal. Snap + dedup + enrich. |
| `relay_decision_authority.py` | 7 | GC authority (G_task + G_auth). Broadcast/collect rounds. |
| `chain_assigner.py` | 8 | authorization → relay_assignment. Verbatim r_target. |
| `strategy_executor.py` | 8 | strategy → current_role. Thin, no condition logic. |
| `relay_position_tracker.py` | 8 | Arrival detection. Publishes movement_status + relay_confirmed + RELAYING. |
| `relay_mover.py` | 8 | Streams NED setpoints via px4_agent client to production daemon. |
| `px4_agent.py` | 8 | CLIENT library (imported by relay_mover). NOT a standalone daemon. |
| `continuous_monitor.py` | 8 | SNR fast-path → reeval_trigger. |

Plus `launch/follower_relay.launch.py` (SITL launch aggregator) and `setup.py`.

### LEGACY / DEAD CODE on disk (BUILDSPEC §1 deletions never applied)

| File | Status | Notes |
|---|---|---|
| `follower_signal_faker.py` | Replaced by unified `signal_faker.py` per §4.1. | Delete when comfortable. |
| `signal_reader.py` | §8 verification: "no file publishes signal_report." Deleted from design; still on disk. | Delete. |
| `gc_radio_health_publisher.py` | Legacy pre-radio_health_reader architecture. | Delete. |
| `leader_radio_health_publisher.py` | Legacy pre-radio_health_reader architecture. | Delete. |

None of these appear in `setup.py` entry_points or any systemd unit. Removing them
would clean up the tree; safe to `rm`. Left in for now to preserve blame history.

### External to package (referenced by relay_mover / state_bridge)

- `/opt/drone-command/px4_agent.py` — production daemon. Owns MAVLink to PX4,
  MQTT command handler, OFFBOARD setpoint sender, SQLite persistence. Patched
  2026-08-18 with `_accept_home_raw()` guard (SESSION_LOG entry backfilled
  2026-08-21). NOT in this package's git history; treat as external dependency.

---

## Known Quirks & Design Decisions

Ranked by likelihood of biting you. Everything in this list has bitten someone
already this cycle.

### 0. SAFETY-CRITICAL — `lost_fc_intent` latch: sustained-loss vs transient hiccup

`FcuTelemetryFresh` sets `bb["lost_fc_intent"] = True` when drone_state ages
past `fcu_telemetry_max_age_s` (default 3 s). `FollowerSafetyExit` reads it and
(a) reports `reason="fcu_telemetry_lost"` regardless of the actual trigger,
(b) SUPPRESSES the RTL `pending_command` (PX4 failsafe owns the airframe).

**Historical trap (fixed 2026-08-26):** the latch was set on stale but NEVER
cleared. A 2-second DDS hiccup (e.g. `state_bridge → capability_assessor`
intermittency during the G1 flight test) permanently disabled RTL for the
rest of the process lifetime. Every future safety exit — battery-critical,
offboard-loss, anything — silently dropped RTL and reported the wrong reason.

**Fix (2-part, defense in depth):**
1. `FcuTelemetryFresh` clears the latch on freshness recovery + logs at WARNING.
   Sustained-loss semantic preserved: latch is TRUE for the duration of the
   stale window, cleared the moment telemetry returns.
2. `FollowerSafetyExit` carries `actual_reason` and `fcu_latched` fields in the
   `alert_intent` payload so RDA/operators see the underlying cause even when
   the reported `reason` is masked to "fcu_telemetry_lost." When suppression
   fires, log.ERROR names the actual trigger with a pointer to SESSION_LOG.

If you're reviewing this file for flight safety: run the 8 tests in
`test_wave5_condition_nodes.py::TestFcuTelemetryFreshLatch` and
`test_wave5_action_nodes.py::TestFollowerSafetyExit` — they lock the invariants.

### 1. §4.9 boundary — `strategy_proposal` has ONE publisher, always

`capability_assessor` writes `pending_proposal` to BB. `relay_strategy_evaluator`
subscribes to `capability_report`, reads pending_proposal, and publishes to
`/{drone}/strategy_proposal` after snapping + dedup + §2.5 enrichment.

**Historical trap:** `capability_assessor` used to also publish to
`strategy_proposal` directly (a "drain-and-forward" convenience added in Session 3).
This bypassed snapping (position jitter propagated into every G8 comparison) and
dedup (RDA got the same proposal every 500ms). Fixed 2026-08-24. Topology
smoke test locks it: `pub_count == 1, pub_matches='relay_strategy_evaluator'`.

### 2. §4.8 `current_role` has TWO publishers by design

`strategy_executor` publishes MOVING_TO_RELAY / OPEN_TO_RELAY on `authorization`
receipt. `relay_position_tracker` publishes RELAYING on physical arrival. Split
is by transition source. §4.8 documents; smoke test asserts `pub_count == 2`.

**Ordering-race fix (2026-08-25):** A fresh re-auth on already-RELAYING drone
briefly stale-flips role to MOVING_TO_RELAY. Was silently swallowing SNR events
in `continuous_monitor` (baseline walked to degraded value even when trigger was
suppressed). Fixed by making `_fire` return bool; caller guards baseline update.

### 3. `signal_report` does not exist (§8 verification)

BUILDSPEC §8 explicitly requires no publisher. `DataFreshness` and
`_collect_inputs` in `capability_assessor` were dead-referencing it; fixed
2026-08-20. If you see `signal_report` in new code, remove it.

### 4. `TaskingIsValid.leader_id` env fallback (single-leader only)

BUILDSPEC §2.4 `relay_tasking` schema is `{round_id, timestamp, trigger}` — no
leader_id. `TaskingIsValid` falls back to `config["leader_id"]` from env. Works
for one leader; silently ambiguous for two or more. If fleet grows past one
leader, revisit: RDA should populate leader_id in tasking payload.

### 5. `alert_intent` payload enrichment (single-follower would work without it)

`FollowerSafetyExit` (Layer 1) writes payload without drone_id. `capability_assessor`
enriches at publish boundary. Same class as leader_id: correct-by-accident at one
follower, ambiguous at two.

### 6. R_target internal "alt" vs §2.7 wire "alt_m"

`geometry.compute_relay_position` returns `{lat, lon, alt}` (internal convention).
§2.7 wire format is `alt_m`. Translation happens in `relay_strategy_evaluator._snap_position`
which also strips "alt" from the output. Do NOT re-add "alt" to wire payloads.

### 7. CycloneDDS Enforce discovery is stateful

After ~20+ rapid service restarts under Enforce, the CycloneDDS discovery FSM
gets starved and `ros2 topic list/echo/info` from a CLI participant sees an
inconsistent view of the fleet. **Fleet-to-fleet DDS still works** — only the
CLI/smoke test participant is affected. Reset via full WSL/box reboot.
`shutdown -r` from inside WSL2 does NOT take effect; use `wsl --shutdown` from
Windows side.

### 8. `follower_position` retired (dead code)

`capability_assessor` used to subscribe to `/{drone}/follower_position` and
write bb["follower_position"] — nothing published it, nothing read the BB key.
`BandSensorNode` reads `drone_state.position` directly. Subscription removed
2026-08-24. Smoke test asserts topic MUST NOT exist (`must_be_absent=True`).

### 10. MAVLink port architecture on SITL — primary vs. secondary

PX4 SITL exposes two MAVLink UDP streams per instance. The port assignments on this
rig are fixed and easy to confuse:

| Drone | Primary port | Listener | Secondary port | Listener |
|-------|-------------|----------|---------------|----------|
| drone-01 | 14560 (PX4 sends here) | `drone-pipeline.service` (telemetry only) | 14556 (PX4 sends here) | `px4-agent-drone-01.service` (`PX4_URL`) |
| drone-02 | 15101 (PX4 sends here) | `drone-pipeline.service` (telemetry only) | 15103 (PX4 sends here) | `px4-agent-drone-02.service` (`PX4_URL`) |

The px4_agent daemon uses its `PX4_URL` port for both reading (heartbeat/state) and
writing (SET_POSITION_TARGET_LOCAL_NED, arm/mode commands). The client library's
`PX4_RX_URL` (default 14558) is a third socket that receives nothing on this SITL rig
and is effectively dead — offboard_mode_held comes from MQTT, not this socket.

**Why this bites you:** the drone-02 systemd drop-in (`sitl-ports.conf`) previously
overrode `PX4_URL` to `14541` — a port no PX4 instance sends to. The daemon silently
blocked in `wait_heartbeat()` for hours with no error beyond the initial hang. If
px4-agent-drone-02 logs stop after startup and never show "Heartbeat OK", check
`/etc/systemd/system/px4-agent-drone-02.service.d/` for a drop-in that overrides
`PX4_URL` to a wrong port. Fixed 2026-08-26 to `PX4_URL=udpin:0.0.0.0:15103`.

### 11. `lost_fc_intent` is write-once for the lifetime of the capability_assessor instance

`FcuTelemetryFresh` (G1) writes `bb["lost_fc_intent"] = True` when `drone_state` goes
stale (age > `fcu_telemetry_max_age_s`, default 3 s). **This flag is never cleared
anywhere in the codebase.** Once set, it changes the behaviour of every subsequent
`FollowerSafetyExit` call for the rest of that process's life:

```python
# action_nodes.py — FollowerSafetyExit.update()
if self.bb.get("lost_fc_intent"):
    reason = "fcu_telemetry_lost"   # RTL NOT dispatched — PX4 failsafe owns airframe
```

Even if G3 (OffboardModeHeld) triggers the exit, the reason reported is `fcu_telemetry_lost`
and no `pending_command=RTL` is written to the BB.

**This is intentional design:** once FCU telemetry is lost, the system cannot trust the
airframe state well enough to command RTL safely; PX4's own failsafe must own recovery for
the remainder of that process instance.

**Operational consequence:** if you need to test the G3→RTL path after a G1 event has
already fired in the same session, you must restart `capability_assessor` to get a fresh
BB (clean `lost_fc_intent`):

```bash
systemctl restart drone-control-capability-assessor.service
```

**Verified live 2026-08-26:** both paths exercised end-to-end against running DDS stack.
G1 correctly suppressed RTL; G3 (on fresh BB) correctly dispatched RTL through the full
chain: capability_assessor → /drone_02/pending_command → relay_mover → MQTT → daemon.

### 9. Colcon incremental builds silently drop source changes

`colcon build --packages-select drone_control` sometimes fails to copy Edit-tool
file changes to `install/`. **Workaround, mandatory:** after every build, grep
the installed file for the changed symbol. If missing:
`rm -rf build/drone_control install/drone_control && colcon build`.

---

## SESSION_LOG Index (major entries by topic)

`/root/SESSION_LOG.md` — 3400+ lines, chronological. Grep for topic keywords or
jump to these anchor dates:

### Architecture / topology corrections
- `[2026-08-20]` — **§4.9 CRITICAL DISCOVERY** (double-publisher on strategy_proposal); reverted 2026-08-24.
- `[2026-08-23]` — §4.8 `current_role` dual-publisher documented.
- `[2026-08-25]` — §4.8 ordering-hole verified + fixed at consumer (continuous_monitor baseline).

### Deployment / systemd (Wave 10)
- `[2026-08-19]` — Deployment DECISION (device-level topology, deploy/systemd/ tree).
- `[2026-08-19]` — SROS 2 Enforce parity.

### End-to-end integration cascade (7-layer fix chain)
- `[2026-08-20]` — RETROSPECTIVE cataloging all 7 layers in dependency order.

### CHECK 1 producer/consumer sweep — the BLOCKERs
- `[2026-08-18]` — CHECK 1 original findings.
- `[2026-08-24]` — `follower_position` reclassified BLOCKER → CLEANUP → closed.
- `[2026-08-24]` — `alert_intent` DECISION option (a) — RDA subscribes.
- `[2026-08-25]` — alert_intent drone_id enrichment closed.

### External production daemon
- `[2026-08-21]` (backfilled) — `_accept_home_raw()` patch to `/opt/drone-command/px4_agent.py`.

### Smoke test tool
- `[2026-08-21]` — topology_smoke_test.py adopted.
- `[2026-08-25]` — GREEN BASELINE (16/16 pass, first fully-green run).

### SITL live tests (against real PX4)
- `[2026-08-26]` — Three OFFBOARD properties verified (Property 1/2/3 PASS) via direct MQTT injection.
- `[2026-08-26]` — Safety-exit branch fired live for the first time: G1 path (fcu_telemetry_lost, RTL suppressed) and G3 path (offboard_unrecoverable, RTL dispatched end-to-end). Full chain confirmed.

### Meta-corrections (self-caught patterns)
- `[2026-08-15 00:06]` — DEVIATED field discipline (first).
- `[2026-08-21]` — colcon incremental drops changes.
- `[2026-08-21]` — leader_id design choice unnamed.
- `[2026-08-21]` — alt_m fix stated as verified, was prediction.
- `[2026-08-21]` — Out-of-package edit unlogged (px4_agent daemon).
- `[2026-08-23]` — Fix without updating related records (current_role).
- `[2026-08-23]` — reeval_trigger BLOCKER stale in ANNOTATIONS.
- `[2026-08-25]` — Ordering-hole "accepted" was unverified impact analysis.

---

## Audit Discipline (rules learned this cycle)

Six patterns of silent failure surfaced and were fixed this cycle. Rules
consolidated so future sessions don't rediscover them:

**Rule (a) — review references to changed surfaces:** For any change to a
file, symbol, or § number, grep BUILDSPEC + ANNOTATIONS + SESSION_LOG for
prior mentions. Update every stale reference in the same commit.

**Rule (b) — no "documentation deferred":** Either update in the same commit
or open a tracked BLOCKER. "Deferred" hides the choice.

**Rule (c) — name adopted options:** When picking from an enumerated set,
CORRECTION entry must name which option was picked and why the others were
rejected. Precedent: leader_id was silently adopted as option (c) with no
option-naming trail.

**Rule (d) — verify against install, not source:** After every colcon build,
grep the installed file for the changed symbol. Live-observation on running
services can be against stale install/. Two-part verification: grep-verify
install AND live-observe topic/log.

**Rule (e) — log out-of-package edits:** Every code change to a file outside
`src/drone_control/` MUST have a SESSION_LOG entry. In-package edits get their
audit trail from git + colcon + pytest; out-of-package edits have none of
that infrastructure.

**Rule (f) — races/ordering issues need reproducers:** When accepting a race
as "OK because small window," acceptance MUST include a test that reproduces
the race and verifies the actual impact. "The window is small" is a hypothesis,
not a verification. If no test can be written, escalate to BLOCKER.

**Rule (g) — BLOCKER vs CLEANUP:** BLOCKER means "work stops pending human
decision." CLEANUP means "obvious action, apply it." When investigating a
BLOCKER reveals a seconds-of-judgment fix, reclassify — don't keep it in the
same bucket as real design forks. Apply the cleanup in the same commit.

**Rule (h) — smoke rules assert END STATE:** A smoke test rule locks in an
invariant. The invariant asserted must be the intended end state, not the
current broken state. If the fix is "delete X," the rule asserts X's absence.
If the fix is "wire Y," the rule asserts Y's presence. A rule matching the
broken state today locks in the wrong invariant.

**Rule (i) — smoke rule for every OPEN BLOCKER:** For every OPEN BLOCKER in
SESSION_LOG, a smoke test rule must exist that FAILS while the blocker is open
and PASSES once resolved. Bridges "known issue in log" and "test coverage."

Applied consistently, these turn silent failures into loud ones and prevent
their recurrence. See SESSION_LOG for the specific incidents each rule caught.

---

## Wave 0 — Configuration

### `drone_control/config/demo_config.py`

| Location | Annotation |
|----------|-----------|
| `_Unresolved` class | Sentinel for unresolved model constants. Arithmetic on an `_Unresolved` instance raises `RuntimeError` immediately — §7.1 hard stop. Do not assign defaults or silently use zeros. |
| `hop_severities` dict | Each hop's severity is set independently. They must NEVER be shared — that is a §4.1 hard rule; sharing means all hops degrade together instead of independently. |
| `noise_range_db` | Controls the mapping from severity ∈ [0,1] to noise_dbm: `noise_dbm = baseline + severity × noise_range`. |
| `follower_radio_range_m` | Nominal range used for FSPL-inverse severity derivation. Larger range → lower severity from the same noise level. |
| Legacy alias keys | Keys retained for backward compat with running experiments; not used in any gate logic. |

### `tests/test_wave0_demo_config.py`

| Location | Annotation |
|----------|-----------|
| `TestPerModelSection` docstring | Sentinels were resolved to placeholder floats in a prior session. Tests prove the per-model section structure, not that sentinels are unresolved. |
| `test_no_shared_severities` | Proves the per-hop severity structure — if two hops referenced the same dict entry, mutating one would change the other. |

---

## Wave 1 — Signal Faker / State Bridge

### `drone_control/signal_faker.py`

| Location | Annotation |
|----------|-----------|
| `_HOP_TOPICS` | All five topics must be published on every tick — no conditional skipping. |
| `compute_hop` docstring | `severity` is an INPUT, never computed here (§4.1). `t` is origin time captured before I/O. |
| `t0 = clock()` before loop | Timestamp captured once at tick start, then stamped on all payloads. Advancing the clock mid-tick does not change published timestamps (§5.6). |
| Missing hop key raises | If a drone_id is absent from positions, a KeyError propagates — deliberately not silenced, because a missing position is a configuration error. |

### `drone_control/state_bridge.py`

| Location | Annotation |
|----------|-----------|
| `STALE_WARN_S = 5.0` | 5 s without a new MQTT message triggers a stale warning. Not a gate — just observability. It does not stop republishing. |
| `_PORT_MAP` | Per-drone MQTT broker ports so multiple drones can run on one machine without topic collisions. |
| `_read_password()` | Tries production path, then dev path, then `MQTT_PASS` env var. Allows tests and CI to run without credential files. |
| `_build_mqtt_client()` | paho-mqtt ≥2.0 requires `CallbackAPIVersion`; older paho uses positional args. The try/except handles both. |
| `self._lock` | Guards `_last_payload` and `_last_recv_ts` written by the MQTT thread and read by the ROS2 timer callback — two different threads. |
| `self._last_payload = None` | `None` means "no message received yet". The guard in `_republish_tick` prevents publishing before the first MQTT message arrives. |
| `self._stale_logged = False` | One-shot flag so the "source silent" warning fires once per silence period, not every tick. |
| `create_timer(1.0, ...)` | 1 Hz republish — downstream nodes expect a steady 1 Hz state feed. |
| `keepalive=60` | Broker drops client after 1.5× this without a PING. 60 s is generous. |
| `loop_start()` | Spawns a background thread; avoids blocking the ROS spin thread. |
| `if "timestamp" not in raw` | Only exception to the "don't restamp" rule: adding a timestamp to a message that arrived without any, not overwriting an existing one (§5.6). |
| `is_first` captured under lock | Prevents race between check and write — must be atomic. |
| `_stale_logged = False` inside lock | Prevents two ticks both seeing `stale_logged=False` and both logging. |
| Log outside lock | Logging can be slow; holding `_lock` during logging would block the MQTT thread. |
| `payload is None` guard | Prevents publishing before the first MQTT message. Downstream freshness guards would see a stale zero-timestamp payload otherwise. |
| `msg.data = json.dumps(payload)` | Republishes last-known payload with its original timestamp — not the current time. Downstream freshness guards measure age from this timestamp. |
| `% 10 == 0` heartbeat log | Log every 10 publishes (~10 s) to confirm liveness without flooding. |
| `_stale_logged = True` inside lock | Prevents two nearly-simultaneous ticks from both emitting the warning. |

### `tests/test_wave1_signal_faker.py`

| Location | Annotation |
|----------|-----------|
| `FakeClock.__call__` | Injectable clock so tests verify timestamp behaviour without sleeping (§3.3). |
| `_make_faker` | All five dependencies injected; no real hardware, no ROS2 context. |
| `sev_map` starts from all-zero | Tests that care about specific values override. Avoids accidentally asserting about zero. |
| `_get_severity(hop_name)` | Called once per hop per tick — must return independent values per hop. |
| `test_per_hop_severity_independent` explanation | noise_dbm = baseline + severity × noise_range, so distinct severities produce distinct noise values only if never collapsed. |

### `tests/test_wave1_state_bridge.py`

| Location | Annotation |
|----------|-----------|
| "no ROS2 or MQTT imports" comment | Tests pure transform logic; MQTT/ROS2 wiring is integration-layer concern. |
| `test_timestamp_added_when_absent` | One exception to the "don't restamp" rule: adding a timestamp to an unstamped message is not the same as overwriting an existing one. |

---

## Wave 2 — Radio Health Readers

### `drone_control/_radio_health_core.py`

| Location | Annotation |
|----------|-----------|
| `on_signal` filter | Hard rule §4.2: only update state for subscribed hops; silently ignore everything else. |
| `publish_tick` always publishes | Silence ≡ crash from a downstream perspective. Publish every tick so the aging timestamp allows freshness gates to detect silence. |
| `snr_db` passthrough (Q16) | SNR is passed through unmodified — not recalculated or smoothed. |
| FSPL-inverse formula | `severity = (noise_dbm - baseline) / noise_range`; then `range_m = nominal × (1 - severity × factor)`. |

### `drone_control/follower_radio_health_reader.py`

| Location | Annotation |
|----------|-----------|
| `subscriptions` list | HARD RULE (§4.2): follower never reads `gc_to_leader`. It only reads hops from the leader and GC to itself. |
| `captured_pubs = {}` | Lazy-create ROS2 publishers. Core layer calls `publish()` with arbitrary topic strings; the ROS2 layer creates the publisher the first time it sees each topic. |
| `lambda msg, t=sub_topic:` | Default argument `t=sub_topic` captures the loop variable by value. Without it, all lambdas would close over the last value of `sub_topic`. |
| `create_timer(1.0, ...)` | BUILDSPEC §4.2: publish every tick (1 Hz) even when input is static. |

### `drone_control/leader_radio_health_reader.py`

| Location | Annotation |
|----------|-----------|
| `("/signal/gc_to_leader", "gc_to_leader")` | The leader only cares about the GC-to-leader link — it does not see follower hops. |
| `create_timer(1.0, ...)` | BUILDSPEC §4.2: publish every tick even when input is static. Downstream freshness guards rely on a continuous stream to detect silence. |

### `drone_control/gc_radio_health_reader.py`

| Location | Annotation |
|----------|-----------|
| `hop_name = f"gc_to_follower_{drone_id}"` | Unique key per follower prevents state dict collisions inside `RadioHealthReaderBase`. `"gc_to_follower"` alone is ambiguous when there are multiple followers. |
| `FOLLOWER_IDS.split(",")` | Comma-separated list; default to `DRONE_ID` so a single-drone deployment works without extra configuration. |
| `create_timer(1.0, ...)` | BUILDSPEC §4.2: publish every tick even when input is static. |

### `tests/test_wave2_radio_health_readers.py`

| Location | Annotation |
|----------|-----------|
| `@pytest.fixture(params=["follower","leader","gc"])` | Parametrize the same test table over all three readers. §4.2 schema identity enforced in one place. |
| `test_publishes_when_input_static` explanation | The aging timestamp is what allows freshness guards to detect stale radio_health. If the publisher stopped on silence, downstream could not distinguish "source crashed" from "link is good and quiet". |

---

## Wave 3 — GC Link Observer / Leader Link Detector

### `drone_control/gc_link_observer.py`

| Location | Annotation |
|----------|-----------|
| `_derive_quality` ASSUMPTION | §4.3 says "SNR-derived quality value" but does not specify the mapping. Linear normalization over [0, 2×marginal] chosen so quality < 0.5 ↔ snr_db < LINK_MARGINAL_QUALITY — consistent with `leader_link_detector`'s raw SNR check. |
| `self._subscriptions = [("/gc/radio_health", ...)]` | HARD RULE (§4.3): GC uses its own radio_health — never subscribe the leader's own radio_health topic. |
| `if payload.get("hop") != "gc_to_leader": return` | Hard rule: only accept `gc_to_leader` — silently ignore all other hops on the same topic. |
| `"timestamp": self._last_health["timestamp"]` | Carry origin timestamp from radio_health — do NOT restamp (§5.6). |

### `drone_control/leader_link_detector.py`

| Location | Annotation |
|----------|-----------|
| INTERPRETATION header | BUILDSPEC §4.4 says "hop leader_to_gc" but leader_radio_health_reader only publishes "gc_to_leader". The link is symmetric in the FSPL model; filtering on "gc_to_leader" satisfies the intent. |
| ASSUMPTION header | Suppression topic `/relay_suppression` and payload `{"active": bool}` are assumed — §2 defines no schema for this. |
| DELIBERATELY ABSENT | No sender-side dedup table. All dedup is relay_decision_authority's job (§4.4 hard rule). |
| `publish_tick` HARD RULE | No dedup, throttling, or debounce. Publish every single tick the condition holds. Do not add rate limiting. |
| `self._ts = payload["timestamp"]` | Origin time from radio_health — not the current time (§5.6). |

### `tests/test_wave3_gc_link_observer.py`

| Location | Annotation |
|----------|-----------|
| `test_never_subscribes_leader_radio_health` | Negative proof of own-measurement rule. If observer subscribed a per-drone topic, GC would be using the leader's measurement instead of its own. |
| `test_publishes_on_static_input` | Three ticks, no new input → still publishes. The aging timestamp lets freshness gates detect link silence. |

### `tests/test_wave3_leader_link_detector.py`

| Location | Annotation |
|----------|-----------|
| `test_no_sender_side_dedup` | 5 ticks × same condition → 5 publishes. Proves the hard rule: no dedup on the sender side, all dedup is GC's job. |
| `test_suppression_resume_automatic` | After suppression clears, publishing resumes without any explicit "re-arm" call — clearing the flag is sufficient. |

---

## Wave 4 — Blackboard / Geometry

### `drone_control/relay_bt/blackboard.py`

| Location | Annotation |
|----------|-----------|
| `clock` injectable | Required by TEST_PROTOCOL §3.3 — no test may sleep. |
| `set()` always re-stamps | HARD RULE (§4.5): re-stamp on every call, even when value is identical. Prevents stale-but-unchanged entries from blocking freshness gates. |
| `get_with_freshness` returns `(None, False, None)` | For a never-written key — NOT `float('inf')`. Callers distinguish never-seen (age is None) from stale (age is a large float). Boot-grace logic depends on this distinction. |

### `drone_control/relay_bt/geometry.py`

| Location | Annotation |
|----------|-----------|
| `estimate_battery_cost` HARD RULE | Do NOT catch `RuntimeError` from `_Unresolved`. §7.1: unresolved model constants must propagate immediately. |
| `return_margin_buffer_pct` | Flat additive buffer: `required = cost + buffer_pct`. NOT a percentage-of-cost multiplier. |
| `predicted_inside_band` | DIAGNOSTIC ONLY — never use in a gate. |
| `bucket_position` | Decision 5 target-stability: grid snap prevents re-authorization on GPS drift. |

### `tests/test_wave4_blackboard.py`

| Location | Annotation |
|----------|-----------|
| `FakeClock.advance()` | Simulates minutes of elapsed time in microseconds — essential for freshness-window boundary tests. |
| `test_never_written_key` explanation | `age=None` not `float('inf')`: callers distinguish never-seen from stale. Boot-grace suppression only fires on `None`. |
| `test_set_restamps_unchanged_value` | Proves `set()` re-stamps even when the value is identical — prevents a value that is re-written on every tick from appearing stale. |

### `tests/test_wave4_geometry.py`

| Location | Annotation |
|----------|-----------|
| `_Unresolved` import | Constructs a bad_model that triggers the §7.1 hard stop. |
| `test_return_margin_arithmetic` | Verifies `required = cost + 10.0` — the 10 is a flat additive, not `cost * 1.1`. |
| `test_buffer_is_flat_not_multiplier` | Explicitly checks `required - cost == 10.0` for a variety of distances to rule out multiplier arithmetic. |

---

## Wave 5 — Condition Nodes / Action Nodes

### `drone_control/relay_bt/condition_nodes.py`

| Location | Annotation |
|----------|-----------|
| `ConditionNodeBase clock` | Injectable for tests using FakeClock (§3.3). |
| `DataFreshness` — signal_report and drone_state only | §4.13 removed all loss-metric references. Subscription list and freshness gate both exclude it. §8 checklist: no "loss" reference remains in any file. |
| DELIBERATELY ABSENT: `GCLinkLossAcceptable` | Gate 7 is SNR-only per §4.6. |
| `LeaderReachabilityFresh` | Never-seen vs stale are DISTINCT: boot-grace suppresses never-seen, stale always fails. F_cap never self-suppresses. |
| `RelayActuallyImproved` — §5.4 OR semantics | FAILURE if EITHER the radius is wrong OR the timer has expired. Not AND. |
| `reauth_requested_at` write guard | Only written if not already set — prevents clock reset on repeated FAILURE ticks. |
| `BandSensorNode` — R_target is single source of truth | Decision 5: never recompute a position that has been authorized. |

### `drone_control/relay_bt/action_nodes.py`

| Location | Annotation |
|----------|-----------|
| `_model_constants(config)` | Returns `(cruise_speed_mps, endurance_s)` from `config["DRONE_MODELS"][drone_model_id]`. `endurance_s` is DERIVED as `100.0 / consumption_rate_pct_per_s` — no standalone key. Storing it independently would create a two-truths problem: the two numbers could drift apart. §7.1. |
| `_cost_from_target` | R_target read from blackboard by caller (Decision 5). Speed and endurance come from `_model_constants(config)` — not from drone_state (removed in Session 5) and not from magic fallbacks (8.0 m/s / 1200 s were silently wrong after Session 4 removed config keys). |
| `ProposeContinuousRelay` | Does NOT compute relay position — reads R_target verbatim from BB. |
| `ProposeChainRelay` | Unreachable in 2-drone demo — `ChainFeasible` always returns FAILURE. |
| `FollowerSafetyExit` | §4.7 item 11: writes `alert_intent` to BB. NEVER publishes directly — Layer 1 rule. |
| `ProposeReposition` threshold | Uses `reposition_improvement_threshold_db` (5 dB), NOT `relay_effective_snr_improvement_db` (3 dB). Different thresholds for different decisions. |
| `ProposeReposition` speed/endurance | Uses `_model_constants(self.config)` — same single source as `_cost_from_target`. |

### `drone_control/relay_bt/tree_builder.py`

| Location | Annotation |
|----------|-----------|
| `_n()` helper passes `clock=` | So time-dependent nodes (G8, `ReauthResponseTimedOut`, `LeaderReachabilityFresh`) can be tested with FakeClock. |
| `memory=False` on all composites | No memory-mode hysteresis — each tick re-evaluates from the first child. Memory mode would skip already-succeeded children, hiding condition re-checks. |
| G8_AUTH `Seq(RelayActuallyImproved, AlwaysFail)` in DIAG_SCAN | Runs unconditionally every tick before ARBITER_SCAN. `AlwaysFail` ensures DIAG_SCAN's Selector falls through to G9 and then `DIAG_CONTINUE`. `RelayActuallyImproved` writes `reauth_requested_at` as a side effect. Previously inside ARBITER_SCAN where G1–G7 could skip it. |
| G9_DIAG `Seq(GpsHealthy, AlwaysFail)` in DIAG_SCAN | GPS health is diagnostic only. Runs unconditionally alongside G8 in DIAG_SCAN — never blocked by ARBITER_SCAN's priority logic. |
| CONTINUE `AlwaysSucceed` | All gates passed → continue relaying. |

### `tests/test_wave5_condition_nodes.py`

| Location | Annotation |
|----------|-----------|
| `FakeClock` explanation | Condition nodes call `clock.now()` on each `update()`. Tests call `clock.advance()` to simulate time without any real delay. |
| `test_gate8_timer_expired_inside_radius` explanation | Gate 8 fails if EITHER radius is wrong OR timer expired. Prevents perpetual re-auth avoidance by a proximity-but-expired authorization. |

### `tests/test_wave5_action_nodes.py`

| Location | Annotation |
|----------|-----------|
| `test_nodes_write_blackboard_only` explanation | Source inspection catches unused/conditional imports that runtime checks miss. An unused networking import is still a Layer 1 violation. |
| `_CFG` has `DRONE_MODELS` + `drone_model_id` | Required after the `_model_constants()` fix — tests that exercise `ProposeContinuousRelay` or `ProposeReposition` now call `_model_constants(config)`, which raises `KeyError` if the model section is absent. |

---

## Wave 6 — Capability Assessor

### `drone_control/capability_assessor.py`

| Location | Annotation |
|----------|-----------|
| `movement_status` unpacked into four BB keys | Each gets its own freshness timestamp — one stale guard does not make all four stale. |
| `gc_link_quality` subscription | LIFECYCLE-MANAGED: created on relay_assignment, destroyed on EXIT/DECLINE. Prevents accumulating stale quality readings after relay ends. |
| F_radio via subscription since v6.4 | FSPL-inverse moved to `follower_radio_health_reader`. capability_assessor subscribes the output, not computes it. |
| `_drain()` | BUILDSPEC §4.11: every output slot cleared after publish. Next tick starts clean. |
| `_apply_relay_assignment` four §6 keys | Written atomically; `reauth_requested_at` reset to None so old re-auth requests don't carry over. |
| `pending_proposal` in report before drain | Report snapshot includes the proposal; drain happens after. If order reversed, report would always show `None`. |
| `capability_report` published unconditionally | HARD RULE §4.11: publish every tick regardless of BT outcome. |

### `tests/test_wave6_capability_assessor.py`

| Location | Annotation |
|----------|-----------|
| `debounce_n=1` | Production uses N=3. Setting to 1 means one tick triggers a gate — prevents tests from needing 3+ ticks. |
| `min_gps_fix_type=0` | Harness has no real GPS; accept fix_type=0 so `GPSFixAdequate` doesn't block other gate assertions. |
| `_TestableAssessorCore.tick()` order | Build report BEFORE draining slots: `pending_proposal` must be visible in the report (§4.11). |
| `_teardown_quality_sub()` before drain | Strategy readable before drain; if drained first, can't check strategy to decide lifecycle. |
| `report_captures.append(report)` | Published unconditionally — §4.11 hard rule. |

---

## Wave 7 — Strategy Evaluator / Decision Authority

### `drone_control/relay_strategy_evaluator.py`

| Location | Annotation |
|----------|-----------|
| DELIBERATELY ABSENT: `/link_state` subscription | File doesn't exist. |
| `_make_content_hash` — battery bucketed at 20%, SNR at 5 dB | Position excluded so R_target GPS jitter doesn't generate repeated proposals. |
| `_publish_proposal` | HARD RULE: no `confidence_score`, no `band_range`, no `strategy_type`. |
| `_snap_position` | BUILDSPEC §4.9: snap before position leaves this node. Only place where bucketing happens (Decision 5). |
| `/relay_tasking` subscription | Shared topic §2.4; used to track `round_id` for echoing in proposals. |

### `drone_control/relay_decision_authority.py`

| Location | Annotation |
|----------|-----------|
| DELIBERATELY ABSENT | No MQTT, Lambda, circuit breaker, retry queue — GC runs entirely local. |
| `_rr_dedup` | HARD RULE: does NOT suppress `leader_link_detector` (which keeps publishing). Suppresses logging only. |
| `_decline_dedup` | Direct tuple key. Dedup is logging-only; entry is still removed regardless. |
| `on_gc_link_quality` | Primary trigger: fire on quality drop, re-arm when quality recovers. |
| `on_strategy_proposal` | Window fixed (§4.10) — late arrivals discarded, window not extended. |
| `_pick_winner` | Lexicographic sort: battery DESC, eta ASC, band DESC, gps DESC. No weighting or thresholds. |
| `_grant_authorization` | r_target echoed VERBATIM from winning proposal — never recomputed (Decision 5). |
| `_handle_decline` | Decline always removes entry even when deduped (HARD RULE §4.10). |
| rclpy wildcard note | rclpy does not support topic wildcards — enumerate drones explicitly from config. |

### `tests/test_wave7_relay_strategy_evaluator.py`

| Location | Annotation |
|----------|-----------|
| `test_r_target_snapped` | Proves bucketing happens before leaving the node — the only place snapping occurs (Decision 5). |
| `test_content_hash_dedup_identical_conditions` | Same report twice → one publish. Prevents re-sending a proposal GC already has. |

### `tests/test_wave7_relay_decision_authority.py`

| Location | Annotation |
|----------|-----------|
| `_make_core()` explanation | `clock.advance()` then `core.check_timers()` replaces the real-time ROS2 wall-clock timer that calls `check_timers()` in production. |
| `test_window_fixed_not_extended` | Window starts on first proposal; a late proposal at 44 s must not push the close to 89 s. |
| `test_decline_still_processed_when_deduped` | Decline always removes the entry even when dedup suppresses the log. |

---

## Wave 8 — Strategy Executor / Chain Assigner / Relay Mover / Position Tracker / Continuous Monitor

### `drone_control/strategy_executor.py`

| Location | Annotation |
|----------|-----------|
| **`/{drone_id}/current_role` publisher — SHARED topic** | This node publishes MOVING_TO_RELAY (on CONTINUOUS_RELAY / CHAIN_RELAY) and OPEN_TO_RELAY (on EXIT_RELAY). `relay_position_tracker` also publishes to the same topic — RELAYING on physical arrival. BUILDSPEC §4.8 documents the dual-publisher design and the known ordering hole. Topology smoke test asserts `pub_count == 2`. |
| Dedup guard on `proposal_id` | §4.8 is silent on dedup; guard added defensively — ROS2 replay can deliver same authorization twice. Mirrors chain_assigner's identical guard. **Note:** dedup prevents redelivery races but NOT stale-overwrite races from a fresh re-auth (new proposal_id) while drone is already RELAYING. See §4.8 "Ordering hole." |
| `REPOSITION_RELAY` pass | Drone already RELAYING; only destination changes, no role transition — this node skips the publish. Prevents the ordering-hole race for REPOSITION specifically; race remains for fresh CONTINUOUS_RELAY / CHAIN_RELAY re-auths. |
| `EXIT_RELAY → OPEN_TO_RELAY` | `OPEN_TO_RELAY` per §2.9 enum, not "IDLE". Indistinguishable regardless of exit cause. |
| `__init__` HARD RULE | Authorization is the ONLY subscription. No other subscriptions allowed (§4.8). |

### `drone_control/chain_assigner.py`

| Location | Annotation |
|----------|-----------|
| `EXIT_RELAY` filter | No position assignment needed; drone returns home. |
| Dedup guard | Same `proposal_id` processed twice → only one `relay_assignment` published. |
| `model["cruise_speed_mps"]` direct access | §7.1 hard stop — no soft fallback with `.get()`. |
| `bare except` on `_compute_eta_s` | `eta_s` is observability-only; swallowing the exception is intentional. Never silences gate-level failures. |

### `drone_control/relay_mover.py`

| Location | Annotation |
|----------|-----------|
| Deactivation resets `stream_start + offboard_sent` | Fresh OFFBOARD arming on re-activation after a role gap. |
| Activation starts `stream_start` clock | `OFFBOARD_PRE_SECS` measurement begins exactly when role becomes active. |
| `self._target["alt_m"]` direct access | §2.7 field name is `alt_m`. No fallback — chain_assigner is the only publisher and it always writes `alt_m`. |
| OFFBOARD guard comment | PX4 needs continuous stream before accepting OFFBOARD mode command — stream must precede the START_LEAD by `OFFBOARD_PRE_SECS`. |

### `drone_control/relay_position_tracker.py`

| Location | Annotation |
|----------|-----------|
| `publish_role_fn` | Publishes `current_role=RELAYING` on CONFIRMED arrival. **Note:** `/{drone_id}/current_role` has TWO publishers by design (see BUILDSPEC §4.8). `strategy_executor` publishes MOVING_TO_RELAY / OPEN_TO_RELAY on `authorization` receipt; this node publishes RELAYING on physical arrival. Split is by transition source. Known ordering hole: a fresh re-auth `authorization` can cause `strategy_executor` to publish `MOVING_TO_RELAY` while this node's last publish was `RELAYING`, briefly stale-overwriting until this node's next 3 Hz tick republishes. §4.8 documents. |
| `assignment_id = drone_id` | §2.7 has no explicit `assignment_id` field; drone_id is the closest unique identifier. |
| Timeout safety factor | Grace beyond eta_s estimate; 120 s fallback when `eta_s=0` (unknown). |
| CONFIRMED vs TIMEOUT | Only CONFIRMED arrival publishes `current_role=RELAYING`. TIMEOUT fires `relay_confirmed` with `status=TIMEOUT` and does NOT publish RELAYING. |

### `drone_control/continuous_monitor.py`

| Location | Annotation |
|----------|-----------|
| `relay_completed` bypass | Race condition: `relay_confirmed` fires the moment arrival is detected, but `relay_position_tracker` hasn't yet published `current_role=RELAYING`. Without this carve-out, the MOVING_TO_RELAY suppression would swallow the trigger that kicks off the re-authorization loop. |
| Baseline update after trigger — **only when trigger actually fired** (2026-08-25) | `_on_signal_report` advances `_baseline_snr_db` only when `_fire` returned True.  Prior version updated baseline unconditionally after any threshold-breach detection, which silently swallowed SNR events during the §4.8 stale-MOVING_TO_RELAY window (baseline walked to the degraded value; next tick computed drop=0 against the new baseline; event never fired).  Fix: `_fire` returns bool; caller guards the baseline advance with `if fired:`.  Proven by `test_suppressed_snr_degradation_is_recovered_after_role_reverts` in `test_wave8_continuous_monitor.py` — pre-fix baseline walks to 14 dB, post-fix stays at 20 dB until role reverts. Real §4.8 ordering-race bite, verified and closed. |
| `_fire()` returns bool | Post-2026-08-25: True iff a reeval_trigger message was actually published (i.e. not suppressed by any of the cooldown/status/role gates).  Return value gates the caller's baseline-update logic; walking the baseline on a suppressed fire permanently swallows the underlying signal drop.  See BUILDSPEC §4.8 "Ordering hole" — verified and mitigated. |
| `STALE_WARN_S = 5.0` | 5 s silence triggers a stale warning — observability only, not a gate. |
| `SNR_TRIGGER_DB` (default 5.0 dB) | Drop threshold. A 5 dB drop is significant in FSPL; smaller drops are measurement noise. |
| `reeval_trigger` subscriber (CLOSED 2026-08-18) | Previously a BLOCKER — no file subscribed to `/{drone}/reeval_trigger` published by continuous_monitor. **Resolved:** SESSION_LOG `[2026-08-18] DECISION` added a subscription in `capability_assessor` (`capability_assessor.py:245`) with `_on_reeval_trigger()` that publishes `reauth_request` on receipt (SNR fast-path parallel to G8's geometric/timer path). Proven by 5 tests: `test_snr_degraded_publishes_reauth`, `test_relay_completed_publishes_reauth`, `test_reason_carries_trigger_reason`, `test_drone_id_in_payload`, `test_not_gated_by_reauth_request_sent_flag`. This subscription is NOT in §4.11's Inputs list — see the corresponding `[2026-08-18] DEVIATION` entry in SESSION_LOG. |

### `drone_control/px4_agent.py`

| Location | Annotation |
|----------|-----------|
| `PX4_RX_URL` | Client library's MAVLink read path. **Not the daemon's write path.** On SITL: drone-01 primary (14560) and secondary (14556) are both occupied — pipeline service on 14560, px4_agent daemon on 14556. The client library `PX4_RX_URL=14558` receives nothing, but that's harmless: `offboard_mode_held` is read from MQTT drone_state, not from this socket. |
| `_BROKER_PORT` | Per-drone ports so multiple drones can co-exist on one machine. |
| `_read_mqtt_pass()` | Same search order as `state_bridge._read_password()`. |
| `_build_mqtt_client()` | Same paho version-compat pattern as state_bridge. |
| `_OFFBOARD_MAIN = 6` | From PX4's `vehicle_status.h` (PX4_CUSTOM_MAIN_MODE_OFFBOARD=6). |
| `_decode_offboard()` | `custom_mode` is a 32-bit packed field in MAVLink HEARTBEAT. Bits 16-23 = main mode, bits 24-31 = sub mode. |
| `self._mqtt_ready` gate | Prevents setpoints from being sent when MQTT is disconnected. |
| Daemon thread on `_rx_loop` | Process can exit even if thread is blocked in `connect()`. |
| Infinite retry loop | MQTT reconnects automatically after broker restarts. 5 s delay prevents busy-looping. |
| `_global_to_ned` | Small-angle flat-earth approximation — valid within a few km of home. East displacement scales by `cos(lat)`. NED z is positive-down. |
| `home_pos` gate in `publish_setpoint` | Without home, WGS-84→NED would compute relative to (0,0,0) and send the drone off-grid. relay_mover retries on the next tick (3 Hz). |
| `(0, 0)` placeholder ignored | PX4 sends `home_pos=(0,0)` before GPS fix. Accepting it would make NED conversion silently wrong. Extracted to `_accept_home()` staticmethod so the invariant is directly testable. |
| `_accept_home()` staticmethod | Pulled out of the `on_message` closure so the guard is reachable from tests. Pure extraction — `on_message` calls it; behavior unchanged. |
| `flight_mode` from MQTT state | px4_agent daemon reads PX4 heartbeats on its `PX4_URL` port (drone-01: 14556, drone-02: 15103) and publishes flight_mode to MQTT drone_state at 1 Hz. Client library reads that MQTT message, not a raw MAVLink socket. `PX4_RX_URL` (14558) is a separate client-side read socket that currently receives nothing on SITL — see Known Quirk §10. |

### `tests/test_wave8_px4_agent.py`

| Location | Annotation |
|----------|-----------|
| `_make_agent()` via `object.__new__` | Bypasses `__init__` to avoid starting threads or connecting to MQTT/MAVLink. Same pattern as `test_wave8_continuous_monitor.py`. Instance state set manually. |
| `_FakeMQTT.published` list | Captures `(topic, payload)` from every `publish()` call so tests can assert on NED content without a real broker. |
| `TestDecodeOffboard._OFFBOARD` constant | `(0 << 24) | (6 << 16) = 393216` — the exact `custom_mode` value PX4 sets in HEARTBEAT when OFFBOARD mode is active. Computed from PX4's `vehicle_status.h` bit layout so the test is self-documenting. |
| `test_ned_east_scales_with_cos_lat` | Verifies the `cos(lat)` term is present: same longitude delta at the equator vs. 60° gives a 2:1 distance ratio. A missing `cos(lat)` would collapse both to the same value. |
| `test_publish_ned_values_match_global_to_ned` | End-to-end round-trip: calls `publish_setpoint`, parses the JSON payload, compares to direct `_global_to_ned` output. Proves the conversion is applied, not just that something is published. |
| `TestAcceptHome` edge cases | `nonzero_lat_zero_lon` and `zero_lat_nonzero_lon` are accepted — the prime meridian and the equator are real locations. Only the `(0,0)` origin is the pre-fix placeholder. |

### `tests/test_wave8_strategy_executor.py`

| Location | Annotation |
|----------|-----------|
| `_make_auth` unique `proposal_id` per test | Prevents the dedup guard from suppressing subsequent `on_authorization()` calls within the same test. |
| `_make_core()` | `_StrategyExecutorCore` is the pure-logic extract; `StrategyExecutor` is the ROS2 wrapper. |
| `test_single_subscription` | Source inspection rather than runtime check — counts `create_subscription` calls in `__init__`. |
| `test_exit_sources_indistinguishable` | Both battery and timeout exits produce `OPEN_TO_RELAY`. Strategy executor cannot and must not try to distinguish them. |
| `test_dedup_same_proposal_not_applied_twice` | Defensive engineering — §4.8 is silent on dedup; guard mirrors chain_assigner to prevent double role-change on ROS2 replay. |

### `tests/test_wave8_chain_assigner.py`

| Location | Annotation |
|----------|-----------|
| `_CFG` supplies `cruise_speed_mps` directly | Tests don't depend on DEMO_CONFIG values. Known speed (12.0 m/s) makes eta_s arithmetic deterministic. |
| `clock=lambda: 10100.0` | Fixed-value clock: timestamp in `relay_assignment` is deterministic; tests don't check it. |
| `test_r_target_not_snapped_or_modified` | Source inspection of `on_authorization` proves `bucket_position` and `haversine` are absent — Decision 5 enforced. |

### `tests/test_wave8_actuation.py`

| Location | Annotation |
|----------|-----------|
| `clock_ref = [0.0]` | List wraps a scalar so tests can advance time by writing `clock_ref[0] = 1000.0` without sleeping. Mutable list instead of mutable int. |
| `roles` capture (6th element) | Added when `relay_position_tracker` was given `publish_role_fn` to fix the RELAYING gap — no other node produces `RELAYING`. |
| `_make_tracker_core()` returns 6-tuple | `core, statuses, reached, confirmed, roles, clock_ref`. Roles added after initial 5-tuple. |

### `tests/test_wave8_continuous_monitor.py`

| Location | Annotation |
|----------|-----------|
| `__new__` bypasses `Node.__init__` | Avoids live ROS2 context requirement. All instance state that `__init__` would set is replicated manually below. |
| `NoopLogger` stub | `_fire()` calls `self.get_logger().info()`. Bypassing `Node.__init__` leaves `_logger` unset; `get_logger()` raises `AttributeError` without this stub. |
| `monitor._last_trigger_ts = 0.0` | Resets cooldown between test calls within the same monitor instance. |

---

## Wave 9 — Package Setup

### `setup.py`

| Location | Annotation |
|----------|-----------|
| 6 stale `console_scripts` removed | `fake_state`, `behavior`, `follower`, `leader_bridge`, `follower_relay`, `signal_reader` were renamed or consolidated in prior waves. Entry points for the renamed scripts are already present under new names. |

---

## Reauth Gap Fix — Post-Wave Patch

### `drone_control/capability_assessor.py` (additions)

| Location | Annotation |
|----------|-----------|
| `self._reauth_request_sent = False` | One-shot flag. After G8 fires, every tick sees `reauth_requested_at` set; without this flag, the GC would be spammed every 0.5s. |
| `self._reauth_pub` | Publisher for `/{drone_id}/reauth_request`. Created alongside other publishers in `__init__`. |
| Reauth check in `_tick()` | After the BT tick, if `reauth_requested_at` is set and `_reauth_request_sent` is False: publish once and set flag. This is the missing link that notifies the GC to start a reauth round. |
| `_reauth_request_sent = False` in `_on_relay_assignment` | New authorization period → fresh send opportunity. Without this reset, the second G8 fire in the drone's lifetime would never notify the GC. |

### `drone_control/relay_decision_authority.py` (additions)

| Location | Annotation |
|----------|-----------|
| `on_reauth_request()` | New `_DecisionCore` method. BUILDSPEC §4.10 "reauth" trigger — was always specified but never implemented. |
| Active-round guard in `on_reauth_request()` | If `_current_round_id is not None and not _window_closed`, a round is collecting proposals — do not reset `_collected` mid-flight. The active round will produce the authorization that answers the reauth. |
| `_start_round("reauth")` | First call site that passes trigger="reauth". Previously only "initial" was used. |
| `/{drone_id}/reauth_request` subscription | Added alongside `relay_request` and `strategy_proposal` in the per-drone loop. |
| **`on_alert_intent()`** (2026-08-24) | Closes CHECK 1 [2026-08-18] BLOCKER — capability_assessor was publishing `/{drone_id}/alert_intent` with nothing subscribed; safety-exit alerts were silently dropped. RDA now receives them per §4.11 item 11 (RDA is G_task per §4.10 header). Observability-only: no control loop consumes; the exit is already actioned by the follower's BT via EXIT_RELAY. |
| `_alert_intents` ring buffer | Per-drone bounded list; default cap 32 (`alert_intent_ring_max` config). Oldest evicted on overflow. §4.10 hard rule "local-process, in-memory" — no MQTT/Lambda/cloud forwarding. |
| `/{drone_id}/alert_intent` subscription | Added alongside `reauth_request` in the per-drone loop. §4.10 subscribes list updated 2026-08-24. |

### `tests/test_wave6_capability_assessor.py` (additions)

| Location | Annotation |
|----------|-----------|
| `self.reauth_captures` | Capture list added to `_TestableAssessorCore` for reauth publish assertions. |
| `self._reauth_request_sent` | Mirrors `CapabilityAssessor._reauth_request_sent` in the test harness. |
| `_reauth_request_sent = False` in `apply_relay_assignment` | Mirrors the reset in `_on_relay_assignment`. |
| `TestReauthRequestPublish` class | Five tests: one-shot publish, no repeat, not published without trigger, reset on assignment, second fire after reset. |

### `tests/test_wave7_relay_decision_authority.py` (additions)

| Location | Annotation |
|----------|-----------|
| `TestReauthRequest` class | Four tests: starts round when idle, starts round after closed window, ignored during active round, ignored before first proposal arrives. |

---

*Generated from inline annotations added across all waves 0–9, plus the reauth gap fix.*

---

## Wave 10 — Deployment (systemd)

Device-level systemd deployment lives at `deploy/systemd/` in the package. Each physical host
in the fleet gets one role directory copied to `/etc/systemd/system/` plus the two shared
files from `common/`. Signal_faker is isolated in `sitl/` and never auto-starts.

### `deploy/systemd/common/drone-control-run` (wrapper script)

| Location | Annotation |
|----------|-----------|
| Wrapper purpose | Systemd starts processes with a bare environment; ROS 2 needs ~10 env vars established by sourcing `setup.bash`. Without this wrapper every unit would either duplicate all those vars or use `bash -lc '…'` in ExecStart. |
| `source /opt/ros/humble/setup.bash` | Base ROS 2 distro — rclpy, DDS libs, `ros2` CLI. |
| `source /root/ros2_ws/install/setup.bash` | Workspace overlay — registers drone_control entry points. Re-sourced at every service start so `colcon build` output is picked up without touching unit files. |
| `exec ros2 run drone_control "$@"` | `exec` replaces the shell with the ros2 process so systemd tracks the right PID. |

### `deploy/systemd/common/drone-control.env` (shared EnvironmentFile)

| Location | Annotation |
|----------|-----------|
| `ROS_DOMAIN_ID=42` | Must be identical across every host in the fleet — DDS discovery domain isolation. |
| `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | CycloneDDS chosen over Fast-DDS for lower latency and better multicast on wifi-mesh. |
| `CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml` | Fleet-wide XML must be deployed to same path on every host. |
| `ROS_SECURITY_ENABLE=true` | SROS 2 enabled fleet-wide. |
| `ROS_SECURITY_STRATEGY=Enforce` | Every participant must present a keystore-signed enclave; unsigned participants (e.g. `ros2 topic list` from a fresh shell) are filtered from discovery. Trade-off documented below. |
| `ROS_SECURITY_KEYSTORE=/root/sros2/keystore` | Fleet CA + per-host enclaves under `enclaves/{drone_01,drone_02,gc}/`. See "SROS 2 enclave binding" below. |

### Unit convention (all services in gc/, leader/, follower/)

| Location | Annotation |
|----------|-----------|
| `Type=simple` | The node runs in the foreground and stays that way; no forking. |
| `User=root` | Matches the existing px4-agent.service convention on drone hardware. |
| `EnvironmentFile=/etc/default/drone-control` | Pulls in the shared fleet env. Unit-level `Environment=DRONE_ID=…` layers per-node values on top. |
| `ExecStart=/usr/local/bin/drone-control-run <entry_point>` | Every service delegates to the wrapper — no direct `ros2 run` invocations. |
| `Restart=always` + `RestartSec=5` | Matches px4-agent.service. Self-heals crashes without any cascade — each node restarts independently. |
| `PartOf=drone-control-<role>.target` | Stopping the target stops all peer services in one command; individual crashes still restart independently. |
| `After=<same-host prereq>.service` | Only same-host ordering (radio_health_reader → capability_assessor; px4-agent → position_tracker/mover). Cross-host coordination is left to DDS async discovery. |
| No `Requires=` anywhere | Prevents cascade kills — one failing service must not take down a whole target. |
| `Wants=network-online.target` | Delays start until network is up for services that talk to other hosts. |

### `deploy/systemd/gc/`

| Service | DRONE_ID env | Role |
|---------|---------|------|
| `drone-control-relay-decision-authority.service` | (n/a) | Fleet authority — one instance across whole deployment. `FOLLOWER_IDS=drone-02` env lists the drones it manages. |
| `drone-control-chain-assigner.service` | (n/a) | Converts `authorization` → `relay_assignment`. Depends on `relay-decision-authority.service` (same-host After=). |
| `drone-control-gc-radio-health-reader.service` | `gc` (+ `FOLLOWER_IDS=drone-02`) | One instance handles all followers via FOLLOWER_IDS list. |
| `drone-control-gc-link-observer.service` | (LEADER_ID=drone-01) | Observes leader-GC link SNR from `/gc/radio_health`. |
| `drone-control-state-bridge-drone-01.service` | `drone-01` | MQTT→ROS bridge for drone-01's `drone_state`. Real: After=mosquitto.service. SITL drop-in: mosquitto-drone01.service. |
| `drone-control-state-bridge-drone-02.service` | `drone-02` | Same, drone-02. SITL drop-in: mosquitto-drone02.service. |

### `deploy/systemd/leader/` (deploys to drone-01 host)

| Service | Notes |
|---------|-------|
| `drone-control-leader-radio-health-reader.service` | DRONE_ID=drone-01. Reads `/signal/gc_to_leader`. |
| `drone-control-leader-link-detector.service` | DRONE_ID=drone-01. After=leader-radio-health-reader. Publishes `/drone_01/relay_request` when SNR < LINK_MARGINAL_QUALITY. |

### `deploy/systemd/follower/` (deploys to drone-02 host)

| Service | Notes |
|---------|-------|
| `drone-control-follower-radio-health-reader.service` | DRONE_ID=drone-02. |
| `drone-control-capability-assessor.service` | DRONE_ID=drone-02, LEADER_ID=drone-01. After=follower-radio-health-reader. Hosts the BT at 0.5 Hz. |
| `drone-control-relay-strategy-evaluator.service` | After=capability-assessor. |
| `drone-control-strategy-executor.service` | After=capability-assessor. |
| `drone-control-relay-position-tracker.service` | After=capability-assessor AND px4-agent.service (real device) — needs `drone_state` from px4-agent. SITL drop-in: px4-agent-drone-02.service. |
| `drone-control-relay-mover.service` | After=relay-position-tracker AND px4-agent.service. `PX4_RX_URL=udpin:0.0.0.0:14558` — px4_agent library's read path listens on the SITL secondary MAVLink output; on real device this points at a spare MAVLink UDP endpoint from PX4 params. |
| `drone-control-continuous-monitor.service` | After=capability-assessor. Publishes SNR fast-path `reeval_trigger`. |

### `deploy/systemd/sitl/drone-control-signal-faker.service`

| Location | Annotation |
|----------|-----------|
| `Environment=DRONE_ID=gc` | Signal_faker's `main()` treats `DRONE_ID=gc` specially: publishes for the full follower set `["drone-01", "drone-02"]` instead of a single follower. |
| **No `[Install]` section** | Deliberately unlinked from any target — `systemctl enable drone-control-gc.target` cannot accidentally start the simulator on a real fleet. SITL rigs must enable manually: `systemctl enable --now drone-control-signal-faker`. |
| MAVLink 15s startup delay | `_make_mavlink_position_provider` calls `wait_heartbeat(timeout=5)` per drone. With 3 drones (leader + drone-01 + drone-02) and no PX4 SITL running, that's 15s of blocking before publishers register. Not an error — proceeds with startup-default positions. |

### SITL drop-in override pattern

Real-device unit files reference `mosquitto.service` and `px4-agent.service` (canonical
names on drone hardware). On a co-hosted SITL rig with per-drone-suffixed instances
(`mosquitto-drone01.service`, `px4-agent-drone-02.service`), drop-in overrides at
`/etc/systemd/system/<unit>.service.d/sitl.conf` redirect the `After=` line without touching
the shipped unit files:

```ini
[Unit]
After=          # empty line clears the inherited After= list
Wants=          # ditto for Wants=
After=network-online.target px4-agent-drone-02.service drone-control-capability-assessor.service
Wants=network-online.target px4-agent-drone-02.service
```

Applied to: state-bridge (×2), relay-position-tracker, relay-mover.

### px4_agent split — client library vs. daemon

| Component | Purpose | Under systemd? |
|-----------|---------|---------------|
| `/opt/drone-command/px4_agent.py` | Production DAEMON. Owns MAVLink UDP to PX4, hosts OFFBOARD setpoint sender, SQLite command persistence, MQTT command handler. | Yes — `px4-agent.service` on real device / `px4-agent.service` + `px4-agent-drone-02.service` on SITL. Pre-existing, unchanged by this deployment work. |
| `drone_control/px4_agent.py` | Client LIBRARY imported by relay_mover. Publishes setpoints via MQTT to the production daemon; reads own state from `drone/{DRONE_ID}/state`. | No — imported in-process by relay_mover, not standalone. |

`/opt/drone-command/px4_agent.py` was patched in this deployment cycle to add
`_accept_home_raw()` — rejects HOME_POSITION messages with `lat=0, lon=0` placeholders
(pre-GPS-fix) that would otherwise corrupt every downstream NED conversion. Fix locations:
MAVLink receiver + drone_state publisher fallback removed. See SESSION_LOG `[2026-08-18]`.

---

## Post-deployment fixes

### DRONE_ID hyphen sanitization (5 files)

ROS 2 topic names disallow hyphens. `DRONE_ID="drone-01"` embedded raw in a topic path
produces `/signal/gc_to_follower/drone-01` → `InvalidTopicNameException`. Convention already
followed by `state_bridge.py` and `chain_assigner.py`; propagated to the other 5 files:

| File | Fix site | Sanitization |
|------|---------|-------------|
| `signal_faker.py` | `_hop_topic()` and `_make_ros2_publisher()` loop | `tid = drone_id.replace('-', '_')` before topic interpolation |
| `follower_radio_health_reader.py` | `make_follower_reader()` top | Same pattern; `tid` used for both subscribe topics and publish topic |
| `gc_radio_health_reader.py` | Per-follower loop in `make_gc_reader()` | `tid` sanitized per iteration; also used in `hop_name` key |
| `leader_radio_health_reader.py` | `publish_topic` line in `make_leader_reader()` | Inline `.replace('-', '_')` |
| `leader_link_detector.py` | `LeaderLinkDetector.__init__` | Adds `self._topic_id = drone_id.replace('-', '_')` alongside `self._drone_id`; the hyphenated form is preserved for the "drone_id" payload field |

**Convention going forward:** any file building a ROS 2 topic path from a hyphenated
identifier must sanitize with `.replace('-', '_')` at the topic-string construction site.
Payload/JSON/MQTT paths keep the original hyphenated form.

### signal_faker.py logger call

Changed `self.get_logger().info("...%s...", arg)` (printf-style) to f-string form. rclpy's
`RcutilsLogger.info()` takes only a formatted string, unlike standard Python `logging`.

---

*Deployment tree: `/root/ros2_ws/src/drone_control/deploy/systemd/`. Install per-host:
copy `common/` + one role directory to `/etc/systemd/system/`, then `systemctl enable
--now drone-control-<role>.target`. Full recipe in `deploy/systemd/README.md`.*

---

## SROS 2 enclave binding (Enforce mode)

Every relay-BT service runs under DDS-Security Enforce mode. Each service loads an
enclave via a drop-in override at `/etc/systemd/system/<service>.d/enclave.conf` that
appends `--ros-args --enclave /<host>/<node>` to the wrapper's ExecStart. The enclave
provides the identity cert + signed permissions doc that authenticate the participant on
DDS domain 42.

### Keystore layout

Under `/root/sros2/keystore/enclaves/`:

- `drone_01/` — enclaves for services deployed to the leader drone (drone-01 host)
- `drone_02/` — enclaves for services deployed to the follower drone (drone-02 host)
- `gc/` — enclaves for services deployed to the ground-control host

Each host directory contains 20 enclave subdirs (5 legacy pre-relay-BT roles + 15
relay-BT roles). Any host generates the full set; only the relevant ones are bound
via drop-in.

### Regeneration workflow

Tooling lives at `/root/sros2/`:

- `generate_keystore.sh <host_id>` — creates fleet CA (once) + per-host enclaves. Signs
  governance and permissions. Accepts any lowercase-alphanumeric-hyphen id, e.g.
  `drone-01`, `drone-02`, `gc`.
- `templates/permissions.xml.j2` — Jinja2 template. Rewritten for relay-BT: allow rules
  cover `rt/signal/*`, `rt/gc/*`, `rt/drone_*/*`, `rt/relay_tasking`, `rt/relay_suppression`,
  `rt/rosout`, `rt/parameter_events` for both pub and sub. Trailing default-deny catches
  everything else.
- `render_permissions.py` — renders template → signs → distributes `permissions.p7s` to
  every enclave under the host.
- `verify_sros2.sh <host_id>` — validates keystore structure (52 checks per host).

To add a new node type, extend `DRONE_NODES=(…)` in `generate_keystore.sh` and rerun
`./generate_keystore.sh <host>` for each host. Existing enclaves are re-signed; the fleet
CA is reused (delete `keystore/public/ca.cert.pem` to force a CA reset — invalidates every
existing cert).

### Service → enclave binding table

| Service | Enclave |
|---------|---------|
| drone-control-leader-radio-health-reader | `/drone_01/leader_radio_health_reader` |
| drone-control-leader-link-detector | `/drone_01/leader_link_detector` |
| drone-control-follower-radio-health-reader | `/drone_02/follower_radio_health_reader` |
| drone-control-capability-assessor | `/drone_02/capability_assessor` |
| drone-control-relay-strategy-evaluator | `/drone_02/relay_strategy_evaluator` |
| drone-control-strategy-executor | `/drone_02/strategy_executor` |
| drone-control-relay-position-tracker | `/drone_02/relay_position_tracker` |
| drone-control-relay-mover | `/drone_02/relay_mover` |
| drone-control-continuous-monitor | `/drone_02/continuous_monitor` |
| drone-control-state-bridge-drone-01 | `/gc/state_bridge` (shared) |
| drone-control-state-bridge-drone-02 | `/gc/state_bridge` (shared) |
| drone-control-gc-radio-health-reader | `/gc/gc_radio_health_reader` |
| drone-control-gc-link-observer | `/gc/gc_link_observer` |
| drone-control-relay-decision-authority | `/gc/relay_decision_authority` |
| drone-control-chain-assigner | `/gc/chain_assigner` |
| drone-control-signal-faker | `/gc/signal_faker` |

**Shared enclave note:** the two state-bridge services (drone-01 and drone-02) share
`/gc/state_bridge` because both run on the same GC host with identical permissions.
SROS 2 authenticates per certificate, not per node instance — two participants presenting
the same cert are two authenticated members of that identity.

### Drop-in override format

```ini
# /etc/systemd/system/drone-control-capability-assessor.service.d/enclave.conf
[Service]
ExecStart=
ExecStart=/usr/local/bin/drone-control-run capability_assessor --ros-args --enclave /drone_02/capability_assessor
```

The empty first `ExecStart=` line clears the unit's inherited ExecStart (systemd
requires this before appending a new one). All 16 drop-ins follow this shape.

**Why drop-ins and not shipped units:** the enclave path is a host-installation choice,
not a package attribute. Shipping `--enclave` in the base unit would hard-code the
host-role assumption. Drop-ins keep the base units host-agnostic.

### CLI trade-off

Under Enforce, `ros2 topic list` (default) from a fresh shell can't discover any topics —
the CLI participant has no enclave. Two workarounds:

1. **Query with a service enclave:** copy an existing enclave to a CLI-scoped path,
   then launch a spy node with `--enclave /gc/gc_link_observer` and read `ros2 topic list`
   output via that node's context.
2. **Temporary Permissive shell:** `env ROS_SECURITY_STRATEGY=Permissive ros2 topic list`
   for one-off diagnostics. Services stay Enforce; only the CLI participant is loose.

Neither affects the running fleet — services discover each other because they all present
valid signed enclaves.

---

## SITL smoke-test scaffolding

Files and patterns used to exercise the full pipeline on a co-hosted SITL rig that has
no PX4 SITL running (drone_state must be faked).

### `/root/fake_drone_state_broadcaster.py`

Publishes MQTT `drone/{drone_id}/state` messages at 1 Hz for both drone-01 and drone-02.
Fields chosen to satisfy every relay-BT data-freshness gate:

| Field | Value | Why |
|-------|-------|-----|
| `battery_pct` | 95 | passes G2 BatteryStillSufficientToRelay, BatteryAboveFloor |
| `gps_fix_type` | 3 | passes GPSFixAdequate (3D fix) |
| `flight_mode` | "OFFBOARD" | passes G3 OffboardModeHeld |
| `position` | Zurich reference + 500m south for drone-02 | inside geofence, non-zero |
| `home_pos` | Same as position | passes _accept_home() (nonzero lat/lon) |
| `avg_speed_ms`, `endurance_s` | legacy fields | published for backward compat |

Run as a systemd-run transient unit so it survives shell exit:

```bash
systemd-run --unit=fake-drone-state-broadcaster \
  /usr/bin/python3 /root/fake_drone_state_broadcaster.py
```

**WARNING — stop it before any real PX4 test:**
The transient unit persists across shell sessions and survives reboots until explicitly
stopped. If it is still running when px4_agent connects to a real PX4 SITL, both the
fake broadcaster and the real daemon publish to the same `drone/{id}/state` topic on the
same broker. The BT reads whichever message arrives last; with fake publishing `flight_mode=OFFBOARD`
at 1 Hz and the real daemon publishing `flight_mode=HOLD`, the relay pipeline sees an
alternating signal and may behave as if the drone is airborne and in OFFBOARD when it is
not. All "success" results from prior SITL runs are suspect if this unit was running.

```bash
# Check if running:
systemctl is-active fake-drone-state-broadcaster

# Stop it before any real PX4 test:
systemctl stop fake-drone-state-broadcaster
```

Not shipped in `deploy/systemd/` — this is dev-only. On a real fleet the production
`px4_agent.service` publishes the same MQTT topic from live MAVLink telemetry.

### Signal degradation via `hop_severities`

To trigger a relay round in SITL, edit `config/demo_config.py`:

```python
"hop_severities": {
    "gc_to_leader":  0.95,   # SITL trigger — force SNR < LINK_MARGINAL_QUALITY
    "leader_to_gc":  0.95,   # symmetric
    ...
}
```

Then `colcon build --packages-select drone_control && systemctl restart
drone-control-signal-faker.service`. Within ~5s, relay_decision_authority logs
`"GC link degraded quality=0.000 — starting broadcast round"` and publishes
`/relay_tasking`.

### Diagnostic: reading topics under Enforce

`ros2 topic list` and `ros2 topic echo` need to present a signed enclave under Enforce.
The pattern:

```bash
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
       CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml \
       ROS_SECURITY_ENABLE=true ROS_SECURITY_STRATEGY=Enforce \
       ROS_SECURITY_KEYSTORE=/root/sros2/keystore \
       ROS_SECURITY_ENCLAVE_OVERRIDE=/gc/gc_link_observer
ros2 topic echo --once --full-length /drone_02/capability_report
```

Any provisioned enclave will work; `/gc/gc_link_observer` is convenient because it's
a subscriber-role enclave with broad read permissions per our template. Use
`--full-length` to avoid ros2's default JSON truncation with `...'`.

To pretty-print the `checks:` dict from a `capability_report` (the actual diagnostic
surface — `reason:` field alone is misleading under IDLE, see Wave 6 known-quirk below):

```bash
python3 -c "
import re, json
raw = open('/tmp/report.txt').read()
m = re.search(r\"^data:\s*'(.+)'\s*\$\", raw, re.MULTILINE|re.DOTALL)
d = json.loads(m.group(1))
for name, info in d['checks'].items():
    mark = '✓' if info['pass'] else '✗'
    print(f'  {mark} {name:32s} {info[\"detail\"]}')
"
```

---

## Wave 6 known quirks (surfaced during SITL smoke test)

### `capability_report.reason` is misleading under `current_role=IDLE`

`_build_report()` sets `reason` to the feedback_message of the FIRST failing condition
node encountered in tree walk order. Under IDLE, the first failing node is always
`IsAlreadyRelaying` (which SHOULD fail for IDLE — Root Selector then tries IDLE_BRANCH).
Reason reports `"current_role=IDLE"` regardless of what's actually blocking downstream.

For real diagnostic, always read the full `checks:` dict, not `reason:` alone. Only
`DataFreshness` failure takes precedence in reason (special-cased in `_build_report`).

### `TaskingIsValid` and `leader_id` — design decision made silently, multi-leader gap open

The setup: `condition_nodes.TaskingIsValid` calls `payload.get("leader_id")`.
BUILDSPEC §2.4 relay_tasking schema is `{round_id, timestamp, trigger}` — no
`leader_id`. `relay_decision_authority._start_round()` correctly follows the spec
and publishes just those three fields. So `payload.get("leader_id")` returns None.

Three options were logged in SESSION_LOG `[2026-08-20]`:
  (a) add `leader_id` to §2.4 + RDA payload — spec change
  (b) drop the leader_id check from TaskingIsValid — code change, loses the
      "am I mis-tasked as relay for myself?" guard
  (c) source `leader_id` from `LEADER_ID` env inside capability_assessor's config
      and use it as fallback — deployment-time workaround

**What was actually done: option (c), silently.** The follow-on "Wave 10 fix log"
entry in this file said "Fixed: env fallback via `config['leader_id']`" without
flagging that a design decision had been made. That's why this section previously
also said "BLOCKER" — the two paragraphs contradicted each other. Reconciled here:
the code works, but the design question was punted, not answered.

**Multi-leader failure mode** (open — worth logging before this ships beyond
single-leader SITL):

Under option (c), every follower validates every tasking against `LEADER_ID` from
its **own env**, not against the leader the GC actually named. With one leader
in the fleet, that's identical to what the GC intended. With more than one leader
it silently diverges — the follower has no way to know which leader a given round
is for, and its answer depends entirely on which leader its env happens to
name.

Concretely:
  - If two leaders each drive their own rounds concurrently, each follower will
    treat both rounds as being for its own configured leader.
  - `TaskingIsValid` will pass for the round even if the GC meant it for the
    other leader. The "not me as leader" check still works (a follower is
    never its own leader), but the "for the right leader" property is gone.
  - Chain still fires; wrong follower may respond to a round the other leader's
    followers should have handled.

**When option (a) becomes required:** any deployment with >1 leader. At that
point add `leader_id` to §2.4, make RDA populate it from its own `LEADER_ID`
env at round-start time, and have TaskingIsValid require the payload field
(env fallback becomes the second-source).

Current status: option (c) is live, single-leader-safe. Multi-leader gap
documented here — not a runtime bug for the current SITL topology
(drone-01 leader, drone-02 follower), but a hard block if the fleet grows.

### `relay_position_tracker` doesn't publish `movement_status` until first assignment

`tick()` returns early when `_target is None`. Before any `relay_assignment` arrives,
no `movement_status` is emitted, so capability_assessor's BB never receives
`offboard_mode_held`, `distance_to_target_decreasing`, `within_acceptance_radius`,
or `last_command_ack` from that source. Not a bug — matches design intent for
movement-phase-only signals. `OffboardModeHeld` reads `drone_state.flight_mode`
directly and is unaffected.

---

## Wave 10 fix log — subscription topic mismatches

Surfaced during Enforce-mode deployment testing.

### `capability_assessor.py` — `/relay_tasking` shared topic

**Before:** subscribed to `{prefix}/relay_tasking` inside a per-drone loop, resolving
to `/drone_02/relay_tasking` for the follower.

**Why wrong:** BUILDSPEC §2.4 explicitly designates `/relay_tasking` as a single shared
topic. Every follower subscribes to it; GC broadcasts once. RDA correctly publishes to
`/relay_tasking`; the follower's per-drone subscription never matched.

**Fix:** removed `relay_tasking` from the per-drone loop; added a dedicated
`create_subscription("/relay_tasking", ...)` outside the loop. Same `_on_msg` dispatcher
writes to `bb["relay_tasking_received"]`.

**Verified:** `ros2 topic info -v /relay_tasking` reports Publisher count=1, Subscription
count=2 (capability_assessor + relay_strategy_evaluator).

**Recommendation:** run a CHECK-1-style topic-wiring sweep against live Enforce topics —
this session found one such mismatch; more likely exist and are silently masked by DDS
not warning about "no publisher" or "no subscriber" cases.

---

## Wave 10 fix log — end-to-end integration bugs

Surfaced while chasing the SITL "signal → drone repositions" flow. Each was silently
masking the next; all 5 fixed together bring the pipeline live end-to-end.

### 1. `DataFreshness` gate dead-referenced deleted `signal_report`

Verified against BUILDSPEC §8 checklist: "No file publishes signal_report — the topic
does not exist." Yet `condition_nodes.DataFreshness` iterated `("signal_report",
"drone_state")` and ALWAYS failed. Under the sequence semantics that short-circuits the
full F_cap chain, this meant every proposal became a `LET_LEADER_ISOLATE` decline —
regardless of actual capability.

Fixed: iterate `("drone_state",)` only. Per-hop freshness is enforced by BandSensorNode
via `radio_health` payloads. Removed matching references in `capability_assessor._on_msg`
per-drone loop, `_build_report` freshness dict, `_collect_inputs` SNR extraction.

### 2. `TaskingIsValid` `leader_id` field not in §2.4

`condition_nodes.TaskingIsValid` required `payload["leader_id"]`; BUILDSPEC §2.4 defines
only `{round_id, timestamp, trigger}`. RDA correctly follows the spec — every authentic
broadcast failed the check.

Fixed: env fallback via `config["leader_id"]` (capability_assessor now injects both
`drone_id` and `leader_id` into config at `__init__` from `DRONE_ID` and `LEADER_ID`
env vars).

### 3. `ProposeContinuousRelay` proposal missing §2.5 fields

Payload was `{proposal_id, timestamp, strategy, relay_position, cost{...}}`. RDA reads
`payload["round_id"]` for the collection-window filter and `payload["drone_id"]` for
routing; both absent → every proposal discarded as `round_id=None`.

Fixed: proposal now includes `drone_id` (from drone_state or config), `round_id`
(echoed from `bb["relay_tasking_received"]`), `r_target` (renamed from `relay_position`,
with `alt_m` conversion — see #4), `eta_s` top-level, `capability_snapshot{battery_pct,
gps_fix_type, t_lo, t_hi, cap_gc_m, cap_leader_m, cap_follower_m, band_feasible,
geofence_ok, return_margin_ok}`, `trigger_context{gate_fired, reason, source}`.
`cost{}` retained for backward-compat.

### 4. `r_target.alt` vs `alt_m` field name

`geometry.compute_relay_position` and `bucket_position` return `{lat, lon, alt}` (internal
convention). §2.7 `r_target` wire format uses `alt_m`. `relay_mover.tick()` reads
`self._target["alt_m"]` → `KeyError` and process crash.

Fixed: translate at the proposal boundary (in `ProposeContinuousRelay`), not at every
geometry call. `r_target = {lat, lon, alt_m: R_target.get("alt_m", R_target.get("alt", 0.0))}`.

### 5. `DRONE_MODEL_ASSIGNMENT` missing from `DEMO_CONFIG` dict

Module-level `DRONE_MODEL_ASSIGNMENT = {"drone-01": ..., "drone-02": ...}` existed but
was NOT listed inside `DEMO_CONFIG = {...}`. `_load_config` returns `DEMO_CONFIG`, so
`cfg.get("DRONE_MODEL_ASSIGNMENT")` returned `None`. RDA's `drone_ids = list(cfg.get(...).keys())`
yielded `[]` → no per-drone strategy_proposal subscriptions were created → every proposal
silently unheard. Startup log revealed it: `"relay_decision_authority started: drones=[]"`.

Fixed: added `"DRONE_MODEL_ASSIGNMENT": DRONE_MODEL_ASSIGNMENT` to DEMO_CONFIG dict.

### 6. `chain_assigner` needs per-follower `DRONE_ID`

`chain_assigner` subscribes `{prefix}/authorization` where prefix comes from `DRONE_ID`
env. On real fleet each drone runs its own instance with its own env. On the SITL box,
the base unit had no `DRONE_ID`, defaulting to `drone-01` — so drone-02 authorizations
were silently unheard.

Fixed: SITL drop-in
`/etc/systemd/system/drone-control-chain-assigner.service.d/drone-id.conf`:
```ini
[Service]
Environment=DRONE_ID=drone-02
```

---

## End-to-end SITL verification

With all six fixes above applied plus the SITL scaffolding (fake_drone_state_broadcaster
+ hop_severities.gc_to_leader=0.95), the full pipeline runs live under SROS 2 Enforce:

```
severity change
  → signal_faker → /signal/gc_to_leader (high noise)
  → gc_radio_health_reader → /gc/radio_health (low snr)
  → gc_link_observer → /gc/gc_link_quality (0.0)
  → RDA: "starting broadcast round" → /relay_tasking
  → capability_assessor: BT ticks CAPABLE, ProposeContinuousRelay → /drone_02/strategy_proposal
  → RDA: collects, closes window, picks winner → /drone_02/authorization
  → strategy_executor: current_role → MOVING_TO_RELAY
  → chain_assigner: /drone_02/relay_assignment with r_target
  → relay_position_tracker: watches arrival
  → relay_mover: START_LEAD → setpoints on MQTT drone/drone-02/setpoint at 3 Hz
  → production px4_agent: SET_POSITION_TARGET_LOCAL_NED to PX4
```

Round-trip: degradation event → first setpoint on the wire ≈ 55s (dominated by the 45s
proposal collection window; the rest is DDS discovery + OFFBOARD pre-stream).

Observed setpoint payload:
```json
{"x": 55.60, "y": -301.10, "z": -50.0, "yaw": 0.0}
```
NED coords → 305 m south-southeast of home, 50 m altitude — matches the proposal's
`repositioning_m=306.2` and `alt_m=50.0`.

---

## Topology smoke test (deploy/smoke/)

Purpose: catch topology bugs (multiple publishers on an exclusive topic, wrong
per-drone vs. shared subscription, missing publisher entirely) that unit tests
structurally cannot see. Three real bugs of this shape have already shipped:
CHECK 1's producer/consumer sweep, the capability_assessor per-drone
`/relay_tasking` mismatch, and the §4.9 `/strategy_proposal` double-publish.
This tool closes the class.

Files:

| Path | Purpose |
|------|---------|
| `deploy/smoke/topology_smoke_test.py` | 17 SPEC-derived rules (§2.x, §4.x). Each asserts publisher count, publisher-node substring, and min subscriber count for one topic. Uses `ros2 topic info -v` under an existing enclave. Exit 0 iff every rule passes. |
| `deploy/smoke/run_topology_smoke.sh` | Wrapper: sources ROS overlay + `/etc/default/drone-control` + `ROS_SECURITY_ENCLAVE_OVERRIDE=/gc/gc_link_observer` (any signed enclave with broad read perms works). |

Usage:
```bash
./deploy/smoke/run_topology_smoke.sh              # full sweep, all 17 rules
./deploy/smoke/run_topology_smoke.sh --topic /drone_02/strategy_proposal
./deploy/smoke/run_topology_smoke.sh --list-spec  # print rules without running
```

Rules encoded (see topology_smoke_test.py `RULES` list for exact syntax):

| Topic | Sole publisher | Min subs | Spec |
|-------|----------------|---------|------|
| `/relay_tasking` | relay_decision_authority | 1 | §2.4 |
| `/{drone}/strategy_proposal` | relay_strategy_evaluator | 0 | §4.9 |
| `/{drone}/authorization` | relay_decision_authority | 0 | §2.6 |
| `/{drone}/relay_assignment` | chain_assigner_{drone} | 0 | §2.7 |
| `/{drone}/relay_confirmed` | relay_position_tracker_{drone} | 0 | §2.10 |
| `/{drone}/reauth_request` | capability_assessor_{drone} | 0 | §2.11 |
| `/{drone}/reeval_trigger` | continuous_monitor_{drone} | 0 | §2.12 |
| `/gc/radio_health` | gc_radio_health_reader | 0 | §4.2 |
| `/{leader}/radio_health` | leader_radio_health_reader_{leader} | 0 | §4.2 |
| `/{drone}/radio_health` | follower_radio_health_reader_{drone} | 0 | §4.2 |
| `/gc/gc_link_quality` | gc_link_observer | 1 | §4.3 |
| `/{drone}/current_role` | strategy_executor + relay_position_tracker (DUAL, pub_count=2) | 0 | §4.8 |
| `/{drone}/capability_report` | capability_assessor_{drone} | 1 | §4.11 |
| `/{leader}/relay_request` | leader_link_detector_{leader} | 0 | §2.3 |

Extending: add a new `TopoRule(topic=..., pub_count=..., pub_matches=...,
min_subs=..., spec_ref="§x.y")` entry in `RULES`. Substring match on the
publisher node name catches the specific §4.9 violation shape (extra publisher
whose name doesn't match the exclusive owner).

**Mandatory at every file gate:** run `./run_topology_smoke.sh`, all 17 rules
must PASS. Any violation is a §-referenced hard block. Add to CI once a
green baseline is established.
