#!/usr/bin/env python3
"""
integration_waves0_to_5.py — end-to-end pipeline, Waves 0–5.

Each wave's output feeds the next wave's input.  Five scenarios show the
complete BT decision path from raw config through to a blackboard proposal.

Pipeline:
  [Wave 0]  DEMO_CONFIG                   config surface (§3)
  [Wave 1]  SignalFaker.tick()            /signal/* (5 hops, §2.1)
  [Wave 2]  *_radio_health_reader         /*/radio_health per role
  [Wave 3]  gc_link_observer              /gc/gc_link_quality
            leader_link_detector          relay_request | silent
  [Wave 4]  TimestampedBlackboard         write from pipeline, age keys
  [Wave 5]  BT tree tick                  condition + action nodes → proposal

Scenarios:
  A — IDLE, clean signals   → FULL_ENTRY → ProposeContinuousRelay
  B — RELAYING, all pass    → ARBITER_SCAN → CONTINUE
  C — RELAYING, 3× bad SNR  → G7 debounce fires → ProposeExitRelay
  D — RELAYING, FCU stale   → G1 fires → FollowerSafetyExit + alert_intent
  E — RELAYING, auth expired → G8 writes reauth → 130s later → ProposeExitRelay

Run:
  python3 tests/integration_waves0_to_5.py
"""

import sys
import os
import math

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
_DC = os.path.join(_PKG_ROOT, "drone_control")
_RBT = os.path.join(_DC, "relay_bt")
for p in (_PKG_ROOT, _DC, _RBT):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.config.demo_config import DEMO_CONFIG
from drone_control.signal_faker import SignalFaker, haversine
from drone_control.leader_radio_health_reader import make_leader_reader
from drone_control.follower_radio_health_reader import make_follower_reader
from drone_control.gc_radio_health_reader import make_gc_reader
from drone_control.gc_link_observer import make_gc_link_observer
from drone_control.leader_link_detector import make_leader_link_detector
from drone_control.relay_bt.blackboard import TimestampedBlackboard
from drone_control.relay_bt.tree_builder import build_relay_decision_tree, dump_state
import py_trees


# ── Topology constants ────────────────────────────────────────────────────────

LEADER_ID = "drone-01"
FOLLOWER_ID = "drone-02"
GC_POS = {"lat": 47.3900, "lon": 8.5400, "alt": 0.0}
LEADER_POS = {"lat": 47.3980, "lon": 8.5480, "alt": 50.0}
RELAY_POS = {"lat": 47.3940, "lon": 8.5440, "alt": 40.0}


# ── Merged config (DEMO_CONFIG + integration overrides) ───────────────────────

CFG = {
    **DEMO_CONFIG,
    "min_gps_fix_type": 0,
    "recover_offboard_max_attempts": 5,
    "debounce_n": 3,
}

MARGINAL = CFG["LINK_MARGINAL_QUALITY"]


# ── FakeClock ─────────────────────────────────────────────────────────────────

class FakeClock:
    def __init__(self, t=1_000.0):
        self._t = t

    def now(self):
        return self._t

    def advance(self, s):
        self._t += s


# ── Display helpers ───────────────────────────────────────────────────────────

W = 76


def _bar(v, lo, hi, w=24, threshold=None):
    span = max(hi - lo, 1e-9)
    filled = int(round(max(0, min(w, (v - lo) / span * w))))
    bar = list("█" * filled + "·" * (w - filled))
    if threshold is not None:
        tp = int(round(max(0, min(w - 1, (threshold - lo) / span * w))))
        bar[tp] = "▼"
    return "[" + "".join(bar) + "]"


def _row(label, value, indent="  │  "):
    print(f"{indent}{label:<30} {value}")


def _hdr(text):
    print(f"\n  ╔{'═' * (W - 4)}╗")
    print(f"  ║  {text:<{W - 6}}║")
    print(f"  ╚{'═' * (W - 4)}╝")


def _wave(num, text):
    print(f"\n  ┌─ WAVE {num}: {text}")


def _sep():
    print("  " + "─" * (W - 2))


def _snr_row(snr_db, label="snr_db", indent="  │  "):
    bar = _bar(snr_db, -20, 60, threshold=MARGINAL)
    cmp = "< BELOW" if snr_db < MARGINAL else "≥ ABOVE"
    print(f"{indent}{label:<30} {bar} {snr_db:+7.2f} dB  {cmp} {MARGINAL}")


def _status_icon(status_name):
    return {"SUCCESS": "✓", "FAILURE": "✗", "RUNNING": "~", "INVALID": "·"}.get(status_name, "?")


def _chk(results, label, cond, detail=""):
    results.append((label, bool(cond), detail))


# ── Wave 1-3 pipeline helper ──────────────────────────────────────────────────

def run_pipeline(severities: dict, clock_t: float) -> dict:
    """
    Runs Waves 1–3 in-process and returns a dict with all pipeline outputs.
    All signal timestamps use clock_t as the fake epoch.
    """

    signal_bus: dict = {}
    faker = SignalFaker(
        cfg=CFG,
        drone_ids=[FOLLOWER_ID],
        get_positions=lambda: {"gc": GC_POS, "leader": LEADER_POS, FOLLOWER_ID: RELAY_POS},
        publish=lambda t, p: signal_bus.update({t: p}),
        get_severity=lambda hop: severities.get(hop, 0.0),
        clock=lambda: clock_t,
    )
    faker.tick()

    ldr_health_bus: dict = {}
    ldr_reader = make_leader_reader(
        LEADER_ID, CFG,
        publish=lambda t, p: ldr_health_bus.update({p["hop"]: p}),
    )
    ldr_reader.on_signal("/signal/gc_to_leader", signal_bus.get("/signal/gc_to_leader", {}))
    ldr_reader.publish_tick()

    flw_health_bus: dict = {}
    flw_reader = make_follower_reader(
        FOLLOWER_ID, CFG,
        publish=lambda t, p: flw_health_bus.update({p["hop"]: p}),
    )
    for topic, hop in [
        (f"/signal/leader_to_follower/{FOLLOWER_ID}", "leader_to_follower"),
        (f"/signal/gc_to_follower/{FOLLOWER_ID}", "gc_to_follower"),
    ]:
        if topic in signal_bus:
            flw_reader.on_signal(topic, signal_bus[topic])
    flw_reader.publish_tick()

    gc_health_bus: dict = {}
    gc_reader = make_gc_reader(
        [FOLLOWER_ID], CFG,
        publish=lambda t, p: gc_health_bus.update({p["hop"]: p}),
    )
    for topic in ["/signal/gc_to_leader", f"/signal/gc_to_follower/{FOLLOWER_ID}"]:
        if topic in signal_bus:
            gc_reader.on_signal(topic, signal_bus[topic])
    gc_reader.publish_tick()

    gc_quality_bus: dict = {}
    gc_obs = make_gc_link_observer(
        CFG, publish=lambda t, p: gc_quality_bus.update({t: p}))
    if "gc_to_leader" in ldr_health_bus:
        gc_obs.on_radio_health(f"/{LEADER_ID}/radio_health", ldr_health_bus["gc_to_leader"])
    gc_obs.publish_tick()

    relay_requests: list = []
    detector = make_leader_link_detector(
        LEADER_ID, CFG,
        publish=lambda t, p: relay_requests.append((t, p)),
        suppression_topic="/relay_suppression",
    )
    if "gc_to_leader" in ldr_health_bus:
        detector.on_radio_health(f"/{LEADER_ID}/radio_health", ldr_health_bus["gc_to_leader"])
    detector.publish_tick()

    return {
        "signal_bus":      signal_bus,
        "ldr_health":      ldr_health_bus,
        "flw_health":      flw_health_bus,
        "gc_health":       gc_health_bus,
        "gc_quality":      gc_quality_bus,
        "relay_requests":  relay_requests,
    }


# ── Print pipeline summary ─────────────────────────────────────────────────────

def _print_pipeline(pipe: dict, severities: dict):
    _wave(1, "SignalFaker.tick() → /signal/* (5 hops, §2.1)")
    hops = [
        ("gc_to_leader",       "/signal/gc_to_leader"),
        ("leader_to_gc",       "/signal/leader_to_gc"),
        (f"gc→flw",            f"/signal/gc_to_follower/{FOLLOWER_ID}"),
        (f"flw→gc",            f"/signal/follower_to_gc/{FOLLOWER_ID}"),
        (f"ldr→flw",           f"/signal/leader_to_follower/{FOLLOWER_ID}"),
    ]
    hop_names = [
        "gc_to_leader", "leader_to_gc",
        "gc_to_follower", "follower_to_gc", "leader_to_follower"
    ]
    print(f"  │  {'hop':<20} {'sev':>4}  {'rssi':>8}  snr")
    print(f"  │  {'─'*60}")
    for (label, topic), name in zip(hops, hop_names):
        sig = pipe["signal_bus"].get(topic)
        sev = severities.get(name, 0.0)
        if sig:
            bar = _bar(sig["snr_db"], -20, 60, w=16, threshold=MARGINAL)
            cmp = "↓BELOW" if sig["snr_db"] < MARGINAL else "ABOVE"
            print(f"  │  {label:<20} {sev:>4.2f}  {sig['rssi_dbm']:>+8.2f}  "
                  f"{bar} {sig['snr_db']:+.1f}dB {cmp}")
        else:
            print(f"  │  {label:<20}  MISSING")

    _wave("2a", f"leader_radio_health_reader → gc_to_leader health")
    lh = pipe["ldr_health"].get("gc_to_leader")
    if lh:
        _snr_row(lh["snr_db"], "  gc_to_leader snr_db")
        _row("  range_m", f"{lh['range_m']:.0f} m  (nominal {CFG['leader_radio_range_m']}m × (1 − sev×factor))")

    _wave("2b", f"follower_radio_health_reader → leader_to_follower, gc_to_follower")
    for hop in ["leader_to_follower", "gc_to_follower"]:
        fh = pipe["flw_health"].get(hop)
        if fh:
            _snr_row(fh["snr_db"], f"  {hop} snr_db")

    _wave("3a", "gc_link_observer → gc_link_quality")
    gq = pipe["gc_quality"].get("/gc/gc_link_quality")
    if gq:
        q_bar = _bar(gq["quality"], 0, 1, threshold=0.5)
        _row("  quality", f"{q_bar} {gq['quality']:.4f}  (threshold 0.5 ↔ snr={MARGINAL}dB)")
    else:
        _row("  quality", "NO OUTPUT (gc_to_leader health missing)")

    _wave("3b", f"leader_link_detector → relay_request | silent")
    lh = pipe["ldr_health"].get("gc_to_leader")
    if pipe["relay_requests"]:
        _, rr = pipe["relay_requests"][0]
        _row("  DECISION", f"FIRE  (snr {lh['snr_db']:+.1f}dB < {MARGINAL}dB threshold)")
    else:
        snr_str = f"{lh['snr_db']:+.1f}" if lh else "N/A"
        _row("  DECISION", f"SILENT  (snr {snr_str}dB ≥ {MARGINAL}dB threshold)")


# ── BT wave 5: tick and print gate results ────────────────────────────────────

def _print_bt_tick(root, tick_num=1):
    state = dump_state(root)
    gated_nodes = {
        k: v for k, v in state.items()
        if v["status"] != "INVALID"
    }
    print(f"  │  [tick {tick_num}]  root → {root.status.name}")
    for name, info in gated_nodes.items():
        icon = _status_icon(info["status"])
        short_fb = info["feedback"][:55]
        print(f"  │    {icon} {name:<44} {info['status']:<9} {short_fb}")


# ═══════════════════════════════════════════════════════════════════════════════
# SCENARIO RUNNERS
# ═══════════════════════════════════════════════════════════════════════════════

_CLEAN_SEV = {k: 0.0 for k in
    ["gc_to_leader", "leader_to_gc", "gc_to_follower", "follower_to_gc", "leader_to_follower"]}

_BAD_SEV = {**_CLEAN_SEV, "gc_to_leader": 0.95, "leader_to_gc": 0.95}


def scenario_a(results):
    """IDLE, clean signals → FULL_ENTRY → ProposeContinuousRelay."""
    _hdr("SCENARIO A — IDLE → FULL_ENTRY → ProposeContinuousRelay")

    _wave(0, "Config (DEMO_CONFIG — relevant keys)")
    _row("gc_pos",              f"lat={GC_POS['lat']}  lon={GC_POS['lon']}")
    _row("leader_pos (fallback)", f"lat={LEADER_POS['lat']}  lon={LEADER_POS['lon']}  ({haversine(GC_POS, LEADER_POS):.0f}m from GC)")
    _row("radio_range_m (all)", f"{CFG['gc_radio_range_m']} m nominal")
    _row("cruise_speed_mps",    f"{CFG['DRONE_MODELS']['generic']['cruise_speed_mps']} m/s  (DRONE_MODELS['generic'])")
    _row("battery_reserve_pct", f"{CFG['battery_reserve_pct']} %")
    _row("boot_grace_window_s", f"{CFG['boot_grace_window_s']} s (LeaderReachabilityFresh)")

    clock = FakeClock()
    pipe = run_pipeline(_CLEAN_SEV, clock.now())
    _print_pipeline(pipe, _CLEAN_SEV)

    _wave(4, "Blackboard ← pipeline outputs + synthetic drone state")
    now = clock.now()
    bb = TimestampedBlackboard(clock=clock.now)
    bb.set("current_role", "IDLE")
    bb.set("relay_tasking_received", {
        "tasking_id": "t-A001", "leader_id": LEADER_ID, "leader_pos": LEADER_POS,
    })
    bb.set("leader_radio_health", {
        **pipe["ldr_health"].get("gc_to_leader", {"snr_db": 30.0, "severity": 0.0}),
        "timestamp": now,
    })
    bb.set("signal_report", {
        "follower_to_gc":      pipe["flw_health"].get("gc_to_follower", {}) or {"snr_db": 25.0},
        "leader_to_follower":  pipe["flw_health"].get("leader_to_follower", {}) or {"snr_db": 22.0},
        "timestamp":           now,
    })
    bb.set("drone_state", {
        "battery_pct": 80.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
        "position":    GC_POS,
        "home_pos":    GC_POS,
        "timestamp":   now,
    })
    lh = pipe["ldr_health"].get("gc_to_leader")
    _row("leader_radio_health.snr_db",    f"{lh['snr_db']:+.2f} dB  (from Wave 2a)" if lh else "N/A")
    _row("current_role",                  "IDLE  → IDLE_BRANCH fires")
    _row("relay_tasking_received",        f"leader_id={LEADER_ID}  leader_pos={LEADER_POS['lat']:.4f},{LEADER_POS['lon']:.4f}")
    _row("drone_state.battery_pct",       "80 %  (entry gate: BatteryAboveFloor > 30 %)")
    _row("drone_state.flight_mode",       "OFFBOARD  (FlightModeAcceptable)")
    _row("drone_state.position",          f"lat={GC_POS['lat']:.4f} lon={GC_POS['lon']:.4f}  (follower starting position near GC)")

    _wave(5, "BT tree tick — Wave 5")
    root = build_relay_decision_tree(bb, CFG, clock=clock.now)
    bt = py_trees.trees.BehaviourTree(root)
    bt.setup()
    bt.tick()
    _print_bt_tick(root, tick_num=1)

    proposal = bb.get("pending_proposal")
    print(f"  │")
    _row("pending_proposal.strategy", proposal["strategy"] if proposal else "NONE")
    if proposal and proposal.get("strategy") == "CONTINUOUS_RELAY":
        rp = proposal["relay_position"]
        co = proposal.get("cost", {})
        _row("relay_position", f"lat={rp['lat']:.5f}  lon={rp['lon']:.5f}  alt={rp['alt']:.0f}m")
        _row("cost.battery_cost_pct", f"{co.get('battery_cost_pct', '?')} %")
        _row("cost.eta_seconds",      f"{co.get('eta_seconds', '?')} s")
        _row("cost.repositioning_m",  f"{co.get('repositioning_m', '?')} m")

    _chk(results, "[A] root status SUCCESS", root.status == py_trees.common.Status.SUCCESS)
    _chk(results, "[A] proposal strategy = CONTINUOUS_RELAY",
         proposal is not None and proposal.get("strategy") == "CONTINUOUS_RELAY",
         str(proposal.get("strategy") if proposal else "None"))


def scenario_b(results):
    """RELAYING, all maintenance gates pass → CONTINUE."""
    _hdr("SCENARIO B — RELAYING, all maintenance gates pass → CONTINUE")

    clock = FakeClock()
    pipe = run_pipeline(_CLEAN_SEV, clock.now())
    _print_pipeline(pipe, _CLEAN_SEV)

    _wave(4, "Blackboard ← pipeline + relaying state")
    now = clock.now()
    bb = TimestampedBlackboard(clock=clock.now)
    bb.set("current_role", "RELAYING")
    bb.set("relay_tasking_received", {
        "tasking_id": "t-B001", "leader_id": LEADER_ID, "leader_pos": LEADER_POS,
    })
    bb.set("drone_state", {
        "battery_pct": 60.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
        "position": RELAY_POS, "home_pos": GC_POS,
        "timestamp": now,
    })
    bb.set("signal_report", {
        "follower_to_gc":     pipe["flw_health"].get("gc_to_follower") or {"snr_db": 25.0},
        "leader_to_follower": pipe["flw_health"].get("leader_to_follower") or {"snr_db": 22.0},
        "timestamp": now,
    })
    _row("current_role",           "RELAYING  → RELAYING_BRANCH fires")
    _row("drone_state.battery_pct","60 %  (above return+reserve ~35.6 %)")
    _row("drone_state.flight_mode","OFFBOARD  (G3 passes)")
    _row("signal_report snr",      f"gc={pipe['flw_health'].get('gc_to_follower', {}).get('snr_db', 25.0):.1f}dB  "
                                   f"ldr={pipe['flw_health'].get('leader_to_follower', {}).get('snr_db', 22.0):.1f}dB  "
                                   f"(both ≥ min_snr={CFG['min_snr_db']}dB → G7 passes)")
    _row("authorization_valid_until","not set  → G8 passes immediately")
    _row("reauth_requested_at",    "not set  → REAUTH_TIMEOUT passes")

    _wave(5, "BT tree tick — Wave 5")
    root = build_relay_decision_tree(bb, CFG, clock=clock.now)
    bt = py_trees.trees.BehaviourTree(root)
    bt.setup()
    bt.tick()
    _print_bt_tick(root, tick_num=1)

    proposal = bb.get("pending_proposal")
    print(f"  │")
    _row("pending_proposal", str(proposal) if proposal else "NONE  (CONTINUE was reached)")
    _row("CONTINUE reached?", "YES" if proposal is None else "NO — a gate fired")

    _chk(results, "[B] root status SUCCESS", root.status == py_trees.common.Status.SUCCESS)
    _chk(results, "[B] no pending_proposal (CONTINUE reached)", proposal is None)
    state = dump_state(root)
    _chk(results, "[B] CONTINUE node reached SUCCESS",
         state.get("CONTINUE", {}).get("status") == "SUCCESS")


def scenario_c(results):
    """RELAYING, signal degrades over 3 ticks — G7 debounce fires → EXIT_RELAY."""
    _hdr("SCENARIO C — RELAYING, link degrades x3 → G7 debounce → ProposeExitRelay")

    clock = FakeClock()
    pipe = run_pipeline(_BAD_SEV, clock.now())
    _print_pipeline(pipe, _BAD_SEV)

    _wave(4, "Blackboard ← pipeline + relaying state  (poor signal_report SNR)")
    now = clock.now()
    bb = TimestampedBlackboard(clock=clock.now)
    bb.set("current_role", "RELAYING")
    bb.set("relay_tasking_received", {
        "tasking_id": "t-C001", "leader_id": LEADER_ID, "leader_pos": LEADER_POS,
    })
    bb.set("drone_state", {
        "battery_pct": 60.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
        "position": RELAY_POS, "home_pos": GC_POS,
        "timestamp": now,
    })
    bb.set("current_relay_target", RELAY_POS)
    bb.set("signal_report", {
        "follower_to_gc":     {"snr_db": 3.0},
        "leader_to_follower": {"snr_db": 5.0},
        "timestamp": now,
    })
    _row("signal_report snr",     "gc=3.0dB  ldr=5.0dB  (both < min_snr=8dB)")
    _row("debounce_n",            f"{CFG['debounce_n']} consecutive bad ticks required before G7 FAILS")

    _wave(5, "BT tree — 3 ticks with sustained bad SNR  (Wave 5)")
    root = build_relay_decision_tree(bb, CFG, clock=clock.now)
    bt = py_trees.trees.BehaviourTree(root)
    bt.setup()

    for tick in range(1, 4):
        now2 = clock.now()
        bb.set("drone_state", {
            "battery_pct": 60.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
            "position": RELAY_POS, "home_pos": GC_POS,
            "timestamp": now2,
        })
        bb.set("signal_report", {
            "follower_to_gc":     {"snr_db": 3.0},
            "leader_to_follower": {"snr_db": 5.0},
            "timestamp": now2,
        })
        bt.tick()
        state = dump_state(root)
        g7 = state.get("RelayLinkAdequate", {})
        prop = bb.get("pending_proposal")
        outcome = (f"→ {g7['status']}  {g7['feedback'][:55]}"
                   + (f"  || proposal: {prop['strategy']}" if prop else ""))
        print(f"  │  [tick {tick}]  G7_RelayLinkAdequate {outcome}")
        clock.advance(CFG["bt_tick_period_s"])

    _print_bt_tick(root, tick_num=3)

    proposal = bb.get("pending_proposal")
    print(f"  │")
    _row("pending_proposal.strategy", proposal["strategy"] if proposal else "NONE")
    _row("pending_proposal.reason",   proposal.get("reason", "N/A") if proposal else "N/A")

    _chk(results, "[C] root status SUCCESS", root.status == py_trees.common.Status.SUCCESS)
    _chk(results, "[C] proposal strategy = EXIT_RELAY",
         proposal is not None and proposal.get("strategy") == "EXIT_RELAY",
         str(proposal.get("strategy") if proposal else "None"))
    _chk(results, "[C] reason = link_ineffective",
         proposal is not None and proposal.get("reason") == "link_ineffective",
         str(proposal.get("reason") if proposal else "None"))


def scenario_d(results):
    """RELAYING, FCU telemetry goes stale — G1 fires → FollowerSafetyExit + alert_intent."""
    _hdr("SCENARIO D — RELAYING, FCU stale → G1 → FollowerSafetyExit + alert_intent")

    clock = FakeClock()
    pipe = run_pipeline(_CLEAN_SEV, clock.now())

    _wave(4, "Blackboard ← pipeline + relaying state, then advance clock past FCU max age")
    now = clock.now()
    bb = TimestampedBlackboard(clock=clock.now)
    bb.set("current_role", "RELAYING")
    bb.set("relay_tasking_received", {
        "tasking_id": "t-D001", "leader_id": LEADER_ID, "leader_pos": LEADER_POS,
    })
    bb.set("drone_state", {
        "battery_pct": 60.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
        "position": RELAY_POS, "home_pos": GC_POS,
        "timestamp": now,
    })
    bb.set("signal_report", {
        "follower_to_gc":     pipe["flw_health"].get("gc_to_follower") or {"snr_db": 25.0},
        "leader_to_follower": pipe["flw_health"].get("leader_to_follower") or {"snr_db": 22.0},
        "timestamp": now,
    })
    _row("drone_state written at", f"t={clock.now():.0f}s")
    _row("fcu_telemetry_max_age_s", f"{CFG['fcu_telemetry_max_age_s']} s")

    clock.advance(10.0)
    _row("clock advanced to", f"t={clock.now():.0f}s  (drone_state age = 10s > 3s → STALE)")
    _row("G1 FcuTelemetryFresh", "→ FAILURE  (Inv(G1) = SUCCESS → FollowerSafetyExit fires)")
    _row("Layer 1 discipline", "FollowerSafetyExit writes alert_intent to BB; NO publish (§5.5, §4.7 item 11)")

    _wave(5, "BT tree tick — Wave 5  (FCU stale)")
    root = build_relay_decision_tree(bb, CFG, clock=clock.now)
    bt = py_trees.trees.BehaviourTree(root)
    bt.setup()
    bt.tick()
    _print_bt_tick(root, tick_num=1)

    alert = bb.get("alert_intent")
    cmd = bb.get("pending_command")
    print(f"  │")
    if alert:
        _row("alert_intent.type",   alert["type"])
        _row("alert_intent.reason", alert["reason"])
        _row("alert_intent.battery_pct", f"{alert['battery_pct']:.0f} %")
        _row("pending_command", "NONE  (LOST_FC → PX4 failsafe owns airframe)")
    else:
        _row("alert_intent", "MISSING — test failure")

    _chk(results, "[D] root status SUCCESS", root.status == py_trees.common.Status.SUCCESS)
    _chk(results, "[D] alert_intent written",
         alert is not None, str(alert))
    _chk(results, "[D] alert_intent.type = FOLLOWER_SAFETY_EXIT",
         alert is not None and alert.get("type") == "FOLLOWER_SAFETY_EXIT",
         str(alert.get("type") if alert else "None"))
    _chk(results, "[D] alert_intent.reason = fcu_telemetry_lost",
         alert is not None and alert.get("reason") == "fcu_telemetry_lost",
         str(alert.get("reason") if alert else "None"))
    _chk(results, "[D] pending_command NOT written (LOST_FC path)", cmd is None,
         str(cmd))


def scenario_e(results):
    """RELAYING, auth timer expired — G8 writes reauth → 130s later REAUTH_TIMEOUT fires."""
    _hdr("SCENARIO E — auth expired → G8 reauth → 130s later → ProposeExitRelay")

    clock = FakeClock()
    pipe = run_pipeline(_CLEAN_SEV, clock.now())

    _wave(4, "Blackboard ← pipeline + relaying state, authorization_valid_until already past")
    now = clock.now()
    bb = TimestampedBlackboard(clock=clock.now)
    bb.set("current_role", "RELAYING")
    bb.set("relay_tasking_received", {
        "tasking_id": "t-E001", "leader_id": LEADER_ID, "leader_pos": LEADER_POS,
    })
    bb.set("current_relay_target", RELAY_POS)
    bb.set("drone_state", {
        "battery_pct": 60.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
        "position": RELAY_POS, "home_pos": GC_POS,
        "timestamp": now,
    })
    bb.set("signal_report", {
        "follower_to_gc":     pipe["flw_health"].get("gc_to_follower") or {"snr_db": 25.0},
        "leader_to_follower": pipe["flw_health"].get("leader_to_follower") or {"snr_db": 22.0},
        "timestamp": now,
    })
    bb.set("authorization_valid_until", now - 5.0)
    _row("authorization_valid_until", f"t={now - 5.0:.0f}s  (5s ago — already expired)")
    _row("reauth_response_timeout_s",  f"{CFG['reauth_response_timeout_s']} s")
    _row("reauth_requested_at",        "not set  (G8 will write it on first FAIL)")

    _wave(5, "BT tree — 2 ticks spanning the 120s reauth window  (Wave 5)")
    root = build_relay_decision_tree(bb, CFG, clock=clock.now)
    bt = py_trees.trees.BehaviourTree(root)
    bt.setup()

    print(f"  │  [tick 1 @ t={clock.now():.0f}s]")
    bt.tick()
    state1 = dump_state(root)
    g8 = state1.get("RelayActuallyImproved", {})
    rt = state1.get("ReauthResponseTimedOut", {})
    cont = state1.get("CONTINUE", {})
    print(f"  │    G8  RelayActuallyImproved: {g8['status']}  {g8['feedback'][:55]}")
    print(f"  │    ReauthResponseTimedOut: {rt['status']}  {rt['feedback'][:55]}")
    print(f"  │    CONTINUE: {cont['status']}")
    print(f"  │    reauth_requested_at written: {bb.get('reauth_requested_at'):.0f}s")
    print(f"  │    pending_proposal: {bb.get('pending_proposal')}")

    clock.advance(130.0)
    print(f"  │  [clock +130s → t={clock.now():.0f}s]  reauth elapsed 130s > 120s threshold")
    now2 = clock.now()
    bb.set("drone_state", {
        "battery_pct": 60.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
        "position": RELAY_POS, "home_pos": GC_POS,
        "timestamp": now2,
    })
    bb.set("signal_report", {
        "follower_to_gc":     {"snr_db": 25.0},
        "leader_to_follower": {"snr_db": 22.0},
        "timestamp": now2,
    })

    print(f"  │  [tick 2 @ t={clock.now():.0f}s]")
    bt.tick()
    state2 = dump_state(root)
    g8b = state2.get("RelayActuallyImproved", {})
    rtb = state2.get("ReauthResponseTimedOut", {})
    print(f"  │    G8  RelayActuallyImproved: {g8b['status']}  {g8b['feedback'][:55]}")
    print(f"  │    ReauthResponseTimedOut: {rtb['status']}  {rtb['feedback'][:55]}")
    print(f"  │    reauth_requested_at (unchanged): {bb.get('reauth_requested_at'):.0f}s  "
          f"(None-guard prevented overwrite)")

    proposal = bb.get("pending_proposal")
    print(f"  │")
    _row("pending_proposal.strategy", proposal["strategy"] if proposal else "NONE")
    _row("pending_proposal.reason",   proposal.get("reason") if proposal else "N/A")

    _chk(results, "[E] root status SUCCESS tick 1", root.status == py_trees.common.Status.SUCCESS)
    _chk(results, "[E] reauth_requested_at written at t=1000",
         math.isclose(bb.get("reauth_requested_at") or -1, 1000.0, abs_tol=0.1))
    _chk(results, "[E] proposal strategy = EXIT_RELAY",
         proposal is not None and proposal.get("strategy") == "EXIT_RELAY",
         str(proposal.get("strategy") if proposal else "None"))


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    total_pass = total_fail = 0

    for runner in (scenario_a, scenario_b, scenario_c, scenario_d, scenario_e):
        results = []
        runner(results)

        n_pass = sum(1 for _, ok, _ in results if ok)
        n_fail = sum(1 for _, ok, _ in results if not ok)
        total_pass += n_pass
        total_fail += n_fail

        print()
        _sep()
        print(f"  Checks: {n_pass} passed  {n_fail} failed  ({len(results)} total)")
        for label, ok, detail in results:
            icon = "✓" if ok else "✗"
            line = f"  {icon}  {label}"
            if detail:
                line += f"  [{detail}]"
            print(line)

    print(f"\n{'═' * W}")
    print(f"  TOTAL: {total_pass} passed, {total_fail} failed across 5 scenarios")
    if total_fail > 0:
        print("  INTEGRATION TEST FAILED")
        sys.exit(1)
    else:
        print("  INTEGRATION TEST PASSED")


if __name__ == "__main__":
    main()
