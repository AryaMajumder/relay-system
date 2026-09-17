#!/usr/bin/env python3
"""
integration_waves0_to_6.py — end-to-end pipeline, Waves 0–6.

Extends integration_waves0_to_5.py by adding Wave 6 (capability_assessor)
after each BT tick.  The assessor's drain cycle converts blackboard proposals,
alerts, and commands into published messages (captured in lists here instead
of real ROS2 topics).

Pipeline:
  [Wave 0]  DEMO_CONFIG                   config surface (§3)
  [Wave 1]  SignalFaker.tick()            /signal/* (5 hops, §2.1)
  [Wave 2]  *_radio_health_reader         /*/radio_health per role
  [Wave 3]  gc_link_observer              /gc/gc_link_quality
            leader_link_detector          relay_request | silent
  [Wave 4]  TimestampedBlackboard         write from pipeline, age keys
  [Wave 5]  BT tree tick                  condition + action nodes → proposal
  [Wave 6]  capability_assessor drain     BB → capability_report + published slots

Scenarios:
  A — IDLE, clean signals   → CONTINUOUS_RELAY proposal drained → capability report
  B — RELAYING, all pass    → CONTINUE, no proposal → report only, relay_assignment lifecycle
  C — RELAYING, 3× bad SNR  → EXIT_RELAY drained → quality sub torn down
  D — RELAYING, FCU stale   → alert_intent drained → report includes alert
  E — RELAYING, auth expired → EXIT_RELAY after 130s → quality sub torn down

Run:
  python3 tests/integration_waves0_to_6.py
"""

import sys
import os
import math

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
_DC  = os.path.join(_PKG_ROOT, "drone_control")
_RBT = os.path.join(_DC, "relay_bt")
for p in (_PKG_ROOT, _DC, _RBT):
    if p not in sys.path:
        sys.path.insert(0, p)

import py_trees

from drone_control.config.demo_config import DEMO_CONFIG
from drone_control.signal_faker import SignalFaker, haversine
from drone_control.leader_radio_health_reader import make_leader_reader
from drone_control.follower_radio_health_reader import make_follower_reader
from drone_control.gc_radio_health_reader import make_gc_reader
from drone_control.gc_link_observer import make_gc_link_observer
from drone_control.leader_link_detector import make_leader_link_detector
from drone_control.relay_bt.blackboard import TimestampedBlackboard
from drone_control.relay_bt.tree_builder import build_relay_decision_tree, dump_state, walk_tree
from drone_control.relay_bt.condition_nodes import ConditionNodeBase, DataFreshness
from drone_control.capability_assessor import (
    _drain,
    _apply_relay_assignment,
    _status_to_string,
)


# ── Topology constants ────────────────────────────────────────────────────────

LEADER_ID   = "drone-01"
FOLLOWER_ID = "drone-02"
GC_POS      = {"lat": 47.3900, "lon": 8.5400, "alt": 0.0}
LEADER_POS  = {"lat": 47.3980, "lon": 8.5480, "alt": 50.0}
RELAY_POS   = {"lat": 47.3940, "lon": 8.5440, "alt": 40.0}


# ── Merged config ─────────────────────────────────────────────────────────────

CFG = {
    **DEMO_CONFIG,
    "min_gps_fix_type":               0,
    "recover_offboard_max_attempts":  5,
    "debounce_n":                     3,
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


# ── Wave 6 harness: pure-Python CapabilityAssessor double ─────────────────────

class _AssessorCore:
    """
    Pure-Python test double for CapabilityAssessor (no ROS2 required).

    Mirrors CapabilityAssessor._tick() and _build_report() without rclpy.
    Publishers → capture lists.  Subscription lifecycle → boolean flag.
    """

    def __init__(self, bb: TimestampedBlackboard, config: dict, clock: FakeClock):
        self._bb       = bb
        self._config   = config
        self._clock_fn = clock.now

        self._tree_root = build_relay_decision_tree(bb, config, clock=clock.now)
        self._bt        = py_trees.trees.BehaviourTree(root=self._tree_root)

        self._df_node = None
        for node in walk_tree(self._tree_root):
            if isinstance(node, DataFreshness):
                self._df_node = node
                break

        self.report_captures   = []
        self.cmd_captures      = []
        self.alert_captures    = []
        self.proposal_captures = []

        self._quality_sub_active = False

    def tick(self) -> dict:
        """BT tick + drain cycle — mirrors CapabilityAssessor._tick()."""
        self._bt.tick()
        report = self._build_report()

        _drain(self._bb, "pending_command", self.cmd_captures.append)

        proposal = self._bb.get("pending_proposal")
        if proposal and proposal.get("strategy") in ("EXIT_RELAY", "LET_LEADER_ISOLATE"):
            self._teardown_quality_sub()

        _drain(self._bb, "pending_proposal", self.proposal_captures.append)
        _drain(self._bb, "alert_intent",     self.alert_captures.append)

        self.report_captures.append(report)
        return report

    def apply_relay_assignment(self, assignment: dict):
        """Simulates _on_relay_assignment(): writes §6 BB keys, starts quality sub."""
        _apply_relay_assignment(self._bb, assignment)
        self._quality_sub_active = True

    def _ensure_quality_sub(self):
        self._quality_sub_active = True

    def _teardown_quality_sub(self):
        self._quality_sub_active = False

    def _build_report(self) -> dict:
        windows = self._config["staleness_windows_s"]
        now     = self._clock_fn()

        freshness = {}
        for key in ("signal_report", "drone_state"):
            data = self._bb.get(key)
            if data is None:
                freshness[key] = {"age_s": None, "fresh": False}
            else:
                ts  = data.get("timestamp") if isinstance(data, dict) else None
                age = round(now - ts, 3) if ts is not None else None
                freshness[key] = {
                    "age_s": age,
                    "fresh": age is not None and age <= windows[key],
                }

        checks = {}
        for node in walk_tree(self._tree_root):
            if isinstance(node, ConditionNodeBase):
                checks[node.name] = {
                    "pass":   node.status == py_trees.common.Status.SUCCESS,
                    "detail": getattr(node, "feedback_message", ""),
                }

        status_str = _status_to_string(self._tree_root, self._df_node)
        return {
            "drone_id":         FOLLOWER_ID,
            "timestamp":        now,
            "capable":          status_str == "CAPABLE",
            "status":           status_str,
            "tick_duration_ms": 0.0,
            "data_freshness":   freshness,
            "checks":           checks,
            "inputs":           {},
            "pending_proposal": self._bb.get("pending_proposal"),
        }


# ── Display helpers ───────────────────────────────────────────────────────────

W = 76


def _bar(v, lo, hi, w=24, threshold=None):
    span   = max(hi - lo, 1e-9)
    filled = int(round(max(0, min(w, (v - lo) / span * w))))
    bar    = list("█" * filled + "·" * (w - filled))
    if threshold is not None:
        tp = int(round(max(0, min(w - 1, (threshold - lo) / span * w))))
        bar[tp] = "▼"
    return "[" + "".join(bar) + "]"


def _row(label, value, indent="  │  "):
    print(f"{indent}{label:<32} {value}")


def _hdr(text):
    print(f"\n  ╔{'═' * (W - 4)}╗")
    print(f"  ║  {text:<{W - 6}}║")
    print(f"  ╚{'═' * (W - 4)}╝")


def _wave(num, text):
    print(f"\n  ┌─ WAVE {num}: {text}")


def _snr_row(snr_db, label="snr_db"):
    bar = _bar(snr_db, -20, 60, threshold=MARGINAL)
    cmp = "< BELOW" if snr_db < MARGINAL else "≥ ABOVE"
    print(f"  │  {label:<32} {bar} {snr_db:+7.2f} dB  {cmp} {MARGINAL}")


def _status_icon(name):
    return {"SUCCESS": "✓", "FAILURE": "✗", "RUNNING": "~", "INVALID": "·"}.get(name, "?")


def _chk(results, label, cond, detail=""):
    results.append((label, bool(cond), detail))


# ── Wave 1–3 pipeline ─────────────────────────────────────────────────────────

def run_pipeline(severities: dict, clock_t: float) -> dict:
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
    for topic, _ in [(f"/signal/leader_to_follower/{FOLLOWER_ID}", "leader_to_follower"),
                     (f"/signal/gc_to_follower/{FOLLOWER_ID}",     "gc_to_follower")]:
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
    gc_obs = make_gc_link_observer(CFG, publish=lambda t, p: gc_quality_bus.update({t: p}))
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
        "signal_bus":     signal_bus,
        "ldr_health":     ldr_health_bus,
        "flw_health":     flw_health_bus,
        "gc_quality":     gc_quality_bus,
        "relay_requests": relay_requests,
    }


def _print_pipeline(pipe: dict, severities: dict):
    _wave(1, "SignalFaker → /signal/* (5 hops)")
    hops = [
        ("gc_to_leader",       "/signal/gc_to_leader"),
        ("leader_to_gc",       "/signal/leader_to_gc"),
        ("gc→flw",             f"/signal/gc_to_follower/{FOLLOWER_ID}"),
        ("flw→gc",             f"/signal/follower_to_gc/{FOLLOWER_ID}"),
        ("ldr→flw",            f"/signal/leader_to_follower/{FOLLOWER_ID}"),
    ]
    hop_names = ["gc_to_leader", "leader_to_gc", "gc_to_follower",
                 "follower_to_gc", "leader_to_follower"]
    print(f"  │  {'hop':<22} {'sev':>4}  {'rssi':>8}  snr")
    print(f"  │  {'─' * 60}")
    for (label, topic), name in zip(hops, hop_names):
        sig = pipe["signal_bus"].get(topic)
        sev = severities.get(name, 0.0)
        if sig:
            bar = _bar(sig["snr_db"], -20, 60, w=14, threshold=MARGINAL)
            cmp = "↓BELOW" if sig["snr_db"] < MARGINAL else "ABOVE"
            print(f"  │  {label:<22} {sev:>4.2f}  {sig['rssi_dbm']:>+8.2f}  "
                  f"{bar} {sig['snr_db']:+.1f}dB {cmp}")

    _wave("2+3", "radio_health_readers → gc_link_observer → leader_link_detector")
    lh = pipe["ldr_health"].get("gc_to_leader")
    if lh:
        _snr_row(lh["snr_db"], "gc_to_leader snr_db")
    gq = pipe["gc_quality"].get("/gc/gc_link_quality")
    if gq:
        q_bar = _bar(gq["quality"], 0, 1, threshold=0.5)
        _row("gc_link_quality", f"{q_bar} {gq['quality']:.4f}")
    if pipe["relay_requests"]:
        _row("relay_request", "FIRED")
    else:
        _row("relay_request", "SILENT")


def _print_bt_tick(core: _AssessorCore, tick_num=1):
    state = dump_state(core._tree_root)
    active = {k: v for k, v in state.items() if v["status"] != "INVALID"}
    print(f"  │  [tick {tick_num}]  root → {core._tree_root.status.name}")
    for name, info in active.items():
        icon     = _status_icon(info["status"])
        short_fb = info["feedback"][:52]
        print(f"  │    {icon} {name:<42} {info['status']:<9} {short_fb}")


def _print_wave6(core: _AssessorCore, report: dict):
    _wave(6, "capability_assessor drain cycle")
    _row("status",         report.get("status", "?"))
    _row("capable",        str(report.get("capable")))

    freshness = report.get("data_freshness", {})
    for key, fd in freshness.items():
        age_str = f"{fd['age_s']:.1f}s" if fd["age_s"] is not None else "never"
        fresh   = "FRESH" if fd["fresh"] else "STALE"
        _row(f"  freshness.{key}", f"{age_str}  →  {fresh}")

    if core.proposal_captures:
        p = core.proposal_captures[-1]
        _row("→ strategy_proposal", f"strategy={p['strategy']}")
        if p.get("relay_position"):
            rp = p["relay_position"]
            _row("  relay_position", f"lat={rp['lat']:.5f}  lon={rp['lon']:.5f}  alt={rp.get('alt_m', rp.get('alt', '?'))}m")
        if p.get("reason"):
            _row("  reason", p["reason"])
    else:
        _row("→ strategy_proposal", "NONE  (CONTINUE path — no drain needed)")

    if core.alert_captures:
        a = core.alert_captures[-1]
        _row("→ alert_intent", f"type={a['type']}  reason={a['reason']}")
    else:
        _row("→ alert_intent", "NONE")

    if core.cmd_captures:
        _row("→ pending_command", str(core.cmd_captures[-1]))
    else:
        _row("→ pending_command", "NONE")

    sub_state = "ACTIVE" if core._quality_sub_active else "INACTIVE"
    _row("quality_sub lifecycle", sub_state)
    _row("capability_reports", f"{len(core.report_captures)} published this run")


# ── Severity presets ──────────────────────────────────────────────────────────

_CLEAN_SEV = {k: 0.0 for k in
    ["gc_to_leader", "leader_to_gc", "gc_to_follower",
     "follower_to_gc", "leader_to_follower"]}

_BAD_SEV = {**_CLEAN_SEV, "gc_to_leader": 0.95, "leader_to_gc": 0.95}


# ═══════════════════════════════════════════════════════════════════════════════
# SCENARIOS
# ═══════════════════════════════════════════════════════════════════════════════

def scenario_a(results):
    """IDLE, clean signals → CONTINUOUS_RELAY proposal drained by assessor."""
    _hdr("SCENARIO A — IDLE → FULL_ENTRY → CONTINUOUS_RELAY drained (Waves 0–6)")

    clock = FakeClock()
    pipe  = run_pipeline(_CLEAN_SEV, clock.now())
    _print_pipeline(pipe, _CLEAN_SEV)

    _wave(4, "Blackboard ← pipeline + synthetic drone state")
    now = clock.now()
    bb  = TimestampedBlackboard(clock=clock.now)
    bb.set("current_role", "IDLE")
    bb.set("relay_tasking_received", {
        "tasking_id": "t-A001", "leader_id": LEADER_ID, "leader_pos": LEADER_POS,
    })
    lh = pipe["ldr_health"].get("gc_to_leader", {"snr_db": 30.0, "severity": 0.0})
    bb.set("leader_radio_health", {**lh, "timestamp": now})
    bb.set("signal_report", {
        "follower_to_gc":     pipe["flw_health"].get("gc_to_follower") or {"snr_db": 25.0},
        "leader_to_follower": pipe["flw_health"].get("leader_to_follower") or {"snr_db": 22.0},
        "timestamp":          now,
    })
    bb.set("drone_state", {
        "battery_pct": 80.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
        "position": GC_POS, "home_pos": GC_POS,
        "timestamp": now,
    })
    _row("current_role",         "IDLE  → IDLE_BRANCH")
    _row("drone_state.battery",  "80 %  (entry gate BatteryAboveFloor)")
    _row("drone_state.position", f"lat={GC_POS['lat']:.4f}  lon={GC_POS['lon']:.4f}  (follower at GC)")

    _wave("5+6", "BT tick → Wave 5 decision + Wave 6 drain")
    core   = _AssessorCore(bb, CFG, clock)
    report = core.tick()
    _print_bt_tick(core, tick_num=1)
    _print_wave6(core, report)

    proposal = core.proposal_captures[-1] if core.proposal_captures else None
    _chk(results, "[A] BT root SUCCESS",
         core._tree_root.status == py_trees.common.Status.SUCCESS)
    _chk(results, "[A] CONTINUOUS_RELAY proposal published",
         proposal is not None and proposal.get("strategy") == "CONTINUOUS_RELAY",
         str(proposal.get("strategy") if proposal else "None"))
    _chk(results, "[A] pending_proposal cleared from BB after drain",
         bb.get("pending_proposal") is None)
    _chk(results, "[A] capability_report produced",
         len(core.report_captures) == 1)
    _chk(results, "[A] quality_sub inactive (no relay_assignment yet)",
         not core._quality_sub_active)


def scenario_b(results):
    """RELAYING, all gates pass → CONTINUE; relay_assignment lifecycle verified."""
    _hdr("SCENARIO B — RELAYING, all pass → CONTINUE; relay_assignment → quality sub active (Waves 0–6)")

    clock = FakeClock()
    pipe  = run_pipeline(_CLEAN_SEV, clock.now())
    _print_pipeline(pipe, _CLEAN_SEV)

    _wave(4, "Blackboard ← pipeline + relaying state + relay_assignment")
    now = clock.now()
    bb  = TimestampedBlackboard(clock=clock.now)
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

    core = _AssessorCore(bb, CFG, clock)

    # Simulate relay_assignment arriving (§6 BB keys + quality sub start).
    core.apply_relay_assignment({
        "relay_target": RELAY_POS,
        "tolerance_radius_m": 10.0,
        "valid_until": now + 300.0,
    })
    _row("relay_assignment received", f"target={RELAY_POS['lat']:.4f},{RELAY_POS['lon']:.4f}  valid_until=+300s")
    _row("quality_sub after assignment", "ACTIVE  (§4.11 item 18)")
    _row("current_role",               "RELAYING  → RELAYING_BRANCH")
    _row("drone_state.battery",        "60 %  (above return+reserve)")

    _wave("5+6", "BT tick → CONTINUE + Wave 6 report")
    report = core.tick()
    _print_bt_tick(core, tick_num=1)
    _print_wave6(core, report)

    _chk(results, "[B] BT root SUCCESS",
         core._tree_root.status == py_trees.common.Status.SUCCESS)
    _chk(results, "[B] no proposal published (CONTINUE path)",
         len(core.proposal_captures) == 0)
    _chk(results, "[B] CONTINUE node reached SUCCESS",
         dump_state(core._tree_root).get("CONTINUE", {}).get("status") == "SUCCESS")
    _chk(results, "[B] quality_sub still active (no EXIT in CONTINUE path)",
         core._quality_sub_active)
    _chk(results, "[B] capability_report produced",
         len(core.report_captures) == 1)
    _chk(results, "[B] BB keys written by relay_assignment",
         bb.get("tolerance_radius_m") == 10.0
         and bb.get("authorization_valid_until") == now + 300.0)


def scenario_c(results):
    """RELAYING, 3× bad SNR → G7 debounce → EXIT_RELAY; quality sub torn down."""
    _hdr("SCENARIO C — RELAYING, link degrades ×3 → EXIT_RELAY drained, sub torn down (Waves 0–6)")

    clock = FakeClock()
    pipe  = run_pipeline(_BAD_SEV, clock.now())
    _print_pipeline(pipe, _BAD_SEV)

    _wave(4, "Blackboard ← poor signal + relaying state + relay_assignment")
    now = clock.now()
    bb  = TimestampedBlackboard(clock=clock.now)
    bb.set("current_role", "RELAYING")
    bb.set("relay_tasking_received", {
        "tasking_id": "t-C001", "leader_id": LEADER_ID, "leader_pos": LEADER_POS,
    })
    bb.set("current_relay_target", RELAY_POS)
    bb.set("drone_state", {
        "battery_pct": 60.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
        "position": RELAY_POS, "home_pos": GC_POS,
        "timestamp": now,
    })
    bb.set("signal_report", {
        "follower_to_gc":     {"snr_db": 3.0},
        "leader_to_follower": {"snr_db": 5.0},
        "timestamp": now,
    })

    core = _AssessorCore(bb, CFG, clock)
    core.apply_relay_assignment({
        "relay_target": RELAY_POS,
        "tolerance_radius_m": 10.0,
        "valid_until": now + 300.0,
    })
    _row("signal snr (gc, ldr)", "3.0 dB, 5.0 dB  (both < min_snr=8 dB)")
    _row("debounce_n",           f"{CFG['debounce_n']}  consecutive bad ticks needed")
    _row("quality_sub initially", "ACTIVE")

    _wave("5+6", f"BT {CFG['debounce_n']} ticks → debounce → EXIT_RELAY + Wave 6 drain")
    report = None
    for tick in range(1, CFG["debounce_n"] + 1):
        now2 = clock.now()
        bb.set("drone_state", {
            "battery_pct": 60.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
            "position": RELAY_POS, "home_pos": GC_POS, "timestamp": now2,
        })
        bb.set("signal_report", {
            "follower_to_gc":     {"snr_db": 3.0},
            "leader_to_follower": {"snr_db": 5.0},
            "timestamp": now2,
        })
        report = core.tick()
        g7 = dump_state(core._tree_root).get("RelayLinkAdequate", {})
        prop = core.proposal_captures[-1]["strategy"] if core.proposal_captures else "pending"
        print(f"  │  [tick {tick}]  G7_RelayLinkAdequate → {g7['status']}  "
              f"{'EXIT_RELAY drained' if core.proposal_captures else g7['feedback'][:40]}")
        clock.advance(CFG["bt_tick_period_s"])

    _print_wave6(core, report)

    proposal = core.proposal_captures[-1] if core.proposal_captures else None
    _chk(results, "[C] EXIT_RELAY proposal published",
         proposal is not None and proposal.get("strategy") == "EXIT_RELAY",
         str(proposal.get("strategy") if proposal else "None"))
    _chk(results, "[C] reason = link_ineffective",
         proposal is not None and proposal.get("reason") == "link_ineffective",
         str(proposal.get("reason") if proposal else "None"))
    _chk(results, "[C] pending_proposal cleared from BB",
         bb.get("pending_proposal") is None)
    _chk(results, "[C] quality_sub torn down on EXIT_RELAY",
         not core._quality_sub_active)
    _chk(results, "[C] capability_reports produced every tick",
         len(core.report_captures) == CFG["debounce_n"])


def scenario_d(results):
    """RELAYING, FCU telemetry stale → G1 → FollowerSafetyExit + alert_intent drained."""
    _hdr("SCENARIO D — RELAYING, FCU stale → alert_intent drained (Waves 0–6)")

    clock = FakeClock()
    pipe  = run_pipeline(_CLEAN_SEV, clock.now())
    _print_pipeline(pipe, _CLEAN_SEV)

    _wave(4, "Blackboard ← pipeline + relaying state, then clock advances past FCU max age")
    now = clock.now()
    bb  = TimestampedBlackboard(clock=clock.now)
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
    _row("drone_state written at", f"t={now:.0f}s")
    _row("fcu_telemetry_max_age_s", f"{CFG['fcu_telemetry_max_age_s']} s")

    clock.advance(10.0)
    _row("clock advanced to",  f"t={clock.now():.0f}s  (drone_state age = 10s > 3s → STALE)")
    _row("G1 FcuTelemetryFresh", "→ FAILURE  → FollowerSafetyExit fires → alert_intent on BB")
    _row("Layer 1 discipline", "alert_intent written to BB only (no publish from Layer 1)")

    _wave("5+6", "BT tick → FollowerSafetyExit + Wave 6 drains alert_intent")
    core   = _AssessorCore(bb, CFG, clock)
    report = core.tick()
    _print_bt_tick(core, tick_num=1)
    _print_wave6(core, report)

    alert = core.alert_captures[-1] if core.alert_captures else None
    _chk(results, "[D] BT root SUCCESS",
         core._tree_root.status == py_trees.common.Status.SUCCESS)
    _chk(results, "[D] alert_intent drained and published",
         alert is not None,
         str(alert))
    _chk(results, "[D] alert_intent.type = FOLLOWER_SAFETY_EXIT",
         alert is not None and alert.get("type") == "FOLLOWER_SAFETY_EXIT",
         str(alert.get("type") if alert else "None"))
    _chk(results, "[D] alert_intent.reason = fcu_telemetry_lost",
         alert is not None and alert.get("reason") == "fcu_telemetry_lost",
         str(alert.get("reason") if alert else "None"))
    _chk(results, "[D] alert_intent cleared from BB",
         bb.get("alert_intent") is None)
    _chk(results, "[D] no proposal published (LOST_FC path)",
         len(core.proposal_captures) == 0)
    _chk(results, "[D] capability_report produced",
         len(core.report_captures) == 1)


def scenario_e(results):
    """RELAYING, auth expired → G8 writes reauth → 130s → EXIT_RELAY drained."""
    _hdr("SCENARIO E — auth expired → G8 reauth → 130s → EXIT_RELAY drained (Waves 0–6)")

    clock = FakeClock()
    pipe  = run_pipeline(_CLEAN_SEV, clock.now())
    _print_pipeline(pipe, _CLEAN_SEV)

    _wave(4, "Blackboard ← relaying state, authorization_valid_until already past")
    now = clock.now()
    bb  = TimestampedBlackboard(clock=clock.now)
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
    _row("authorization_valid_until", f"t={now - 5.0:.0f}s  (5s in the past → G8 FAILS)")
    _row("reauth_response_timeout_s",  f"{CFG['reauth_response_timeout_s']} s")

    core = _AssessorCore(bb, CFG, clock)
    core.apply_relay_assignment({
        "relay_target": RELAY_POS,
        "tolerance_radius_m": 10.0,
        "valid_until": now - 5.0,
    })

    _wave("5+6", "Tick 1 — G8 fails, reauth_requested_at written; no proposal yet")
    report1 = core.tick()
    _print_bt_tick(core, tick_num=1)
    _row("  reauth_requested_at", f"t={bb.get('reauth_requested_at'):.0f}s  (written by G8)")
    _row("  quality_sub", "ACTIVE  (no EXIT proposal yet)")

    clock.advance(130.0)
    now2 = clock.now()
    bb.set("drone_state", {
        "battery_pct": 60.0, "flight_mode": "OFFBOARD", "gps_fix_type": 3,
        "position": RELAY_POS, "home_pos": GC_POS, "timestamp": now2,
    })
    bb.set("signal_report", {
        "follower_to_gc":     {"snr_db": 25.0},
        "leader_to_follower": {"snr_db": 22.0},
        "timestamp": now2,
    })

    print(f"  │  [clock +130s → t={clock.now():.0f}s  reauth elapsed > {CFG['reauth_response_timeout_s']}s threshold]")
    _wave("5+6", "Tick 2 — ReauthResponseTimedOut fires → EXIT_RELAY drained, sub torn down")
    report2 = core.tick()
    _print_bt_tick(core, tick_num=2)
    _print_wave6(core, report2)

    proposal = core.proposal_captures[-1] if core.proposal_captures else None
    _chk(results, "[E] reauth_requested_at written at t=1000",
         math.isclose(bb.get("reauth_requested_at") or -1, 1000.0, abs_tol=0.1))
    _chk(results, "[E] EXIT_RELAY proposal published on tick 2",
         proposal is not None and proposal.get("strategy") == "EXIT_RELAY",
         str(proposal.get("strategy") if proposal else "None"))
    _chk(results, "[E] pending_proposal cleared from BB",
         bb.get("pending_proposal") is None)
    _chk(results, "[E] quality_sub torn down after EXIT_RELAY",
         not core._quality_sub_active)
    _chk(results, "[E] 2 capability_reports produced (one per tick)",
         len(core.report_captures) == 2)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    total_pass = total_fail = 0

    for runner in (scenario_a, scenario_b, scenario_c, scenario_d, scenario_e):
        results = []
        runner(results)

        print(f"\n  ┌─ RESULTS")
        for label, ok, detail in results:
            icon = "✓" if ok else "✗"
            line = f"  │  {icon}  {label}"
            if not ok and detail:
                line += f"  ← got: {detail}"
            print(line)
        print("  └─")

        passed = sum(1 for _, ok, _ in results if ok)
        failed = sum(1 for _, ok, _ in results if not ok)
        total_pass += passed
        total_fail += failed
        print(f"  {passed}/{passed + failed} checks passed")

    print(f"\n{'═' * 76}")
    print(f"  TOTAL: {total_pass} passed / {total_pass + total_fail} checks"
          + ("  ALL PASS" if total_fail == 0 else f"  {total_fail} FAILED"))
    print(f"{'═' * 76}\n")

    if total_fail:
        sys.exit(1)


if __name__ == "__main__":
    main()
