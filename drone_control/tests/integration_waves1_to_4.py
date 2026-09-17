#!/usr/bin/env python3
"""
integration_waves1_to_4.py — end-to-end pipeline test, Waves 1–4.

Every component from Wave 1 through Wave 4 runs in-process.  Each stage
fires and prints its live output before the next stage begins.

Pipeline:
  [Wave 1]  SignalFaker.tick()
            → /signal/*  (5 hops, §2.1)
            note: state_bridge (Wave 1) bridges MQTT→ROS2 and is not
                  testable in-process; contract-level tests are in
                  test_wave1_state_bridge.py

  [Wave 2a] leader_radio_health_reader   → /{leader}/radio_health
  [Wave 2b] follower_radio_health_reader → /{follower}/radio_health
  [Wave 2c] gc_radio_health_reader       → /gc/radio_health

  [Wave 3a] gc_link_observer             → /gc/gc_link_quality
  [Wave 3b] leader_link_detector         → /{leader}/relay_request | silent

  [Wave 4a] TimestampedBlackboard        → write / age / freshness
  [Wave 4b] geometry                     → band_bounds, relay pos, battery check

Scenarios:
  1 — All clean:         close range, zero severity
  2 — Leader far:        6 km GC↔Leader, zero severity
  3 — GC-leader jammed:  close range, gc_to_leader sev=0.90
  4 — Per-hop mix:       leader=2.5 km, gc_to_leader sev=0.80 follower hops sev=0.10
  5 — Geometry focus:    feasible band, battery boundary cases, §7.1 sentinel

Run:
  python3 tests/integration_waves1_to_4.py
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

from drone_control.signal_faker                import SignalFaker, haversine
from drone_control.leader_radio_health_reader  import make_leader_reader
from drone_control.follower_radio_health_reader import make_follower_reader
from drone_control.gc_radio_health_reader       import make_gc_reader
from drone_control.gc_link_observer            import make_gc_link_observer
from drone_control.leader_link_detector        import make_leader_link_detector
from drone_control.relay_bt.blackboard         import TimestampedBlackboard
from drone_control.relay_bt.geometry           import (
    haversine as geo_haversine,
    band_bounds, band_center, estimate_battery_cost,
)

# ── Config ────────────────────────────────────────────────────────────────────

CFG = {
    "baseline_noise_dbm":     -95.0,
    "noise_range_db":          40.0,
    "tx_power_dbm":            20.0,
    "frequency_mhz":           915,
    "jamming_severity_factor":  0.6,
    "leader_radio_range_m":   800.0,
    "follower_radio_range_m": 800.0,
    "gc_radio_range_m":       800.0,
    "LINK_MARGINAL_QUALITY":   13,
    "return_margin_buffer_pct": 10.0,
}

# Fake model config with real floats for battery cost tests (§7.1: real build
# uses _Unresolved sentinel; we use reals here to prove the arithmetic path).
_FAKE_MODEL = {
    "cruise_speed_mps":           10.0,
    "consumption_rate_pct_per_s":  0.01,  # 1 %/s
}

LEADER_ID  = "drone-01"
FOLLOWER_ID = "drone-02"
GC_POS     = {"lat": 47.3900, "lon": 8.5400, "alt": 0.0}
MARGINAL   = CFG["LINK_MARGINAL_QUALITY"]

# ── Display helpers ───────────────────────────────────────────────────────────

W = 74

def _bar(v, lo, hi, w=26, threshold=None):
    span   = max(hi - lo, 1e-9)
    filled = int(round(max(0, min(w, (v - lo) / span * w))))
    bar    = list("█" * filled + "·" * (w - filled))
    if threshold is not None:
        tp = int(round(max(0, min(w - 1, (threshold - lo) / span * w))))
        bar[tp] = "▼"
    return "[" + "".join(bar) + "]"

def _row(label, value, indent="  │  "):
    print(f"{indent}{label:<26} {value}")

def _hdr(text):
    print(f"\n  ╔{'═'*(W-4)}╗")
    print(f"  ║  {text:<{W-6}}║")
    print(f"  ╚{'═'*(W-4)}╝")

def _stage(tag, text):
    print(f"\n  ┌─ STAGE {tag}: {text}")

def _div():
    print("  " + "─" * (W - 2))

def _snr_row(snr_db, label="snr_db"):
    bar = _bar(snr_db, -20, 60, threshold=MARGINAL)
    cmp = "< BELOW" if snr_db < MARGINAL else "≥ ABOVE"
    print(f"  │  {label:<26} {bar} {snr_db:+7.2f} dB  {cmp} {MARGINAL}")

def _chk(results, label, cond, detail=""):
    results.append((label, bool(cond), detail))

# ── Scenario runner ───────────────────────────────────────────────────────────

class FakeClock:
    def __init__(self, t=0.0): self._t = t
    def now(self): return self._t
    def advance(self, s): self._t += s

def run_scenario(sc):
    results = []

    name       = sc["name"]
    leader_pos = sc["leader_pos"]
    flw_pos    = sc["follower_pos"]
    sev        = sc["severities"]   # dict of hop → float

    _hdr(name)

    dist_gc_ldr = haversine(GC_POS, leader_pos)
    dist_gc_flw = haversine(GC_POS, flw_pos)
    dist_ldr_flw = haversine(leader_pos, flw_pos)

    # ── STAGE 0: Config ───────────────────────────────────────────────────────
    _stage(0, "Config values active for this scenario")
    _row("tx_power_dbm",            f"{CFG['tx_power_dbm']} dBm")
    _row("baseline_noise_dbm",      f"{CFG['baseline_noise_dbm']} dBm  (clean floor)")
    _row("noise_range_db",          f"{CFG['noise_range_db']} dB   (max jamming adds this)")
    _row("frequency_mhz",           f"{CFG['frequency_mhz']} MHz")
    _row("LINK_MARGINAL_QUALITY",   f"{MARGINAL} dB   (relay_request fires below this)")
    _row("*_radio_range_m",         f"{CFG['leader_radio_range_m']} m  (nominal, all roles)")
    _row("return_margin_buffer_pct",f"{CFG['return_margin_buffer_pct']} %   (flat additive)")
    print(f"  │  Per-hop severities:")
    for hop, s in sev.items():
        bar = _bar(s, 0, 1, w=16)
        print(f"  │    {hop:<28} {bar} {s:.2f}")

    # ── STAGE 1: SignalFaker ──────────────────────────────────────────────────
    _stage(1, "Wave 1 — SignalFaker.tick() → /signal/* (5 hops, §2.1)")
    print("  │  note: state_bridge.py bridges MQTT→ROS2; "
          "not testable in-process — contract-level tests in test_wave1_state_bridge.py")
    print(f"  │")
    _row("GC pos",  f"lat={GC_POS['lat']:.4f}  lon={GC_POS['lon']:.4f}  alt={GC_POS['alt']:.0f}m")
    _row("Leader pos", f"lat={leader_pos['lat']:.4f}  lon={leader_pos['lon']:.4f}  alt={leader_pos['alt']:.0f}m  ({dist_gc_ldr:.0f}m from GC)")
    _row("Follower pos",f"lat={flw_pos['lat']:.4f}  lon={flw_pos['lon']:.4f}  alt={flw_pos['alt']:.0f}m  ({dist_gc_flw:.0f}m from GC, {dist_ldr_flw:.0f}m from leader)")
    print(f"  │")

    signal_bus: dict = {}
    faker = SignalFaker(
        cfg=CFG,
        drone_ids=[FOLLOWER_ID],
        get_positions=lambda: {"gc": GC_POS, "leader": leader_pos, FOLLOWER_ID: flw_pos},
        publish=lambda t, p: signal_bus.update({t: p}),
        get_severity=lambda hop: sev[hop],
        clock=lambda: 1_000_000.0,
    )
    faker.tick()   # ← fires here

    hop_map = [
        ("gc_to_leader",       "/signal/gc_to_leader",                          GC_POS,     leader_pos),
        ("leader_to_gc",       "/signal/leader_to_gc",                          leader_pos, GC_POS),
        ("gc_to_follower",     f"/signal/gc_to_follower/{FOLLOWER_ID}",          GC_POS,     flw_pos),
        ("follower_to_gc",     f"/signal/follower_to_gc/{FOLLOWER_ID}",          flw_pos,    GC_POS),
        ("leader_to_follower", f"/signal/leader_to_follower/{FOLLOWER_ID}",      leader_pos, flw_pos),
    ]

    print(f"  │  {'hop':<22} {'dist':>6}  {'sev':>4}  {'rssi':>8}  {'noise':>8}  snr")
    print(f"  │  {'─'*68}")
    for hop_name, topic, pa, pb in hop_map:
        sig = signal_bus.get(topic)
        if sig is None:
            print(f"  │  {hop_name:<22}  MISSING")
            _chk(results, f"[1] {hop_name} published", False, "missing")
            continue
        d = haversine(pa, pb)
        bar = _bar(sig["snr_db"], -20, 60, threshold=MARGINAL)
        cmp = "↓BELOW" if sig["snr_db"] < MARGINAL else "ABOVE"
        print(f"  │  {hop_name:<22} {d:>5.0f}m  {sig['severity']:>4.2f}  "
              f"{sig['rssi_dbm']:>+8.2f}  {sig['noise_dbm']:>+8.2f}  {bar} {sig['snr_db']:+.2f}dB {cmp}")
        _chk(results, f"[1] {hop_name} snr=rssi-noise",
             math.isclose(sig["snr_db"], sig["rssi_dbm"] - sig["noise_dbm"], abs_tol=1e-9),
             f"snr={sig['snr_db']:.4f}")
    _chk(results, "[1] all 5 hops published", len(signal_bus) == 5,
         f"got {len(signal_bus)}")

    # ── STAGE 2a: leader_radio_health_reader ──────────────────────────────────
    _stage("2a", f"Wave 2 — leader_radio_health_reader → /{LEADER_ID}/radio_health")

    ldr_health_bus: dict = {}
    ldr_reader = make_leader_reader(
        LEADER_ID, CFG,
        publish=lambda t, p: ldr_health_bus.update({p["hop"]: p}),
    )
    ldr_reader.on_signal("/signal/gc_to_leader", signal_bus["/signal/gc_to_leader"])
    ldr_reader.publish_tick()   # ← fires here

    lh = ldr_health_bus.get("gc_to_leader")
    if lh:
        sev_bar  = _bar(lh["severity"], 0, 1)
        rng_bar  = _bar(lh["range_m"], 0, CFG["leader_radio_range_m"])
        _row("hop",       lh["hop"])
        _row("severity",  f"{sev_bar} {lh['severity']:.4f}")
        _row("range_m",   f"{rng_bar} {lh['range_m']:.1f} m")
        _snr_row(lh["snr_db"])
        _row("timestamp", f"{lh['timestamp']:.1f}  (origin, §5.6)")
        exp_sev = max(0.0, min(1.0,
            (signal_bus["/signal/gc_to_leader"]["noise_dbm"] - CFG["baseline_noise_dbm"])
            / CFG["noise_range_db"]))
        exp_rng = CFG["leader_radio_range_m"] * (1 - exp_sev * CFG["jamming_severity_factor"])
        _chk(results, "[2a] gc_to_leader severity",
             math.isclose(lh["severity"], exp_sev, abs_tol=1e-9), f"{lh['severity']:.6f}")
        _chk(results, "[2a] gc_to_leader range_m",
             math.isclose(lh["range_m"], exp_rng, abs_tol=1e-6), f"{lh['range_m']:.3f}")
        _chk(results, "[2a] snr passthrough",
             math.isclose(lh["snr_db"],
                          signal_bus["/signal/gc_to_leader"]["snr_db"], abs_tol=1e-9))
    else:
        _chk(results, "[2a] gc_to_leader health published", False, "missing")

    # ── STAGE 2b: follower_radio_health_reader ────────────────────────────────
    _stage("2b", f"Wave 2 — follower_radio_health_reader → /{FOLLOWER_ID}/radio_health")

    flw_health_bus: dict = {}
    flw_reader = make_follower_reader(
        FOLLOWER_ID, CFG,
        publish=lambda t, p: flw_health_bus.update({p["hop"]: p}),
    )
    for topic, hop in [
        (f"/signal/leader_to_follower/{FOLLOWER_ID}", "leader_to_follower"),
        (f"/signal/gc_to_follower/{FOLLOWER_ID}",     "gc_to_follower"),
    ]:
        if topic in signal_bus:
            flw_reader.on_signal(topic, signal_bus[topic])
    flw_reader.publish_tick()   # ← fires here

    for hop_name in ["leader_to_follower", "gc_to_follower"]:
        fh = flw_health_bus.get(hop_name)
        if fh:
            sig_key = f"/signal/{hop_name.replace('gc_to_follower', f'gc_to_follower/{FOLLOWER_ID}').replace('leader_to_follower', f'leader_to_follower/{FOLLOWER_ID}')}"
            src_snr = signal_bus.get(sig_key, {}).get("snr_db", float("nan"))
            sev_bar = _bar(fh["severity"], 0, 1)
            rng_bar = _bar(fh["range_m"], 0, CFG["follower_radio_range_m"])
            print(f"  │  {hop_name}:")
            _row("  severity", f"{sev_bar} {fh['severity']:.4f}")
            _row("  range_m",  f"{rng_bar} {fh['range_m']:.1f} m")
            _snr_row(fh["snr_db"], "  snr_db")
            _chk(results, f"[2b] {hop_name} snr passthrough",
                 math.isclose(fh["snr_db"], src_snr, abs_tol=1e-9) if not math.isnan(src_snr) else False)
        else:
            _chk(results, f"[2b] {hop_name} published", False, "missing")

    # ── STAGE 2c: gc_radio_health_reader ─────────────────────────────────────
    _stage("2c", "Wave 2 — gc_radio_health_reader → /gc/radio_health")

    gc_health_bus: dict = {}
    gc_reader = make_gc_reader(
        [FOLLOWER_ID], CFG,
        publish=lambda t, p: gc_health_bus.update({p["hop"]: p}),
    )
    for topic, key in [
        ("/signal/gc_to_leader",                   "gc_to_leader"),
        (f"/signal/gc_to_follower/{FOLLOWER_ID}",  f"gc_to_follower_{FOLLOWER_ID}"),
    ]:
        if topic in signal_bus:
            gc_reader.on_signal(topic, signal_bus[topic])
    gc_reader.publish_tick()   # ← fires here

    for hop_name in ["gc_to_leader", f"gc_to_follower_{FOLLOWER_ID}"]:
        gh = gc_health_bus.get(hop_name)
        if gh:
            sev_bar = _bar(gh["severity"], 0, 1)
            rng_bar = _bar(gh["range_m"], 0, CFG["gc_radio_range_m"])
            print(f"  │  {hop_name}:")
            _row("  severity", f"{sev_bar} {gh['severity']:.4f}")
            _row("  range_m",  f"{rng_bar} {gh['range_m']:.1f} m")
            _snr_row(gh["snr_db"], "  snr_db")
        else:
            _chk(results, f"[2c] {hop_name} published", False, "missing")

    _chk(results, "[2c] two gc hops published", len(gc_health_bus) == 2,
         f"got {len(gc_health_bus)}: {list(gc_health_bus.keys())}")

    # ── STAGE 3a: gc_link_observer ────────────────────────────────────────────
    _stage("3a", "Wave 3 — gc_link_observer → /gc/gc_link_quality")

    gc_quality_bus: dict = {}
    gc_obs = make_gc_link_observer(
        CFG, publish=lambda t, p: gc_quality_bus.update({t: p}))

    # Feed the gc reader's gc_to_leader output into the observer
    if "gc_to_leader" in gc_health_bus:
        gc_obs.on_radio_health("/gc/radio_health", gc_health_bus["gc_to_leader"])
    gc_obs.publish_tick()   # ← fires here

    gq = gc_quality_bus.get("/gc/gc_link_quality")
    if gq:
        q_bar = _bar(gq["quality"], 0, 1, threshold=0.5)
        _row("quality", f"{q_bar} {gq['quality']:.4f}  (threshold=0.5 ↔ snr={MARGINAL}dB)")
        _snr_row(gq["snr_db"])
        _row("timestamp", f"{gq['timestamp']:.1f}  (origin, §5.6)")
        exp_quality = max(0.0, min(1.0, gq["snr_db"] / (2.0 * MARGINAL)))
        _chk(results, "[3a] gc_link_quality published", True)
        _chk(results, "[3a] quality formula",
             math.isclose(gq["quality"], exp_quality, abs_tol=1e-9),
             f"got {gq['quality']:.4f}  exp {exp_quality:.4f}")
        _chk(results, "[3a] snr passthrough",
             math.isclose(gq["snr_db"],
                          gc_health_bus["gc_to_leader"]["snr_db"], abs_tol=1e-9))
    else:
        _chk(results, "[3a] gc_link_quality published", False, "missing")
        print("  │  no output (gc_to_leader health missing upstream)")

    # ── STAGE 3b: leader_link_detector ────────────────────────────────────────
    _stage("3b", f"Wave 3 — leader_link_detector → /{LEADER_ID}/relay_request | silent")

    relay_requests: list = []
    detector = make_leader_link_detector(
        LEADER_ID, CFG,
        publish=lambda t, p: relay_requests.append((t, p)),
        suppression_topic="/relay_suppression",
    )

    if lh:
        detector.on_radio_health(f"/{LEADER_ID}/radio_health", lh)
    detector.publish_tick()   # ← fires here

    if relay_requests:
        _, rr = relay_requests[0]
        _row("→ DECISION", f"FIRE  (snr {lh['snr_db']:+.2f} dB < {MARGINAL} dB threshold)")
        _row("  drone_id",  rr["drone_id"])
        _row("  snr_db",    f"{rr['snr_db']:+.4f} dB")
        _row("  timestamp", f"{rr['timestamp']:.1f}")
        _row("  topic",     relay_requests[0][0])
        _chk(results, "[3b] relay_request fired",
             lh["snr_db"] < MARGINAL,
             f"snr={lh['snr_db']:.2f}")
        _chk(results, "[3b] §2.3 schema",
             set(rr.keys()) == {"drone_id", "snr_db", "timestamp"})
        _chk(results, "[3b] snr_db echoes pipeline value",
             math.isclose(rr["snr_db"], lh["snr_db"], abs_tol=1e-9))
    else:
        _row("→ DECISION", f"SILENT  (snr {lh['snr_db'] if lh else 'N/A':+.2f} dB ≥ {MARGINAL} dB)")
        _chk(results, "[3b] no relay_request (link healthy)",
             lh is not None and lh["snr_db"] >= MARGINAL)

    # ── STAGE 4a: Blackboard ──────────────────────────────────────────────────
    _stage("4a", "Wave 4 — TimestampedBlackboard — write, age, freshness")

    clock = FakeClock(t=0.0)
    bb = TimestampedBlackboard(clock=clock.now)

    # Write health data as capability_assessor would after relay_assignment
    print(f"  │  t=0.0s: write leader radio_health and follower radio_health")
    if lh:
        bb.set("leader_radio_health", lh)
    flw_ldr_h = flw_health_bus.get("leader_to_follower")
    if flw_ldr_h:
        bb.set("follower_radio_health_leader_hop", flw_ldr_h)

    if lh:
        v, fresh, age = bb.get_with_freshness("leader_radio_health", 30.0)
        _row("  leader_radio_health (t=0)", f"age={age:.1f}s  fresh(max=30s)={fresh}  snr={v['snr_db']:+.2f}dB")
        _chk(results, "[4a] freshly written is fresh", fresh is True)

    print(f"  │  t=5.0s: advance clock +5s")
    clock.advance(5.0)
    if lh:
        v, fresh, age = bb.get_with_freshness("leader_radio_health", 30.0)
        _row("  leader_radio_health (t=5)", f"age={age:.1f}s  fresh(max=30s)={fresh}")
        _chk(results, "[4a] 5s within 30s window is fresh", fresh is True)

    print(f"  │  t=35.0s: advance clock +30s more (past max_age=30s)")
    clock.advance(30.0)
    if lh:
        v, fresh, age = bb.get_with_freshness("leader_radio_health", 30.0)
        stale_bar = _bar(min(age, 50), 0, 50, threshold=30)
        _row("  leader_radio_health (t=35)", f"age={age:.1f}s  fresh(max=30s)={fresh}  {stale_bar}")
        _row("  value still readable?", f"{'YES  snr=' + str(round(v['snr_db'],2)) if v else 'NO'}")
        _chk(results, "[4a] 35s past 30s window is stale", fresh is False)
        _chk(results, "[4a] stale value still readable", v is not None and "snr_db" in v)

    print(f"  │  never-written key test:")
    val, fresh, age = bb.get_with_freshness("ghost_key", 10.0)
    _row("  ghost_key → (val, fresh, age)", f"({val!r}, {fresh!r}, {age!r})")
    _chk(results, "[4a] never-written → (None, False, None)",
         val is None and fresh is False and age is None,
         f"got ({val!r}, {fresh!r}, {age!r})")

    # ── STAGE 4b: Geometry ────────────────────────────────────────────────────
    _stage("4b", "Wave 4 — Geometry — relay band + return margin")

    r_G = flw_health_bus.get("gc_to_follower", {}).get("range_m", CFG["follower_radio_range_m"])
    r_L = flw_health_bus.get("leader_to_follower", {}).get("range_m", CFG["follower_radio_range_m"])
    D   = geo_haversine(GC_POS, leader_pos)

    t_lo, t_hi = band_bounds(D, r_G, r_L)
    feasible = t_hi >= t_lo

    _row("D  GC↔Leader",  f"{D:.0f} m")
    _row("r_G  (→GC)",    f"{r_G:.1f} m  (follower gc_to_follower range_m)")
    _row("r_L  (→Leader)",f"{r_L:.1f} m  (follower leader_to_follower range_m)")
    _row("r_G + r_L",     f"{r_G + r_L:.1f} m  {'≥' if r_G + r_L >= D else '<'} D  → {'FEASIBLE' if feasible else 'INFEASIBLE'}")

    if feasible:
        relay_pos = band_center(t_lo, t_hi, GC_POS, leader_pos)
        band_bar = _bar((t_lo + t_hi) / 2, -1, 2, w=20)
        _row("band  t_lo / t_hi", f"{t_lo:.3f} / {t_hi:.3f}  (fractions along GC→Leader line)")
        _row("relay position",    f"lat={relay_pos['lat']:.5f}  lon={relay_pos['lon']:.5f}")
        _chk(results, "[4b] band feasible (t_hi >= t_lo)", True)
    else:
        _row("band  t_lo / t_hi", f"{t_lo:.3f} / {t_hi:.3f}  (INVERTED → follower cannot bridge gap)")
        _row("relay position",    "— not computable (infeasible) —")
        _chk(results, "[4b] band infeasible confirmed (r_G+r_L < D)",
             r_G + r_L < D, f"{r_G+r_L:.0f} < {D:.0f}")

    # Battery cost: follower return to GC
    dist_home = geo_haversine(flw_pos, GC_POS)
    print(f"  │")
    print(f"  │  Battery return-to-home check (fake model: speed=10m/s, consumption=0.01%/s)")
    _row("  follower → GC distance", f"{dist_home:.0f} m")

    for batt_pct in sc.get("battery_tests", [50.0, 5.0]):
        ok, req, cost = estimate_battery_cost(
            flw_pos, GC_POS, batt_pct, _FAKE_MODEL, CFG)
        verdict = "PASSES ✓" if ok else "FAILS  ✗"
        _row(f"  battery={batt_pct:.1f}%",
             f"cost={cost:.3f}%  required={req:.3f}%  → {verdict}")
        _chk(results,
             f"[4b] battery {batt_pct:.0f}% return margin {'ok' if ok else 'fail'}",
             ok == sc.get("expect_margin_ok", {}).get(batt_pct, ok))

    # §7.1 sentinel demonstration (only shown in scenario 5)
    if sc.get("show_sentinel"):
        print(f"  │")
        print(f"  │  §7.1 sentinel — _Unresolved raises on arithmetic:")
        sys.path.insert(0, os.path.join(_DC))
        from config.demo_config import DRONE_MODELS
        bad_model = DRONE_MODELS["generic"]
        raised = False
        try:
            estimate_battery_cost(flw_pos, GC_POS, 50.0, bad_model, CFG)
        except RuntimeError as e:
            raised = True
            _row("  RuntimeError raised", f"✓  [{str(e)[:60]}…]")
        _chk(results, "[4b] §7.1 sentinel raises RuntimeError", raised)

    return results


# ── Scenarios ─────────────────────────────────────────────────────────────────

SCENARIOS = [
    {
        "name":        "1 — All clean: close range, zero severity everywhere",
        "leader_pos":  {"lat": 47.3904, "lon": 8.5405, "alt": 50.0},   # ~58m
        "follower_pos":{"lat": 47.3902, "lon": 8.5402, "alt": 30.0},   # ~28m from GC
        "severities":  {k: 0.0 for k in ["gc_to_leader","leader_to_gc",
                        "gc_to_follower","follower_to_gc","leader_to_follower"]},
        "battery_tests": [50.0, 5.0],
        "expect_margin_ok": {50.0: True, 5.0: False},   # 10% flat buffer makes 5% fail always
    },
    {
        "name":        "2 — Leader far: 6 km GC↔Leader, zero severity",
        "leader_pos":  {"lat": 47.4350, "lon": 8.5850, "alt": 100.0},  # ~6 km
        "follower_pos":{"lat": 47.4050, "lon": 8.5600, "alt": 60.0},   # ~1.9 km from GC
        "severities":  {k: 0.0 for k in ["gc_to_leader","leader_to_gc",
                        "gc_to_follower","follower_to_gc","leader_to_follower"]},
        "battery_tests": [50.0, 3.0],
        "expect_margin_ok": {50.0: True, 3.0: False},
    },
    {
        "name":        "3 — GC-leader jammed: close range, gc_to_leader sev=0.90",
        "leader_pos":  {"lat": 47.3904, "lon": 8.5405, "alt": 50.0},
        "follower_pos":{"lat": 47.3902, "lon": 8.5402, "alt": 30.0},
        "severities":  {"gc_to_leader": 0.90, "leader_to_gc": 0.90,
                        "gc_to_follower": 0.0, "follower_to_gc": 0.0,
                        "leader_to_follower": 0.0},
        "battery_tests": [50.0, 5.0],
        "expect_margin_ok": {50.0: True, 5.0: False},   # 10% flat buffer makes 5% fail always
    },
    {
        "name":        "4 — Per-hop mix: leader 2.5 km, gc_to_leader sev=0.80 follower sev=0.10",
        "leader_pos":  {"lat": 47.4100, "lon": 8.5620, "alt": 75.0},   # ~2.5 km
        "follower_pos":{"lat": 47.4000, "lon": 8.5500, "alt": 50.0},   # ~1.4 km from GC
        "severities":  {"gc_to_leader": 0.80, "leader_to_gc": 0.80,
                        "gc_to_follower": 0.10, "follower_to_gc": 0.10,
                        "leader_to_follower": 0.10},
        "battery_tests": [30.0, 8.0],
        "expect_margin_ok": {30.0: True, 8.0: False},
    },
    {
        "name":        "5 — Geometry focus: leader 2 km, follower 1 km, band + §7.1",
        "leader_pos":  {"lat": 47.4080, "lon": 8.5580, "alt": 70.0},   # ~2 km
        "follower_pos":{"lat": 47.3990, "lon": 8.5490, "alt": 50.0},   # ~1 km from GC
        "severities":  {k: 0.0 for k in ["gc_to_leader","leader_to_gc",
                        "gc_to_follower","follower_to_gc","leader_to_follower"]},
        "battery_tests": [15.0, 2.0],
        "expect_margin_ok": {15.0: True, 2.0: False},
        "show_sentinel": True,
    },
]


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    total_pass = total_fail = 0

    for sc in SCENARIOS:
        results = run_scenario(sc)

        n_pass = sum(1 for _, ok, _ in results if ok)
        n_fail = sum(1 for _, ok, _ in results if not ok)
        total_pass += n_pass
        total_fail += n_fail

        print()
        _div()
        print(f"  Checks: {n_pass} passed  {n_fail} failed  ({len(results)} total)")
        for label, ok, detail in results:
            icon = "✓" if ok else "✗"
            line = f"  {icon}  {label}"
            if detail:
                line += f"  [{detail}]"
            print(line)

    print(f"\n{'═'*W}")
    print(f"  TOTAL: {total_pass} passed, {total_fail} failed across {len(SCENARIOS)} scenarios")
    if total_fail > 0:
        print("  INTEGRATION TEST FAILED")
        sys.exit(1)
    else:
        print("  INTEGRATION TEST PASSED")


if __name__ == "__main__":
    main()
