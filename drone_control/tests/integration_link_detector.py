#!/usr/bin/env python3
"""
integration_link_detector.py — live-output pipeline integration test.

Wires signal_faker → leader_radio_health_reader → leader_link_detector
in-process (no ROS2).  Each stage fires and prints its own live output
before the next stage runs.

Pipeline:
  [1] SignalFaker.tick()
      → /signal/gc_to_leader  (§2.1)
  [2] leader_radio_health_reader.on_signal() + .publish_tick()
      → /{leader_id}/radio_health  (§2.2, hop="gc_to_leader")
  [3] leader_link_detector.on_radio_health() + .publish_tick()
      → /{leader_id}/relay_request  (§2.3)  — or silent

Scenarios:
  A — close range, clean:            SNR >> 13  → no relay_request
  B — long range, clean:             SNR <  13  → relay_request published
  C — close range, heavy jamming:    SNR <  13  → relay_request published
  D — bad link + suppression active: SNR <  13, but suppressed → no relay_request
  E — bad link, 3 ticks:             3 relay_requests (no sender-side dedup)

Run:
  python3 tests/integration_link_detector.py
"""

import sys
import os
import math

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
_DC       = os.path.join(_PKG_ROOT, "drone_control")
for p in (_PKG_ROOT, _DC):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.signal_faker               import SignalFaker, haversine
from drone_control.leader_radio_health_reader import make_leader_reader
from drone_control.leader_link_detector       import make_leader_link_detector

# ── Config ────────────────────────────────────────────────────────────────────

CFG = {
    "baseline_noise_dbm":     -95.0,
    "noise_range_db":          40.0,
    "tx_power_dbm":            20.0,
    "frequency_mhz":           915,
    "jamming_severity_factor":  0.6,
    "leader_radio_range_m":   800.0,
    "LINK_MARGINAL_QUALITY":   13,
}

LEADER_ID = "drone-01"
DRONE_ID  = "drone-02"
GC_POS    = {"lat": 47.3900, "lon": 8.5400, "alt": 0.0}

MARGINAL  = CFG["LINK_MARGINAL_QUALITY"]

# ── Display helpers ───────────────────────────────────────────────────────────

_W = 72   # column width

def _bar(v, lo, hi, w=28, threshold=None):
    """Filled bar from lo to hi.  Optional threshold marker."""
    span   = max(hi - lo, 1e-9)
    filled = int(round(max(0, min(w, (v - lo) / span * w))))
    bar    = "█" * filled + "·" * (w - filled)
    tmark  = ""
    if threshold is not None:
        tp = int(round(max(0, min(w, (threshold - lo) / span * w))))
        bar_list = list(bar)
        if 0 <= tp < w:
            bar_list[tp] = "▼"
        bar = "".join(bar_list)
    return f"[{bar}]"

def _snr_line(snr_db):
    bar = _bar(snr_db, -20, 50, threshold=MARGINAL)
    cmp = "<  BELOW threshold" if snr_db < MARGINAL else ">= ABOVE threshold"
    return f"  snr_db   {bar} {snr_db:+7.2f} dB   ({cmp} {MARGINAL} dB)"

def _check(results, label, cond, detail=""):
    results.append((label, bool(cond), detail))

def _divider(char="─"): print("  " + char * (_W - 2))

def _hdr(text): print(f"\n  ╔{'═'*(_W-4)}╗"); print(f"  ║  {text:<{_W-6}}║"); print(f"  ╚{'═'*(_W-4)}╝")

def _stage(n, text): print(f"\n  ┌─ STAGE {n}: {text}")

def _row(label, value): print(f"  │  {label:<22} {value}")

# ── Per-scenario runner ───────────────────────────────────────────────────────

def run_scenario(sc):
    results = []
    name       = sc["name"]
    leader_pos = sc["leader_pos"]
    severity   = sc["severity"]
    suppressed = sc["suppressed"]
    ticks      = sc["ticks"]

    _hdr(name)

    dist_m = haversine(GC_POS, leader_pos)
    _row("GC position", f"lat={GC_POS['lat']:.4f}  lon={GC_POS['lon']:.4f}  alt={GC_POS['alt']:.0f} m")
    _row("Leader position", f"lat={leader_pos['lat']:.4f}  lon={leader_pos['lon']:.4f}  alt={leader_pos['alt']:.0f} m")
    _row("GC ↔ Leader distance", f"{dist_m:.0f} m")
    _row("gc_to_leader severity", f"{severity:.2f}  (0=clean, 1=worst jamming)")
    _row("Follower suppression", "ON" if suppressed else "OFF")
    _row("Publish ticks", str(ticks))

    # ── STAGE 1: signal_faker ─────────────────────────────────────────────────

    _stage(1, "signal_faker.tick()  →  /signal/gc_to_leader")

    signal_bus: dict = {}
    all_sev = {
        "gc_to_leader":       severity,
        "leader_to_gc":       0.0,
        "gc_to_follower":     0.0,
        "follower_to_gc":     0.0,
        "leader_to_follower": 0.0,
    }
    positions = {"gc": GC_POS, "leader": leader_pos, DRONE_ID: GC_POS}

    faker = SignalFaker(
        cfg=CFG,
        drone_ids=[DRONE_ID],
        get_positions=lambda: dict(positions),
        publish=lambda t, p: signal_bus.update({t: p}),
        get_severity=lambda hop: all_sev[hop],
        clock=lambda: 1_000_000.0,
    )
    faker.tick()   # ← fires here; signal_bus populated immediately

    sig = signal_bus.get("/signal/gc_to_leader")
    _check(results, "signal/gc_to_leader present", sig is not None,
           f"bus={list(signal_bus.keys())}")

    if sig is None:
        print("  │  ERROR: /signal/gc_to_leader not published")
        return results

    # Print live output from this stage
    pl = _bar(sig["rssi_dbm"], -110, 0)
    _row("rssi_dbm", f"{pl} {sig['rssi_dbm']:+7.2f} dBm")
    nl = _bar(sig["noise_dbm"], -110, -55)
    _row("noise_dbm", f"{nl} {sig['noise_dbm']:+7.2f} dBm")
    print(_snr_line(sig["snr_db"]))
    _row("severity (input)", f"{sig['severity']:.4f}")
    _row("timestamp", f"{sig['timestamp']:.1f}")

    _check(results, "rssi = tx_power - FSPL",
           math.isclose(sig["rssi_dbm"],
                        CFG["tx_power_dbm"] - (20*math.log10(max(dist_m,1))
                                               + 20*math.log10(CFG["frequency_mhz"])
                                               - 27.55), abs_tol=1e-6),
           f"got {sig['rssi_dbm']:.4f}")
    _check(results, "noise = baseline + sev×range",
           math.isclose(sig["noise_dbm"],
                        CFG["baseline_noise_dbm"] + severity * CFG["noise_range_db"],
                        abs_tol=1e-6),
           f"got {sig['noise_dbm']:.4f}")
    _check(results, "snr = rssi - noise",
           math.isclose(sig["snr_db"], sig["rssi_dbm"] - sig["noise_dbm"], abs_tol=1e-9),
           f"got {sig['snr_db']:.4f}")

    # ── STAGE 2: leader_radio_health_reader ───────────────────────────────────

    _stage(2, "leader_radio_health_reader  →  /drone-01/radio_health")

    health_bus: dict = {}
    reader = make_leader_reader(
        LEADER_ID, CFG,
        publish=lambda t, p: health_bus.update({(t, p["hop"]): p}),
    )

    reader.on_signal("/signal/gc_to_leader", sig)   # feed stage-1 output
    reader.publish_tick()                           # ← fires here

    hkey = (f"/{LEADER_ID}/radio_health", "gc_to_leader")
    h = health_bus.get(hkey)
    _check(results, "radio_health gc_to_leader published", h is not None,
           f"bus={list(health_bus.keys())}")

    if h is None:
        print("  │  ERROR: radio_health not published")
        return results

    # Print live output from this stage
    sl = _bar(h["severity"], 0, 1)
    _row("severity (derived)", f"{sl} {h['severity']:.4f}")
    rl = _bar(h["range_m"], 0, CFG["leader_radio_range_m"])
    _row("range_m", f"{rl} {h['range_m']:.1f} m")
    print(_snr_line(h["snr_db"]))
    _row("hop", h["hop"])
    _row("timestamp (origin)", f"{h['timestamp']:.1f}")

    exp_sev = max(0.0, min(1.0,
        (sig["noise_dbm"] - CFG["baseline_noise_dbm"]) / CFG["noise_range_db"]))
    exp_rng = CFG["leader_radio_range_m"] * (1.0 - exp_sev * CFG["jamming_severity_factor"])

    _check(results, "health severity correct",
           math.isclose(h["severity"], exp_sev, abs_tol=1e-9),
           f"got {h['severity']:.6f}  exp {exp_sev:.6f}")
    _check(results, "health range_m correct",
           math.isclose(h["range_m"], exp_rng, abs_tol=1e-6),
           f"got {h['range_m']:.3f}  exp {exp_rng:.3f}")
    _check(results, "snr_db passthrough (Q16)",
           math.isclose(h["snr_db"], sig["snr_db"], abs_tol=1e-9),
           f"got {h['snr_db']:.4f}  stage1 {sig['snr_db']:.4f}")
    _check(results, "timestamp = origin (§5.6)",
           h["timestamp"] == sig["timestamp"],
           f"got {h['timestamp']}")
    _check(results, "hop field = gc_to_leader",
           h["hop"] == "gc_to_leader",
           f"got {h['hop']!r}")

    # ── STAGE 3: leader_link_detector ─────────────────────────────────────────

    _stage(3, "leader_link_detector  →  /drone-01/relay_request  (or silent)")

    relay_requests: list = []
    detector = make_leader_link_detector(
        LEADER_ID, CFG,
        publish=lambda t, p: relay_requests.append((t, p)),
        suppression_topic="/relay_suppression",
    )

    if suppressed:
        detector.on_suppression("/relay_suppression", {"active": True})
        _row("suppression injected", "active=True")

    detector.on_radio_health(f"/{LEADER_ID}/radio_health", h)  # feed stage-2 output

    for tick_n in range(1, ticks + 1):
        detector.publish_tick()   # ← fires here; relay_requests appended if condition holds
        if relay_requests and len(relay_requests) == tick_n:
            _, rr = relay_requests[-1]
            _row(f"  tick {tick_n} → relay_request",
                 f"snr_db={rr['snr_db']:.2f}  drone_id={rr['drone_id']}")
        else:
            _row(f"  tick {tick_n} → relay_request", "silent")

    below = h["snr_db"] < MARGINAL

    if suppressed:
        _row("DECISION", f"SNR {h['snr_db']:+.2f} < {MARGINAL} dB → True, BUT suppressed → SILENT")
        _check(results, "suppressed: zero relay_requests",
               len(relay_requests) == 0,
               f"got {len(relay_requests)}")
    elif below:
        _row("DECISION", f"SNR {h['snr_db']:+.2f} dB < {MARGINAL} dB → FIRE ({len(relay_requests)}/{ticks} ticks)")
        _check(results, f"relay_request published ({ticks} tick(s))",
               len(relay_requests) == ticks,
               f"got {len(relay_requests)}  exp {ticks}")
        if relay_requests:
            _, rr = relay_requests[0]
            _check(results, "§2.3 schema: exactly {drone_id, snr_db, timestamp}",
                   set(rr.keys()) == {"drone_id", "snr_db", "timestamp"},
                   f"got keys={set(rr.keys())}")
            _check(results, "§2.3 drone_id matches leader",
                   rr["drone_id"] == LEADER_ID,
                   f"got {rr['drone_id']!r}")
            _check(results, "§2.3 snr_db matches pipeline value",
                   math.isclose(rr["snr_db"], h["snr_db"], abs_tol=1e-9),
                   f"rr {rr['snr_db']:.4f}  stage2 {h['snr_db']:.4f}")
            _check(results, "§2.3 timestamp present",
                   isinstance(rr.get("timestamp"), float),
                   f"got {rr.get('timestamp')!r}")
            _check(results, "relay_request on correct topic",
                   relay_requests[0][0] == f"/{LEADER_ID}/relay_request",
                   f"got {relay_requests[0][0]!r}")
    else:
        _row("DECISION", f"SNR {h['snr_db']:+.2f} dB ≥ {MARGINAL} dB → SILENT")
        _check(results, "no relay_request (SNR above threshold)",
               len(relay_requests) == 0,
               f"got {len(relay_requests)}")

    return results


# ── Scenarios ─────────────────────────────────────────────────────────────────

SCENARIOS = [
    {
        "name":       "A — close range (50 m), severity 0.0 — clean link",
        "leader_pos": {"lat": 47.3904, "lon": 8.5405, "alt": 50.0},
        "severity":   0.0,
        "suppressed": False,
        "ticks":      1,
    },
    {
        "name":       "B — long range (≈5 km), severity 0.0 — distance degrades link",
        "leader_pos": {"lat": 47.4350, "lon": 8.5850, "alt": 100.0},
        "severity":   0.0,
        "suppressed": False,
        "ticks":      1,
    },
    {
        "name":       "C — close range (50 m), severity 0.95 — heavy jamming",
        "leader_pos": {"lat": 47.3904, "lon": 8.5405, "alt": 50.0},
        "severity":   0.95,
        "suppressed": False,
        "ticks":      1,
    },
    {
        "name":       "D — long range (bad link) + follower suppression active",
        "leader_pos": {"lat": 47.4350, "lon": 8.5850, "alt": 100.0},
        "severity":   0.0,
        "suppressed": True,
        "ticks":      1,
    },
    {
        "name":       "E — bad link, 3 consecutive ticks — no sender-side dedup",
        "leader_pos": {"lat": 47.4350, "lon": 8.5850, "alt": 100.0},
        "severity":   0.0,
        "suppressed": False,
        "ticks":      3,
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
        _divider()
        print(f"  Checks:  {n_pass} passed  {n_fail} failed  ({len(results)} total)")
        for label, ok, detail in results:
            icon = "✓" if ok else "✗"
            line = f"  {icon}  {label}"
            if detail:
                line += f"  [{detail}]"
            print(line)

    print(f"\n{'═'*_W}")
    print(f"  TOTAL: {total_pass} passed, {total_fail} failed")
    if total_fail > 0:
        print("  INTEGRATION TEST FAILED")
        sys.exit(1)
    else:
        print("  INTEGRATION TEST PASSED")


if __name__ == "__main__":
    main()
