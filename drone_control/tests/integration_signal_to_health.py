#!/usr/bin/env python3
"""
integration_signal_to_health.py — pipeline smoke test.

Wires signal_faker → radio_health_readers in-process with no ROS2.
Vary GPS positions and per-hop severities, confirm the radio health
values coming out the other side are mathematically consistent.

Run:
  python3 tests/integration_signal_to_health.py
"""

import sys
import os
import math

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
_DC       = os.path.join(_PKG_ROOT, "drone_control")
for p in (_PKG_ROOT, _DC):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.signal_faker import SignalFaker, haversine
from drone_control.follower_radio_health_reader import make_follower_reader
from drone_control.leader_radio_health_reader   import make_leader_reader
from drone_control.gc_radio_health_reader       import make_gc_reader

# ── Config (pulled from demo_config, or inline here for explicitness) ─────────

CFG = {
    "baseline_noise_dbm":    -95.0,
    "noise_range_db":         40.0,
    "tx_power_dbm":           20.0,
    "frequency_mhz":          915,
    "jamming_severity_factor": 0.6,
    "follower_radio_range_m": 800.0,
    "leader_radio_range_m":   800.0,
    "gc_radio_range_m":       800.0,
}

DRONE_ID = "drone-02"

# ── Scenario table ─────────────────────────────────────────────────────────────

SCENARIOS = [
    {
        "name": "clean — short range",
        "gc_pos":       {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},
        "leader_pos":   {"lat": 47.3920, "lon": 8.5420, "alt": 50.0},   # ~260 m
        "follower_pos": {"lat": 47.3910, "lon": 8.5410, "alt": 30.0},   # ~130 m from GC
        "severities": {k: 0.0 for k in
                       ["gc_to_leader","leader_to_gc","gc_to_follower",
                        "follower_to_gc","leader_to_follower"]},
    },
    {
        "name": "moderate jamming (sev=0.5) — medium range",
        "gc_pos":       {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},
        "leader_pos":   {"lat": 47.4100, "lon": 8.5600, "alt": 50.0},   # ~2800 m
        "follower_pos": {"lat": 47.4000, "lon": 8.5500, "alt": 40.0},   # ~1400 m from GC
        "severities": {k: 0.5 for k in
                       ["gc_to_leader","leader_to_gc","gc_to_follower",
                        "follower_to_gc","leader_to_follower"]},
    },
    {
        "name": "per-hop mix — leader link stressed, follower clean",
        "gc_pos":       {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},
        "leader_pos":   {"lat": 47.4100, "lon": 8.5600, "alt": 50.0},
        "follower_pos": {"lat": 47.4000, "lon": 8.5500, "alt": 40.0},
        "severities": {
            "gc_to_leader":       0.8,
            "leader_to_gc":       0.8,
            "gc_to_follower":     0.1,
            "follower_to_gc":     0.1,
            "leader_to_follower": 0.1,
        },
    },
    {
        "name": "long range — approaching radio limit",
        "gc_pos":       {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},
        "leader_pos":   {"lat": 47.4700, "lon": 8.6200, "alt": 100.0},  # ~10 km
        "follower_pos": {"lat": 47.4300, "lon": 8.5800, "alt": 80.0},
        "severities": {k: 0.0 for k in
                       ["gc_to_leader","leader_to_gc","gc_to_follower",
                        "follower_to_gc","leader_to_follower"]},
    },
]

# ── Reference formulas (same as signal_faker.py and _radio_health_core.py) ────

def ref_noise_dbm(severity: float) -> float:
    return CFG["baseline_noise_dbm"] + severity * CFG["noise_range_db"]

def ref_path_loss(dist_m: float) -> float:
    return 20*math.log10(max(dist_m,1)) + 20*math.log10(CFG["frequency_mhz"]) - 27.55

def ref_rssi(dist_m: float) -> float:
    return CFG["tx_power_dbm"] - ref_path_loss(dist_m)

def ref_snr(dist_m: float, severity: float) -> float:
    return ref_rssi(dist_m) - ref_noise_dbm(severity)

def ref_severity_from_noise(noise_dbm: float) -> float:
    return max(0.0, min(1.0,
        (noise_dbm - CFG["baseline_noise_dbm"]) / CFG["noise_range_db"]))

def ref_range_m(nominal: float, severity: float) -> float:
    return nominal * (1.0 - severity * CFG["jamming_severity_factor"])

# ── Run one scenario ──────────────────────────────────────────────────────────

def run_scenario(sc: dict) -> list:
    """Returns list of (check_name, passed, detail) tuples."""
    results = []
    def check(name, cond, detail=""):
        results.append((name, cond, detail))

    positions = {
        "gc":     sc["gc_pos"],
        "leader": sc["leader_pos"],
        DRONE_ID: sc["follower_pos"],
    }
    severities = sc["severities"]

    # ── 1. Run signal_faker tick ──────────────────────────────────────────────
    signal_bus: dict = {}   # topic → payload (last write wins)

    faker = SignalFaker(
        cfg=CFG,
        drone_ids=[DRONE_ID],
        get_positions=lambda: dict(positions),
        publish=lambda t, p: signal_bus.update({t: p}),
        get_severity=lambda hop: severities[hop],
        clock=lambda: 7777.0,
    )
    faker.tick()

    # Expect exactly 5 topics published
    check("5 signal topics published", len(signal_bus) == 5,
          f"got {list(signal_bus.keys())}")

    # ── 2. Verify signal_faker math against reference formulas ────────────────
    for hop_name, (pos_a_key, pos_b_key) in [
        ("gc_to_leader",           ("gc",     "leader")),
        ("leader_to_gc",           ("leader", "gc")),
        ("gc_to_follower",         ("gc",     DRONE_ID)),
        ("follower_to_gc",         (DRONE_ID, "gc")),
        ("leader_to_follower",     ("leader", DRONE_ID)),
    ]:
        topic = {
            "gc_to_leader":       "/signal/gc_to_leader",
            "leader_to_gc":       "/signal/leader_to_gc",
            "gc_to_follower":     f"/signal/gc_to_follower/{DRONE_ID}",
            "follower_to_gc":     f"/signal/follower_to_gc/{DRONE_ID}",
            "leader_to_follower": f"/signal/leader_to_follower/{DRONE_ID}",
        }[hop_name]

        sig = signal_bus.get(topic)
        if sig is None:
            check(f"{hop_name} topic present", False, "missing from bus")
            continue

        dist_m = haversine(positions[pos_a_key], positions[pos_b_key])
        sev    = severities[hop_name]

        exp_rssi  = ref_rssi(dist_m)
        exp_noise = ref_noise_dbm(sev)
        exp_snr   = exp_rssi - exp_noise

        tol = 1e-6
        check(f"{hop_name} rssi_dbm",  math.isclose(sig["rssi_dbm"],  exp_rssi,  abs_tol=tol),
              f"got {sig['rssi_dbm']:.4f}  expected {exp_rssi:.4f}")
        check(f"{hop_name} noise_dbm", math.isclose(sig["noise_dbm"], exp_noise, abs_tol=tol),
              f"got {sig['noise_dbm']:.4f}  expected {exp_noise:.4f}")
        check(f"{hop_name} snr_db",    math.isclose(sig["snr_db"],    exp_snr,   abs_tol=tol),
              f"got {sig['snr_db']:.4f}  expected {exp_snr:.4f}")
        check(f"{hop_name} severity",  math.isclose(sig["severity"],  sev,       abs_tol=tol),
              f"got {sig['severity']}  expected {sev}")
        check(f"{hop_name} timestamp", sig["timestamp"] == 7777.0,
              f"got {sig['timestamp']}")

    # ── 3. Feed signal bus into radio health readers ──────────────────────────
    health_bus: dict = {}

    follower_reader = make_follower_reader(
        DRONE_ID, CFG, publish=lambda t, p: health_bus.update({(t, p['hop']): p}))
    leader_reader   = make_leader_reader(
        "drone-01", CFG, publish=lambda t, p: health_bus.update({(t, p['hop']): p}))
    gc_reader       = make_gc_reader(
        [DRONE_ID], CFG, publish=lambda t, p: health_bus.update({(t, p['hop']): p}))

    # Feed each reader the signal(s) it subscribes to
    reader_feeds = {
        follower_reader: [
            (f"/signal/leader_to_follower/{DRONE_ID}", "leader_to_follower"),
            (f"/signal/gc_to_follower/{DRONE_ID}",     "gc_to_follower"),
        ],
        leader_reader: [
            ("/signal/gc_to_leader", "gc_to_leader"),
        ],
        gc_reader: [
            ("/signal/gc_to_leader",                   "gc_to_leader"),
            (f"/signal/gc_to_follower/{DRONE_ID}",     f"gc_to_follower_{DRONE_ID}"),
        ],
    }

    for reader, feeds in reader_feeds.items():
        for topic, hop_name in feeds:
            if topic in signal_bus:
                reader.on_signal(topic, signal_bus[topic])
        reader.publish_tick()

    # ── 4. Verify radio health values against reference formulas ──────────────
    # Check follower reader: leader_to_follower and gc_to_follower hops
    for hop_name in ["leader_to_follower", "gc_to_follower"]:
        key = (f"/{DRONE_ID}/radio_health", hop_name)
        h = health_bus.get(key)
        if h is None:
            check(f"follower health {hop_name} published", False, "not in bus")
            continue
        sev = severities[hop_name]
        exp_sev = ref_severity_from_noise(ref_noise_dbm(sev))
        exp_rng = ref_range_m(CFG["follower_radio_range_m"], exp_sev)

        check(f"follower/{hop_name} severity",
              math.isclose(h["severity"], exp_sev, abs_tol=1e-9),
              f"got {h['severity']:.6f}  expected {exp_sev:.6f}")
        check(f"follower/{hop_name} range_m",
              math.isclose(h["range_m"], exp_rng, abs_tol=1e-6),
              f"got {h['range_m']:.3f}  expected {exp_rng:.3f}")
        check(f"follower/{hop_name} snr passthrough",
              math.isclose(h["snr_db"],
                           signal_bus[f"/signal/{hop_name.replace('_', '_', 1)}"
                                      .replace('gc_to_follower', f'gc_to_follower/{DRONE_ID}')
                                      .replace('leader_to_follower', f'leader_to_follower/{DRONE_ID}')
                                      ]["snr_db"],
                           abs_tol=1e-9) if (
                    f"/signal/{hop_name.replace('gc_to_follower', f'gc_to_follower/{DRONE_ID}').replace('leader_to_follower', f'leader_to_follower/{DRONE_ID}')}"
                    in signal_bus
               ) else True,
              f"snr={h['snr_db']:.3f}")
        check(f"follower/{hop_name} timestamp passthrough",
              h["timestamp"] == 7777.0,
              f"got {h['timestamp']}")

    # Check leader reader: gc_to_leader hop
    key = ("/drone-01/radio_health", "gc_to_leader")
    h = health_bus.get(key)
    if h is None:
        check("leader/gc_to_leader health published", False, "not in bus")
    else:
        sev = severities["gc_to_leader"]
        exp_sev = ref_severity_from_noise(ref_noise_dbm(sev))
        exp_rng = ref_range_m(CFG["leader_radio_range_m"], exp_sev)
        check("leader/gc_to_leader severity",
              math.isclose(h["severity"], exp_sev, abs_tol=1e-9),
              f"got {h['severity']:.6f}  expected {exp_sev:.6f}")
        check("leader/gc_to_leader range_m",
              math.isclose(h["range_m"], exp_rng, abs_tol=1e-6),
              f"got {h['range_m']:.3f}  expected {exp_rng:.3f}")
        check("leader/gc_to_leader timestamp",
              h["timestamp"] == 7777.0, f"got {h['timestamp']}")

    return results


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    total_pass = total_fail = 0
    for sc in SCENARIOS:
        print(f"\n{'='*70}")
        print(f"SCENARIO: {sc['name']}")
        gc  = sc["gc_pos"]
        ldr = sc["leader_pos"]
        flw = sc["follower_pos"]
        print(f"  GC       → lat={gc['lat']:.4f}  lon={gc['lon']:.4f}  alt={gc['alt']:.0f}m")
        print(f"  Leader   → lat={ldr['lat']:.4f}  lon={ldr['lon']:.4f}  alt={ldr['alt']:.0f}m")
        print(f"  Follower → lat={flw['lat']:.4f}  lon={flw['lon']:.4f}  alt={flw['alt']:.0f}m")
        dist_gc_ldr = haversine(gc, ldr)
        dist_gc_flw = haversine(gc, flw)
        print(f"  Distances: GC↔Leader={dist_gc_ldr:.0f}m  GC↔Follower={dist_gc_flw:.0f}m")
        print(f"  Severities: {sc['severities']}")
        print()

        results = run_scenario(sc)
        n_pass = sum(1 for _, ok, _ in results if ok)
        n_fail = sum(1 for _, ok, _ in results if not ok)
        total_pass += n_pass
        total_fail += n_fail

        for name, ok, detail in results:
            icon = "✓" if ok else "✗"
            line = f"  {icon}  {name}"
            if detail:
                line += f"  [{detail}]"
            print(line)

        print(f"\n  {n_pass}/{len(results)} checks passed")

        # Also print representative signal values for one hop
        # Re-run to get the bus (cheap)
        signal_bus: dict = {}
        from drone_control.signal_faker import SignalFaker
        faker = SignalFaker(
            cfg=CFG,
            drone_ids=[DRONE_ID],
            get_positions=lambda: {"gc": sc["gc_pos"], "leader": sc["leader_pos"], DRONE_ID: sc["follower_pos"]},
            publish=lambda t, p: signal_bus.update({t: p}),
            get_severity=lambda hop: sc["severities"][hop],
            clock=lambda: 7777.0,
        )
        faker.tick()
        sig = signal_bus.get("/signal/gc_to_leader")
        if sig:
            dist = haversine(sc["gc_pos"], sc["leader_pos"])
            print(f"\n  gc_to_leader signal (dist={dist:.0f}m, sev={sc['severities']['gc_to_leader']:.2f}):")
            print(f"    rssi_dbm  = {sig['rssi_dbm']:.2f}")
            print(f"    noise_dbm = {sig['noise_dbm']:.2f}")
            print(f"    snr_db    = {sig['snr_db']:.2f}")

    print(f"\n{'='*70}")
    print(f"TOTAL: {total_pass} passed, {total_fail} failed")
    if total_fail > 0:
        print("INTEGRATION SMOKE TEST FAILED")
        sys.exit(1)
    else:
        print("INTEGRATION SMOKE TEST PASSED")


if __name__ == "__main__":
    main()
