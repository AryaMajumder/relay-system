#!/usr/bin/env python3
"""
integration_pipeline_trace.py — full pipeline trace: config → faker → readers.

Shows every value at every stage so the signal model is easy to audit.

Run:
  python3 tests/integration_pipeline_trace.py
"""

import sys, os, math
_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
_DC       = os.path.join(_PKG_ROOT, "drone_control")
for p in (_PKG_ROOT, _DC):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.signal_faker import SignalFaker, haversine
from drone_control.follower_radio_health_reader import make_follower_reader
from drone_control.leader_radio_health_reader   import make_leader_reader
from drone_control.gc_radio_health_reader       import make_gc_reader

DRONE_ID = "drone-02"

# ── Scenarios ─────────────────────────────────────────────────────────────────

SCENARIOS = [
    {
        "name": "Clean / short range  (no jamming, 270 m gap)",
        "gc_pos":       {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},
        "leader_pos":   {"lat": 47.3920, "lon": 8.5420, "alt": 50.0},
        "follower_pos": {"lat": 47.3910, "lon": 8.5410, "alt": 30.0},
        "severities":   {k: 0.0 for k in
                         ["gc_to_leader","leader_to_gc","gc_to_follower",
                          "follower_to_gc","leader_to_follower"]},
    },
    {
        "name": "Moderate jamming / medium range  (sev=0.5 all hops, 2700 m gap)",
        "gc_pos":       {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},
        "leader_pos":   {"lat": 47.4100, "lon": 8.5600, "alt": 50.0},
        "follower_pos": {"lat": 47.4000, "lon": 8.5500, "alt": 40.0},
        "severities":   {k: 0.5 for k in
                         ["gc_to_leader","leader_to_gc","gc_to_follower",
                          "follower_to_gc","leader_to_follower"]},
    },
    {
        "name": "Per-hop mix  (leader link stressed sev=0.8, follower clean sev=0.1)",
        "gc_pos":       {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},
        "leader_pos":   {"lat": 47.4100, "lon": 8.5600, "alt": 50.0},
        "follower_pos": {"lat": 47.4000, "lon": 8.5500, "alt": 40.0},
        "severities":   {
            "gc_to_leader":       0.8,
            "leader_to_gc":       0.8,
            "gc_to_follower":     0.1,
            "follower_to_gc":     0.1,
            "leader_to_follower": 0.1,
        },
    },
    {
        "name": "Long range / thermal limit  (no jamming, 10.7 km gap)",
        "gc_pos":       {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},
        "leader_pos":   {"lat": 47.4700, "lon": 8.6200, "alt": 100.0},
        "follower_pos": {"lat": 47.4300, "lon": 8.5800, "alt": 80.0},
        "severities":   {k: 0.0 for k in
                         ["gc_to_leader","leader_to_gc","gc_to_follower",
                          "follower_to_gc","leader_to_follower"]},
    },
]

# ── Formatting helpers ────────────────────────────────────────────────────────

W = 72
def banner(text):    print(f"\n{'='*W}\n  {text}\n{'='*W}")
def section(text):   print(f"\n  ── {text} {'─'*(W-6-len(text))}")
def row(label, val): print(f"    {label:<38}  {val}")
def subrow(label, val): print(f"      {label:<36}  {val}")

def fmt_pos(p):
    return f"lat={p['lat']:.4f}  lon={p['lon']:.4f}  alt={p.get('alt',0):.0f} m"

def snr_bar(snr_db):
    """ASCII bar: ████░░ style, 20 chars wide, range -20 to +40 dB."""
    lo, hi = -20.0, 40.0
    frac = max(0.0, min(1.0, (snr_db - lo) / (hi - lo)))
    filled = round(frac * 20)
    return f"[{'█'*filled}{'░'*(20-filled)}] {snr_db:+.1f} dB"

def sev_bar(sev):
    filled = round(sev * 20)
    label  = "clean" if sev < 0.1 else ("moderate" if sev < 0.5 else ("stressed" if sev < 0.8 else "severe"))
    return f"[{'█'*filled}{'░'*(20-filled)}] {sev:.2f}  ({label})"

def range_bar(r_m, nominal=800):
    frac   = max(0.0, min(1.0, r_m / nominal))
    filled = round(frac * 20)
    return f"[{'█'*filled}{'░'*(20-filled)}] {r_m:.0f} m  ({frac*100:.0f}% of {nominal:.0f} m nominal)"

# ── Reference formulas ────────────────────────────────────────────────────────

def ref_path_loss(dist_m, freq_mhz=915):
    return 20*math.log10(max(dist_m,1)) + 20*math.log10(freq_mhz) - 27.55

def ref_rssi(dist_m, tx_pwr=20.0, freq_mhz=915):
    return tx_pwr - ref_path_loss(dist_m, freq_mhz)

def ref_noise(sev, baseline=-95.0, rng=40.0):
    return baseline + sev * rng

def ref_severity_inv(noise_dbm, baseline=-95.0, rng=40.0):
    return max(0.0, min(1.0, (noise_dbm - baseline) / rng))

def ref_range(sev, nominal=800.0, factor=0.6):
    return nominal * (1.0 - sev * factor)

# ── Main trace ────────────────────────────────────────────────────────────────

def trace_scenario(sc):
    banner(sc["name"])

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

    # ── STEP 0: Config ────────────────────────────────────────────────────────
    section("STEP 0 · CONFIG (signal model constants)")
    row("baseline_noise_dbm",       f"{CFG['baseline_noise_dbm']} dBm  (noise floor at sev=0)")
    row("noise_range_db",           f"{CFG['noise_range_db']} dB    (noise rises by this at sev=1)")
    row("tx_power_dbm",             f"{CFG['tx_power_dbm']} dBm")
    row("frequency_mhz",            f"{CFG['frequency_mhz']} MHz")
    row("jamming_severity_factor",  f"{CFG['jamming_severity_factor']}")
    row("nominal radio range",      f"{CFG['gc_radio_range_m']} m  (all radios, same hardware)")
    print()
    print("    Forward model  (faker → signal):  noise_dbm = baseline + sev × noise_range")
    print("    Inverse model  (reader → health): severity  = (noise_dbm − baseline) / noise_range")
    print("    Effective range:                  range_m   = nominal × (1 − sev × factor)")

    # ── STEP 1: Positions ─────────────────────────────────────────────────────
    section("STEP 1 · POSITIONS (faker inputs)")
    gc  = sc["gc_pos"]
    ldr = sc["leader_pos"]
    flw = sc["follower_pos"]
    row("GC       (fixed ground station)",  fmt_pos(gc))
    row("Leader   (target drone)",          fmt_pos(ldr))
    row(f"Follower (relay candidate {DRONE_ID})", fmt_pos(flw))
    print()
    d_gc_ldr = haversine(gc, ldr)
    d_gc_flw = haversine(gc, flw)
    d_ldr_flw = haversine(ldr, flw)
    row("  GC  ↔ Leader   distance",   f"{d_gc_ldr:.0f} m")
    row("  GC  ↔ Follower distance",   f"{d_gc_flw:.0f} m")
    row("  Leader ↔ Follower distance",f"{d_ldr_flw:.0f} m")

    # ── STEP 2: Severities ────────────────────────────────────────────────────
    section("STEP 2 · PER-HOP SEVERITIES (faker inputs, config-dialled)")
    print("    Each hop has an INDEPENDENT severity dial — never shared.")
    print()
    sevs = sc["severities"]
    for hop, sev in sevs.items():
        row(f"  {hop}", sev_bar(sev))

    # ── STEP 3: Run faker ─────────────────────────────────────────────────────
    section("STEP 3 · SIGNAL FAKER OUTPUT  (one tick, 2 Hz)")
    print("    Formulae applied per hop:")
    print("      path_loss_db = 20·log₁₀(dist_m) + 20·log₁₀(915) − 27.55")
    print("      rssi_dbm     = tx_power_dbm − path_loss_db")
    print("      noise_dbm    = baseline_noise_dbm + severity × noise_range_db")
    print("      snr_db       = rssi_dbm − noise_dbm")
    print()

    signal_bus = {}
    faker = SignalFaker(
        cfg=CFG,
        drone_ids=[DRONE_ID],
        get_positions=lambda: {"gc": gc, "leader": ldr, DRONE_ID: flw},
        publish=lambda t, p: signal_bus.update({t: p}),
        get_severity=lambda hop: sevs[hop],
        clock=lambda: 9000.0,
    )
    faker.tick()

    # Map topic → (hop_name, dist, pos_a_key, pos_b_key)
    hop_meta = [
        ("/signal/gc_to_leader",                   "gc_to_leader",       d_gc_ldr,  "GC",  "Leader"),
        ("/signal/leader_to_gc",                   "leader_to_gc",       d_gc_ldr,  "Leader","GC"),
        (f"/signal/gc_to_follower/{DRONE_ID}",     "gc_to_follower",     d_gc_flw,  "GC",  "Follower"),
        (f"/signal/follower_to_gc/{DRONE_ID}",     "follower_to_gc",     d_gc_flw,  "Follower","GC"),
        (f"/signal/leader_to_follower/{DRONE_ID}", "leader_to_follower", d_ldr_flw, "Leader","Follower"),
    ]

    errors = []
    for topic, hop, dist, src, dst in hop_meta:
        sig = signal_bus.get(topic)
        if sig is None:
            errors.append(f"MISSING {topic}")
            continue
        sev = sevs[hop]
        exp_rssi  = ref_rssi(dist)
        exp_noise = ref_noise(sev)
        exp_snr   = exp_rssi - exp_noise
        ok_r = math.isclose(sig["rssi_dbm"],  exp_rssi,  abs_tol=1e-4)
        ok_n = math.isclose(sig["noise_dbm"], exp_noise, abs_tol=1e-4)
        ok_s = math.isclose(sig["snr_db"],    exp_snr,   abs_tol=1e-4)

        print(f"\n    ▶  {hop}  ({src} → {dst},  dist={dist:.0f} m,  sev={sev:.2f})")
        subrow("topic",        topic)
        subrow("rssi_dbm",     f"{sig['rssi_dbm']:+.4f} dBm  {'✓' if ok_r else '✗ expected '+str(round(exp_rssi,4))}")
        subrow("noise_dbm",    f"{sig['noise_dbm']:+.4f} dBm  {'✓' if ok_n else '✗ expected '+str(round(exp_noise,4))}")
        subrow("snr_db",       snr_bar(sig["snr_db"]) + f"  {'✓' if ok_s else '✗'}")
        subrow("severity (in)",f"{sig['severity']:.3f}  (passed through from config)")
        subrow("timestamp",    f"{sig['timestamp']}  (origin time, not publish time)")

    if errors:
        for e in errors:
            print(f"\n    ✗ {e}")

    # ── STEP 4: What each reader subscribes to ────────────────────────────────
    section("STEP 4 · READER SUBSCRIPTIONS  (which signals each reader consumes)")

    health_bus = {}

    follower_reader = make_follower_reader(
        DRONE_ID, CFG, publish=lambda t, p: health_bus.update({(t, p["hop"]): p}))
    leader_reader   = make_leader_reader(
        "drone-01", CFG, publish=lambda t, p: health_bus.update({(t, p["hop"]): p}))
    gc_reader       = make_gc_reader(
        [DRONE_ID], CFG, publish=lambda t, p: health_bus.update({(t, p["hop"]): p}))

    reader_info = [
        ("follower_radio_health_reader", follower_reader, f"/{DRONE_ID}/radio_health"),
        ("leader_radio_health_reader",   leader_reader,   "/drone-01/radio_health"),
        ("gc_radio_health_reader",       gc_reader,       "/gc/radio_health"),
    ]

    for name, reader, pub_topic in reader_info:
        print(f"\n    {name}")
        subrow("publishes to", pub_topic)
        for sub_topic, hop_name in reader._subscriptions:
            signal_available = sub_topic in signal_bus
            subrow(f"  subscribes", f"{sub_topic}  {'[signal present ✓]' if signal_available else '[no signal yet]'}")

    # ── STEP 5: Feed signals into readers ────────────────────────────────────
    section("STEP 5 · FEEDING SIGNALS INTO READERS")

    for name, reader, _ in reader_info:
        for sub_topic, hop_name in reader._subscriptions:
            if sub_topic in signal_bus:
                reader.on_signal(sub_topic, signal_bus[sub_topic])
                print(f"    {name}")
                print(f"      received: {sub_topic}")
                sig = signal_bus[sub_topic]
                subrow("  noise_dbm in",  f"{sig['noise_dbm']:+.4f} dBm")
                subrow("  snr_db in",     f"{sig['snr_db']:+.4f} dB")
                subrow("  timestamp in",  f"{sig['timestamp']}")
        reader.publish_tick()

    # ── STEP 6: Reader outputs ────────────────────────────────────────────────
    section("STEP 6 · RADIO HEALTH READER OUTPUTS  (§2.2 schema)")
    print("    Computation:")
    print("      severity  = (noise_dbm − baseline) / noise_range  [clamped 0–1]")
    print("      range_m   = nominal × (1 − severity × factor)")
    print("      snr_db    = passed through unmodified  (Q16)")
    print("      timestamp = origin timestamp from signal  (§5.6)")
    print()

    reader_hop_checks = [
        ("follower_radio_health_reader", f"/{DRONE_ID}/radio_health",
         [("leader_to_follower", sevs["leader_to_follower"], CFG["follower_radio_range_m"]),
          ("gc_to_follower",     sevs["gc_to_follower"],     CFG["follower_radio_range_m"])]),
        ("leader_radio_health_reader",   "/drone-01/radio_health",
         [("gc_to_leader",       sevs["gc_to_leader"],       CFG["leader_radio_range_m"])]),
        ("gc_radio_health_reader",       "/gc/radio_health",
         [("gc_to_leader",       sevs["gc_to_leader"],       CFG["gc_radio_range_m"]),
          (f"gc_to_follower_{DRONE_ID}", sevs["gc_to_follower"], CFG["gc_radio_range_m"])]),
    ]

    total_ok = total_checks = 0
    for reader_name, pub_topic, hops in reader_hop_checks:
        print(f"    {reader_name}  →  {pub_topic}")
        for hop_name, input_sev, nominal in hops:
            key = (pub_topic, hop_name)
            h = health_bus.get(key)
            if h is None:
                print(f"      ✗  hop={hop_name}  NOT PUBLISHED")
                continue

            exp_sev = ref_severity_inv(ref_noise(input_sev))
            exp_rng = ref_range(exp_sev, nominal)

            ok_sev = math.isclose(h["severity"], exp_sev, abs_tol=1e-9)
            ok_rng = math.isclose(h["range_m"],  exp_rng, abs_tol=1e-6)
            ok_ts  = h["timestamp"] == 9000.0
            total_checks += 3
            total_ok += sum([ok_sev, ok_rng, ok_ts])

            print(f"\n      hop = {hop_name}")
            subrow("severity  (derived)",
                   f"{h['severity']:.6f}  ref={exp_sev:.6f}  {'✓' if ok_sev else '✗'}")
            subrow("range_m   (derived)",
                   range_bar(h["range_m"], nominal) + f"  ref={exp_rng:.1f}  {'✓' if ok_rng else '✗'}")
            subrow("snr_db    (passthrough)",
                   f"{h['snr_db']:+.4f} dB  (unchanged from signal faker)")
            subrow("hop label",
                   f"{h['hop']!r}")
            subrow("timestamp (passthrough)",
                   f"{h['timestamp']}  {'✓' if ok_ts else '✗'}")
        print()

    print(f"    Checks: {total_ok}/{total_checks} ✓")
    return total_ok, total_checks


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    grand_ok = grand_total = 0
    for sc in SCENARIOS:
        ok, total = trace_scenario(sc)
        grand_ok    += ok
        grand_total += total

    print(f"\n{'='*W}")
    print(f"  GRAND TOTAL: {grand_ok}/{grand_total} checks passed across all scenarios")
    if grand_ok == grand_total:
        print("  PIPELINE TRACE PASSED")
    else:
        print("  PIPELINE TRACE FAILED")
        sys.exit(1)

if __name__ == "__main__":
    main()
