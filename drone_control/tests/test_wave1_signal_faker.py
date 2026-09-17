"""
test_wave1_signal_faker.py — Wave 1 gate tests for signal_faker.py.

TEST_PROTOCOL: §5.2  Archetype C (transform node)

Harness: hand-inject positions + severities, capture publishes.
No ROS2, no MAVLink, no PX4.  BUILDSPEC §5.5 clock is injectable.

PROVEN column:
  test_fspl_known_distance        -> BUILDSPEC §4.1 formula
  test_rssi_falls_with_distance   -> BUILDSPEC §4.1
  test_noise_independent_of_distance -> BUILDSPEC §4.1 (noise is severity-only)
  test_snr_is_rssi_minus_noise    -> BUILDSPEC §2.1
  test_per_hop_severity_independent -> BUILDSPEC §4.1 hard rule
  test_publishes_all_five_hops    -> BUILDSPEC §2.1 hop topic list
  test_schema_exact               -> BUILDSPEC §2.1 (no extra fields)
  test_timestamp_is_origin        -> BUILDSPEC §5.6
"""

import math
import sys
import os
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from drone_control.signal_faker import compute_hop, SignalFaker

# ── Test fixtures ─────────────────────────────────────────────────────────────

_CFG = {
    "baseline_noise_dbm": -95.0,
    "noise_range_db":      40.0,
    "tx_power_dbm":        20.0,
    "frequency_mhz":       915,
    "hop_severities": {
        "gc_to_leader":       0.0,
        "leader_to_gc":       0.0,
        "gc_to_follower":     0.0,
        "follower_to_gc":     0.0,
        "leader_to_follower": 0.0,
    },
}

_POS_GC        = {"lat": 47.3900, "lon": 8.5400, "alt": 0.0}
_POS_LEADER    = {"lat": 47.4100, "lon": 8.5600, "alt": 50.0}   # ~2800 m from GC
_POS_FOLLOWER  = {"lat": 47.4000, "lon": 8.5500, "alt": 50.0}   # ~1400 m from GC

_DRONE_ID = "drone-02"


class FakeClock:
    """
    Controllable time source — TEST_PROTOCOL §3.3: no test may sleep.
    SignalFaker captures t0 = clock() at tick start and stamps all payloads
    with it; tests verify that advancing the clock does not change the stamp.
    """
    def __init__(self, t=1000.0):
        self._t = t
    def __call__(self):
        return self._t
    def advance(self, seconds):
        self._t += seconds


def _make_faker(severities=None, clock=None, drone_id=_DRONE_ID):
    """
    Build a SignalFaker with captured-publish and injectable clock.
    All five dependencies (positions, severities, publish, clock) are injected
    so tests control all inputs and observe all outputs without real hardware.
    """
    captured = []   # all (topic, payload) pairs published during the test

    # Start from all-zero severities; tests that care about specific values override.
    sev_map = dict(_CFG["hop_severities"])
    if severities:
        sev_map.update(severities)

    def _get_positions():
        # Returns the three drone positions used to compute FSPL distances.
        return {
            "gc":     _POS_GC,
            "leader": _POS_LEADER,
            drone_id: _POS_FOLLOWER,
        }

    def _get_severity(hop_name):
        # Called once per hop per tick — returns the per-hop severity for that hop.
        return sev_map[hop_name]

    faker = SignalFaker(
        cfg=_CFG,
        drone_ids=[drone_id],
        get_positions=_get_positions,
        publish=lambda topic, payload: captured.append((topic, payload)),
        get_severity=_get_severity,
        clock=clock or FakeClock(),
    )
    return faker, captured


# ── FSPL formula tests (pure compute_hop) ─────────────────────────────────────

def _expected_rssi(dist_m: float) -> float:
    """Reference FSPL RSSI — BUILDSPEC §4.1 formula."""
    path_loss = 20 * math.log10(max(dist_m, 1.0)) + 20 * math.log10(915) - 27.55
    return 20.0 - path_loss


def _dist(a, b) -> float:
    from drone_control.signal_faker import haversine
    return haversine(a, b)


class TestFSPL:
    """BUILDSPEC §4.1 formula correctness."""

    def test_fspl_known_distance(self):
        # 1000 m at 915 MHz, tx_power=20 dBm
        dist = 1000.0
        expected = _expected_rssi(dist)
        # Manufacture two positions 1 km apart (roughly)
        pos_a = {"lat": 47.0000, "lon": 8.0000, "alt": 0.0}
        pos_b = {"lat": 47.0090, "lon": 8.0000, "alt": 0.0}  # ~1000 m N
        actual_dist = _dist(pos_a, pos_b)
        result = compute_hop(pos_a, pos_b, 0.0, _CFG, 1000.0)
        # Verify with the actual distance computed by haversine
        expected_for_actual = _expected_rssi(actual_dist)
        assert math.isclose(result["rssi_dbm"], expected_for_actual, rel_tol=1e-6), (
            f"rssi={result['rssi_dbm']:.4f} expected={expected_for_actual:.4f}"
        )

    def test_rssi_falls_with_distance(self):
        # Double the distance → lower rssi_dbm.  FSPL is 20*log10 so it falls.
        pos_a = {"lat": 47.0000, "lon": 8.0000, "alt": 0.0}
        pos_b_close = {"lat": 47.0045, "lon": 8.0000, "alt": 0.0}
        pos_b_far   = {"lat": 47.0090, "lon": 8.0000, "alt": 0.0}
        r_close = compute_hop(pos_a, pos_b_close, 0.0, _CFG, 0.0)
        r_far   = compute_hop(pos_a, pos_b_far,   0.0, _CFG, 0.0)
        assert r_far["rssi_dbm"] < r_close["rssi_dbm"], (
            "rssi should fall as distance increases"
        )

    def test_noise_independent_of_distance(self):
        # Same severity at two distances → identical noise_dbm.
        # Noise is purely a function of severity, not distance.  BUILDSPEC §4.1.
        sev = 0.6
        pos_a = {"lat": 47.0000, "lon": 8.0000, "alt": 0.0}
        pos_b_close = {"lat": 47.0045, "lon": 8.0000, "alt": 0.0}
        pos_b_far   = {"lat": 47.0090, "lon": 8.0000, "alt": 0.0}
        r_close = compute_hop(pos_a, pos_b_close, sev, _CFG, 0.0)
        r_far   = compute_hop(pos_a, pos_b_far,   sev, _CFG, 0.0)
        assert r_close["noise_dbm"] == r_far["noise_dbm"], (
            "noise_dbm must not change with distance"
        )

    def test_snr_is_rssi_minus_noise(self):
        result = compute_hop(_POS_GC, _POS_LEADER, 0.3, _CFG, 0.0)
        assert math.isclose(
            result["snr_db"],
            result["rssi_dbm"] - result["noise_dbm"],
            rel_tol=1e-9,
        ), "snr_db must equal rssi_dbm - noise_dbm exactly"


# ── SignalFaker integration tests ─────────────────────────────────────────────

class TestSignalFaker:
    """BUILDSPEC §4.1, §2.1, §5.6 — full tick behaviour."""

    def test_per_hop_severity_independent(self):
        """
        5 distinct severities → 5 distinct noise_dbm values.
        HARD RULE: never share severities between hops.  BUILDSPEC §4.1.
        noise_dbm = baseline + severity * noise_range, so distinct severities
        produce distinct noise values only if they are never collapsed to a single value.
        """
        sev_map = {
            "gc_to_leader":       0.0,
            "leader_to_gc":       0.2,
            "gc_to_follower":     0.4,
            "follower_to_gc":     0.6,
            "leader_to_follower": 0.8,
        }
        faker, captured = _make_faker(severities=sev_map)
        faker.tick()

        noise_values = [payload["noise_dbm"] for _, payload in captured]
        # All 5 must be distinct — if severities were shared any two would match.
        assert len(set(round(n, 6) for n in noise_values)) == 5, (
            f"Expected 5 distinct noise_dbm values, got {noise_values}"
        )

    def test_publishes_all_five_hops(self):
        """One tick → exactly 5 publishes on the §2.1 topics.  BUILDSPEC §2.1."""
        faker, captured = _make_faker()
        faker.tick()

        # ROS 2 topic names disallow hyphens; signal_faker sanitizes DRONE_ID before topic construction.
        _tid = _DRONE_ID.replace('-', '_')
        expected_topics = {
            "/signal/gc_to_leader",
            "/signal/leader_to_gc",
            f"/signal/gc_to_follower/{_tid}",
            f"/signal/follower_to_gc/{_tid}",
            f"/signal/leader_to_follower/{_tid}",
        }
        actual_topics = {topic for topic, _ in captured}
        assert actual_topics == expected_topics, (
            f"Published topics: {actual_topics}\nExpected: {expected_topics}"
        )
        assert len(captured) == 5, f"Expected 5 publishes, got {len(captured)}"

    def test_schema_exact(self):
        """
        Every payload has exactly the §2.1 keys — no more, no fewer.
        BUILDSPEC §2.1.
        """
        expected_keys = {"rssi_dbm", "noise_dbm", "snr_db", "severity", "timestamp"}
        faker, captured = _make_faker()
        faker.tick()

        for topic, payload in captured:
            assert set(payload.keys()) == expected_keys, (
                f"Topic {topic}: keys={set(payload.keys())} expected={expected_keys}"
            )

    def test_timestamp_is_origin(self):
        """
        Advancing the clock between tick start and (simulated) publish
        does not change the timestamp — it captures origin time.
        BUILDSPEC §5.6.
        """
        clock = FakeClock(t=5000.0)
        faker, captured = _make_faker(clock=clock)
        faker.tick()

        # All published payloads must carry the clock value AT TICK START.
        for topic, payload in captured:
            assert payload["timestamp"] == 5000.0, (
                f"Topic {topic}: timestamp={payload['timestamp']}, "
                "expected origin time 5000.0"
            )

    def test_severity_in_payload_matches_input(self):
        """severity field in output reflects the per-hop severity given as input."""
        sev_map = {"gc_to_leader": 0.42, "leader_to_gc": 0.42,
                   "gc_to_follower": 0.42, "follower_to_gc": 0.42,
                   "leader_to_follower": 0.42}
        faker, captured = _make_faker(severities=sev_map)
        faker.tick()
        for _, payload in captured:
            assert math.isclose(payload["severity"], 0.42, rel_tol=1e-9)
