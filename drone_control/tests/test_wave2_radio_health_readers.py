"""
test_wave2_radio_health_readers.py — Wave 2 gate tests for the three readers.

TEST_PROTOCOL: §5.3  Archetype C (transform node)
Run the whole table against each of the three readers.

PROVEN column:
  test_schema_identical_across_readers -> BUILDSPEC §4.2, §2.2
  test_snr_passthrough_unmodified      -> BUILDSPEC §2.2 Q16
  test_timestamp_preserved             -> BUILDSPEC §5.6
  test_publishes_when_input_static     -> BUILDSPEC §4.2 hard rule
  test_subscribes_only_own_hops        -> BUILDSPEC §4.2 hard rule
  test_severity_derivation             -> BUILDSPEC §4.2 FSPL-inverse
"""

import sys
import os
import math
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

_DC = os.path.join(_PKG_ROOT, "drone_control")
if _DC not in sys.path:
    sys.path.insert(0, _DC)

from drone_control.follower_radio_health_reader import make_follower_reader
from drone_control.leader_radio_health_reader   import make_leader_reader
from drone_control.gc_radio_health_reader       import make_gc_reader
from drone_control._radio_health_core           import compute_radio_health

# ── Shared test config ────────────────────────────────────────────────────────

_CFG = {
    "baseline_noise_dbm":    -95.0,
    "noise_range_db":         40.0,
    "jamming_severity_factor": 0.6,
    "follower_radio_range_m": 800.0,
    "leader_radio_range_m":   800.0,
    "gc_radio_range_m":       800.0,
}

_DRONE_ID  = "drone-02"
_SEV_31    = 0.5   # test severity
_NOISE_31  = -95.0 + _SEV_31 * 40.0   # = -75.0 dBm

# A §2.1 signal payload for testing
def _make_signal(snr_db=-10.0, severity=_SEV_31, ts=5000.0) -> dict:
    noise_dbm = -95.0 + severity * 40.0
    rssi_dbm  = noise_dbm + snr_db
    return {
        "rssi_dbm":  rssi_dbm,
        "noise_dbm": noise_dbm,
        "snr_db":    snr_db,
        "severity":  severity,
        "timestamp": ts,
    }

_SCHEMA_22_KEYS = {"severity", "range_m", "snr_db", "hop", "timestamp"}


# ── Reader factories ──────────────────────────────────────────────────────────

def _make_follower():
    """Returns (reader, captured_publishes)."""
    captured = []
    reader = make_follower_reader(_DRONE_ID, _CFG,
                                  publish=lambda t, p: captured.append((t, p)))
    return reader, captured


def _make_leader():
    captured = []
    reader = make_leader_reader("drone-01", _CFG,
                                publish=lambda t, p: captured.append((t, p)))
    return reader, captured


def _make_gc():
    captured = []
    reader = make_gc_reader([_DRONE_ID], _CFG,
                             publish=lambda t, p: captured.append((t, p)))
    return reader, captured


# ── Parametrize over all three readers ───────────────────────────────────────

@pytest.fixture(params=["follower", "leader", "gc"])
def reader_and_capture(request):
    """
    Parametrize the same test table over all three readers.
    BUILDSPEC §4.2: schema identity is enforced in one place for all three readers.
    Each fixture instance receives its first subscription topic for feeding input
    (follower: leader_to_follower, leader: gc_to_leader, gc: gc_to_leader).
    """
    name = request.param
    if name == "follower":
        r, c = _make_follower()
        # Use first subscribed topic for feeding input
        sub_topic, hop_name = r._subscriptions[0]
    elif name == "leader":
        r, c = _make_leader()
        sub_topic, hop_name = r._subscriptions[0]
    else:
        r, c = _make_gc()
        sub_topic, hop_name = r._subscriptions[0]   # gc_to_leader
    return r, c, sub_topic, hop_name


# ── §5.3 test table — run against every reader ───────────────────────────────

class TestRadioHealthReaders:

    def test_schema_identical_across_readers(self, reader_and_capture):
        """All three readers emit exactly the §2.2 key set.  BUILDSPEC §4.2."""
        reader, captured, sub_topic, _ = reader_and_capture
        reader.on_signal(sub_topic, _make_signal())
        reader.publish_tick()
        assert len(captured) >= 1
        for _, payload in captured:
            assert set(payload.keys()) == _SCHEMA_22_KEYS, (
                f"Keys mismatch: {set(payload.keys())} vs {_SCHEMA_22_KEYS}"
            )

    def test_snr_passthrough_unmodified(self, reader_and_capture):
        """Input snr_db appears unchanged on output.  BUILDSPEC §2.2 Q16."""
        reader, captured, sub_topic, _ = reader_and_capture
        signal = _make_signal(snr_db=-7.5)
        reader.on_signal(sub_topic, signal)
        reader.publish_tick()
        for _, payload in captured:
            assert payload["snr_db"] == -7.5, (
                f"snr_db was modified: {payload['snr_db']} != -7.5"
            )

    def test_timestamp_preserved(self, reader_and_capture):
        """Output timestamp == input origin timestamp.  BUILDSPEC §5.6."""
        reader, captured, sub_topic, _ = reader_and_capture
        signal = _make_signal(ts=9999.123)
        reader.on_signal(sub_topic, signal)
        reader.publish_tick()
        for _, payload in captured:
            assert payload["timestamp"] == 9999.123, (
                f"timestamp restamped: {payload['timestamp']} != 9999.123"
            )

    def test_publishes_when_input_static(self, reader_and_capture):
        """
        No new input for N ticks → still publishes.
        BUILDSPEC §4.2 hard rule: publish every tick, downstream sees aging timestamp.
        The aging timestamp is what allows freshness guards to detect stale radio_health —
        if the publisher stopped on silence, downstream would see nothing at all and
        could not distinguish "source crashed" from "link is good and quiet".
        """
        reader, captured, sub_topic, _ = reader_and_capture
        reader.on_signal(sub_topic, _make_signal(ts=1000.0))

        # Three ticks with no new input — must still publish each time.
        reader.publish_tick()
        reader.publish_tick()
        reader.publish_tick()

        # Must have 3 publishes with the same (aging) origin timestamp.
        assert len(captured) >= 3, f"Expected ≥3 publishes, got {len(captured)}"
        # Timestamps are all the origin timestamp (aging because no new signal received).
        for _, payload in captured:
            assert payload["timestamp"] == 1000.0

    def test_subscribes_only_own_hops(self, reader_and_capture):
        """
        Feed a hop the reader is not subscribed to → no publish.
        BUILDSPEC §4.2 hard rule: follower never sees gc_to_leader, etc.
        """
        reader, captured, _, _ = reader_and_capture
        # Pick a topic that no reader is subscribed to
        wrong_topic = "/signal/leader_to_gc"
        reader.on_signal(wrong_topic, _make_signal())
        reader.publish_tick()
        # State for all hops is None → publish_tick skips them
        assert len(captured) == 0, (
            f"Reader published on wrong-topic feed: {captured}"
        )

    def test_severity_derivation(self, reader_and_capture):
        """
        Known noise_dbm → expected severity via FSPL-inverse.
        severity = (noise_dbm - baseline) / noise_range.  BUILDSPEC §4.2.
        """
        reader, captured, sub_topic, _ = reader_and_capture
        # Known: severity=0.5 → noise_dbm = -95 + 0.5*40 = -75
        signal = _make_signal(severity=0.5, snr_db=-5.0)
        reader.on_signal(sub_topic, signal)
        reader.publish_tick()
        assert len(captured) >= 1
        for _, payload in captured:
            assert math.isclose(payload["severity"], 0.5, abs_tol=1e-9), (
                f"severity={payload['severity']}, expected 0.5"
            )


# ── Follower-specific: hard rule gc_to_leader never read ─────────────────────

class TestFollowerNeverSeesGcToLeader:
    """BUILDSPEC §4.2 hard rule: the follower never sees gc_to_leader."""

    def test_gc_to_leader_not_in_follower_subscriptions(self):
        reader, _ = _make_follower()
        sub_topics = [t for t, _ in reader._subscriptions]
        assert "/signal/gc_to_leader" not in sub_topics, (
            "Follower reader must NOT subscribe to gc_to_leader"
        )

    def test_leader_to_gc_not_in_follower_subscriptions(self):
        reader, _ = _make_follower()
        sub_topics = [t for t, _ in reader._subscriptions]
        assert "/signal/leader_to_gc" not in sub_topics


# ── compute_radio_health unit test ───────────────────────────────────────────

class TestComputeRadioHealth:
    """Unit test the core computation directly — BUILDSPEC §4.2."""

    def test_severity_derivation_exact(self):
        signal = _make_signal(severity=0.3, snr_db=-2.0)
        result = compute_radio_health(signal, 800.0, "gc_to_leader", _CFG)
        assert math.isclose(result["severity"], 0.3, abs_tol=1e-9)

    def test_range_m_from_severity(self):
        # severity=0.5, factor=0.6 → range = 800*(1 - 0.5*0.6) = 800*0.7 = 560
        signal = _make_signal(severity=0.5, snr_db=-5.0)
        result = compute_radio_health(signal, 800.0, "gc_to_leader", _CFG)
        assert math.isclose(result["range_m"], 560.0, abs_tol=1e-6)

    def test_severity_clamped_above_one(self):
        # noise_dbm > baseline + noise_range → severity would be >1, clamp to 1.0
        bad_signal = {"noise_dbm": -50.0, "snr_db": -99.0, "timestamp": 0.0}
        result = compute_radio_health(bad_signal, 800.0, "gc_to_leader", _CFG)
        assert result["severity"] <= 1.0

    def test_severity_clamped_below_zero(self):
        # noise_dbm < baseline → severity would be <0, clamp to 0.0
        clean_signal = {"noise_dbm": -100.0, "snr_db": 30.0, "timestamp": 0.0}
        result = compute_radio_health(clean_signal, 800.0, "gc_to_leader", _CFG)
        assert result["severity"] >= 0.0
