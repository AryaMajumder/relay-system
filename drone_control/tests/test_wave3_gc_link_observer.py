"""
test_wave3_gc_link_observer.py — Wave 3 gate tests for gc_link_observer.py.

TEST_PROTOCOL: §5 Archetype C (transform node)

PROVEN column:
  test_filters_to_gc_to_leader_hop_only  -> BUILDSPEC §4.3 hard rule (own measurement)
  test_no_loss_blending_required         -> BUILDSPEC §4.3 "SNR only. No loss. No blending."
  test_timestamp_passthrough             -> BUILDSPEC §5.6
  test_publish_topic                     -> BUILDSPEC §4.3 (publishes /gc/gc_link_quality)
  test_schema_keys                       -> BUILDSPEC §4.3 (no candidate/mission fields)
  test_no_publish_before_first_input     -> archetype C correctness
  test_snr_quality_formula_at_marginal   -> BUILDSPEC §4.3 SNR-derived quality (four parametric variants)
  test_never_subscribes_leader_radio_health -> BUILDSPEC §4.3 hard rule
"""

import sys
import os
import math
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for p in (_PKG_ROOT, os.path.join(_PKG_ROOT, "drone_control")):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.gc_link_observer import make_gc_link_observer, _derive_quality

_CFG = {
    "LINK_MARGINAL_QUALITY": 13,
}

_SCHEMA_KEYS = {"quality", "snr_db", "hop", "timestamp"}

# A §2.2 radio_health payload for the gc_to_leader hop
def _make_health(snr_db=10.0, hop="gc_to_leader", ts=5000.0) -> dict:
    return {
        "severity":  0.3,
        "range_m":   700.0,
        "snr_db":    snr_db,
        "hop":       hop,
        "timestamp": ts,
    }


def _make_obs():
    captured = []
    obs = make_gc_link_observer(_CFG, publish=lambda t, p: captured.append((t, p)))
    return obs, captured


class TestGcLinkObserver:

    def test_filters_to_gc_to_leader_hop_only(self):
        """Wrong hop → state not updated, nothing published.  BUILDSPEC §4.3."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health(hop="gc_to_follower_drone-02"))
        obs.publish_tick()
        assert len(captured) == 0, "Wrong-hop message should not trigger publish"

    def test_no_loss_blending_required(self):
        """
        Publishes from SNR alone — no second input (no loss report, no blending).
        BUILDSPEC §4.3: 'SNR only. No loss. No blending. No second input.'
        """
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health(snr_db=15.0))
        obs.publish_tick()
        assert len(captured) == 1
        _, payload = captured[0]
        # Payload must not have loss-related fields
        assert "loss_pct" not in payload
        assert "loss_elevated" not in payload
        assert "loss_quality" not in payload

    def test_timestamp_passthrough(self):
        """Origin timestamp carried through unchanged.  BUILDSPEC §5.6."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health(ts=9876.543))
        obs.publish_tick()
        assert len(captured) == 1
        _, payload = captured[0]
        assert payload["timestamp"] == 9876.543, (
            f"Timestamp restamped: got {payload['timestamp']}, expected 9876.543"
        )

    def test_publish_topic(self):
        """Publishes to /gc/gc_link_quality.  BUILDSPEC §4.3."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health())
        obs.publish_tick()
        assert len(captured) == 1
        topic, _ = captured[0]
        assert topic == "/gc/gc_link_quality", f"Wrong topic: {topic}"

    def test_schema_keys(self):
        """Payload has exactly {{quality, snr_db, hop, timestamp}} — no extra fields."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health())
        obs.publish_tick()
        assert len(captured) == 1
        _, payload = captured[0]
        assert set(payload.keys()) == _SCHEMA_KEYS, (
            f"Schema mismatch: {set(payload.keys())} vs {_SCHEMA_KEYS}"
        )

    def test_no_publish_before_first_input(self):
        """No data received → publish_tick is a no-op.  Archetype C correctness."""
        obs, captured = _make_obs()
        obs.publish_tick()
        obs.publish_tick()
        assert len(captured) == 0

    def test_snr_quality_formula_at_marginal(self):
        """snr_db == LINK_MARGINAL_QUALITY → quality == 0.5 (midpoint)."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health(snr_db=13.0))
        obs.publish_tick()
        _, payload = captured[0]
        assert math.isclose(payload["quality"], 0.5, abs_tol=1e-9), (
            f"quality at marginal SNR: {payload['quality']}, expected 0.5"
        )

    def test_snr_quality_formula_above_double_marginal(self):
        """snr_db == 2 × LINK_MARGINAL_QUALITY → quality == 1.0 (clamped)."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health(snr_db=26.0))
        obs.publish_tick()
        _, payload = captured[0]
        assert math.isclose(payload["quality"], 1.0, abs_tol=1e-9)

    def test_snr_quality_formula_at_zero(self):
        """snr_db == 0 → quality == 0.0."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health(snr_db=0.0))
        obs.publish_tick()
        _, payload = captured[0]
        assert math.isclose(payload["quality"], 0.0, abs_tol=1e-9)

    def test_snr_quality_formula_negative_clamped(self):
        """Negative snr_db → quality clamped to 0.0."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health(snr_db=-10.0))
        obs.publish_tick()
        _, payload = captured[0]
        assert payload["quality"] == 0.0

    def test_snr_db_passed_through(self):
        """snr_db in output equals input snr_db exactly."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health(snr_db=7.5))
        obs.publish_tick()
        _, payload = captured[0]
        assert payload["snr_db"] == 7.5

    def test_hop_field_is_gc_to_leader(self):
        """Output hop field is always 'gc_to_leader'."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health())
        obs.publish_tick()
        _, payload = captured[0]
        assert payload["hop"] == "gc_to_leader"

    def test_publishes_on_static_input(self):
        """Three ticks with no new input → still publishes (aging timestamp)."""
        obs, captured = _make_obs()
        obs.on_radio_health("/gc/radio_health", _make_health(ts=1000.0))
        obs.publish_tick()
        obs.publish_tick()
        obs.publish_tick()
        assert len(captured) == 3
        for _, p in captured:
            assert p["timestamp"] == 1000.0   # origin time, not republish time

    def test_never_subscribes_leader_radio_health(self):
        """
        Subscription list must not include the leader's own radio_health topic.
        BUILDSPEC §4.3 hard rule: GC uses its own measurement.
        """
        obs, _ = _make_obs()
        sub_topics = [t for t, _ in obs._subscriptions]
        for topic in sub_topics:
            assert "drone-01" not in topic and "drone_01" not in topic, (
                f"Observer subscribes leader's topic: {topic}"
            )
        # Must subscribe /gc/radio_health
        assert "/gc/radio_health" in sub_topics


class TestDeriveQuality:
    """Unit tests for the quality formula in isolation."""

    def test_at_marginal_is_half(self):
        assert math.isclose(_derive_quality(13.0, 13.0), 0.5, abs_tol=1e-9)

    def test_at_double_marginal_is_one(self):
        assert math.isclose(_derive_quality(26.0, 13.0), 1.0, abs_tol=1e-9)

    def test_above_double_clamped_at_one(self):
        assert _derive_quality(100.0, 13.0) == 1.0

    def test_at_zero_is_zero(self):
        assert _derive_quality(0.0, 13.0) == 0.0

    def test_negative_clamped_at_zero(self):
        assert _derive_quality(-50.0, 13.0) == 0.0

    def test_zero_marginal_safe(self):
        # Edge case: degenerate config — should not raise.
        assert _derive_quality(10.0, 0.0) == 0.0
