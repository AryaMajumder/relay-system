"""
test_wave3_leader_link_detector.py — Wave 3 gate tests for leader_link_detector.py.

TEST_PROTOCOL: §5 Archetype C (transform node)

PROVEN column:
  test_publishes_when_below_threshold    -> BUILDSPEC §4.4 core logic
  test_no_publish_when_above_threshold   -> BUILDSPEC §4.4 core logic
  test_no_sender_side_dedup              -> BUILDSPEC §4.4 HARD RULE
  test_suppression_blocks_publish        -> BUILDSPEC §4.4 hard rule
  test_suppression_resume_automatic      -> BUILDSPEC §4.4 "no separate resume logic"
  test_schema_matches_section_23         -> BUILDSPEC §2.3
  test_subscribes_radio_health_not_signal -> BUILDSPEC §4.4
  test_no_publish_before_first_input     -> archetype C correctness
  test_timestamp_passthrough             -> BUILDSPEC §5.6
"""

import sys
import os
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for p in (_PKG_ROOT, os.path.join(_PKG_ROOT, "drone_control")):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.leader_link_detector import make_leader_link_detector

_CFG = {
    "LINK_MARGINAL_QUALITY": 13,
}
_DRONE_ID = "drone-01"

# §2.3 schema
_SCHEMA_23_KEYS = {"drone_id", "snr_db", "timestamp"}

# §2.2 radio_health payload (gc_to_leader hop from leader_radio_health_reader)
def _make_health(snr_db=10.0, hop="gc_to_leader", ts=7000.0) -> dict:
    return {
        "severity":  0.3,
        "range_m":   700.0,
        "snr_db":    snr_db,
        "hop":       hop,
        "timestamp": ts,
    }

def _make_suppression(active: bool) -> dict:
    return {"active": active}


def _make_det():
    captured = []
    det = make_leader_link_detector(
        _DRONE_ID, _CFG,
        publish=lambda t, p: captured.append((t, p)),
        suppression_topic="/relay_suppression",
    )
    return det, captured


class TestLeaderLinkDetector:

    def test_publishes_when_below_threshold(self):
        """snr_db < LINK_MARGINAL_QUALITY → publish on tick.  BUILDSPEC §4.4."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=5.0))
        det.publish_tick()
        assert len(captured) == 1

    def test_no_publish_when_above_threshold(self):
        """snr_db >= LINK_MARGINAL_QUALITY → no publish.  BUILDSPEC §4.4."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=13.0))
        det.publish_tick()
        assert len(captured) == 0, "Must not publish at exactly marginal threshold"

    def test_no_publish_strictly_above_threshold(self):
        """snr_db > LINK_MARGINAL_QUALITY → no publish."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=20.0))
        det.publish_tick()
        assert len(captured) == 0

    def test_no_sender_side_dedup(self):
        """
        5 ticks with the same below-threshold condition → 5 publishes.
        BUILDSPEC §4.4 HARD RULE: no dedup, throttling, or debounce on sender side.
        All dedup is relay_decision_authority's job.
        """
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=5.0))
        for _ in range(5):
            det.publish_tick()
        assert len(captured) == 5, (
            f"Expected 5 publishes (no dedup), got {len(captured)}"
        )

    def test_suppression_blocks_publish(self):
        """Suppression active → no publish even when snr_db < threshold.  §4.4."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=5.0))
        det.on_suppression("/relay_suppression", _make_suppression(active=True))
        det.publish_tick()
        assert len(captured) == 0, "Suppression should block publish"

    def test_suppression_resume_automatic(self):
        """
        Suppression cleared → resumes publishing.  No separate resume logic needed.
        BUILDSPEC §4.4: 'Resume automatically when it isn't — no separate resume logic.'
        """
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=5.0))

        det.on_suppression("/relay_suppression", _make_suppression(active=True))
        det.publish_tick()
        assert len(captured) == 0, "Suppressed tick should not publish"

        # Suppression clears
        det.on_suppression("/relay_suppression", _make_suppression(active=False))
        det.publish_tick()
        assert len(captured) == 1, "Should resume automatically after suppression clears"

    def test_schema_matches_section_23(self):
        """Payload keys exactly match §2.3: {drone_id, snr_db, timestamp}."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=5.0))
        det.publish_tick()
        assert len(captured) == 1
        _, payload = captured[0]
        assert set(payload.keys()) == _SCHEMA_23_KEYS, (
            f"Schema mismatch: {set(payload.keys())} vs {_SCHEMA_23_KEYS}"
        )

    def test_drone_id_in_payload(self):
        """relay_request carries the leader's drone_id.  §2.3."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=5.0))
        det.publish_tick()
        _, payload = captured[0]
        assert payload["drone_id"] == _DRONE_ID

    def test_snr_db_in_payload(self):
        """relay_request carries the raw snr_db value that triggered it.  §2.3."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=7.3))
        det.publish_tick()
        _, payload = captured[0]
        assert payload["snr_db"] == 7.3

    def test_publish_topic(self):
        """Publishes to /{drone_id}/relay_request.  BUILDSPEC §4.4."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=5.0))
        det.publish_tick()
        topic, _ = captured[0]
        # ROS 2 topic names disallow hyphens; detector sanitizes DRONE_ID for the topic path only.
        assert topic == f"/{_DRONE_ID.replace('-', '_')}/relay_request", f"Wrong topic: {topic}"

    def test_subscribes_radio_health_not_signal(self):
        """
        Subscription is /{own_id}/radio_health, not /signal/*.
        BUILDSPEC §4.4 — reads radio_health layer, not raw signal.
        """
        det, _ = _make_det()
        sub_topics = [t for t, _ in det._subscriptions]
        for topic in sub_topics:
            assert not topic.startswith("/signal/"), (
                f"Detector must not subscribe raw signal topic: {topic}"
            )
        assert f"/{_DRONE_ID.replace('-', '_')}/radio_health" in sub_topics

    def test_no_publish_before_first_input(self):
        """No radio_health received → publish_tick is a no-op.  Archetype C."""
        det, captured = _make_det()
        det.publish_tick()
        det.publish_tick()
        assert len(captured) == 0

    def test_timestamp_passthrough(self):
        """Output timestamp is origin timestamp from radio_health.  BUILDSPEC §5.6."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=5.0, ts=8765.4))
        det.publish_tick()
        _, payload = captured[0]
        assert payload["timestamp"] == 8765.4, (
            f"Timestamp restamped: got {payload['timestamp']}, expected 8765.4"
        )

    def test_ignores_wrong_hop(self):
        """Only processes gc_to_leader hop; wrong hop doesn't update state."""
        det, captured = _make_det()
        det.on_radio_health(
            f"/{_DRONE_ID}/radio_health",
            _make_health(snr_db=5.0, hop="gc_to_follower_drone-02"),
        )
        det.publish_tick()
        assert len(captured) == 0, "Wrong hop should not update detector state"

    def test_suppression_persists_until_cleared(self):
        """Suppression stays active across multiple ticks until a clear message arrives."""
        det, captured = _make_det()
        det.on_radio_health(f"/{_DRONE_ID}/radio_health", _make_health(snr_db=5.0))
        det.on_suppression("/relay_suppression", _make_suppression(active=True))
        det.publish_tick()
        det.publish_tick()
        det.publish_tick()
        assert len(captured) == 0, "Suppression should persist without explicit clear"

    def test_suppression_topic_subscription(self):
        """Detector subscribes the suppression topic.  BUILDSPEC §4.4."""
        det, _ = _make_det()
        sub_topics = [t for t, _ in det._subscriptions]
        assert "/relay_suppression" in sub_topics
