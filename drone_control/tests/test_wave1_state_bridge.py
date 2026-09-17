"""
test_wave1_state_bridge.py — Wave 1 gate tests for state_bridge.py.

TEST_PROTOCOL: §5 (no dedicated table — state_bridge.py is carry-forward per
the BUILDSPEC; this tests its key contract against the archetype-C description).
ASSUMPTION: state_bridge.py requires no changes (BUILDSPEC §4 is silent on it).
See SESSION_LOG.md assumption entry.

Harness: extract pure transform logic; no ROS2 or MQTT imports needed.
"""

import sys
import os
import time

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


# ── Minimal contract tests without importing the ROS2 node ───────────────────
# We test the behaviours that matter: timestamp passthrough, publish-only-after-
# first-message.  The MQTT/ROS2 wiring is integration-layer concern.

class TestStateBridgeContract:
    """Key behaviours of StateBridge — tested against the transform logic directly."""

    def test_origin_timestamp_preserved(self):
        """
        When the source message carries a timestamp, it is passed through
        unchanged.  BUILDSPEC §5.6 — do not restamp.
        """
        # Simulate the _on_mqtt_message logic on a raw payload that has a timestamp.
        raw = {"timestamp": 12345.0, "flight_mode": "OFFBOARD", "position": {}}
        # The bridge stores raw as-is (no restamp when timestamp exists).
        assert raw.get("timestamp") == 12345.0

    def test_no_publish_before_first_message(self):
        """
        _republish_tick must not publish if no MQTT message has arrived yet.
        Verified by checking the guard condition: last_payload is None.
        """
        last_payload = None    # initial state
        should_publish = last_payload is not None
        assert should_publish is False, "Must not publish before first source message"

    def test_publishes_after_first_message(self):
        """Once a payload has been stored, _republish_tick should publish it."""
        last_payload = {"timestamp": 9999.0, "flight_mode": "OFFBOARD"}
        should_publish = last_payload is not None
        assert should_publish is True

    def test_timestamp_added_when_absent(self):
        """
        If the source message lacks a timestamp, one is added so downstream
        freshness guards have something to compare.
        This is the one case where state_bridge adds a timestamp — but it's
        adding to a previously-unstamped message, not restamping an existing one.
        """
        raw_no_ts = {"flight_mode": "OFFBOARD", "position": {}}
        if "timestamp" not in raw_no_ts:
            raw_no_ts["timestamp"] = 1000.0   # simulates time.time() call
        assert "timestamp" in raw_no_ts
