"""
test_wave4_blackboard.py — Wave 4 gate tests for relay_bt/blackboard.py.

TEST_PROTOCOL: §5.4  Archetype B (pure state container)

PROVEN column:
  test_never_written_key         -> BUILDSPEC §4.5 — the reason this file is touched
  test_fresh_within_window       -> BUILDSPEC §4.5
  test_stale_past_window         -> BUILDSPEC §4.5 — stale ≠ absent
  test_set_restamps_unchanged_value -> BUILDSPEC §4.5 explicit requirement
"""

import sys
import os
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
_RBT = os.path.join(_PKG_ROOT, "drone_control", "relay_bt")
for p in (_PKG_ROOT, _RBT):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.relay_bt.blackboard import TimestampedBlackboard


class FakeClock:
    """
    Controllable time source.  TEST_PROTOCOL §3.3 — no test may sleep.
    advance() lets tests simulate minutes of elapsed time in microseconds,
    which is essential for freshness-window boundary tests.
    """
    def __init__(self, t=1000.0):
        self._t = t
    def now(self):
        return self._t
    def advance(self, seconds: float):
        self._t += seconds


class TestBlackboard:

    def test_never_written_key(self):
        """
        get_with_freshness on a key that was never set returns (None, False, None).
        Does NOT raise.  Does NOT return float('inf') for the age field.
        BUILDSPEC §4.5 — the primary reason this file is being touched.

        Why age=None and not float('inf'): callers distinguish "never seen" from "stale"
        by checking age is None.  float('inf') would look like a very stale entry,
        but callers may behave differently (boot-grace suppression only fires on None).
        """
        bb = TimestampedBlackboard(clock=FakeClock().now)
        value, is_fresh, age = bb.get_with_freshness("nonexistent_key", 10.0)
        assert value    is None,  f"value should be None, got {value!r}"
        assert is_fresh is False, f"is_fresh should be False, got {is_fresh!r}"
        assert age      is None,  f"age should be None (not float('inf')), got {age!r}"

    def test_never_written_key_does_not_raise(self):
        """Accessing a never-written key must never raise — any exception is a bug."""
        bb = TimestampedBlackboard(clock=FakeClock().now)
        try:
            bb.get_with_freshness("missing", 5.0)
        except Exception as e:
            pytest.fail(f"get_with_freshness raised on missing key: {e}")

    def test_fresh_within_window(self):
        """Write a key, advance 5 s, query with max_age=10 → is_fresh True."""
        clock = FakeClock(t=1000.0)
        bb = TimestampedBlackboard(clock=clock.now)
        bb.set("snr_db", 15.0)
        clock.advance(5.0)
        value, is_fresh, age = bb.get_with_freshness("snr_db", 10.0)
        assert value    == 15.0
        assert is_fresh is True,  f"should be fresh at 5s with max_age=10s, got {is_fresh}"
        assert abs(age - 5.0) < 1e-9

    def test_stale_past_window(self):
        """
        Write, advance 15 s, query with max_age=10 → is_fresh False, value
        still returned.  Stale ≠ absent: the value is still accessible.
        BUILDSPEC §4.5.
        """
        clock = FakeClock(t=1000.0)
        bb = TimestampedBlackboard(clock=clock.now)
        bb.set("battery_pct", 80.0)
        clock.advance(15.0)
        value, is_fresh, age = bb.get_with_freshness("battery_pct", 10.0)
        assert value    == 80.0,  "stale value must still be returned"
        assert is_fresh is False, f"should be stale at 15s with max_age=10s"
        assert abs(age - 15.0) < 1e-9

    def test_set_restamps_unchanged_value(self):
        """
        Write X, advance 20 s (now stale), write X again → fresh.
        set() must re-stamp on every call, even when value is identical.
        BUILDSPEC §4.5 explicit requirement.
        """
        clock = FakeClock(t=1000.0)
        bb = TimestampedBlackboard(clock=clock.now)
        bb.set("role", "OPEN_TO_RELAY")
        clock.advance(20.0)

        # Verify it's stale before the second set
        _, is_fresh, _ = bb.get_with_freshness("role", 10.0)
        assert is_fresh is False, "should be stale before re-stamp"

        # Write same value — must re-stamp
        bb.set("role", "OPEN_TO_RELAY")
        value, is_fresh, age = bb.get_with_freshness("role", 10.0)
        assert value    == "OPEN_TO_RELAY"
        assert is_fresh is True, "should be fresh immediately after re-stamp with same value"
        assert age < 1e-9

    def test_get_returns_none_for_missing(self):
        """get() (no freshness) returns None for a never-set key."""
        bb = TimestampedBlackboard(clock=FakeClock().now)
        assert bb.get("not_there") is None

    def test_get_returns_value_when_set(self):
        """get() returns the stored value."""
        bb = TimestampedBlackboard(clock=FakeClock().now)
        bb.set("pos", {"lat": 47.39, "lon": 8.54})
        assert bb.get("pos") == {"lat": 47.39, "lon": 8.54}

    def test_age_returns_none_for_missing(self):
        """age() returns None for a never-set key (consistent with get_with_freshness)."""
        bb = TimestampedBlackboard(clock=FakeClock().now)
        assert bb.age("ghost") is None

    def test_age_returns_elapsed(self):
        """age() returns elapsed seconds since last set."""
        clock = FakeClock(t=500.0)
        bb = TimestampedBlackboard(clock=clock.now)
        bb.set("x", 1)
        clock.advance(7.5)
        assert abs(bb.age("x") - 7.5) < 1e-9

    def test_overwrite_updates_value(self):
        """Writing a different value replaces the old one."""
        bb = TimestampedBlackboard(clock=FakeClock().now)
        bb.set("status", "A")
        bb.set("status", "B")
        assert bb.get("status") == "B"

    def test_at_exact_max_age_is_fresh(self):
        """
        age == max_age_s is still fresh (<=, not <).
        Boundary condition: exactly at the window edge.
        """
        clock = FakeClock(t=0.0)
        bb = TimestampedBlackboard(clock=clock.now)
        bb.set("k", "v")
        clock.advance(10.0)
        _, is_fresh, _ = bb.get_with_freshness("k", 10.0)
        assert is_fresh is True, "age == max_age should still be fresh (<=)"

    def test_one_tick_past_max_age_is_stale(self):
        """age == max_age_s + epsilon → stale."""
        clock = FakeClock(t=0.0)
        bb = TimestampedBlackboard(clock=clock.now)
        bb.set("k", "v")
        clock.advance(10.001)
        _, is_fresh, _ = bb.get_with_freshness("k", 10.0)
        assert is_fresh is False

    def test_multiple_keys_independent(self):
        """Each key has its own timestamp; one stale key doesn't affect another."""
        clock = FakeClock(t=0.0)
        bb = TimestampedBlackboard(clock=clock.now)
        bb.set("old_key", "old")
        clock.advance(20.0)
        bb.set("new_key", "new")

        _, old_fresh, _ = bb.get_with_freshness("old_key", 10.0)
        _, new_fresh, _ = bb.get_with_freshness("new_key", 10.0)
        assert old_fresh is False
        assert new_fresh is True

    def test_all_keys_lists_set_keys(self):
        """all_keys() returns exactly the keys that have been set."""
        bb = TimestampedBlackboard(clock=FakeClock().now)
        bb.set("a", 1)
        bb.set("b", 2)
        assert set(bb.all_keys()) == {"a", "b"}
