"""
blackboard.py — per-key timestamped blackboard for the relay BT.

BUILDSPEC: §4.5
LAYER:     1 (pure state container — no I/O, no network)
SUBSCRIBES: none
PUBLISHES:  none

Hard rules this file must satisfy (BUILDSPEC §4.5):
  - get_with_freshness(key, max_age_s) returns (None, False, None) for a
    never-written key — does NOT raise, does NOT return float('inf')
    -> proven by test_never_written_key
  - Stale-but-present differs from never-written: stale returns the value,
    never-written returns None — callers can distinguish
    -> proven by test_stale_past_window vs test_never_written_key
  - set() re-stamps on every call, even when value is unchanged
    -> proven by test_set_restamps_unchanged_value
  - Clock is injectable (TEST_PROTOCOL §3.3) — no direct time.time() calls
    -> required for freshness tests without sleeping
"""

import time
import logging

log = logging.getLogger(__name__)


class TimestampedBlackboard:
    """
    Per-key timestamped blackboard.  Freshness checks compare key age against
    a caller-supplied max_age_s window.  Clock is injectable for testing.
    BUILDSPEC §4.5.
    """

    def __init__(self, clock=None):
        # Injectable clock — defaults to wall time; tests substitute FakeClock.now.
        # Required by TEST_PROTOCOL §3.3: no test may sleep to advance timers.
        self._clock  = clock or time.time
        self._data:       dict = {}
        self.timestamps:  dict = {}

    def set(self, key: str, value) -> None:
        # HARD RULE (§4.5): re-stamp on every set, even when the value is unchanged.
        # Callers that write the same value to keep it fresh depend on this.
        self._data[key]      = value
        self.timestamps[key] = self._clock()
        summary = str(value)[:80] if not isinstance(value, dict) else list(value.keys())
        log.debug("BB SET %s=%s", key, summary)

    def get(self, key: str):
        """Returns stored value or None if never set.  Does NOT check staleness."""
        return self._data.get(key)

    def get_with_freshness(self, key: str, max_age_s: float) -> tuple:
        """
        Returns (value, is_fresh, age_seconds).

        Never-written key  → (None,  False, None)
        Stale-but-present  → (value, False, age)
        Fresh              → (value, True,  age)

        HARD RULE (BUILDSPEC §4.5): never-written returns None for age,
        NOT float('inf').  Callers that need to distinguish never-seen from
        stale check `value is None` on the first element.
        """
        if key not in self.timestamps:
            log.debug("BB GET %s → never written", key)
            return None, False, None   # HARD RULE: None, not float('inf')

        age      = self._clock() - self.timestamps[key]
        is_fresh = age <= max_age_s
        log.debug("BB GET %s age=%.2fs fresh=%s", key, age, is_fresh)
        return self._data.get(key), is_fresh, age

    def age(self, key: str):
        """
        Returns seconds since last set, or None if never set.
        Consistent with get_with_freshness third element.
        """
        if key not in self.timestamps:
            return None
        return self._clock() - self.timestamps[key]

    def all_keys(self) -> list:
        return list(self._data.keys())

    def dump(self) -> dict:
        """Returns full state for debug logging."""
        now = self._clock()
        return {
            k: {
                "value":     self._data[k],
                "age_s":     round(now - self.timestamps[k], 3),
                "timestamp": self.timestamps[k],
            }
            for k in self._data
        }
