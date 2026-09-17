"""
test_wave8_continuous_monitor.py — Wave 8 gate tests for continuous_monitor.py.

TEST_PROTOCOL: §5.14  Archetype C (subscribe → compute → publish)
BUILDSPEC:     §4.13

PROVEN column:
  test_no_loss_branch                    -> BUILDSPEC §4.13 ("Remove the loss branch")
  test_snr_step_change_triggers          -> BUILDSPEC §4.13 ("SNR step-changes only")
  test_relay_completed_bypasses_moving_to_relay   -> race-condition bypass fires despite MOVING_TO_RELAY
  test_non_relay_completed_suppressed_by_moving_to_relay -> other triggers still suppressed by MOVING_TO_RELAY
"""

import sys
import os
import inspect

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for p in (_PKG_ROOT, os.path.join(_PKG_ROOT, "drone_control")):
    if p not in sys.path:
        sys.path.insert(0, p)

import drone_control.continuous_monitor as mod


# ── §4.13: no loss branch ─────────────────────────────────────────────────────

class TestNoLossBranch:
    def _code_lines(self, src: str) -> str:
        return "\n".join(
            ln for ln in src.splitlines()
            if not ln.strip().startswith("#")
        )

    def test_no_loss_branch(self):
        """
        continuous_monitor must have no loss_report subscription, no _on_loss_report(),
        no LOSS_TRIGGER_PCT env var, and no _baseline_loss_pct state.
        BUILDSPEC §4.13: 'Remove the loss branch and LOSS_TRIGGER_PCT dependency.'
        """
        src = self._code_lines(inspect.getsource(mod))

        assert "loss_report" not in src, (
            "loss_report subscription must be removed (BUILDSPEC §4.13)"
        )
        assert "_on_loss_report" not in src, (
            "_on_loss_report method must be removed (BUILDSPEC §4.13)"
        )
        assert "LOSS_TRIGGER_PCT" not in src, (
            "LOSS_TRIGGER_PCT env var must be removed (BUILDSPEC §4.13)"
        )
        assert "_baseline_loss_pct" not in src, (
            "_baseline_loss_pct state must be removed (BUILDSPEC §4.13)"
        )

    def test_no_loss_reference_anywhere(self):
        """No 'loss_pct' or 'loss_increased' strings in non-comment code."""
        src = self._code_lines(inspect.getsource(mod))
        assert "loss_pct" not in src, (
            "loss_pct must not appear in non-comment code (§4.13)"
        )
        assert "loss_increased" not in src, (
            "loss_increased trigger must be removed (§4.13)"
        )


# ── §4.13: SNR step-changes trigger ──────────────────────────────────────────

class TestSnrStepChangeTriggers:
    def _make_monitor(self):
        """Instantiate ContinuousMonitor with mocked ROS2 publisher and noop logger."""
        triggers = []

        # __new__ bypasses Node.__init__ so tests run without a live ROS2 context.
        # All instance state that __init__ would set is replicated below.
        monitor = mod.ContinuousMonitor.__new__(mod.ContinuousMonitor)
        monitor._last_trigger_ts = 0.0
        monitor._last_cap_status = None
        monitor._current_role    = None
        monitor._baseline_snr_db = None

        class FakePub:
            def publish(self, msg):
                import json
                triggers.append(json.loads(msg.data))

        class NoopLogger:
            # _fire() calls self.get_logger().info(); bypassing Node.__init__ leaves
            # _logger unset, so get_logger() raises AttributeError without this stub.
            def info(self, *a, **kw): pass
            def warning(self, *a, **kw): pass
            def debug(self, *a, **kw): pass

        monitor._pub = FakePub()
        monitor.get_logger = lambda: NoopLogger()
        return monitor, triggers

    def _signal_msg(self, snr_db: float) -> str:
        import json
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps({"follower_to_gc": {"snr_db": snr_db}})
        return msg

    def test_snr_step_change_triggers(self):
        """
        A SNR drop > SNR_TRIGGER_DB fires a reeval_trigger with reason='snr_degraded'.
        BUILDSPEC §4.13: 'SNR step-changes only.'
        """
        monitor, triggers = self._make_monitor()

        monitor._on_signal_report(self._signal_msg(20.0))
        assert len(triggers) == 0, "First reading sets baseline, must not trigger"

        monitor._last_trigger_ts = 0.0
        monitor._on_signal_report(self._signal_msg(14.0))
        assert len(triggers) == 1, (
            f"Drop of 6dB (> default 5dB threshold) must fire trigger; got {triggers}"
        )
        assert triggers[0]["reason"] == "snr_degraded"

    def test_small_snr_change_no_trigger(self):
        """SNR drop ≤ SNR_TRIGGER_DB does not fire."""
        monitor, triggers = self._make_monitor()

        monitor._on_signal_report(self._signal_msg(20.0))
        monitor._last_trigger_ts = 0.0
        monitor._on_signal_report(self._signal_msg(16.0))

        assert len(triggers) == 0, (
            f"Drop of 4dB (≤ 5dB threshold) must not trigger; got {triggers}"
        )

    def test_snr_baseline_updated_after_trigger(self):
        """After a trigger fires, the SNR baseline is updated so the next trigger is relative."""
        monitor, triggers = self._make_monitor()

        monitor._on_signal_report(self._signal_msg(20.0))
        monitor._last_trigger_ts = 0.0
        monitor._on_signal_report(self._signal_msg(14.0))
        assert len(triggers) == 1

        assert monitor._baseline_snr_db == 14.0, (
            "Baseline must update to current SNR after trigger fires"
        )

    def test_trigger_schema(self):
        """reeval_trigger has required fields: trigger_id, drone_id, timestamp, reason, delta."""
        monitor, triggers = self._make_monitor()
        monitor._on_signal_report(self._signal_msg(20.0))
        monitor._last_trigger_ts = 0.0
        monitor._on_signal_report(self._signal_msg(10.0))

        assert len(triggers) == 1
        t = triggers[0]
        for field in ("trigger_id", "drone_id", "timestamp", "reason", "delta"):
            assert field in t, f"reeval_trigger missing field: {field}"


# ── relay_completed MOVING_TO_RELAY bypass ────────────────────────────────────

class TestRelayCompletedBypass:
    """
    When current_role == MOVING_TO_RELAY, _should_suppress() returns True.
    Normally that would eat the trigger. But relay_completed gets a carve-out:
    relay_confirmed fires on the same tick relay_position_tracker detects arrival,
    before the RELAYING role update has been received. Without the bypass, the
    MOVING_TO_RELAY suppression would swallow the trigger that restarts the
    re-authorization loop.  SESSION_LOG: ASSUMPTION entry 2026-08-18.
    """

    def _make_monitor(self):
        import json
        triggers = []

        monitor = mod.ContinuousMonitor.__new__(mod.ContinuousMonitor)
        monitor._last_trigger_ts = 0.0
        monitor._last_cap_status = None
        monitor._current_role    = None
        monitor._baseline_snr_db = None

        class FakePub:
            def publish(self, msg):
                triggers.append(json.loads(msg.data))

        class NoopLogger:
            def info(self, *a, **kw): pass
            def warning(self, *a, **kw): pass
            def debug(self, *a, **kw): pass

        monitor._pub = FakePub()
        monitor.get_logger = lambda: NoopLogger()
        return monitor, triggers

    def _relay_confirmed_msg(self, status="CONFIRMED"):
        import json
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps({"assignment_id": "assign-test", "status": status})
        return msg

    def _signal_msg(self, snr_db: float):
        import json
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps({"follower_to_gc": {"snr_db": snr_db}})
        return msg

    def test_relay_completed_bypasses_moving_to_relay(self):
        """
        relay_completed fires even when current_role == MOVING_TO_RELAY.
        This is the race-condition bypass: the RELAYING role publish from
        relay_position_tracker may not have arrived yet.
        """
        monitor, triggers = self._make_monitor()
        monitor._current_role = "MOVING_TO_RELAY"

        monitor._on_relay_confirmed(self._relay_confirmed_msg())

        assert len(triggers) == 1, (
            "relay_completed must fire despite MOVING_TO_RELAY suppression "
            f"(race-condition bypass); got {triggers}"
        )
        assert triggers[0]["reason"] == "relay_completed"

    def test_non_relay_completed_suppressed_by_moving_to_relay(self):
        """
        Other trigger reasons (e.g. snr_degraded) are still suppressed when
        current_role == MOVING_TO_RELAY — the bypass is relay_completed-only.
        """
        monitor, triggers = self._make_monitor()
        monitor._current_role = "MOVING_TO_RELAY"

        # Establish baseline then drop SNR to trigger snr_degraded.
        monitor._on_signal_report(self._signal_msg(20.0))
        monitor._last_trigger_ts = 0.0
        monitor._on_signal_report(self._signal_msg(10.0))

        assert len(triggers) == 0, (
            "snr_degraded must be suppressed when MOVING_TO_RELAY; "
            f"got {triggers}"
        )

    def test_suppressed_snr_degradation_is_recovered_after_role_reverts(self):
        """
        §4.8 ordering-hole race verification.

        Scenario: drone is RELAYING; a fresh re-auth causes strategy_executor
        to publish stale MOVING_TO_RELAY for up to ~333 ms (BT tick cadence).
        continuous_monitor's `_on_signal_report` MUST NOT lose track of a
        real SNR degradation event that arrives during that window.

        Baseline BEFORE fix (2026-08-25): the baseline update at line 195 ran
        unconditionally after `_fire`, including when `_fire` was suppressed.
        A 6 dB drop during the stale window would suppress the trigger AND
        walk the baseline forward, so the degradation was silently swallowed
        forever — the next tick post-window computed drop=0dB against the
        new (lower) baseline.

        After fix: baseline is only advanced when the trigger actually fires.
        The degradation persists in the baseline delta until the role reverts,
        then the very next signal_report catches it.
        """
        monitor, triggers = self._make_monitor()

        # Setup: baseline 20 dB, drone is RELAYING (not suppressed).
        monitor._current_role = "RELAYING"
        monitor._on_signal_report(self._signal_msg(20.0))
        assert monitor._baseline_snr_db == 20.0
        assert len(triggers) == 0, "baseline-establishing tick must not trigger"

        # Race window opens: role stale-flips to MOVING_TO_RELAY.
        monitor._current_role = "MOVING_TO_RELAY"

        # SNR drops 6 dB during the window.  Threshold is 5 dB.
        monitor._last_trigger_ts = 0.0   # bypass cooldown
        monitor._on_signal_report(self._signal_msg(14.0))
        assert len(triggers) == 0, (
            "trigger correctly suppressed during MOVING_TO_RELAY window"
        )

        # BEFORE FIX: baseline walked to 14.0, permanently swallowing the drop.
        # AFTER FIX: baseline stays at 20.0 because the fire was suppressed.
        assert monitor._baseline_snr_db == 20.0, (
            f"baseline must NOT advance while trigger is suppressed — "
            f"was {monitor._baseline_snr_db}, expected 20.0.  "
            f"If this fails, §4.8 ordering race silently loses SNR events."
        )

        # Race window closes: role reverts to RELAYING.  A follow-up
        # signal_report at the SAME (persistent) degraded SNR must now trigger.
        monitor._current_role = "RELAYING"
        monitor._last_trigger_ts = 0.0
        monitor._on_signal_report(self._signal_msg(14.0))

        assert len(triggers) == 1, (
            "after role reverts, the SNR degradation MUST fire on the next tick; "
            f"got {triggers}"
        )
        assert triggers[0]["reason"] == "snr_degraded"
        # Post-trigger baseline update SHOULD happen — this is the normal path.
        assert monitor._baseline_snr_db == 14.0
