"""
test_wave5_condition_nodes.py — Wave 5 gate tests for relay_bt/condition_nodes.py.

TEST_PROTOCOL: §5.6  Archetype D (BT node — blackboard in → status + blackboard out)

PROVEN column:
  test_exactly_nine_gates                 -> BUILDSPEC §4.6
  test_no_loss_gate                       -> BUILDSPEC §4.6 removal
  test_gate8_inside_radius_valid_timer    -> BUILDSPEC §4.6
  test_gate8_outside_radius               -> BUILDSPEC §4.6
  test_gate8_timer_expired_inside_radius  -> BUILDSPEC §5.4 (OR not AND)
  test_gate8_both_fail                    -> BUILDSPEC §5.4
  test_gate8_failure_writes_no_movement   -> BUILDSPEC §4.6 (request, not move)
  test_reauth_timeout_not_outstanding     -> BUILDSPEC §4.6
  test_reauth_timeout_within_window       -> BUILDSPEC §4.6
  test_reauth_timeout_expired             -> BUILDSPEC §4.6
  test_fcap_never_seen_vs_stale_distinct  -> BUILDSPEC §4.6 hard rule
  test_fcap_boot_grace_suppresses_never_seen -> BUILDSPEC §4.6
  test_fcap_no_self_suppression           -> BUILDSPEC §4.6 hard rule
"""

import sys
import os
import math
import pytest
import py_trees

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
_RBT = os.path.join(_PKG_ROOT, "drone_control", "relay_bt")
for p in (_PKG_ROOT, _RBT):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.relay_bt.blackboard import TimestampedBlackboard
import drone_control.relay_bt.condition_nodes as cn
from drone_control.relay_bt.condition_nodes import (
    MAINTENANCE_GATES,
    RelayActuallyImproved,
    ReauthResponseTimedOut,
    LeaderReachabilityFresh,
    FcuTelemetryFresh,
    BatteryStillSufficientToRelay,
    OffboardModeHeld,
    PositionServiceable,
    RfLinkTelemetryFresh,
    RelayStillNeeded,
    RelayLinkAdequate,
    GpsHealthy,
)

_S = py_trees.common.Status.SUCCESS
_F = py_trees.common.Status.FAILURE


class FakeClock:
    """
    Controllable time source (TEST_PROTOCOL §3.3 — no test may sleep).
    Condition nodes receive clock=clock.now so they call clock.now() on each update().
    Tests call clock.advance() to simulate time passing without any real delay.
    """
    def __init__(self, t=1000.0):
        self._t = t
    def now(self):
        return self._t
    def advance(self, seconds: float):
        self._t += seconds


def _bb(clock=None):
    c = clock or FakeClock()
    return TimestampedBlackboard(clock=c.now)


_CFG = {
    "gc_pos": {"lat": 47.39, "lon": 8.54},
    "tolerance_radius_m": 10.0,
    "reauth_response_timeout_s": 120,
    "boot_grace_window_s": 600,
    "radio_health_max_age_s": 10.0,
}


# ═══════════════════════════════════════════════════════════════════════════════
# Gate registry
# ═══════════════════════════════════════════════════════════════════════════════

class TestGateRegistry:

    def test_exactly_nine_gates(self):
        """BUILDSPEC §4.6: gate list has exactly nine entries."""
        expected = [
            FcuTelemetryFresh,
            BatteryStillSufficientToRelay,
            OffboardModeHeld,
            PositionServiceable,
            RfLinkTelemetryFresh,
            RelayStillNeeded,
            RelayLinkAdequate,
            RelayActuallyImproved,
            GpsHealthy,
        ]
        assert len(MAINTENANCE_GATES) == 9, (
            f"Expected 9 maintenance gates, got {len(MAINTENANCE_GATES)}: {MAINTENANCE_GATES}"
        )
        assert MAINTENANCE_GATES == expected, (
            f"Gate order/content mismatch:\n  got: {MAINTENANCE_GATES}\n  exp: {expected}"
        )

    def test_no_loss_gate(self):
        """GCLinkLossAcceptable must not exist in the module. BUILDSPEC §4.6 removal."""
        assert not hasattr(cn, "GCLinkLossAcceptable"), (
            "GCLinkLossAcceptable still exists — must be removed per BUILDSPEC §4.6"
        )


# ═══════════════════════════════════════════════════════════════════════════════
# Gate 8: RelayActuallyImproved
# ═══════════════════════════════════════════════════════════════════════════════

class TestGate8RelayActuallyImproved:

    def _make(self, clock):
        bb = _bb(clock)
        node = RelayActuallyImproved(bb=bb, config=_CFG, clock=clock.now)
        return bb, node

    def _near_pos(self):
        """R_target and current_relay_target 5 m apart — inside 10 m tolerance."""
        return (
            {"lat": 47.39000, "lon": 8.54000, "alt": 0.0},   # current_relay_target
            {"lat": 47.39003, "lon": 8.54000, "alt": 0.0},   # R_target (~3 m north)
        )

    def _far_pos(self):
        """R_target 200 m from current_relay_target — outside 10 m tolerance."""
        return (
            {"lat": 47.39000, "lon": 8.54000, "alt": 0.0},
            {"lat": 47.39180, "lon": 8.54000, "alt": 0.0},   # ~200 m north
        )

    def test_gate8_inside_radius_valid_timer(self):
        """inside_radius=True AND timer_ok=True → SUCCESS. BUILDSPEC §4.6."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        current_target, r_target_now = self._near_pos()
        bb.set("current_relay_target", current_target)
        bb.set("authorization_valid_until", clock.now() + 1800.0)
        bb.set("tolerance_radius_m", 10.0)
        bb.set("R_target", r_target_now)
        assert node.update() == _S

    def test_gate8_outside_radius(self):
        """inside_radius=False → FAILURE regardless of timer. BUILDSPEC §4.6."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        current_target, r_target_now = self._far_pos()
        bb.set("current_relay_target", current_target)
        bb.set("authorization_valid_until", clock.now() + 1800.0)
        bb.set("tolerance_radius_m", 10.0)
        bb.set("R_target", r_target_now)
        assert node.update() == _F

    def test_gate8_timer_expired_inside_radius(self):
        """
        timer_ok=False → FAILURE even when inside radius. BUILDSPEC §5.4 (OR not AND).
        Gate 8 fails if EITHER the radius is wrong OR the timer expired —
        not only when both fail.  This prevents perpetual re-auth avoidance
        by an authorization that has expired but whose relay_position is nearby.
        """
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        current_target, r_target_now = self._near_pos()
        bb.set("current_relay_target", current_target)
        bb.set("authorization_valid_until", clock.now() - 1.0)   # expired 1 s ago
        bb.set("tolerance_radius_m", 10.0)
        bb.set("R_target", r_target_now)
        assert node.update() == _F

    def test_gate8_both_fail(self):
        """Both outside radius AND timer expired → FAILURE. BUILDSPEC §5.4."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        current_target, r_target_now = self._far_pos()
        bb.set("current_relay_target", current_target)
        bb.set("authorization_valid_until", clock.now() - 60.0)
        bb.set("tolerance_radius_m", 10.0)
        bb.set("R_target", r_target_now)
        assert node.update() == _F

    def test_gate8_failure_writes_no_movement(self):
        """
        On FAILURE, current_relay_target is unchanged.
        BUILDSPEC §4.6: reauth is a REQUEST, not permission to move.
        """
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        current_target, r_target_now = self._far_pos()
        saved_target = dict(current_target)
        bb.set("current_relay_target", current_target)
        bb.set("authorization_valid_until", clock.now() + 1800.0)
        bb.set("tolerance_radius_m", 10.0)
        bb.set("R_target", r_target_now)

        result = node.update()
        assert result == _F
        # current_relay_target must be identical to what was set
        assert bb.get("current_relay_target") == saved_target, (
            "Gate 8 FAILURE must not change current_relay_target"
        )
        # reauth_requested_at must be written
        assert bb.get("reauth_requested_at") is not None, (
            "Gate 8 FAILURE must write reauth_requested_at"
        )

    def test_gate8_no_assignment_passes(self):
        """No relay assignment yet → SUCCESS (nothing to validate)."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        # current_relay_target and authorization_valid_until not set
        assert node.update() == _S

    def test_gate8_writes_reauth_at_on_failure(self):
        """reauth_requested_at written with current clock value on FAILURE."""
        clock = FakeClock(t=5000.0)
        bb, node = self._make(clock)
        current_target, r_target_now = self._far_pos()
        bb.set("current_relay_target", current_target)
        bb.set("authorization_valid_until", clock.now() + 1800.0)
        bb.set("tolerance_radius_m", 10.0)
        bb.set("R_target", r_target_now)
        node.update()
        assert math.isclose(bb.get("reauth_requested_at"), 5000.0, abs_tol=1e-6)


# ═══════════════════════════════════════════════════════════════════════════════
# ReauthResponseTimedOut
# ═══════════════════════════════════════════════════════════════════════════════

class TestReauthResponseTimedOut:

    def _make(self, clock):
        bb = _bb(clock)
        node = ReauthResponseTimedOut(bb=bb, config=_CFG, clock=clock.now)
        return bb, node

    def test_reauth_timeout_not_outstanding(self):
        """No pending reauth → SUCCESS. BUILDSPEC §4.6."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        # reauth_requested_at not set
        assert node.update() == _S

    def test_reauth_timeout_within_window(self):
        """119s elapsed (< 120s timeout) → SUCCESS. BUILDSPEC §4.6."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        bb.set("reauth_requested_at", clock.now())
        clock.advance(119.0)
        assert node.update() == _S

    def test_reauth_timeout_expired(self):
        """121s elapsed (> 120s timeout) → FAILURE. BUILDSPEC §4.6."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        bb.set("reauth_requested_at", clock.now())
        clock.advance(121.0)
        assert node.update() == _F

    def test_reauth_timeout_exactly_at_boundary(self):
        """119.999s → SUCCESS; 120.001s → FAILURE."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        bb.set("reauth_requested_at", clock.now())
        clock.advance(119.999)
        assert node.update() == _S
        clock.advance(0.002)   # now 120.001s
        assert node.update() == _F

    def test_reauth_none_after_clearance(self):
        """After reauth_requested_at cleared, gate returns SUCCESS again."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        bb.set("reauth_requested_at", clock.now())
        clock.advance(50.0)
        assert node.update() == _S   # still in window
        # Simulate: new authorization arrived, capability_assessor clears the key
        bb.set("reauth_requested_at", None)
        # With None stored, gate should treat as not outstanding
        # (None stored is same as never-set for this check)
        result = node.update()
        # reauth_requested_at == None → treated as not outstanding
        assert result == _S


# ═══════════════════════════════════════════════════════════════════════════════
# LeaderReachabilityFresh (F_cap addition)
# ═══════════════════════════════════════════════════════════════════════════════

class TestLeaderReachabilityFresh:

    def _make(self, clock):
        bb = TimestampedBlackboard(clock=clock.now)
        node = LeaderReachabilityFresh(bb=bb, config=_CFG, clock=clock.now)
        return bb, node

    def test_fcap_fresh_leader_radio_health(self):
        """Fresh leader_radio_health → SUCCESS."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        bb.set("leader_radio_health", {"snr_db": 20.0, "hop": "gc_to_leader"})
        # Within max_age (10s)
        clock.advance(5.0)
        assert node.update() == _S

    def test_fcap_never_seen_outside_grace(self):
        """Never-seen outside boot grace → FAILURE."""
        clock = FakeClock(t=1000.0)
        cfg = dict(_CFG, boot_grace_window_s=5)   # very short grace
        bb = TimestampedBlackboard(clock=clock.now)
        node = LeaderReachabilityFresh(bb=bb, config=cfg, clock=clock.now)
        clock.advance(10.0)   # past 5s grace
        assert node.update() == _F

    def test_fcap_stale_leader_radio_health(self):
        """Stale leader_radio_health → FAILURE."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock)
        bb.set("leader_radio_health", {"snr_db": 15.0})
        clock.advance(15.0)   # past 10s max_age
        assert node.update() == _F

    def test_fcap_never_seen_vs_stale_distinct(self):
        """
        Never-seen and stale produce DIFFERENT feedback messages.
        BUILDSPEC §4.6 hard rule: never collapse them.
        """
        clock = FakeClock(t=1000.0)
        cfg = dict(_CFG, boot_grace_window_s=5)

        # Never-seen node (outside grace)
        bb1 = TimestampedBlackboard(clock=clock.now)
        node1 = LeaderReachabilityFresh(bb=bb1, config=cfg, clock=clock.now)
        clock.advance(10.0)   # past grace
        node1.update()
        never_seen_msg = node1.feedback_message

        # Stale node
        clock2 = FakeClock(t=2000.0)
        bb2 = TimestampedBlackboard(clock=clock2.now)
        node2 = LeaderReachabilityFresh(bb=bb2, config=cfg, clock=clock2.now)
        bb2.set("leader_radio_health", {"snr_db": 15.0})
        clock2.advance(15.0)
        node2.update()
        stale_msg = node2.feedback_message

        assert never_seen_msg != stale_msg, (
            f"never_seen and stale must produce distinct messages; "
            f"both returned: {never_seen_msg!r}"
        )
        assert "never_seen" in never_seen_msg, f"Missing 'never_seen' in: {never_seen_msg}"
        assert "stale" in stale_msg, f"Missing 'stale' in: {stale_msg}"

    def test_fcap_boot_grace_suppresses_never_seen(self):
        """
        Within boot_grace_window_s, never-seen → SUCCESS (suppressed).
        BUILDSPEC §4.6.
        """
        clock = FakeClock(t=1000.0)
        cfg = dict(_CFG, boot_grace_window_s=600)
        bb = TimestampedBlackboard(clock=clock.now)
        node = LeaderReachabilityFresh(bb=bb, config=cfg, clock=clock.now)
        # Node just constructed — well within 600s grace
        clock.advance(1.0)
        result = node.update()
        assert result == _S, (
            f"Within boot grace, never-seen should be SUCCESS, got {result.name}: "
            f"{node.feedback_message}"
        )

    def test_fcap_no_self_suppression(self):
        """
        Same stale condition 5 times → 5 FAILURE results.
        BUILDSPEC §4.6 hard rule: F_cap never dedups or suppresses its own output.
        """
        clock = FakeClock(t=1000.0)
        cfg = dict(_CFG, boot_grace_window_s=5)
        bb = TimestampedBlackboard(clock=clock.now)
        node = LeaderReachabilityFresh(bb=bb, config=cfg, clock=clock.now)
        # Write stale data
        bb.set("leader_radio_health", {"snr_db": 15.0})
        clock.advance(20.0)   # well past grace and max_age

        results = [node.update() for _ in range(5)]
        assert all(r == _F for r in results), (
            f"All 5 ticks should be FAILURE, got: {[r.name for r in results]}"
        )

    def test_fcap_never_seen_suppressed_at_grace_boundary(self):
        """At exactly grace boundary - 1s → still suppressed (SUCCESS)."""
        clock = FakeClock(t=1000.0)
        cfg = dict(_CFG, boot_grace_window_s=60)
        bb = TimestampedBlackboard(clock=clock.now)
        node = LeaderReachabilityFresh(bb=bb, config=cfg, clock=clock.now)
        clock.advance(59.0)   # 1s before grace expires
        assert node.update() == _S

    def test_fcap_never_seen_fires_after_grace_expires(self):
        """After grace expires, never-seen → FAILURE."""
        clock = FakeClock(t=1000.0)
        cfg = dict(_CFG, boot_grace_window_s=60)
        bb = TimestampedBlackboard(clock=clock.now)
        node = LeaderReachabilityFresh(bb=bb, config=cfg, clock=clock.now)
        clock.advance(61.0)   # past grace
        assert node.update() == _F


# ═══════════════════════════════════════════════════════════════════════════════
# FcuTelemetryFresh — lost_fc_intent latching + recovery
# SESSION_LOG [2026-08-26] CRITICAL DISCOVERY
# ═══════════════════════════════════════════════════════════════════════════════

class TestFcuTelemetryFreshLatch:
    """
    Locks the invariant that lost_fc_intent behaves as a *transient* latch,
    not a permanent one.  Pre-fix, the latch was set on stale but NEVER
    cleared, permanently disabling FollowerSafetyExit's RTL command for
    the process lifetime.  A 2-second DDS hiccup was sufficient to trip it.
    """

    def _make(self, clock, max_age=3.0):
        bb  = TimestampedBlackboard(clock=clock.now)
        cfg = dict(_CFG, fcu_telemetry_max_age_s=max_age)
        return bb, FcuTelemetryFresh(bb=bb, config=cfg, clock=clock.now)

    def test_fcu_telemetry_stale_sets_latch(self):
        """Baseline behavior: stale drone_state sets lost_fc_intent."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock, max_age=3.0)
        # Write stale drone_state (5s old, threshold 3s).
        bb.set("drone_state", {"battery_pct": 80, "flight_mode": "OFFBOARD"})
        clock.advance(5.0)
        assert node.update() == _F
        assert bb.get("lost_fc_intent") is True, (
            "FcuTelemetryFresh must set lost_fc_intent on stale telemetry"
        )

    def test_fcu_latch_persists_across_ticks_while_stale(self):
        """Sustained loss: latch stays True across many stale ticks."""
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock, max_age=3.0)
        bb.set("drone_state", {"battery_pct": 80, "flight_mode": "OFFBOARD"})
        clock.advance(5.0)
        for _ in range(10):
            assert node.update() == _F
            assert bb.get("lost_fc_intent") is True
            clock.advance(1.0)   # each tick after the first: still stale (5+k seconds)

    def test_fcu_telemetry_fresh_clears_latch_on_recovery(self):
        """
        SAFETY-CRITICAL invariant: once drone_state becomes fresh again,
        lost_fc_intent MUST clear.  Pre-fix (2026-08-26 CRITICAL DISCOVERY):
        latch persisted forever, silently disabling RTL from any subsequent
        FollowerSafetyExit call.
        """
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock, max_age=3.0)

        # Step 1: stale → set latch.
        bb.set("drone_state", {"battery_pct": 80, "flight_mode": "OFFBOARD"})
        clock.advance(5.0)
        assert node.update() == _F
        assert bb.get("lost_fc_intent") is True

        # Step 2: fresh telemetry arrives.  Simulate by resetting the BB
        # timestamp on drone_state via a fresh write at current time.
        bb.set("drone_state", {"battery_pct": 80, "flight_mode": "OFFBOARD"})
        # No clock.advance() — write is at "now", so age=0, fresh.

        assert node.update() == _S
        assert bb.get("lost_fc_intent") is False, (
            "SAFETY: lost_fc_intent must be cleared when telemetry recovers.  "
            "If this assertion fails, RTL will be permanently disabled after "
            "any transient telemetry loss — SESSION_LOG [2026-08-26]."
        )

    def test_fcu_latch_recovers_after_transient_hiccup(self):
        """
        End-to-end scenario: 2-second DDS hiccup mid-flight.
        - Fresh for 10s → latch not set.
        - Stale for 2s (hiccup) → latch set.
        - Fresh recovers → latch clears.
        - Subsequent stale-then-fresh cycles work independently.
        """
        clock = FakeClock(t=1000.0)
        bb, node = self._make(clock, max_age=3.0)

        # Phase 1: fresh
        bb.set("drone_state", {"battery_pct": 80, "flight_mode": "OFFBOARD"})
        assert node.update() == _S
        assert bb.get("lost_fc_intent") in (None, False)

        # Phase 2: hiccup — no update to drone_state for 5s
        clock.advance(5.0)
        assert node.update() == _F
        assert bb.get("lost_fc_intent") is True

        # Phase 3: telemetry recovers, latch clears
        bb.set("drone_state", {"battery_pct": 80, "flight_mode": "OFFBOARD"})
        assert node.update() == _S
        assert bb.get("lost_fc_intent") is False

        # Phase 4: another hiccup — latch fires again, then clears again
        clock.advance(5.0)
        assert node.update() == _F
        assert bb.get("lost_fc_intent") is True
        bb.set("drone_state", {"battery_pct": 80, "flight_mode": "OFFBOARD"})
        assert node.update() == _S
        assert bb.get("lost_fc_intent") is False
