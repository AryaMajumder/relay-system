"""
test_wave5_action_nodes.py — Wave 5 gate tests for relay_bt/action_nodes.py.

TEST_PROTOCOL: §5.7  Archetype D (BT node — blackboard in → status + blackboard out)

PROVEN column:
  test_nodes_write_blackboard_only       -> BUILDSPEC §5.5 Layer 1 discipline
  test_propose_exit_writes_pending_proposal -> BUILDSPEC §4.7
  test_safety_exit_writes_alert_intent   -> BUILDSPEC §4.7 item 11
"""

import sys
import os
import importlib
import py_trees
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
_RBT = os.path.join(_PKG_ROOT, "drone_control", "relay_bt")
for p in (_PKG_ROOT, _RBT):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.relay_bt.blackboard import TimestampedBlackboard
from drone_control.relay_bt.action_nodes import (
    ProposeContinuousRelay,
    ProposeIncumbentContinuousRelay,
    ProposeChainRelay,
    ProposeLetLeaderIsolate,
    FollowerSafetyExit,
    ProposeExitRelay,
)
import drone_control.relay_bt.action_nodes as _action_mod

_S = py_trees.common.Status.SUCCESS
_F = py_trees.common.Status.FAILURE

_CFG = {
    "gc_pos": {"lat": 47.39, "lon": 8.54},
    "battery_reserve_pct": 30,
    "abort_battery_pct": 20,
    "relay_exit_quality_threshold": 0.85,
    # _model_constants() reads cruise_speed_mps and consumption_rate_pct_per_s
    # from DRONE_MODELS[drone_model_id].  Both must be present; endurance_s is
    # derived as 100.0 / consumption_rate_pct_per_s (no standalone key).
    "drone_model_id": "generic",
    "DRONE_MODELS": {
        "generic": {
            "cruise_speed_mps":           12.0,
            "consumption_rate_pct_per_s": 0.05,
        }
    },
}


class FakeClock:
    def __init__(self, t=1000.0):
        self._t = t
    def now(self):
        return self._t
    def advance(self, s):
        self._t += s


def _bb(clock=None):
    c = clock or FakeClock()
    return TimestampedBlackboard(clock=c.now)


# ═══════════════════════════════════════════════════════════════════════════════
# Layer 1 discipline
# ═══════════════════════════════════════════════════════════════════════════════

class TestLayerDiscipline:

    def test_nodes_write_blackboard_only(self):
        """
        No node in action_nodes.py may perform network I/O.
        Check: module imports no networking libraries.
        BUILDSPEC §5.5 Layer 1 discipline.

        Why source inspection instead of runtime check: runtime checks only catch
        code that actually runs.  An unused import or conditional import of a
        networking library is still a Layer 1 violation — source inspection catches it.
        """
        forbidden = {"socket", "requests", "aiohttp", "rclpy", "mqtt",
                     "paho", "zmq", "urllib"}
        module_source = open(_action_mod.__file__).read()
        for lib in forbidden:
            assert f"import {lib}" not in module_source, (
                f"action_nodes.py imports '{lib}' — Layer 1 nodes must not do network I/O"
            )


# ═══════════════════════════════════════════════════════════════════════════════
# ProposeExitRelay
# ═══════════════════════════════════════════════════════════════════════════════

class TestProposeExitRelay:

    def test_propose_exit_writes_pending_proposal(self):
        """pending_proposal.strategy == 'EXIT_RELAY'. BUILDSPEC §4.7."""
        bb = _bb()
        node = ProposeExitRelay(bb=bb, config=_CFG)
        result = node.update()
        assert result == _S
        proposal = bb.get("pending_proposal")
        assert proposal is not None, "pending_proposal not written"
        assert proposal["strategy"] == "EXIT_RELAY", (
            f"Expected 'EXIT_RELAY', got '{proposal.get('strategy')}'"
        )

    def test_propose_exit_has_reason_field(self):
        """EXIT_RELAY proposal carries a reason string."""
        bb = _bb()
        node = ProposeExitRelay(bb=bb, config=_CFG)
        node.update()
        proposal = bb.get("pending_proposal")
        assert "reason" in proposal
        assert isinstance(proposal["reason"], str)

    def test_propose_exit_reason_direct_link_recovered(self):
        """Reason = direct_link_recovered when gc_leader_direct_quality >= threshold."""
        bb = _bb()
        bb.set("gc_leader_direct_quality", 0.90)
        node = ProposeExitRelay(bb=bb, config=_CFG)
        node.update()
        assert bb.get("pending_proposal")["reason"] == "direct_link_recovered"

    def test_propose_exit_reason_position_infeasible(self):
        """Reason = position_infeasible when band_fillable is False."""
        bb = _bb()
        bb.set("band_fillable", False)
        node = ProposeExitRelay(bb=bb, config=_CFG)
        node.update()
        assert bb.get("pending_proposal")["reason"] == "position_infeasible"


# ═══════════════════════════════════════════════════════════════════════════════
# FollowerSafetyExit — alert intent
# ═══════════════════════════════════════════════════════════════════════════════

class TestFollowerSafetyExit:

    def _drone_state(self, battery=50.0, mode="OFFBOARD"):
        return {"battery_pct": battery, "flight_mode": mode,
                "position": {"lat": 47.39, "lon": 8.54, "alt": 30.0},
                "home_pos": {"lat": 47.39, "lon": 8.54, "alt": 0.0}}

    def test_safety_exit_writes_alert_intent(self):
        """
        FollowerSafetyExit writes alert_intent to blackboard.
        Must NOT publish — Layer 1 discipline (§5.5).
        BUILDSPEC §4.7 item 11.
        """
        bb = _bb()
        bb.set("drone_state", self._drone_state(battery=50.0, mode="OFFBOARD"))
        node = FollowerSafetyExit(bb=bb, config=_CFG)
        result = node.update()
        assert result == _S
        intent = bb.get("alert_intent")
        assert intent is not None, "alert_intent must be written to blackboard"
        assert intent["type"] == "FOLLOWER_SAFETY_EXIT", (
            f"Expected type 'FOLLOWER_SAFETY_EXIT', got: {intent.get('type')}"
        )
        assert "reason" in intent
        assert "timestamp" in intent

    def test_safety_exit_alert_intent_on_battery_critical(self):
        """alert_intent written when battery < abort_battery_pct."""
        cfg = dict(_CFG, abort_battery_pct=30)
        bb = _bb()
        bb.set("drone_state", self._drone_state(battery=15.0, mode="OFFBOARD"))
        node = FollowerSafetyExit(bb=bb, config=cfg)
        node.update()
        intent = bb.get("alert_intent")
        assert intent is not None
        assert "battery_critical" in intent["reason"]

    def test_safety_exit_alert_intent_on_offboard_loss(self):
        """alert_intent written when flight_mode != OFFBOARD."""
        bb = _bb()
        bb.set("drone_state", self._drone_state(battery=80.0, mode="POSCTL"))
        node = FollowerSafetyExit(bb=bb, config=_CFG)
        node.update()
        intent = bb.get("alert_intent")
        assert intent is not None
        assert "offboard_unrecoverable" in intent["reason"]

    def test_safety_exit_alert_intent_on_lost_fc(self):
        """alert_intent written on LOST_FC path; pending_command NOT written."""
        bb = _bb()
        bb.set("drone_state", self._drone_state())
        bb.set("lost_fc_intent", True)
        node = FollowerSafetyExit(bb=bb, config=_CFG)
        result = node.update()
        assert result == _S
        intent = bb.get("alert_intent")
        assert intent is not None, "alert_intent must be written even on LOST_FC path"
        assert intent["reason"] == "fcu_telemetry_lost"
        # On LOST_FC, pending_command must NOT be written (FCU is unreachable)
        assert bb.get("pending_command") is None, (
            "pending_command must not be written when FCU telemetry is lost"
        )

    def test_safety_exit_writes_rtl_command_when_fc_alive(self):
        """When FCU is alive (no lost_fc_intent), pending_command RTL is written."""
        bb = _bb()
        bb.set("drone_state", self._drone_state(battery=80.0, mode="POSCTL"))
        node = FollowerSafetyExit(bb=bb, config=_CFG)
        node.update()
        cmd = bb.get("pending_command")
        assert cmd is not None
        assert cmd["command"] == "RTL"

    # ── SESSION_LOG [2026-08-26] latch-misattribution defense in depth ────────

    def test_alert_intent_carries_actual_reason_and_latch_flag(self):
        """
        SAFETY: even when lost_fc_intent latches the reported `reason` to
        "fcu_telemetry_lost", the alert_intent payload must also carry the
        underlying trigger cause in `actual_reason` and the latch state in
        `fcu_latched`.  RDA's ring buffer preserves these; operators can
        see when a battery-critical or offboard-loss exit was masked by a
        stale FCU latch.
        """
        cfg = dict(_CFG, abort_battery_pct=30)
        bb  = _bb()
        # Battery-critical trigger with stale FCU latch → reported reason
        # is fcu_telemetry_lost, actual_reason retains battery_critical.
        bb.set("drone_state", self._drone_state(battery=10.0, mode="OFFBOARD"))
        bb.set("lost_fc_intent", True)
        FollowerSafetyExit(bb=bb, config=cfg).update()

        intent = bb.get("alert_intent")
        assert intent is not None
        assert intent["reason"]        == "fcu_telemetry_lost"
        assert intent["actual_reason"] == "battery_critical:10pct", (
            "SAFETY: alert_intent must expose the actual trigger when the "
            "latch masks the reported reason.  Otherwise every exit under a "
            "stale FCU latch appears as fcu_telemetry_lost with no way for "
            "the GC to see the real cause."
        )
        assert intent["fcu_latched"] is True

    def test_alert_intent_actual_reason_matches_reason_when_not_latched(self):
        """When not latched, actual_reason equals reason and fcu_latched is False."""
        cfg = dict(_CFG, abort_battery_pct=30)
        bb  = _bb()
        bb.set("drone_state", self._drone_state(battery=10.0, mode="OFFBOARD"))
        FollowerSafetyExit(bb=bb, config=cfg).update()
        intent = bb.get("alert_intent")
        assert intent["reason"]        == "battery_critical:10pct"
        assert intent["actual_reason"] == "battery_critical:10pct"
        assert intent["fcu_latched"]   is False

    def test_follower_safety_exit_rtl_restored_after_latch_clears(self):
        """
        End-to-end: latch set, RTL suppressed; latch cleared, RTL resumes.
        Mirrors what FcuTelemetryFresh recovery does at BT tick boundary
        after this bug's fix.  Previously the latch never cleared, so this
        test would fail on the second FollowerSafetyExit invocation.
        """
        cfg = dict(_CFG, abort_battery_pct=30)
        bb  = _bb()
        bb.set("drone_state", self._drone_state(battery=10.0, mode="OFFBOARD"))

        # Phase 1: latched → no RTL, alert only.
        bb.set("lost_fc_intent", True)
        FollowerSafetyExit(bb=bb, config=cfg).update()
        assert bb.get("pending_command") is None, (
            "latched: RTL suppressed (PX4 failsafe owns airframe)"
        )
        assert bb.get("alert_intent")["reason"] == "fcu_telemetry_lost"

        # Phase 2: latch clears (simulate FcuTelemetryFresh recovery).
        bb.set("lost_fc_intent", False)
        FollowerSafetyExit(bb=bb, config=cfg).update()
        cmd = bb.get("pending_command")
        assert cmd is not None, (
            "SAFETY: RTL must be commanded after latch clears.  If this "
            "fails, the safety-exit-after-FCU-recovery path is broken — "
            "SESSION_LOG [2026-08-26]."
        )
        assert cmd["command"] == "RTL"
        assert "battery_critical" in cmd["reason"]

    def test_follower_safety_exit_logs_actual_reason_when_latched(self, capsys):
        """
        LOUD LOG: when RTL is suppressed by the latch, log.error must name
        the actual trigger so the misattribution is impossible to miss in
        the journal.  Pre-2026-08-26, suppression was completely silent.

        Uses capsys (captures stderr directly) rather than caplog because the
        action_nodes module logger writes to stderr via root propagation
        set up by pytest's default logging config.
        """
        import logging
        # Ensure log records reach stderr for capsys to see.
        logging.basicConfig(level=logging.DEBUG, force=False)
        cfg = dict(_CFG, abort_battery_pct=30)
        bb  = _bb()
        bb.set("drone_state", self._drone_state(battery=10.0, mode="OFFBOARD"))
        bb.set("lost_fc_intent", True)

        FollowerSafetyExit(bb=bb, config=cfg).update()

        captured = capsys.readouterr()
        combined = captured.err + captured.out
        assert "SUPPRESSED" in combined and "battery_critical" in combined, (
            "SAFETY: latch-suppressed RTL must log LOUDLY naming the actual "
            f"trigger so the misattribution is visible.  Log output was:\n"
            f"stderr: {captured.err!r}\nstdout: {captured.out!r}"
        )


# ═══════════════════════════════════════════════════════════════════════════════
# ProposeContinuousRelay — basic smoke
# ═══════════════════════════════════════════════════════════════════════════════

class TestProposeContinuousRelay:

    def test_propose_continuous_writes_strategy(self):
        """pending_proposal.strategy == 'CONTINUOUS_RELAY'."""
        bb = _bb()
        bb.set("R_target", {"lat": 47.395, "lon": 8.545, "alt": 40.0})
        bb.set("drone_state", {
            "drone_id": "drone-02",
            "battery_pct": 80.0,
            "position": {"lat": 47.39, "lon": 8.54, "alt": 30.0},
            "home_pos": {"lat": 47.39, "lon": 8.54, "alt": 0.0},
        })
        node = ProposeContinuousRelay(bb=bb, config=_CFG)
        result = node.update()
        assert result == _S
        proposal = bb.get("pending_proposal")
        assert proposal["strategy"] == "CONTINUOUS_RELAY"

    def test_propose_continuous_no_r_target(self):
        """Without R_target on blackboard → FAILURE."""
        bb = _bb()
        node = ProposeContinuousRelay(bb=bb, config=_CFG)
        assert node.update() == _F


# ═══════════════════════════════════════════════════════════════════════════════
# ProposeLetLeaderIsolate — always succeeds
# ═══════════════════════════════════════════════════════════════════════════════

class TestProposeLetLeaderIsolate:

    def test_propose_let_isolate_always_succeeds(self):
        """Fallback — always returns SUCCESS and writes LET_LEADER_ISOLATE."""
        bb = _bb()
        bb.set("relay_tasking_received", {"tasking_id": "t1", "leader_id": "drone-01"})
        node = ProposeLetLeaderIsolate(bb=bb, config=_CFG)
        result = node.update()
        assert result == _S
        proposal = bb.get("pending_proposal")
        assert proposal["strategy"] == "LET_LEADER_ISOLATE"


# ═══════════════════════════════════════════════════════════════════════════════
# ProposeIncumbentContinuousRelay — RELAYING-branch terminal bid
# ═══════════════════════════════════════════════════════════════════════════════


class TestProposeIncumbentContinuousRelay:
    """
    SESSION_LOG 2026-10-06 DEVIATION: the pre-existing ARBITER_SCAN
    terminal was _succeed("CONTINUE") — the incumbent submitted nothing
    on reauth rounds. The new terminal bids CONTINUOUS_RELAY for the live
    current_relay_target so a reauth round can renew the incumbent.
    """

    def _bb(self):
        bb = _bb()
        bb.set("drone_state", {
            "drone_id":     "drone-02",
            "battery_pct":  80.0,
            "position":     {"lat": 47.395, "lon": 8.545, "alt": 40.0},
            "home_pos":     {"lat": 47.390, "lon": 8.540, "alt":  0.0},
        })
        return bb

    def test_incumbent_bids_continuous_relay_for_live_target(self):
        """Node writes CONTINUOUS_RELAY proposal for current_relay_target."""
        bb = self._bb()
        R = {"lat": 47.400, "lon": 8.550, "alt_m": 50.0}
        bb.set("current_relay_target", R)
        node = ProposeIncumbentContinuousRelay(bb=bb, config=_CFG)
        assert node.update() == _S
        proposal = bb.get("pending_proposal")
        assert proposal is not None, "incumbent must write a pending_proposal"
        assert proposal["strategy"] == "CONTINUOUS_RELAY"
        assert proposal["relay_position"] == R, (
            "incumbent must bid for the LIVE current_relay_target, verbatim"
        )
        assert proposal["trigger"] == "incumbent"
        assert proposal["reason"]  == "incumbent_bid"

    def test_eta_s_derived_from_distance_to_target(self):
        """eta_seconds in cost is non-zero when follower is away from target."""
        bb = self._bb()
        bb.set("current_relay_target", {"lat": 47.500, "lon": 8.650, "alt_m": 50.0})
        node = ProposeIncumbentContinuousRelay(bb=bb, config=_CFG)
        node.update()
        proposal = bb.get("pending_proposal")
        assert proposal["cost"]["eta_seconds"] > 0.0, (
            "incumbent eta must reflect follower-to-target distance; got "
            f"{proposal['cost']['eta_seconds']}"
        )

    def test_succeeds_without_current_relay_target(self):
        """
        If current_relay_target is unset the node must still SUCCESS so
        ARBITER_SCAN's terminal child does not fail the RELAYING_BRANCH
        Sequence. No pending_proposal is written in that case.
        """
        bb = self._bb()
        node = ProposeIncumbentContinuousRelay(bb=bb, config=_CFG)
        assert node.update() == _S
        assert bb.get("pending_proposal") is None, (
            "no bid should be written when current_relay_target is absent"
        )

    def test_new_authorized_target_updates_bid(self):
        """When current_relay_target changes (new auth), next tick bids new target."""
        bb = self._bb()
        bb.set("current_relay_target", {"lat": 47.400, "lon": 8.550, "alt_m": 50.0})
        node = ProposeIncumbentContinuousRelay(bb=bb, config=_CFG)
        node.update()
        assert bb.get("pending_proposal")["relay_position"]["lat"] == 47.400

        # Simulate the capability_assessor applying a fresh relay_assignment.
        bb.set("current_relay_target", {"lat": 47.410, "lon": 8.560, "alt_m": 50.0})
        node.update()
        assert bb.get("pending_proposal")["relay_position"]["lat"] == 47.410, (
            "incumbent bid must track live current_relay_target when auth updates"
        )


# ═══════════════════════════════════════════════════════════════════════════════
# ProposeLetLeaderIsolate — decline reason attribution (2026-10-06)
# ═══════════════════════════════════════════════════════════════════════════════


class TestLetLeaderIsolateReason:
    """
    SESSION_LOG 2026-10-06: ProposeLetLeaderIsolate must publish the
    specific decline reason into the proposal so it can flow through
    relay_strategy_evaluator → RDA's log.
      - CapFail  → reason = bb["last_capability_check"]
      - NoStrategy → reason = "no_strategy"
    """

    def _node(self, mode):
        node = ProposeLetLeaderIsolate(bb=_bb(), config=_CFG)
        node.name = f"ProposeLetLeaderIsolate({mode})"
        return node

    def test_capfail_reason_is_last_capability_check_name(self):
        """CapFail reads bb['last_capability_check'] as the decline reason."""
        node = self._node("CapFail")
        node.bb.set("last_capability_check", "LeaderReachabilityFresh")
        node.update()
        p = node.bb.get("pending_proposal")
        assert p["strategy"] == "LET_LEADER_ISOLATE"
        assert p["trigger"] == "CapFail"
        assert p["reason"]  == "LeaderReachabilityFresh"

    def test_capfail_fallback_reason_when_bb_missing(self):
        """CapFail with no last_capability_check uses an explicit placeholder."""
        node = self._node("CapFail")
        node.update()
        assert node.bb.get("pending_proposal")["reason"] == "unknown_capability_check"

    def test_nostrategy_reason_is_literal_no_strategy(self):
        """NoStrategy ignores bb state; reason is always 'no_strategy'."""
        node = self._node("NoStrategy")
        node.bb.set("last_capability_check", "GeometryFeasible")  # must be ignored
        node.update()
        p = node.bb.get("pending_proposal")
        assert p["trigger"] == "NoStrategy"
        assert p["reason"]  == "no_strategy"

    def test_condition_node_set_writes_last_capability_check(self):
        """ConditionNodeBase._set must record self.name to bb on every tick."""
        from drone_control.relay_bt.condition_nodes import ConditionNodeBase
        bb = _bb()
        class _Dummy(ConditionNodeBase):
            def update(self): return self._set(_S, "ok")
        dummy = _Dummy(bb=bb, config=_CFG, name="FakeCheck")
        dummy.update()
        assert bb.get("last_capability_check") == "FakeCheck"
