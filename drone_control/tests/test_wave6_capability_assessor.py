"""
test_wave6_capability_assessor.py — Wave 6 gate tests for capability_assessor.py.

TEST_PROTOCOL: §5.8  Archetype E (stateful decision node)

PROVEN column:
  test_capability_report_every_tick          -> BUILDSPEC §4.11 (unconditional publish)
  test_drains_pending_command / test_drains_alert_intent / test_drains_pending_proposal / test_empty_slots_produce_no_publish -> BUILDSPEC §4.11 (drain-and-clear per slot)
  test_no_severity_computation              -> BUILDSPEC §4.11 (F_radio moved out v6.4)
  test_subscription_lifecycle_start        -> BUILDSPEC §4.11 item 18
  test_subscription_lifecycle_stop_washout -> BUILDSPEC §4.11 (F_reject/F_decline path)
  test_subscription_lifecycle_stop_exit    -> BUILDSPEC §4.11 (EXIT_RELAY path)
  test_writes_radius_and_timer_keys        -> BUILDSPEC §6
  test_reauth_request_published_once       -> gap fix (reauth_request one-shot publish)
  test_reauth_request_not_repeated         -> gap fix (one-shot flag suppresses repeat)
  test_reauth_request_cleared_on_new_assignment -> gap fix (flag reset on relay_assignment)
  test_snr_degraded_publishes_reauth / test_relay_completed_publishes_reauth / test_reason_carries_trigger_reason / test_drone_id_in_payload / test_not_gated_by_reauth_request_sent_flag -> DEVIATION: reeval_trigger subscriber (SNR fast-path)

Harness: _TestableAssessorCore — pure-Python test double for CapabilityAssessor.
  Mirrors _tick() and _build_report() logic; uses injectable capture lists instead
  of ROS2 publishers.  No rclpy.init() or DDS required.
  Pure functions _drain() and _apply_relay_assignment() imported and called directly.
"""

import os
import sys
import time
import py_trees
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for _p in (_PKG_ROOT, os.path.join(_PKG_ROOT, "drone_control")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from drone_control.relay_bt.blackboard import TimestampedBlackboard
from drone_control.relay_bt.tree_builder import build_relay_decision_tree, walk_tree
from drone_control.relay_bt.condition_nodes import ConditionNodeBase, DataFreshness
from drone_control.config.demo_config import DEMO_CONFIG
from drone_control.capability_assessor import (
    _drain,
    _apply_relay_assignment,
    _status_to_string,
)
import drone_control.capability_assessor as _ca_mod


# ── Shared test config ────────────────────────────────────────────────────────

_CFG = {
    **DEMO_CONFIG,
    # debounce_n=1: production uses N=3 to debounce gate transitions.  Setting to 1
    # means a single tick triggers the gate — prevents tests from needing 3+ ticks.
    "debounce_n": 1,
    # min_gps_fix_type=0: harness has no real GPS; accept fix_type=0 (no fix) so
    # GPSFixAdequate doesn't block other gate assertions.
    "min_gps_fix_type": 0,
}


# ── FakeClock ─────────────────────────────────────────────────────────────────

class FakeClock:
    def __init__(self, t=1000.0):
        self._t = t
    def now(self):
        return self._t
    def advance(self, s):
        self._t += s


# ── Test harness: pure-Python CapabilityAssessor double ──────────────────────

class _TestableAssessorCore:
    """
    Pure-Python test double for CapabilityAssessor (no ROS2 required).

    Mirrors the tick/drain/build logic without any rclpy infrastructure.
    Publishers are replaced by capture lists; subscription lifecycle is
    represented by a boolean flag instead of a real ROS2 subscription handle.

    TEST_PROTOCOL §5.8 archetype E harness.
    """

    def __init__(self, bb: TimestampedBlackboard, config: dict, clock: FakeClock):
        self._bb = bb
        self._config = config
        self._clock_fn = clock.now
        # Mirrors CapabilityAssessor.__init__: drone_id is a first-class
        # attribute used to enrich outbound payloads (alert_intent since 08-25).
        self._drone_id = "test-drone"

        self._tree_root = build_relay_decision_tree(bb, config, clock=clock.now)
        self._bt = py_trees.trees.BehaviourTree(root=self._tree_root)

        self._data_freshness_node = None
        for node in walk_tree(self._tree_root):
            if isinstance(node, DataFreshness):
                self._data_freshness_node = node
                break

        # Capture lists — substitute for ROS2 publishers.
        # Each publish call appends the raw dict so tests can assert on content.
        self.report_captures   = []
        self.cmd_captures      = []
        self.alert_captures    = []
        self.proposal_captures = []
        self.reauth_captures   = []   # one entry per reauth_request publish

        # Lifecycle flag — substitute for the ROS2 subscription handle.
        # True = gc_leader_direct_quality subscription is active.
        self._quality_sub_active = False

        # One-shot flag: mirrors CapabilityAssessor._reauth_request_sent.
        self._reauth_request_sent = False

    def tick(self):
        """
        One full BT tick + drain cycle — mirrors CapabilityAssessor._tick().
        Order matters: build report BEFORE draining slots so pending_proposal
        is visible in the report (BUILDSPEC §4.11: report includes current pending_proposal).
        """
        self._bt.tick()

        # Publish reauth_request once when G8 writes reauth_requested_at.
        # Mirrors CapabilityAssessor._tick() reauth block (gap fix).
        if self._bb.get("reauth_requested_at") is not None and not self._reauth_request_sent:
            self.reauth_captures.append({
                "drone_id":  "test-drone",
                "timestamp": self._clock_fn(),
                "reason":    "authorization_expiring",
            })
            self._reauth_request_sent = True

        # Snapshot report while all BB slots still hold their post-tick values.
        report = self._build_report()

        # Drain pending_command first — relay_mover reads it from here.
        _drain(self._bb, "pending_command", self.cmd_captures.append)

        # Detect lifecycle-closing strategy before draining pending_proposal.
        # Must happen before drain so the strategy value is still readable.
        proposal = self._bb.get("pending_proposal")
        if proposal and proposal.get("strategy") in ("EXIT_RELAY", "LET_LEADER_ISOLATE"):
            # gc_link_quality subscription no longer needed — stop it to avoid leaking handles.
            self._teardown_quality_sub()

        # Drain pending_proposal and alert_intent.
        _drain(self._bb, "pending_proposal", self.proposal_captures.append)
        # alert_intent enrichment mirrors CapabilityAssessor._pub_alert:
        # add drone_id at publish boundary so downstream (RDA / G_task) can
        # attribute the alert.  FollowerSafetyExit (Layer 1) can't know
        # drone_id from the BB alone.
        def _capture_alert(intent):
            enriched = dict(intent)
            enriched.setdefault("drone_id", self._drone_id)
            self.alert_captures.append(enriched)
        _drain(self._bb, "alert_intent", _capture_alert)

        # Report published unconditionally — §4.11 hard rule: every tick, even when not capable.
        self.report_captures.append(report)
        return report

    def _build_report(self) -> dict:
        cfg     = self._config
        windows = cfg["staleness_windows_s"]
        now     = self._clock_fn()

        data_freshness = {}
        for key in ("signal_report", "drone_state"):
            max_age = windows[key]
            data    = self._bb.get(key)
            if data is None:
                data_freshness[key] = {"age_s": None, "fresh": False}
            else:
                ts  = data.get("timestamp") if isinstance(data, dict) else None
                if ts is None:
                    data_freshness[key] = {"age_s": None, "fresh": False}
                else:
                    age = round(now - ts, 3)
                    data_freshness[key] = {"age_s": age, "fresh": age <= max_age}

        checks = {}
        for node in walk_tree(self._tree_root):
            if isinstance(node, ConditionNodeBase):
                checks[node.name] = {
                    "pass":   node.status == py_trees.common.Status.SUCCESS,
                    "detail": getattr(node, "feedback_message", ""),
                }

        df_node    = self._data_freshness_node
        status_str = _status_to_string(self._tree_root, df_node)

        return {
            "drone_id":         "test-drone",
            "timestamp":        now,
            "capable":          status_str == "CAPABLE",
            "status":           status_str,
            "reason":           "",
            "tick_duration_ms": 0.0,
            "data_freshness":   data_freshness,
            "checks":           checks,
            "inputs":           {},
            "pending_proposal": self._bb.get("pending_proposal"),
        }

    def apply_relay_assignment(self, assignment: dict):
        """Mirrors _on_relay_assignment(): writes §6 BB keys, starts quality sub, resets reauth flag."""
        _apply_relay_assignment(self._bb, assignment)
        self._reauth_request_sent = False
        self._ensure_quality_sub()

    def receive_reeval_trigger(self, reason: str):
        """
        Mirrors capability_assessor._on_reeval_trigger(): publishes reauth_request
        with reason='reeval:{reason}'. Does NOT check _reauth_request_sent — the
        SNR fast-path is not one-shot; continuous_monitor COOLDOWN_S debounces.
        """
        self.reauth_captures.append({
            "drone_id":  "test-drone",
            "timestamp": self._clock_fn(),
            "reason":    f"reeval:{reason}",
        })

    def _ensure_quality_sub(self):
        self._quality_sub_active = True

    def _teardown_quality_sub(self):
        self._quality_sub_active = False


def _make_core(clock: FakeClock = None) -> _TestableAssessorCore:
    """Return a fresh harness with an empty blackboard."""
    clock = clock or FakeClock()
    bb    = TimestampedBlackboard(clock=clock.now)
    return _TestableAssessorCore(bb, _CFG, clock)


# ── §5.8 tests ────────────────────────────────────────────────────────────────


class TestCapabilityReportEveryTick:

    def test_capability_report_every_tick(self):
        """
        5 BT ticks → 5 capability reports published, regardless of BT outcome.
        BUILDSPEC §4.11 hard rule: published every tick, unconditional.
        """
        core = _make_core()
        # No blackboard data — BT will FAIL or RUNNING every tick.
        # Reports must still be produced.
        for _ in range(5):
            core.tick()

        assert len(core.report_captures) == 5, (
            f"Expected 5 reports; got {len(core.report_captures)}"
        )
        for i, report in enumerate(core.report_captures):
            assert isinstance(report, dict), f"Report {i} is not a dict"
            assert "capable"  in report,    f"Report {i} missing 'capable'"
            assert "status"   in report,    f"Report {i} missing 'status'"
            assert "drone_id" in report,    f"Report {i} missing 'drone_id'"

    def test_report_produced_even_on_bt_failure(self):
        """Report produced when BT returns FAILURE (no data — INCAPABLE path)."""
        core = _make_core()
        report = core.tick()
        assert report is not None
        # With no blackboard data the BT can't succeed.
        assert report["capable"] is False

    def test_report_produced_even_on_bt_success(self):
        """Report produced when BT returns SUCCESS (full CAPABLE path)."""
        clock = FakeClock()
        bb    = TimestampedBlackboard(clock=clock.now)
        now   = clock.now()

        # Minimal state for the RELAYING branch to reach CONTINUE.
        bb.set("current_role", "RELAYING")
        bb.set("signal_report", {
            "follower_to_gc":     {"snr_db": 25.0, "timestamp": now},
            "leader_to_follower": {"snr_db": 22.0, "timestamp": now},
            "timestamp": now,
        })
        bb.set("drone_state", {
            "battery_pct": 70.0, "flight_mode": "OFFBOARD",
            "gps_fix_type": 3,
            "position": {"lat": 47.394, "lon": 8.544, "alt": 40.0},
            "home_pos":  {"lat": 47.390, "lon": 8.540, "alt": 0.0},
            "timestamp": now,
        })
        bb.set("R_target", {"lat": 47.394, "lon": 8.544, "alt": 40.0})

        core = _TestableAssessorCore(bb, _CFG, clock)
        report = core.tick()
        assert report is not None
        assert "status" in report


class TestDrainsAndClearsSlots:

    def _idle_bb(self, clock: FakeClock) -> TimestampedBlackboard:
        """Blackboard in IDLE/no-tasking state — BT exits early, writes nothing."""
        bb = TimestampedBlackboard(clock=clock.now)
        bb.set("current_role", "IDLE")
        # No relay_tasking_received → RelayRequestReceived FAILURE → BT writes nothing.
        return bb

    def test_drains_pending_command(self):
        """pending_command: published once then cleared; absent on next tick."""
        clock = FakeClock()
        bb    = self._idle_bb(clock)
        bb.set("pending_command", {"command": "RTL", "reason": "test"})

        core = _TestableAssessorCore(bb, _CFG, clock)
        core.tick()

        assert len(core.cmd_captures) == 1
        assert core.cmd_captures[0]["command"] == "RTL"
        assert bb.get("pending_command") is None, "BB slot must be cleared after drain"

        # Second tick — same slot must NOT republish.
        core.tick()
        assert len(core.cmd_captures) == 1, "pending_command re-published; must drain exactly once"

    def test_drains_alert_intent(self):
        """alert_intent: published once then cleared; absent on next tick."""
        clock = FakeClock()
        bb    = self._idle_bb(clock)
        bb.set("alert_intent", {
            "type": "FOLLOWER_SAFETY_EXIT",
            "reason": "test_reason",
            "timestamp": clock.now(),
        })

        core = _TestableAssessorCore(bb, _CFG, clock)
        core.tick()

        assert len(core.alert_captures) == 1
        assert core.alert_captures[0]["type"] == "FOLLOWER_SAFETY_EXIT"
        assert bb.get("alert_intent") is None

        core.tick()
        assert len(core.alert_captures) == 1, "alert_intent re-published; must drain exactly once"

    def test_alert_intent_enriched_with_drone_id(self):
        """
        alert_intent published to /{drone_id}/alert_intent must carry drone_id
        in the PAYLOAD so RDA (G_task) can attribute across a multi-follower
        fleet without relying on "which topic did this arrive on."  Same class
        as the leader_id multi-leader gap — correct-by-accident at one
        follower, silently ambiguous at two or more.  Fix applied 2026-08-25:
        capability_assessor enriches at the publish boundary.
        """
        clock = FakeClock()
        bb    = self._idle_bb(clock)
        # FollowerSafetyExit (Layer 1) writes payload WITHOUT drone_id — it
        # cannot know it from BB.  Enrichment happens at drain-and-publish.
        bb.set("alert_intent", {
            "type":        "FOLLOWER_SAFETY_EXIT",
            "reason":      "battery_below_floor",
            "battery_pct": 12.0,
            "timestamp":   clock.now(),
        })
        core = _TestableAssessorCore(bb, _CFG, clock)
        core.tick()

        assert len(core.alert_captures) == 1
        captured = core.alert_captures[0]
        assert captured.get("drone_id") == "test-drone", (
            "alert_intent must be enriched with drone_id at publish boundary; "
            f"got {captured!r}"
        )
        # Original fields preserved
        assert captured["type"]        == "FOLLOWER_SAFETY_EXIT"
        assert captured["reason"]      == "battery_below_floor"
        assert captured["battery_pct"] == 12.0

    def test_alert_intent_explicit_drone_id_not_overwritten(self):
        """
        If FollowerSafetyExit ever starts including drone_id (future work),
        the enrichment must NOT overwrite it.  setdefault semantics.
        """
        clock = FakeClock()
        bb    = self._idle_bb(clock)
        bb.set("alert_intent", {
            "drone_id":  "explicit-drone",
            "type":      "FOLLOWER_SAFETY_EXIT",
            "reason":    "x",
            "timestamp": clock.now(),
        })
        core = _TestableAssessorCore(bb, _CFG, clock)
        core.tick()
        assert core.alert_captures[0]["drone_id"] == "explicit-drone", (
            "enrichment must use setdefault — explicit drone_id must be preserved"
        )

    def test_drains_pending_proposal(self):
        """pending_proposal: published once then cleared; absent on next tick."""
        clock = FakeClock()
        bb    = self._idle_bb(clock)
        bb.set("pending_proposal", {"strategy": "CONTINUOUS_RELAY", "reason": "test"})

        core = _TestableAssessorCore(bb, _CFG, clock)
        core.tick()

        assert len(core.proposal_captures) == 1
        assert core.proposal_captures[0]["strategy"] == "CONTINUOUS_RELAY"
        assert bb.get("pending_proposal") is None

        core.tick()
        assert len(core.proposal_captures) == 1, "pending_proposal re-published; must drain exactly once"

    def test_empty_slots_produce_no_publish(self):
        """Empty BB slots → no publish on any of the three channels."""
        core = _make_core()
        core.tick()

        assert len(core.cmd_captures)      == 0
        assert len(core.alert_captures)    == 0
        assert len(core.proposal_captures) == 0

    def test_report_includes_proposal_before_drain(self):
        """
        pending_proposal is included in capability_report in the same tick it
        is drained — report is built before the slot is cleared.
        """
        clock = FakeClock()
        bb    = self._idle_bb(clock)
        bb.set("pending_proposal", {"strategy": "EXIT_RELAY", "reason": "link_ineffective"})

        core = _TestableAssessorCore(bb, _CFG, clock)
        report = core.tick()

        assert report["pending_proposal"] is not None, (
            "Report must capture pending_proposal before it is drained"
        )
        assert report["pending_proposal"]["strategy"] == "EXIT_RELAY"
        # And still drained from BB for next tick.
        assert bb.get("pending_proposal") is None


class TestNoSeverityComputation:

    def test_no_severity_computation(self):
        """
        FSPL-inverse formula absent from capability_assessor.py.
        Severity arrives via /drone_NN/radio_health subscription from
        follower_radio_health_reader.py.  BUILDSPEC §4.11: F_radio moved out v6.4.
        """
        source = open(_ca_mod.__file__).read()

        # The FSPL-inverse formula uses this pattern to convert noise_dbm → severity.
        assert "(noise_dbm - baseline" not in source, (
            "FSPL-inverse found in capability_assessor.py — must be in follower_radio_health_reader.py"
        )
        # Guard against alternative formulations.
        assert "noise_dbm - baseline_noise" not in source
        assert "/ noise_range_db" not in source or "noise_range_db" not in source.split("/ noise_range_db")[0].split("\n")[-1].strip().replace(" ", "")


class TestSubscriptionLifecycle:

    def _assignment(self, clock: FakeClock) -> dict:
        return {
            "relay_target":      {"lat": 47.394, "lon": 8.544, "alt_m": 40.0},
            "tolerance_radius_m": 10.0,
            "valid_until":        clock.now() + 1800,
        }

    def test_subscription_lifecycle_start(self):
        """
        gc_leader_direct_quality subscription starts inactive; becomes active
        only after relay_assignment is received (F_tvalid passed).
        BUILDSPEC §4.11 item 18.
        """
        clock = FakeClock()
        core  = _make_core(clock)

        # Before any relay_assignment: quality sub must NOT be active.
        assert not core._quality_sub_active, (
            "gc_leader_direct_quality must not be subscribed before relay_assignment"
        )

        # After relay_assignment: quality sub becomes active.
        core.apply_relay_assignment(self._assignment(clock))

        assert core._quality_sub_active, (
            "gc_leader_direct_quality must be subscribed after relay_assignment"
        )

    def test_subscription_lifecycle_stop_washout(self):
        """
        LET_LEADER_ISOLATE in pending_proposal → subscription torn down.
        This is the F_decline/washout path: drone could not satisfy any strategy.
        BUILDSPEC §4.11.
        """
        clock = FakeClock()
        bb    = TimestampedBlackboard(clock=clock.now)
        bb.set("current_role", "IDLE")

        core = _TestableAssessorCore(bb, _CFG, clock)
        core._quality_sub_active = True  # pre-activate as if relay_assignment was received

        # Plant the LET_LEADER_ISOLATE proposal.
        bb.set("pending_proposal", {"strategy": "LET_LEADER_ISOLATE", "reason": "no_feasible_strategy"})
        core.tick()

        assert not core._quality_sub_active, (
            "gc_leader_direct_quality must be unsubscribed on LET_LEADER_ISOLATE"
        )
        # Proposal must have been drained.
        assert len(core.proposal_captures) == 1
        assert core.proposal_captures[0]["strategy"] == "LET_LEADER_ISOLATE"

    def test_subscription_lifecycle_stop_exit(self):
        """
        EXIT_RELAY in pending_proposal → subscription torn down.
        This is the maintenance-gate exit path: relay is no longer needed or viable.
        BUILDSPEC §4.11.
        """
        clock = FakeClock()
        bb    = TimestampedBlackboard(clock=clock.now)
        bb.set("current_role", "IDLE")

        core = _TestableAssessorCore(bb, _CFG, clock)
        core._quality_sub_active = True

        bb.set("pending_proposal", {"strategy": "EXIT_RELAY", "reason": "direct_link_recovered"})
        core.tick()

        assert not core._quality_sub_active, (
            "gc_leader_direct_quality must be unsubscribed on EXIT_RELAY"
        )
        assert len(core.proposal_captures) == 1
        assert core.proposal_captures[0]["strategy"] == "EXIT_RELAY"

    def test_quality_sub_not_torn_down_on_other_strategies(self):
        """CONTINUOUS_RELAY proposal must NOT tear down the quality subscription."""
        clock = FakeClock()
        bb    = TimestampedBlackboard(clock=clock.now)
        bb.set("current_role", "IDLE")

        core = _TestableAssessorCore(bb, _CFG, clock)
        core._quality_sub_active = True

        bb.set("pending_proposal", {"strategy": "CONTINUOUS_RELAY", "reason": "test"})
        core.tick()

        assert core._quality_sub_active, (
            "gc_leader_direct_quality must NOT be torn down on CONTINUOUS_RELAY"
        )

    def test_subscription_not_active_without_assignment(self):
        """Multiple ticks without relay_assignment: quality sub stays inactive."""
        core = _make_core()
        for _ in range(3):
            core.tick()
        assert not core._quality_sub_active


class TestWritesRadiusAndTimerKeys:

    def test_writes_radius_and_timer_keys(self):
        """
        relay_assignment writes all four §6 blackboard keys.
        BUILDSPEC §6.
        """
        clock      = FakeClock(t=1000.0)
        core       = _make_core(clock)
        valid_until = clock.now() + 1800.0

        assignment = {
            # BUILDSPEC §2.7 / §5.3 Decision 5: the authorized target on the
            # relay_assignment payload is `r_target`, echoed verbatim from the
            # authorization.  The blackboard key it writes to is
            # `current_relay_target` — the rename happens at the write boundary.
            "r_target":           {"lat": 47.394, "lon": 8.544, "alt_m": 40.0},
            "tolerance_radius_m": 10.0,
            "valid_until":        valid_until,
        }
        core.apply_relay_assignment(assignment)

        assert core._bb.get("current_relay_target") == assignment["r_target"], (
            "current_relay_target not written"
        )
        assert core._bb.get("tolerance_radius_m") == 10.0, (
            "tolerance_radius_m not written"
        )
        assert core._bb.get("authorization_valid_until") == valid_until, (
            "authorization_valid_until not written"
        )
        assert core._bb.get("reauth_requested_at") is None, (
            "reauth_requested_at must be reset to None on new assignment"
        )

    def test_relay_assignment_resets_reauth_timer(self):
        """
        reauth_requested_at reset to None even if it was non-None before.
        A new authorization period starts fresh — no carried-over reauth clock.
        """
        clock = FakeClock(t=1000.0)
        core  = _make_core(clock)

        # Simulate a prior reauth episode having written a timestamp.
        core._bb.set("reauth_requested_at", 950.0)

        core.apply_relay_assignment({
            "r_target":           {"lat": 47.394, "lon": 8.544, "alt_m": 40.0},
            "tolerance_radius_m": 10.0,
            "valid_until":        1000.0 + 1800.0,
        })

        assert core._bb.get("reauth_requested_at") is None, (
            "reauth_requested_at must be cleared (None) when a new assignment arrives"
        )

    def test_drain_function_publishes_and_clears(self):
        """
        _drain() pure function: publishes value via pub_fn, clears BB key.
        BUILDSPEC §4.11 drain-and-clear contract.
        """
        bb      = TimestampedBlackboard()
        captured = []

        bb.set("pending_command", {"command": "RTL"})
        drained = _drain(bb, "pending_command", captured.append)

        assert drained is True
        assert len(captured) == 1
        assert captured[0]["command"] == "RTL"
        assert bb.get("pending_command") is None

    def test_drain_function_no_op_when_empty(self):
        """_drain() returns False and calls pub_fn zero times when slot is empty."""
        bb      = TimestampedBlackboard()
        captured = []

        drained = _drain(bb, "pending_command", captured.append)

        assert drained is False
        assert len(captured) == 0

    def test_apply_relay_assignment_pure_function(self):
        """_apply_relay_assignment() writes exactly the four §6 keys, no more."""
        bb = TimestampedBlackboard()
        _apply_relay_assignment(bb, {
            "r_target":           {"lat": 47.0, "lon": 8.0, "alt_m": 30.0},
            "tolerance_radius_m": 15.0,
            "valid_until":        9999.0,
        })

        assert bb.get("current_relay_target")      == {"lat": 47.0, "lon": 8.0, "alt_m": 30.0}
        assert bb.get("tolerance_radius_m")        == 15.0
        assert bb.get("authorization_valid_until") == 9999.0
        assert bb.get("reauth_requested_at")       is None


# ── Reauth request publish (gap fix) ──────────────────────────────────────────

class TestReauthRequestPublish:

    def test_reauth_request_published_once(self):
        """
        When reauth_requested_at is set on BB and _reauth_request_sent is False,
        a single tick must publish exactly one reauth_request.
        Gap fix: without this publish, the GC never starts a reauth round.
        """
        clock = FakeClock(t=1000.0)
        core  = _make_core(clock)
        # Simulate G8 having fired: RelayActuallyImproved wrote reauth_requested_at.
        core._bb.set("reauth_requested_at", clock.now())
        assert not core._reauth_request_sent

        core.tick()

        assert len(core.reauth_captures) == 1, (
            "Exactly one reauth_request must be published on first tick after G8 fires"
        )
        published = core.reauth_captures[0]
        assert published["drone_id"]  == "test-drone"
        assert published["reason"]    == "authorization_expiring"
        assert published["timestamp"] == clock.now()

    def test_reauth_request_not_repeated(self):
        """
        _reauth_request_sent is set to True after the first publish.
        Subsequent ticks with reauth_requested_at still set must NOT publish again.
        """
        clock = FakeClock(t=1000.0)
        core  = _make_core(clock)
        core._bb.set("reauth_requested_at", clock.now())

        for _ in range(5):
            core.tick()

        assert len(core.reauth_captures) == 1, (
            "reauth_request must be published exactly once regardless of tick count "
            f"(got {len(core.reauth_captures)})"
        )

    def test_reauth_request_not_published_without_reauth_requested_at(self):
        """If reauth_requested_at is None (not set), no reauth_request is published."""
        core = _make_core()
        assert core._bb.get("reauth_requested_at") is None

        core.tick()

        assert len(core.reauth_captures) == 0, (
            "reauth_request must not be published when reauth_requested_at is not set"
        )

    def test_reauth_request_cleared_on_new_assignment(self):
        """
        apply_relay_assignment() resets _reauth_request_sent to False so the next
        authorization period gets a fresh send opportunity.
        """
        clock = FakeClock(t=1000.0)
        core  = _make_core(clock)
        # Simulate G8 firing and the reauth being sent.
        core._bb.set("reauth_requested_at", clock.now())
        core.tick()
        assert core._reauth_request_sent is True
        assert len(core.reauth_captures) == 1

        # New relay_assignment arrives: reset flag.
        core.apply_relay_assignment({
            "relay_target":       {"lat": 47.394, "lon": 8.544, "alt_m": 40.0},
            "tolerance_radius_m": 10.0,
            "valid_until":        clock.now() + 1800.0,
        })
        assert core._reauth_request_sent is False, (
            "_reauth_request_sent must be reset to False on new relay_assignment"
        )
        assert core._bb.get("reauth_requested_at") is None, (
            "reauth_requested_at must be cleared by _apply_relay_assignment"
        )

        # Next tick: reauth_requested_at is None, so no publish.
        core.tick()
        assert len(core.reauth_captures) == 1, (
            "No additional reauth_request after assignment reset reauth_requested_at"
        )

    def test_reauth_request_published_again_after_new_assignment(self):
        """
        After a new relay_assignment clears the flag, a subsequent G8 fire
        (reauth_requested_at set again) triggers a second reauth_request publish.
        Verifies the flag is truly reset and not permanently latched.
        """
        clock = FakeClock(t=1000.0)
        core  = _make_core(clock)

        # First G8 fire → publish #1.
        core._bb.set("reauth_requested_at", clock.now())
        core.tick()
        assert len(core.reauth_captures) == 1

        # New authorization arrives.
        core.apply_relay_assignment({
            "relay_target":       {"lat": 47.394, "lon": 8.544, "alt_m": 40.0},
            "tolerance_radius_m": 10.0,
            "valid_until":        clock.now() + 1800.0,
        })

        # Second G8 fire in new authorization period → publish #2.
        clock.advance(600.0)
        core._bb.set("reauth_requested_at", clock.now())
        core.tick()
        assert len(core.reauth_captures) == 2, (
            "Second G8 fire in a new authorization period must publish a second reauth_request"
        )


# ── reeval_trigger → reauth_request (SNR fast-path, DEVIATION) ───────────────

class TestReevalTriggerToReauth:
    """
    Tests for capability_assessor._on_reeval_trigger() — the SNR fast-path added
    as a DEVIATION (not in BUILDSPEC §4.11).  continuous_monitor publishes
    reeval_trigger on snr_degraded; capability_assessor forwards it as a
    reauth_request with reason='reeval:{trigger_reason}'.
    SESSION_LOG DEVIATION 2026-08-18.
    """

    def test_snr_degraded_publishes_reauth(self):
        """receive_reeval_trigger('snr_degraded') appends one reauth_request."""
        core = _make_core()
        core.receive_reeval_trigger("snr_degraded")

        assert len(core.reauth_captures) == 1, (
            f"Expected 1 reauth_request; got {len(core.reauth_captures)}"
        )

    def test_relay_completed_publishes_reauth(self):
        """receive_reeval_trigger('relay_completed') also appends a reauth_request."""
        core = _make_core()
        core.receive_reeval_trigger("relay_completed")

        assert len(core.reauth_captures) == 1, (
            "relay_completed reeval_trigger must publish reauth_request"
        )

    def test_reason_carries_trigger_reason(self):
        """reauth_request reason field is 'reeval:{trigger_reason}'."""
        core = _make_core()
        core.receive_reeval_trigger("snr_degraded")

        assert core.reauth_captures[0]["reason"] == "reeval:snr_degraded", (
            f"Expected reason='reeval:snr_degraded'; got {core.reauth_captures[0]['reason']!r}"
        )

    def test_drone_id_in_payload(self):
        """reauth_request payload includes drone_id."""
        core = _make_core()
        core.receive_reeval_trigger("snr_degraded")

        assert "drone_id" in core.reauth_captures[0], (
            "reauth_request payload must include drone_id"
        )

    def test_not_gated_by_reauth_request_sent_flag(self):
        """
        receive_reeval_trigger fires even when _reauth_request_sent is True.
        The one-shot flag gates only the G8 timer path; the SNR fast-path is
        debounced by continuous_monitor's COOLDOWN_S instead.
        """
        core = _make_core()
        core._reauth_request_sent = True

        core.receive_reeval_trigger("snr_degraded")

        assert len(core.reauth_captures) == 1, (
            "reeval_trigger must not be gated by _reauth_request_sent flag; "
            f"got {len(core.reauth_captures)} captures"
        )
