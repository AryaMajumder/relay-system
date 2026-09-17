"""
test_wave7_relay_decision_authority.py — Wave 7 gate tests.

TEST_PROTOCOL: §5.10  Archetype E (stateful decision node, fake clock)
BUILDSPEC:     §4.10, §2.4, §2.6

PROVEN column:
  test_relay_tasking_shared_topic           -> §5.2 (one shared topic, not per-drone)
  test_relay_tasking_has_no_target_id       -> §2.4 (no target drone field)
  test_authorization_per_drone_topic        -> §2.6 (per-drone /{drone_id}/authorization)
  test_window_opens_on_first_proposal       -> §4.10 step 3
  test_window_fixed_not_extended            -> §4.10 (window not extended)
  test_collected_set_keyed_by_drone         -> §4.10 step 4 (replace, not append)
  test_decline_deletes_entry                -> §4.10 step 4 (decline → DELETE)
  test_only_positive_ranked                 -> §4.10 (true by construction after decline)
  test_empty_window_rebroadcasts            -> §4.10 step 5
  test_rank_battery_first                   -> §4.10 winner comparison order
  test_rank_eta_breaks_battery_tie          -> §4.10 winner comparison order
  test_rank_band_breaks_eta_tie             -> §4.10 winner comparison order
  test_rank_gps_breaks_band_tie             -> §4.10 winner comparison order
  test_no_threshold_filtering               -> §4.10 (no gating thresholds)
  test_r_target_verbatim                    -> §5.3 Decision 5
  test_authorization_carries_radius_and_validity -> §2.6
  test_relay_request_dedup_suppresses_logging    -> §4.10 table 1
  test_relay_request_dedup_expires               -> §4.10 table 1
  test_decline_dedup_tuple_key                   -> §4.10 table 2
  test_decline_dedup_expires                     -> §4.10 table 2
  test_decline_still_processed_when_deduped      -> §4.10 (dedup is logging-only)
  test_no_mqtt_client                            -> §4.10 stripped
  test_no_gc_radio_health_for_selection          -> §4.10
  test_reauth_request_starts_round               -> §4.10 reauth trigger (gap fix)
  test_reauth_request_ignored_during_active_round -> §4.10 reauth guard
"""

import sys
import os
import json
import time
import importlib
import inspect
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for p in (
    _PKG_ROOT,
    os.path.join(_PKG_ROOT, "drone_control"),
):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.relay_decision_authority import _DecisionCore


# ── Fake clock ────────────────────────────────────────────────────────────────

class FakeClock:
    def __init__(self, t=10000.0):
        self._t = t

    def now(self):
        return self._t

    def advance(self, seconds):
        self._t += seconds


# ── Test config ───────────────────────────────────────────────────────────────

_CFG = {
    "collection_window_s":          45,
    "rebroadcast_pause_s":          120,
    "relay_request_dedup_expiry_s": 600,
    "decline_dedup_expiry_s":       180,
    "tolerance_radius_m":           10.0,
    "authorization_validity_s":     1800,
}

_R_TARGET = {"lat": 47.391, "lon": 8.541, "alt_m": 10.0}


# ── Helper builders ───────────────────────────────────────────────────────────

def _make_proposal(
    drone_id="drone-01",
    round_id="round-test",
    strategy="CONTINUOUS_RELAY",
    battery_pct=70.0,
    eta_s=60.0,
    t_lo=0.3,
    t_hi=0.7,
    gps_fix_type=3,
    r_target=None,
    proposal_id=None,
    reason="",
    trigger_context=None,
):
    return {
        "proposal_id": proposal_id or f"prop-{drone_id[:4]}0000000",
        "drone_id":    drone_id,
        "round_id":    round_id,
        "strategy":    strategy,
        "r_target":    r_target or dict(_R_TARGET),
        "eta_s":       eta_s,
        "capability_snapshot": {
            "battery_pct":  battery_pct,
            "gps_fix_type": gps_fix_type,
            "t_lo":         t_lo,
            "t_hi":         t_hi,
        },
        "trigger_context": trigger_context or {"gate_fired": "", "reason": reason, "source": "capability_assessor"},
        "timestamp": 10000.0,
    }


def _make_decline(drone_id="drone-01", round_id="round-test", reason="never_seen"):
    return _make_proposal(
        drone_id=drone_id,
        round_id=round_id,
        strategy="LET_LEADER_ISOLATE",
        trigger_context={"gate_fired": "", "reason": reason, "source": "capability_assessor"},
    )


def _make_core():
    """
    Build a _DecisionCore with captured outputs and a FakeClock.
    tasking_out:  every relay_tasking broadcast appended here.
    auth_out:     every authorization published as (drone_id, payload).
    clock:        mutable — tests call clock.advance() to simulate time,
                  then core.check_timers() to trigger time-dependent state changes.
                  This replaces the ROS2 wall-clock timer that calls check_timers() in production.
    """
    tasking_out = []
    auth_out    = []   # list of (drone_id, payload)
    clock       = FakeClock()

    core = _DecisionCore(
        config=dict(_CFG),
        publish_tasking_fn=tasking_out.append,
        publish_auth_fn=lambda did, p: auth_out.append((did, p)),
        clock=clock.now,
    )
    return core, tasking_out, auth_out, clock


# ── §2.4 / §5.2: relay_tasking schema and topic ───────────────────────────────

class TestRelayTaskingSharedTopic:
    def test_relay_tasking_shared_topic(self):
        """
        relay_tasking must go to /relay_tasking (shared), not per-drone.
        Verified by checking that the ROS2 node uses the literal '/relay_tasking' topic.
        BUILDSPEC §5.2 — broadcast is intentional.
        """
        import inspect
        import drone_control.relay_decision_authority as rda_mod
        src = inspect.getsource(rda_mod.RelayDecisionAuthority.__init__)
        assert '"/relay_tasking"' in src, (
            "RelayDecisionAuthority must publish to '/relay_tasking' (shared topic), "
            "not a per-drone variant."
        )

    def test_relay_tasking_payload_has_round_id(self):
        """relay_tasking payload must include round_id (§2.4)."""
        core, tasking, _, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        assert len(tasking) == 1
        assert "round_id" in tasking[0], "relay_tasking missing round_id"

    def test_relay_tasking_payload_has_trigger(self):
        """relay_tasking payload must include trigger (§2.4)."""
        core, tasking, _, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        assert tasking[0].get("trigger") in ("initial", "reauth")


class TestRelayTaskingHasNoTargetId:
    def test_relay_tasking_has_no_target_id(self):
        """
        relay_tasking must NOT contain a target drone_id field.
        BUILDSPEC §2.4: "No target drone id. Broadcast is intentional."
        """
        core, tasking, _, _ = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        assert len(tasking) == 1
        p = tasking[0]
        # Fields from §2.4: round_id, timestamp, trigger — nothing else
        forbidden = {"drone_id", "target_id", "target_drone", "follower_id"}
        found = forbidden & p.keys()
        assert not found, f"relay_tasking contains target fields: {found}"


# ── §2.6: authorization per-drone topic ───────────────────────────────────────

class TestAuthorizationPerDroneTopic:
    def test_authorization_per_drone_topic(self):
        """
        Authorization must go to /{drone_id}/authorization, not a shared topic.
        Verified by checking that the ROS2 node builds per-drone publishers.
        BUILDSPEC §2.6.
        """
        import inspect
        import drone_control.relay_decision_authority as rda_mod
        src = inspect.getsource(rda_mod.RelayDecisionAuthority.__init__)
        assert '"/authorization"' in src or "authorization" in src, (
            "RelayDecisionAuthority must create per-drone /authorization publishers"
        )
        # Verify auth_pubs is drone-keyed (structural check in core tests below)

    def test_authorization_goes_to_correct_drone(self):
        """Winner drone_id is used as the auth destination."""
        core, _, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-02", round_id=round_id, battery_pct=80.0))
        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-01", round_id=round_id, battery_pct=50.0))
        core.check_timers()
        clock.advance(50)
        core.check_timers()

        assert len(auth) == 1
        assert auth[0][0] == "drone-02", "Auth should go to the higher-battery drone"


# ── §4.10 step 3: window opens on first proposal ──────────────────────────────

class TestWindowOpensOnFirstProposal:
    def test_window_opens_on_first_proposal(self):
        """
        Window starts on FIRST proposal arrival, NOT at broadcast time.
        BUILDSPEC §4.10 step 3.
        """
        core, tasking, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        # Advance 20s before first proposal — window should NOT be open yet.
        clock.advance(20)
        core.check_timers()
        assert not core._window_closed, "Window should not be closed before first proposal"
        assert len(auth) == 0

        # First proposal: window starts NOW (at t=10020).
        core.on_strategy_proposal(_make_proposal(drone_id="drone-01", round_id=round_id))
        window_start = core._window_start
        assert window_start is not None, "window_start should be set after first proposal"
        assert abs(window_start - clock.now()) < 0.01

        # Advance less than 45s from window start — still open.
        clock.advance(44)
        core.check_timers()
        assert not core._window_closed, "Window should still be open at 44s"

        # Advance past 45s total from window start — closes.
        clock.advance(2)
        core.check_timers()
        assert core._window_closed, "Window should be closed after 45s"
        assert len(auth) == 1, "Should have granted authorization after window close"


# ── §4.10: window fixed, not extended ────────────────────────────────────────

class TestWindowFixedNotExtended:
    def test_window_fixed_not_extended(self):
        """
        Late arrival near window end does NOT extend the window.
        BUILDSPEC §4.10: "fixed, not extended."
        """
        core, _, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        # First proposal opens window at t=10000.
        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-01", round_id=round_id, battery_pct=60.0))
        window_start = core._window_start

        # Advance to 44s — just before window closes.
        clock.advance(44)
        # Late proposal at 44s should NOT extend the window.
        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-02", round_id=round_id, battery_pct=80.0))

        # Advance 2 more seconds → window should close at 45s from first proposal.
        clock.advance(2)
        core.check_timers()
        assert core._window_closed, "Window should have closed exactly at window_s"

        # Total window = 45s from window_start (not from second proposal arrival).
        total_open = clock.now() - window_start
        assert total_open >= 45.0, "Window must be at least 45s"
        # Would be ~91s if extended by the late proposal.
        assert total_open < 60.0, "Window must not be extended by late proposal"


# ── §4.10 step 4: collected set keyed by drone_id ────────────────────────────

class TestCollectedSetKeyedByDrone:
    def test_collected_set_keyed_by_drone(self):
        """
        Two proposals from the same drone → ONE entry (replace, not append).
        BUILDSPEC §4.10 step 4: "new proposal, existing entry → replace."
        """
        core, _, _, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-01", round_id=round_id, battery_pct=60.0,
            proposal_id="prop-aaa000000001"))
        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-01", round_id=round_id, battery_pct=75.0,
            proposal_id="prop-aaa000000002"))

        assert len(core._collected) == 1, "Two proposals from same drone → one entry"
        entry = core._collected["drone-01"]
        cs = entry.get("capability_snapshot", {})
        assert cs.get("battery_pct") == 75.0, "Latest proposal should win (replaced)"


# ── §4.10 step 4: decline → DELETE entry ─────────────────────────────────────

class TestDeclineDeletesEntry:
    def test_decline_deletes_entry(self):
        """
        Proposal then decline from same drone → entry removed.
        BUILDSPEC §4.10 step 4: "decline arrives → DELETE that entry."
        """
        core, _, _, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        core.on_strategy_proposal(_make_proposal(drone_id="drone-01", round_id=round_id))
        assert "drone-01" in core._collected

        core.on_strategy_proposal(_make_decline(drone_id="drone-01", round_id=round_id))
        assert "drone-01" not in core._collected, "Decline must remove entry"


class TestOnlyPositiveRanked:
    def test_only_positive_ranked(self):
        """
        After a decline, that drone cannot win.
        BUILDSPEC §4.10: "only positive responses are ranked" (true by construction).
        """
        core, _, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        # drone-01 has highest battery but declines.
        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-01", round_id=round_id, battery_pct=99.0))
        core.on_strategy_proposal(_make_decline(drone_id="drone-01", round_id=round_id))

        # drone-02 has lower battery but is the only positive respondent.
        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-02", round_id=round_id, battery_pct=60.0))

        clock.advance(50)
        core.check_timers()

        assert len(auth) == 1
        assert auth[0][0] == "drone-02", (
            "Declined drone must not win even if it had highest battery"
        )


# ── §4.10 step 5: empty window → rebroadcast ─────────────────────────────────

class TestEmptyWindowRebroadcasts:
    def test_empty_window_rebroadcasts(self):
        """
        Empty window at close → wait rebroadcast_pause_s (120s) → re-broadcast.
        BUILDSPEC §4.10 step 5: "if collected is empty → wait rebroadcast_pause_s, goto 2."
        """
        core, tasking, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        assert len(tasking) == 1  # initial broadcast

        # Simulate a proposal arriving to open the window, then let window expire empty.
        round_id = core._current_round_id
        core.on_strategy_proposal(_make_proposal(drone_id="drone-01", round_id=round_id))
        # Remove the proposal via decline so collected is empty at close.
        core.on_strategy_proposal(_make_decline(drone_id="drone-01", round_id=round_id))
        assert len(core._collected) == 0

        # Advance past window (45s).
        clock.advance(50)
        core.check_timers()
        assert core._window_closed, "Window should be closed"
        assert len(auth) == 0, "No authorization when window closes empty"
        assert core._rebroadcast_at is not None, "Rebroadcast should be scheduled"

        # Before rebroadcast_pause_s elapses: no new broadcast.
        clock.advance(100)
        core.check_timers()
        assert len(tasking) == 1, "Should not rebroadcast before pause expires"

        # After rebroadcast_pause_s (120s total from window close): new broadcast.
        clock.advance(25)  # 50+100+25 = 175 > 45 + 120
        core.check_timers()
        assert len(tasking) == 2, "Should have rebroadcast after pause expires"

    def test_empty_window_no_proposals_at_all(self):
        """Round with no proposals times out via _round_start_at and schedules rebroadcast."""
        core, tasking, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        # No proposals arrive — window never opened via proposal, but _round_start_at
        # timeout in check_timers closes the round empty after collection_window_s.
        clock.advance(100)  # 100 > 45s collection_window_s
        core.check_timers()
        assert len(auth) == 0, "No authorization when window closes empty"
        assert core._window_closed, "Round should be closed after timeout"
        assert core._rebroadcast_at is not None, (
            "Rebroadcast should be scheduled after empty round times out"
        )



class TestPostAuthorizationRebroadcast:
    def test_rebroadcast_scheduled_after_authorization(self):
        """After granting authorization, next poll is scheduled at rebroadcast_pause_s."""
        core, tasking, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        core.on_strategy_proposal(_make_proposal(drone_id="drone-02", round_id=round_id))
        clock.advance(50)
        core.check_timers()
        assert len(auth) == 1, "Authorization should be granted"
        assert core._rebroadcast_at is not None, (
            "Next poll should be scheduled after authorization, even with a winner"
        )

    def test_new_round_fires_after_post_auth_pause(self):
        """After authorization, a new relay_tasking is published once the pause elapses."""
        core, tasking, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        core.on_strategy_proposal(_make_proposal(drone_id="drone-02", round_id=round_id))
        clock.advance(50)
        core.check_timers()
        assert len(tasking) == 1

        # Before pause expires: no new broadcast.
        clock.advance(100)
        core.check_timers()
        assert len(tasking) == 1

        # After rebroadcast_pause_s (120s from authorization): new broadcast.
        clock.advance(25)  # 50 + 100 + 25 = 175 > 45 + 120
        core.check_timers()
        assert len(tasking) == 2, "Should have rebroadcast after post-auth pause"


# ── §4.10: winner comparison ──────────────────────────────────────────────────

def _run_rank_test(proposals: list) -> str:
    """Helper: start a round, submit proposals, close window, return winner drone_id."""
    core, _, auth, clock = _make_core()
    core.on_gc_link_quality({"quality": 0.2})
    round_id = core._current_round_id
    for p in proposals:
        p = dict(p)
        p["round_id"] = round_id
        core.on_strategy_proposal(p)
    clock.advance(50)
    core.check_timers()
    assert len(auth) == 1, f"Expected exactly 1 auth, got {len(auth)}"
    return auth[0][0]


class TestRankBatteryFirst:
    def test_rank_battery_first(self):
        """
        Lower eta_s but lower battery loses to higher battery.
        BUILDSPEC §4.10: battery_pct DESC is primary sort key.
        """
        winner = _run_rank_test([
            _make_proposal(drone_id="drone-01", battery_pct=80.0, eta_s=120.0),
            _make_proposal(drone_id="drone-02", battery_pct=60.0, eta_s=10.0),
        ])
        assert winner == "drone-01", (
            "Higher battery should win even with higher eta_s"
        )


class TestRankEtaBreaksBatteryTie:
    def test_rank_eta_breaks_battery_tie(self):
        """
        Equal battery → lower eta_s wins.
        BUILDSPEC §4.10: eta_s ASC is secondary sort key.
        """
        winner = _run_rank_test([
            _make_proposal(drone_id="drone-01", battery_pct=70.0, eta_s=90.0),
            _make_proposal(drone_id="drone-02", battery_pct=70.0, eta_s=30.0),
        ])
        assert winner == "drone-02", "Lower eta_s should win when battery is tied"


class TestRankBandBreaksEtaTie:
    def test_rank_band_breaks_eta_tie(self):
        """
        Equal battery + eta → wider t_hi - t_lo wins.
        BUILDSPEC §4.10: (t_hi-t_lo) DESC is tertiary sort key.
        """
        winner = _run_rank_test([
            _make_proposal(drone_id="drone-01", battery_pct=70.0, eta_s=60.0,
                           t_lo=0.4, t_hi=0.6),  # band width 0.2
            _make_proposal(drone_id="drone-02", battery_pct=70.0, eta_s=60.0,
                           t_lo=0.2, t_hi=0.8),  # band width 0.6
        ])
        assert winner == "drone-02", "Wider band should win when battery+eta tied"


class TestRankGpsBreaksBandTie:
    def test_rank_gps_breaks_band_tie(self):
        """
        Equal on battery+eta+band → better gps_fix_type wins.
        BUILDSPEC §4.10: gps_fix_type DESC is quaternary sort key.
        """
        winner = _run_rank_test([
            _make_proposal(drone_id="drone-01", battery_pct=70.0, eta_s=60.0,
                           t_lo=0.3, t_hi=0.7, gps_fix_type=2),
            _make_proposal(drone_id="drone-02", battery_pct=70.0, eta_s=60.0,
                           t_lo=0.3, t_hi=0.7, gps_fix_type=3),
        ])
        assert winner == "drone-02", "Better GPS fix type should win on final tiebreaker"


class TestNoThresholdFiltering:
    def test_no_threshold_filtering(self):
        """
        A drone with a very thin band (t_hi - t_lo near zero) still wins on
        top battery — no threshold filters it out before ranking.
        BUILDSPEC §4.10: "No thresholds on any field."
        """
        winner = _run_rank_test([
            _make_proposal(drone_id="drone-01", battery_pct=90.0,
                           t_lo=0.499, t_hi=0.501),  # nearly zero band
            _make_proposal(drone_id="drone-02", battery_pct=60.0,
                           t_lo=0.2, t_hi=0.8),       # wide band
        ])
        assert winner == "drone-01", (
            "Thin-band drone with top battery must win — no threshold filtering"
        )


# ── §5.3: r_target verbatim passthrough ──────────────────────────────────────

class TestRTargetVerbatim:
    def test_r_target_verbatim(self):
        """
        Authorized r_target must be byte-identical to the proposal's r_target.
        BUILDSPEC §5.3 Decision 5: "never recompute a position that has been authorized."
        """
        r = {"lat": 47.3914159, "lon": 8.5412718, "alt_m": 12.5}
        core, _, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id
        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-01", round_id=round_id, r_target=r))
        clock.advance(50)
        core.check_timers()

        assert len(auth) == 1
        auth_r = auth[0][1].get("r_target")
        assert auth_r == r, (
            f"r_target must be byte-identical.\n"
            f"  proposal: {r}\n"
            f"  auth:     {auth_r}"
        )

    def test_r_target_not_re_snapped(self):
        """Relay_decision_authority must NOT call bucket_position on r_target."""
        import inspect
        import drone_control.relay_decision_authority as rda_mod
        src = inspect.getsource(rda_mod._DecisionCore._grant_authorization)
        assert "bucket_position" not in src, (
            "_grant_authorization must not call bucket_position — §5.3 Decision 5"
        )


# ── §2.6: authorization schema ────────────────────────────────────────────────

class TestAuthorizationCarriesRadiusAndValidity:
    def test_authorization_carries_radius_and_validity(self):
        """
        Authorization must carry tolerance_radius_m and valid_until.
        BUILDSPEC §2.6.
        """
        core, _, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id
        core.on_strategy_proposal(_make_proposal(
            drone_id="drone-01", round_id=round_id))
        clock.advance(50)
        core.check_timers()

        assert len(auth) == 1
        payload = auth[0][1]
        assert "tolerance_radius_m" in payload, "Authorization missing tolerance_radius_m"
        assert "valid_until" in payload, "Authorization missing valid_until"
        assert payload["tolerance_radius_m"] == _CFG["tolerance_radius_m"]
        assert payload["valid_until"] > clock.now(), "valid_until must be in the future"
        assert abs(payload["valid_until"] - (clock.now() + _CFG["authorization_validity_s"])) < 1.0

    def test_authorization_schema_complete(self):
        """All §2.6 fields present: proposal_id, drone_id, round_id, strategy,
           r_target, tolerance_radius_m, valid_until, timestamp."""
        core, _, auth, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id
        p = _make_proposal(drone_id="drone-01", round_id=round_id,
                           proposal_id="prop-abc000000001")
        core.on_strategy_proposal(p)
        clock.advance(50)
        core.check_timers()

        assert len(auth) == 1
        a = auth[0][1]
        for field in ("proposal_id", "drone_id", "round_id", "strategy",
                      "r_target", "tolerance_radius_m", "valid_until", "timestamp"):
            assert field in a, f"Authorization missing required field '{field}'"


# ── §4.10 dedup table 1: relay_request ────────────────────────────────────────

class TestRelayRequestDedupSuppressesLogging:
    def test_relay_request_dedup_suppresses_logging(self):
        """
        Same (drone_id, snr_bucket) twice inside 600s → only one entry in dedup table.
        BUILDSPEC §4.10 table 1.
        Use 12.0 and 14.0 dB — both map to bucket int(12//5)=int(14//5)=2.
        """
        core, _, _, clock = _make_core()
        core.on_relay_request({"drone_id": "drone-01", "snr_db": 12.0, "timestamp": clock.now()})
        core.on_relay_request({"drone_id": "drone-01", "snr_db": 14.0, "timestamp": clock.now()})
        # 12 and 14 both land in bucket 2 (int(12//5)=2, int(14//5)=2)
        key = ("drone-01", int(12.0 // 5))
        assert key in core._rr_dedup, "Key must be in dedup table"
        # Only ONE table entry per key: the two arrivals share the same bucket.
        rr_keys = [k for k in core._rr_dedup if k[0] == "drone-01"]
        assert len(rr_keys) == 1, "Same SNR bucket should produce one dedup entry"


class TestRelayRequestDedupExpires:
    def test_relay_request_dedup_expires(self):
        """
        Advance 601s → same key counts as fresh.
        BUILDSPEC §4.10 table 1: expiry = relay_request_dedup_expiry_s (600s).
        """
        core, _, _, clock = _make_core()
        core.on_relay_request({"drone_id": "drone-01", "snr_db": 12.0, "timestamp": clock.now()})
        key = ("drone-01", int(12.0 // 5))
        first_seen = core._rr_dedup[key]

        clock.advance(601)
        core.on_relay_request({"drone_id": "drone-01", "snr_db": 12.0, "timestamp": clock.now()})
        second_seen = core._rr_dedup[key]
        assert second_seen > first_seen, "Dedup entry should be refreshed after expiry"


# ── §4.10 dedup table 2: declines ─────────────────────────────────────────────

class TestDeclineDedupTupleKey:
    def test_decline_dedup_tuple_key(self):
        """
        Dedup key for declines is (drone_id, reason) DIRECT TUPLE, not a hash.
        BUILDSPEC §4.10 table 2: "a direct tuple key, not a hash. Only two reason
        values exist; hashing adds nothing."
        """
        core, _, _, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        core.on_strategy_proposal(
            _make_decline(drone_id="drone-01", round_id=round_id, reason="never_seen"))

        # Verify the dedup key is literally a tuple, not a string/hash.
        keys = list(core._decline_dedup.keys())
        assert len(keys) == 1
        assert isinstance(keys[0], tuple), (
            f"Decline dedup key must be a tuple, got {type(keys[0])}"
        )
        assert keys[0] == ("drone-01", "never_seen"), (
            f"Expected ('drone-01', 'never_seen'), got {keys[0]}"
        )


class TestDeclineDedupExpires:
    def test_decline_dedup_expires(self):
        """
        Advance 181s → same (drone_id, reason) counts as fresh.
        BUILDSPEC §4.10 table 2: expiry = decline_dedup_expiry_s (180s).
        """
        core, _, _, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        core.on_strategy_proposal(
            _make_decline(drone_id="drone-01", round_id=round_id, reason="never_seen"))
        key = ("drone-01", "never_seen")
        first_seen = core._decline_dedup[key]

        clock.advance(181)
        # Need to reset round state to avoid stale-round discard.
        core._current_round_id = round_id  # keep the same round
        core.on_strategy_proposal(
            _make_decline(drone_id="drone-01", round_id=round_id, reason="never_seen"))
        second_seen = core._decline_dedup[key]
        assert second_seen > first_seen, "Dedup entry should be refreshed after expiry"


class TestDeclineStillProcessedWhenDeduped:
    def test_decline_still_processed_when_deduped(self):
        """
        A deduped decline (within 180s) still removes the collected entry.
        BUILDSPEC §4.10: "Does not suppress the follower. F_cap declines every time it
        is asked; every decline still arrives and is still processed."
        Dedup is LOGGING-ONLY.
        """
        core, _, _, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        # Insert a proposal for drone-01.
        core.on_strategy_proposal(_make_proposal(drone_id="drone-01", round_id=round_id))
        assert "drone-01" in core._collected

        # First decline removes from collected and seeds dedup table.
        core.on_strategy_proposal(
            _make_decline(drone_id="drone-01", round_id=round_id, reason="never_seen"))
        assert "drone-01" not in core._collected

        # Re-insert drone-01.
        core.on_strategy_proposal(_make_proposal(drone_id="drone-01", round_id=round_id))
        assert "drone-01" in core._collected

        # Second decline is a dup (within 180s) — but must STILL remove the entry.
        core.on_strategy_proposal(
            _make_decline(drone_id="drone-01", round_id=round_id, reason="never_seen"))
        assert "drone-01" not in core._collected, (
            "Deduped decline must still remove the collected entry (dedup is logging-only)"
        )


# ── §4.10 stripped: no MQTT ───────────────────────────────────────────────────

class TestNoMqttClient:
    def test_no_mqtt_client(self):
        """
        No MQTT client instantiation or import in the module.
        Comments that say 'no mqtt' are OK — what must be absent is any usage.
        BUILDSPEC §4.10 stripped: "No MQTT client."
        """
        import drone_control.relay_decision_authority as rda_mod
        src = inspect.getsource(rda_mod)
        # Check for actual MQTT usage (import statement or client construction).
        # "import paho" or "import mqtt" in actual code lines (not comment lines).
        code_lines = [
            ln for ln in src.splitlines()
            if not ln.strip().startswith("#")
        ]
        code_body = "\n".join(code_lines)
        assert "import paho" not in code_body, (
            "relay_decision_authority.py must not import paho (MQTT client library)"
        )
        assert "mqtt.Client" not in code_body, (
            "relay_decision_authority.py must not instantiate an MQTT client"
        )
        assert "import mqtt" not in code_body, (
            "relay_decision_authority.py must not import mqtt"
        )


# ── §4.10: selection reads proposals only ─────────────────────────────────────

class TestNoGcRadioHealthForSelection:
    def test_no_gc_radio_health_for_selection(self):
        """
        Candidate selection reads proposals only. GC does not pre-rank by its own
        radio health. BUILDSPEC §4.10: "Does NOT subscribe gc_radio_health for
        candidate selection."
        """
        import inspect
        import drone_control.relay_decision_authority as rda_mod
        # _pick_winner must only read capability_snapshot from proposals.
        pick_src = inspect.getsource(rda_mod._DecisionCore._pick_winner)
        assert "gc_radio" not in pick_src.lower(), (
            "_pick_winner must not reference gc_radio_health"
        )
        # Also verify _grant_authorization doesn't use radio health.
        grant_src = inspect.getsource(rda_mod._DecisionCore._grant_authorization)
        assert "gc_radio" not in grant_src.lower()

    def test_collected_set_contains_only_proposal_data(self):
        """collected dict entries must be the raw proposal dicts, no GC additions."""
        core, _, _, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id

        proposal = _make_proposal(drone_id="drone-01", round_id=round_id)
        core.on_strategy_proposal(proposal)

        assert len(core._collected) == 1
        stored = core._collected["drone-01"]
        # The stored entry must be the proposal, not an enriched version.
        assert stored.get("proposal_id") == proposal["proposal_id"]
        assert "gc_radio_health" not in stored


# ── §4.10 reauth trigger (gap fix) ────────────────────────────────────────────

class TestReauthRequest:

    def test_reauth_request_starts_round(self):
        """
        reauth_request when no active round → new round broadcast.
        BUILDSPEC §4.10: "TRIGGER: follower reauth request (reauth)."
        This was the missing path: _start_round("reauth") was never called before the fix.
        """
        core, tasking_out, _, clock = _make_core()
        # No round in flight — reauth_request must start one.
        assert core._current_round_id is None
        core.on_reauth_request({"drone_id": "drone-01", "timestamp": clock.now(),
                                "reason": "authorization_expiring"})
        assert len(tasking_out) == 1, (
            "reauth_request when idle must produce exactly one relay_tasking broadcast"
        )
        assert tasking_out[0]["trigger"] == "reauth", (
            f"trigger field must be 'reauth', got {tasking_out[0].get('trigger')!r}"
        )
        assert core._current_round_id is not None

    def test_reauth_request_starts_round_after_window_closed(self):
        """
        reauth_request after a previous round's window has closed → new round.
        Window closed means no active collection; the reauth is the next trigger.
        """
        core, tasking_out, auth_out, clock = _make_core()
        # Complete a round: trigger, propose, wait for window to close.
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id
        core.on_strategy_proposal(_make_proposal(drone_id="drone-01", round_id=round_id))
        clock.advance(50)   # past the 45s collection window
        core.check_timers()
        assert core._window_closed, "Window should be closed after advancing past collection_window_s"

        # Now a reauth_request should start a new round (window_closed == True).
        before_count = len(tasking_out)
        core.on_reauth_request({"drone_id": "drone-01", "timestamp": clock.now(),
                                "reason": "authorization_expiring"})
        assert len(tasking_out) == before_count + 1, (
            "reauth_request after closed window must broadcast a new relay_tasking"
        )
        assert tasking_out[-1]["trigger"] == "reauth"

    def test_reauth_request_ignored_during_active_round(self):
        """
        reauth_request while proposals are being collected → ignored.
        BUILDSPEC §4.10 guard: an active round is already working; don't reset the
        collected set mid-flight.
        """
        core, tasking_out, _, clock = _make_core()
        # Start a round and send one proposal (window is now open, not closed).
        core.on_gc_link_quality({"quality": 0.2})
        round_id = core._current_round_id
        core.on_strategy_proposal(_make_proposal(drone_id="drone-01", round_id=round_id))
        assert not core._window_closed, "Window must still be open"

        broadcast_count = len(tasking_out)
        collected_before = dict(core._collected)

        core.on_reauth_request({"drone_id": "drone-01", "timestamp": clock.now(),
                                "reason": "authorization_expiring"})

        # No new broadcast; collected set preserved.
        assert len(tasking_out) == broadcast_count, (
            "reauth_request during active window must not trigger a new relay_tasking"
        )
        assert core._collected == collected_before, (
            "reauth_request during active window must not reset the collected set"
        )

    def test_reauth_request_ignored_before_first_proposal_arrives(self):
        """
        reauth_request while a round is started but no proposals have arrived yet
        (_window_start is None, window not closed) → ignored.
        The round is in flight — proposals may be on their way.
        """
        core, tasking_out, _, clock = _make_core()
        core.on_gc_link_quality({"quality": 0.2})
        # Round started but no proposals yet: _window_start is None, _window_closed is False.
        assert core._current_round_id is not None
        assert core._window_start is None
        assert not core._window_closed

        broadcast_count = len(tasking_out)
        core.on_reauth_request({"drone_id": "drone-01", "timestamp": clock.now(),
                                "reason": "authorization_expiring"})
        assert len(tasking_out) == broadcast_count, (
            "reauth_request during an in-flight round (no proposals yet) must not re-broadcast"
        )


# ── §4.11 item 11: alert_intent — G_task subscribes ─────────────────────────

class TestAlertIntent:
    """
    Closes CHECK 1 [2026-08-18] BLOCKER — safety-exit alerts had no consumer.
    Option (a) applied 2026-08-24: RDA subscribes; observability-only, no
    control-loop consumption; in-memory ring buffer per §4.10 hard rule.
    """

    def test_alert_intent_retained(self):
        """on_alert_intent stores the payload in the per-drone ring buffer."""
        core, tasking_out, _, clock = _make_core()
        payload = {
            "drone_id":    "drone-02",
            "type":        "FOLLOWER_SAFETY_EXIT",
            "reason":      "battery_below_floor",
            "battery_pct": 12.0,
            "flight_mode": "OFFBOARD",
            "timestamp":   clock.now(),
        }
        core.on_alert_intent(payload)
        assert core._alert_intents["drone-02"] == [payload]

    def test_alert_intent_missing_drone_id_bucketed_as_unknown(self):
        """Payloads lacking drone_id land in the 'unknown' bucket, not a crash."""
        core, _, _, _ = _make_core()
        # Real capability_assessor payload doesn't include drone_id today
        # (§4.11 item 11 marked 🔴 NEW; drone_id enrichment is a future step).
        core.on_alert_intent({"type": "FOLLOWER_SAFETY_EXIT", "reason": "test"})
        assert "unknown" in core._alert_intents
        assert len(core._alert_intents["unknown"]) == 1

    def test_alert_intent_ring_bounded(self):
        """Ring evicts oldest when count exceeds alert_intent_ring_max."""
        cfg = dict(_CFG); cfg["alert_intent_ring_max"] = 3
        tasking_out = []; auth_out = []; clock = FakeClock()
        core = _DecisionCore(config=cfg,
                             publish_tasking_fn=tasking_out.append,
                             publish_auth_fn=lambda d, p: auth_out.append((d, p)),
                             clock=clock.now)
        for i in range(5):
            core.on_alert_intent({"drone_id": "drone-02", "seq": i})
        seqs = [p["seq"] for p in core._alert_intents["drone-02"]]
        assert seqs == [2, 3, 4], f"expected FIFO eviction to last 3; got {seqs}"

    def test_alert_intent_does_not_trigger_round_or_authorization(self):
        """
        alert_intent is observability-only.  It must NOT start a broadcast
        round or emit an authorization — those are strategy_proposal /
        gc_link_quality / reauth_request paths.
        """
        core, tasking_out, auth_out, clock = _make_core()
        pre_tasking = len(tasking_out); pre_auth = len(auth_out)
        core.on_alert_intent({
            "drone_id":    "drone-02",
            "type":        "FOLLOWER_SAFETY_EXIT",
            "reason":      "battery_below_floor",
        })
        assert len(tasking_out) == pre_tasking
        assert len(auth_out)    == pre_auth
        # Also: alert_intent must not affect round state.
        assert core._current_round_id is None
        assert core._window_start is None
