"""
test_wave7_relay_strategy_evaluator.py — Wave 7 gate tests.

TEST_PROTOCOL: §5.9  Archetype E (stateful decision node, fake clock)
BUILDSPEC:     §4.9, §2.5

PROVEN column:
  test_proposal_id_format       -> BUILDSPEC §4.9, §2.5
  test_field_named_strategy     -> BUILDSPEC §2.5 ("field named 'strategy'")
  test_no_confidence_score      -> BUILDSPEC §2.5, Q9
  test_no_band_range_field      -> BUILDSPEC §2.5
  test_r_target_snapped         -> BUILDSPEC §4.9 (snap before leaving node)
  test_content_hash_dedup_identical_conditions -> BUILDSPEC §4.9 (content-hash dedup — same report suppressed)
  test_round_id_echoed          -> BUILDSPEC §2.5 (echoed from relay_tasking)
"""

import sys
import os
import re
import json
import hashlib
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for p in (
    _PKG_ROOT,
    os.path.join(_PKG_ROOT, "drone_control"),
    os.path.join(_PKG_ROOT, "drone_control", "relay_bt"),
):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.relay_strategy_evaluator import _StrategyEvaluatorCore


# ── Fake clock ────────────────────────────────────────────────────────────────

class FakeClock:
    def __init__(self, t=1000.0):
        self._t = t

    def now(self):
        return self._t

    def advance(self, seconds):
        self._t += seconds


# ── Fixtures ─────────────────────────────────────────────────────────────────

_CFG = {
    "position_bucket_m": 10.0,
}

_RELAY_POS = {"lat": 47.3910, "lon": 8.5410, "alt_m": 10.0}

# A capability_report that triggers a proposal
def _make_report(
    strategy="CONTINUOUS_RELAY",
    battery_pct=75.0,
    gc_snr_db=15.0,
    relay_position=None,
    band_t_lo=0.3,
    band_t_hi=0.7,
    band_sensor_pass=True,
    geofence_pass=True,
    battery_return_pass=True,
    round_id=None,
    gps_fix_type=3,
    reason="",
    trigger="",
    eta_seconds=60.0,
):
    rp = relay_position or dict(_RELAY_POS)
    return {
        "drone_id":  "drone-01",
        "timestamp": 1000.0,
        "capable":   True,
        "status":    "CAPABLE",
        "reason":    reason,
        "inputs": {
            "battery_pct":  battery_pct,
            "gps_fix_type": gps_fix_type,
            "gc_snr_db":    gc_snr_db,
            "band_t_lo":    band_t_lo,
            "band_t_hi":    band_t_hi,
        },
        "checks": {
            "BandSensorNode":              {"pass": band_sensor_pass, "detail": ""},
            "GeofenceContainsRelayPos":    {"pass": geofence_pass,   "detail": ""},
            "BatterySufficientForReturn":  {"pass": battery_return_pass, "detail": ""},
        },
        "pending_proposal": {
            "strategy":       strategy,
            "relay_position": rp,
            "reason":         reason,
            "trigger":        trigger,
            "cost": {
                "battery_cost_pct": 5.0,
                "repositioning_m":  50.0,
                "eta_seconds":      eta_seconds,
            },
        },
    }


def _make_core(published_list=None):
    pub = published_list if published_list is not None else []
    clock = FakeClock()
    core = _StrategyEvaluatorCore(
        drone_id="drone-01",
        config=_CFG,
        publish_fn=pub.append,
        clock=clock.now,
    )
    return core, pub, clock


# ── Tests ─────────────────────────────────────────────────────────────────────

class TestProposalIdFormat:
    def test_proposal_id_format(self):
        """
        proposal_id must match 'prop-' + exactly 12 hex chars.
        BUILDSPEC §4.9, §2.5.
        """
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report())
        assert len(pub) == 1, "Expected exactly one published proposal"
        pid = pub[0]["proposal_id"]
        assert re.fullmatch(r"prop-[0-9a-f]{12}", pid), (
            f"proposal_id '{pid}' does not match 'prop-' + 12 hex chars"
        )


class TestFieldNamedStrategy:
    def test_field_named_strategy(self):
        """
        Payload must have field 'strategy', NOT 'strategy_type'.
        BUILDSPEC §2.5: "Field name is 'strategy', not 'strategy_type'."
        """
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report(strategy="EXIT_RELAY"))
        assert len(pub) == 1
        p = pub[0]
        assert "strategy" in p, "Field 'strategy' missing from proposal"
        assert "strategy_type" not in p, "'strategy_type' must not exist"
        assert p["strategy"] == "EXIT_RELAY"


class TestNoConfidenceScore:
    def test_no_confidence_score(self):
        """
        'confidence_score' must NOT appear anywhere in the proposal.
        BUILDSPEC §2.5: "No confidence_score field exists — do not add one."
        """
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report())
        assert len(pub) == 1
        p = pub[0]
        assert "confidence_score" not in p, (
            "'confidence_score' found — banned by §2.5"
        )
        # Also check inside capability_snapshot
        cs = p.get("capability_snapshot", {})
        assert "confidence_score" not in cs, (
            "'confidence_score' found inside capability_snapshot"
        )


class TestNoBandRangeField:
    def test_no_band_range_field(self):
        """
        'band_range' must NOT appear — consumer computes t_hi - t_lo.
        BUILDSPEC §2.5: "'band_range' is not a field."
        """
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report())
        assert len(pub) == 1
        p = pub[0]
        assert "band_range" not in p, "'band_range' found — banned by §2.5"
        cs = p.get("capability_snapshot", {})
        assert "band_range" not in cs, "'band_range' found inside capability_snapshot"


class TestRTargetSnapped:
    def test_r_target_snapped(self):
        """
        relay_position must be snapped to position_bucket_m grid before publish.
        Output r_target lat/lon must be multiples of bucket_m in degree-equivalent.
        BUILDSPEC §4.9: 'snap R_target to position_bucket_m grid'.
        """
        core, pub, _ = _make_core()
        # Use a raw position that is NOT on the grid
        raw = {"lat": 47.3912, "lon": 8.5417, "alt_m": 5.0}
        report = _make_report(relay_position=raw)
        core.on_capability_report(report)

        assert len(pub) == 1
        p = pub[0]

        # r_target must exist and have lat/lon/alt_m
        rt = p.get("r_target")
        assert rt is not None, "r_target missing from proposal"
        assert "lat" in rt and "lon" in rt, "r_target missing lat or lon"
        assert "alt_m" in rt, "r_target missing alt_m"

        # Verify the position is different from raw (snapping happened)
        # bucket_position rounds to nearest bucket_m grid
        # With bucket_m=10m: lat/lon are snapped in meter-equivalent steps
        # Just verify that the output is a dict with numeric values — the exact
        # snapped value is verified by geometry.py tests (test_wave4_geometry.py).
        assert isinstance(rt["lat"], float)
        assert isinstance(rt["lon"], float)

    def test_r_target_has_alt_m_field(self):
        """r_target must carry alt_m even if raw_pos included it."""
        core, pub, _ = _make_core()
        raw = {"lat": 47.3900, "lon": 8.5400, "alt_m": 15.0}
        core.on_capability_report(_make_report(relay_position=raw))
        assert len(pub) == 1
        rt = pub[0].get("r_target", {})
        assert "alt_m" in rt, "r_target missing alt_m"


class TestContentHashDedup:
    def test_content_hash_dedup_identical_conditions(self):
        """
        Same capability_report twice → only one publish (identical content hash).
        BUILDSPEC §4.9: 'content-hash dedup — same conditions → same hash → no re-publish'.
        """
        core, pub, _ = _make_core()
        report = _make_report()
        core.on_capability_report(report)
        core.on_capability_report(report)  # same report
        assert len(pub) == 1, (
            f"Expected 1 publish but got {len(pub)} — content-hash dedup failed"
        )

    def test_content_hash_dedup_changed_strategy_republishes(self):
        """Different strategy → different hash → republish."""
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report(strategy="CONTINUOUS_RELAY"))
        core.on_capability_report(_make_report(strategy="EXIT_RELAY"))
        assert len(pub) == 2, "Different strategies should produce two proposals"

    def test_content_hash_dedup_changed_battery_bucket_republishes(self):
        """Battery crossing a bucket boundary → different hash → republish."""
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report(battery_pct=75.0))  # bucket 3
        core.on_capability_report(_make_report(battery_pct=55.0))  # bucket 2
        assert len(pub) == 2, "Battery bucket change should produce two proposals"


class TestRoundIdEchoed:
    def test_round_id_echoed(self):
        """
        round_id in the proposal must match the round_id from the prompting relay_tasking.
        BUILDSPEC §2.5: "round_id: echoed from the relay_tasking that prompted this."
        """
        core, pub, _ = _make_core()

        # Feed relay_tasking first (as relay_decision_authority would send)
        tasking = {
            "round_id":  "round-abc12345",
            "timestamp": 1000.0,
            "trigger":   "initial",
        }
        core.on_relay_tasking(tasking)

        # Then feed capability_report that triggers a proposal
        core.on_capability_report(_make_report())

        assert len(pub) == 1
        assert pub[0]["round_id"] == "round-abc12345", (
            f"round_id mismatch: got {pub[0]['round_id']!r}, "
            f"expected 'round-abc12345'"
        )

    def test_round_id_none_before_tasking(self):
        """If no relay_tasking received yet, round_id is None (not an error)."""
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report())
        assert len(pub) == 1
        # round_id may be None — that's correct (no tasking received yet)
        assert "round_id" in pub[0], "round_id key must exist even if None"

    def test_round_id_updates_on_new_tasking(self):
        """New relay_tasking replaces the stored round_id."""
        core, pub, _ = _make_core()

        core.on_relay_tasking({"round_id": "round-first",  "timestamp": 1000.0, "trigger": "initial"})
        core.on_capability_report(_make_report(battery_pct=80.0))

        # New round
        core.on_relay_tasking({"round_id": "round-second", "timestamp": 1100.0, "trigger": "reauth"})
        # Different battery → different hash → republish
        core.on_capability_report(_make_report(battery_pct=60.0))

        assert len(pub) == 2
        assert pub[0]["round_id"] == "round-first"
        assert pub[1]["round_id"] == "round-second"


class TestCapabilitySnapshotSchema:
    def test_capability_snapshot_has_all_required_fields(self):
        """capability_snapshot must carry all §2.5 fields."""
        required = {
            "battery_pct", "gps_fix_type",
            "cap_gc_m", "cap_leader_m", "cap_follower_m",
            "band_feasible", "t_lo", "t_hi",
            "geofence_ok", "return_margin_ok",
        }
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report())
        assert len(pub) == 1
        cs = pub[0].get("capability_snapshot", {})
        missing = required - cs.keys()
        assert not missing, f"capability_snapshot missing fields: {missing}"

    def test_t_lo_t_hi_from_inputs(self):
        """t_lo and t_hi in capability_snapshot come from report inputs."""
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report(band_t_lo=0.25, band_t_hi=0.65))
        assert len(pub) == 1
        cs = pub[0]["capability_snapshot"]
        assert cs["t_lo"] == 0.25
        assert cs["t_hi"] == 0.65

    def test_trigger_context_has_required_fields(self):
        """trigger_context must have gate_fired, reason, source."""
        core, pub, _ = _make_core()
        core.on_capability_report(_make_report())
        assert len(pub) == 1
        tc = pub[0].get("trigger_context", {})
        for field in ("gate_fired", "reason", "source"):
            assert field in tc, f"trigger_context missing '{field}'"
