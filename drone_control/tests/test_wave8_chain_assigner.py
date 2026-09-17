"""
test_wave8_chain_assigner.py — Wave 8 gate tests for chain_assigner.py.

TEST_PROTOCOL: §5.12  Archetype C (subscribe → compute → publish)
BUILDSPEC:     §2.7, §5.3 Decision 5

PROVEN column:
  test_r_target_verbatim              -> BUILDSPEC §5.3 Decision 5
  test_tolerance_radius_m_passthrough -> BUILDSPEC §2.7 (radius verbatim)
  test_valid_until_passthrough        -> BUILDSPEC §2.7 (validity verbatim)
  test_only_eta_computed_not_r_target -> BUILDSPEC §2.7 ("eta_s is the sole computed field")
"""

import sys
import os
import math
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for p in (
    _PKG_ROOT,
    os.path.join(_PKG_ROOT, "drone_control"),
    os.path.join(_PKG_ROOT, "drone_control", "relay_bt"),
):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.chain_assigner import _ChainAssignerCore


# ── Fixtures ──────────────────────────────────────────────────────────────────

_R_TARGET = {"lat": 47.3914159, "lon": 8.5412718, "alt_m": 12.5}

_CFG = {
    # Supplies cruise_speed_mps directly so tests don't depend on DEMO_CONFIG values.
    # Tests that check eta_s arithmetic use this speed (12.0 m/s) as the known divisor.
    "DRONE_MODELS":   {"generic": {"cruise_speed_mps": 12.0, "consumption_rate_pct_per_s": 0.05}},
    "drone_model_id": "generic",
}

_CURRENT_POS = {"lat": 47.3900, "lon": 8.5400, "alt_m": 0.0}


def _make_auth(
    strategy="CONTINUOUS_RELAY",
    r_target=None,
    tolerance_radius_m=10.0,
    valid_until=12000.0,
    proposal_id="prop-aaa000000001",
):
    return {
        "proposal_id":        proposal_id,
        "drone_id":           "drone-01",
        "round_id":           "round-test",
        "strategy":           strategy,
        "r_target":           r_target or dict(_R_TARGET),
        "tolerance_radius_m": tolerance_radius_m,
        "valid_until":        valid_until,
        "timestamp":          10000.0,
    }


def _make_core(with_pos=True):
    assignments = []
    core = _ChainAssignerCore(
        drone_id="drone-01",
        config=dict(_CFG),
        publish_fn=assignments.append,
        # Fixed-value clock: timestamp in relay_assignment is deterministic; tests
        # don't check it, so a lambda beats injecting a real clock.
        clock=lambda: 10100.0,
    )
    if with_pos:
        core.on_drone_state({"position": dict(_CURRENT_POS)})
    return core, assignments


# ── §5.3 Decision 5: r_target verbatim ───────────────────────────────────────

class TestRTargetVerbatim:
    def test_r_target_verbatim(self):
        """
        relay_assignment.r_target must be byte-identical to authorization.r_target.
        BUILDSPEC §5.3 Decision 5: "never recompute a position that has been authorized."
        """
        r = {"lat": 47.3914159, "lon": 8.5412718, "alt_m": 12.5}
        core, assignments = _make_core()
        core.on_authorization(_make_auth(r_target=r))

        assert len(assignments) == 1
        assert assignments[0]["r_target"] == r, (
            f"r_target must be byte-identical.\n"
            f"  authorization: {r}\n"
            f"  assignment:    {assignments[0]['r_target']}"
        )

    def test_r_target_not_snapped_or_modified(self):
        """chain_assigner must not call bucket_position() on r_target."""
        import inspect
        import drone_control.chain_assigner as mod
        src = inspect.getsource(mod._ChainAssignerCore.on_authorization)
        assert "bucket_position" not in src, (
            "chain_assigner must not snap r_target (Decision 5)"
        )
        assert "haversine" not in src, (
            "chain_assigner must not call haversine on r_target in on_authorization "
            "(haversine is only for eta_s in _compute_eta_s)"
        )

    def test_all_three_movement_strategies_produce_assignment(self):
        """CONTINUOUS_RELAY, CHAIN_RELAY, REPOSITION_RELAY all produce assignments."""
        for strategy, pid in [
            ("CONTINUOUS_RELAY",  "prop-001"),
            ("CHAIN_RELAY",       "prop-002"),
            ("REPOSITION_RELAY",  "prop-003"),
        ]:
            core, assignments = _make_core()
            core.on_authorization(_make_auth(strategy=strategy, proposal_id=pid))
            assert len(assignments) == 1, f"{strategy} must produce exactly one assignment"

    def test_exit_relay_produces_no_assignment(self):
        """EXIT_RELAY does not produce a relay_assignment."""
        core, assignments = _make_core()
        core.on_authorization(_make_auth(strategy="EXIT_RELAY"))
        assert len(assignments) == 0, "EXIT_RELAY must not produce a relay_assignment"


# ── §2.7: tolerance_radius_m and valid_until passthrough ─────────────────────

class TestRadiusAndValidityPassthrough:
    def test_tolerance_radius_m_passthrough(self):
        """tolerance_radius_m must be carried unchanged from authorization to assignment."""
        core, assignments = _make_core()
        core.on_authorization(_make_auth(tolerance_radius_m=15.0))
        assert len(assignments) == 1
        assert assignments[0]["tolerance_radius_m"] == 15.0, (
            "tolerance_radius_m must be verbatim from authorization"
        )

    def test_valid_until_passthrough(self):
        """valid_until must be carried unchanged from authorization to assignment."""
        core, assignments = _make_core()
        core.on_authorization(_make_auth(valid_until=13800.0))
        assert len(assignments) == 1
        assert assignments[0]["valid_until"] == 13800.0, (
            "valid_until must be verbatim from authorization"
        )

    def test_schema_complete(self):
        """All §2.7 fields present in relay_assignment."""
        required = {"drone_id", "r_target", "tolerance_radius_m", "valid_until",
                    "eta_s", "timestamp"}
        core, assignments = _make_core()
        core.on_authorization(_make_auth())
        assert len(assignments) == 1
        missing = required - assignments[0].keys()
        assert not missing, f"relay_assignment missing §2.7 fields: {missing}"


# ── §2.7: eta_s is the sole computed field ────────────────────────────────────

class TestComputesOnlyEta:
    def test_eta_s_is_computed(self):
        """eta_s is computed from distance / speed. Other fields are passthroughs."""
        core, assignments = _make_core(with_pos=True)
        core.on_authorization(_make_auth())
        assert len(assignments) == 1
        a = assignments[0]

        # eta_s should be > 0 (current_pos and r_target are ~100m apart)
        assert isinstance(a["eta_s"], float), "eta_s must be a float"
        assert a["eta_s"] >= 0.0, "eta_s must be non-negative"

    def test_eta_zero_without_current_pos(self):
        """eta_s = 0.0 when current_pos is not known yet."""
        core, assignments = _make_core(with_pos=False)
        core.on_authorization(_make_auth())
        assert len(assignments) == 1
        assert assignments[0]["eta_s"] == 0.0, (
            "eta_s must be 0.0 when current_pos is unknown"
        )

    def test_eta_uses_cruise_speed_from_config(self):
        """eta_s = distance / cruise_speed_mps from DRONE_MODELS config."""
        from relay_bt.geometry import haversine

        # Put current_pos and r_target exactly 1200m apart (approx)
        # So that eta_s = 1200 / 12.0 = 100s
        pos = {"lat": 47.3900, "lon": 8.5400, "alt_m": 0.0}
        # r_target ~0.01 deg north ≈ 1111m
        r   = {"lat": 47.4010, "lon": 8.5400, "alt_m": 10.0}

        core, assignments = _make_core(with_pos=False)
        core.on_drone_state({"position": pos})
        core.on_authorization(_make_auth(r_target=r))

        assert len(assignments) == 1
        eta = assignments[0]["eta_s"]
        dist = haversine(pos, r)
        expected = round(dist / 12.0, 1)
        assert math.isclose(eta, expected, rel_tol=1e-3), (
            f"eta_s={eta:.1f} expected {expected:.1f} (dist={dist:.1f}m / speed=12.0)"
        )

    def test_only_eta_computed_not_r_target(self):
        """r_target, tolerance_radius_m, valid_until are NOT recomputed."""
        r = {"lat": 47.3914159, "lon": 8.5412718, "alt_m": 12.5}
        auth = _make_auth(r_target=r, tolerance_radius_m=7.5, valid_until=14400.0)

        core, assignments = _make_core()
        core.on_authorization(auth)
        a = assignments[0]

        # These must be byte-identical to the authorization — not recomputed.
        assert a["r_target"] == r
        assert a["tolerance_radius_m"] == 7.5
        assert a["valid_until"] == 14400.0
        # Only eta_s differs from the authorization (it's computed, not copied).
        assert "eta_s" in a
