"""
test_wave4_geometry.py — Wave 4 gate tests for relay_bt/geometry.py.

TEST_PROTOCOL: §5.5  Archetype A (pure functions)

PROVEN column:
  test_haversine_known_pair                   -> BUILDSPEC §4.12
  test_band_bounds_ordering                   -> BUILDSPEC §4.12
  test_return_margin_arithmetic               -> BUILDSPEC §4.12 (flat buffer)
  test_battery_cost_raises_without_model_constants -> BUILDSPEC §7.1 hard stop
"""

import sys
import os
import math
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
_RBT = os.path.join(_PKG_ROOT, "drone_control", "relay_bt")
for p in (_PKG_ROOT, _RBT):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.relay_bt.geometry import (
    haversine, band_bounds, estimate_battery_cost,
    band_center, bucket_position,
)

# ── Known positions for haversine check ──────────────────────────────────────

# Zürich → Bern: ~95 km great-circle
_ZRH = {"lat": 47.3769, "lon": 8.5417}
_BRN = {"lat": 46.9480, "lon": 7.4474}
_ZRH_BRN_KM = 95.0   # ±5% tolerance

# ── Config stubs ──────────────────────────────────────────────────────────────

_CFG = {"return_margin_buffer_pct": 10.0}

# Fake model config with real floats for arithmetic tests
_REAL_MODEL = {
    "cruise_speed_mps":           10.0,
    "consumption_rate_pct_per_s":  0.01,   # 1 %/s
}


class TestHaversine:

    def test_haversine_known_pair(self):
        """Zürich → Bern ≈ 95 km.  Within ±5% tolerance.  BUILDSPEC §4.12."""
        dist_m = haversine(_ZRH, _BRN)
        assert abs(dist_m / 1000 - _ZRH_BRN_KM) / _ZRH_BRN_KM < 0.05, (
            f"haversine(ZRH,BRN) = {dist_m/1000:.1f} km, expected ~{_ZRH_BRN_KM} km"
        )

    def test_haversine_same_point_is_zero(self):
        pos = {"lat": 47.39, "lon": 8.54}
        assert haversine(pos, pos) == 0.0

    def test_haversine_symmetric(self):
        assert math.isclose(haversine(_ZRH, _BRN), haversine(_BRN, _ZRH), rel_tol=1e-9)

    def test_haversine_short_range(self):
        """50 m north displacement ≈ 0.00045° lat ≈ 50 m.  Within 1 m."""
        a = {"lat": 47.3900, "lon": 8.5400}
        b = {"lat": 47.3905, "lon": 8.5400}   # ~55 m north
        dist = haversine(a, b)
        assert 50 < dist < 60, f"Expected ~55 m, got {dist:.1f} m"


class TestBandBounds:

    def test_band_bounds_ordering_feasible(self):
        """t_hi > t_lo when r_G + r_L > D.  BUILDSPEC §4.12."""
        D, r_G, r_L = 1000.0, 700.0, 700.0   # sum > D → feasible
        t_lo, t_hi = band_bounds(D, r_G, r_L)
        assert t_hi > t_lo, f"Feasible case: t_hi={t_hi:.3f} should > t_lo={t_lo:.3f}"

    def test_band_bounds_inverted_when_infeasible(self):
        """t_hi < t_lo when r_G + r_L < D — the EXIT signal."""
        D, r_G, r_L = 2000.0, 700.0, 700.0   # sum < D → infeasible
        t_lo, t_hi = band_bounds(D, r_G, r_L)
        assert t_hi < t_lo, f"Infeasible case: t_hi={t_hi:.3f} should < t_lo={t_lo:.3f}"

    def test_band_bounds_symmetric_ranges(self):
        """Equal r_G and r_L → band centred at t=0.5."""
        t_lo, t_hi = band_bounds(1000.0, 600.0, 600.0)
        assert math.isclose((t_lo + t_hi) / 2, 0.5, abs_tol=1e-9)

    def test_band_bounds_zero_distance(self):
        """D=0 → (0.0, 1.0) guard."""
        t_lo, t_hi = band_bounds(0.0, 500.0, 500.0)
        assert t_lo == 0.0 and t_hi == 1.0


class TestEstimateBatteryCost:

    def test_battery_cost_raises_without_model_constants(self):
        """
        Passing _Unresolved sentinels as model constants must raise RuntimeError.
        BUILDSPEC §7.1 hard stop: no default, no fallback.
        """
        sys.path.insert(0, os.path.join(_PKG_ROOT, "drone_control"))
        from config.demo_config import _Unresolved
        bad_model = {
            "consumption_rate_pct_per_s": _Unresolved("consumption_rate_pct_per_s"),
            "cruise_speed_mps":           _Unresolved("cruise_speed_mps"),
        }

        pos_a = {"lat": 47.39, "lon": 8.54}
        pos_b = {"lat": 47.39, "lon": 8.54}

        with pytest.raises(RuntimeError):
            estimate_battery_cost(pos_a, pos_b, 50.0, bad_model, _CFG)

    def test_return_margin_arithmetic(self):
        """
        cost_pct + 10.0 flat buffer.  battery_pct == required → return_margin_ok True.
        BUILDSPEC §4.12: flat buffer, NOT a multiplier.
        """
        # 1000 m at 10 m/s = 100 s.  100 s * 0.01 %/s = 1.0 %
        # required = 1.0 + 10.0 = 11.0 %
        # battery_pct = 11.0 → exactly at requirement → True
        pos_a = {"lat": 47.3900, "lon": 8.5400}
        pos_b = {"lat": 47.3900, "lon": 8.5490}   # ~approx 540 m east

        ok, required, cost = estimate_battery_cost(pos_a, pos_b, 999.0, _REAL_MODEL, _CFG)
        # Recompute expected values
        dist = haversine(pos_a, pos_b)
        exp_cost = (dist / 10.0) * 0.01
        exp_required = exp_cost + 10.0

        assert math.isclose(cost, exp_cost, rel_tol=1e-6), (
            f"cost={cost:.4f}  exp={exp_cost:.4f}"
        )
        assert math.isclose(required, exp_required, rel_tol=1e-6), (
            f"required={required:.4f}  exp={exp_required:.4f}"
        )
        assert ok is True   # 999% battery >> required

    def test_return_margin_exactly_at_boundary(self):
        """battery_pct == required_pct → True (>= not >)."""
        pos = {"lat": 47.39, "lon": 8.54}
        _, required, _ = estimate_battery_cost(pos, pos, 50.0, _REAL_MODEL, _CFG)
        # Distance 0 → cost = 0 → required = 0 + 10 = 10
        ok, req, cost = estimate_battery_cost(pos, pos, 10.0, _REAL_MODEL, _CFG)
        assert math.isclose(req, 10.0, abs_tol=1e-9)
        assert ok is True, "battery_pct == required should pass (>=)"

    def test_return_margin_just_below_boundary(self):
        """battery_pct == required - epsilon → False."""
        pos = {"lat": 47.39, "lon": 8.54}
        ok, req, _ = estimate_battery_cost(pos, pos, 9.999, _REAL_MODEL, _CFG)
        assert ok is False, f"battery_pct 9.999 < required {req:.3f} should fail"

    def test_buffer_is_flat_not_multiplier(self):
        """
        Flat buffer = cost + 10.0, not cost * 1.1.
        Verify required - cost == 10.0 exactly, regardless of cost magnitude.
        BUILDSPEC §4.12: 'flat buffer, added to computed cost'.
        """
        pos_near = {"lat": 47.390, "lon": 8.540}
        pos_far  = {"lat": 47.450, "lon": 8.610}   # ~8 km

        _, required, cost = estimate_battery_cost(
            pos_near, pos_far, 50.0, _REAL_MODEL, _CFG)

        buffer_applied = required - cost
        assert math.isclose(buffer_applied, 10.0, abs_tol=1e-9), (
            f"Buffer should be 10.0 flat, got {buffer_applied:.6f} "
            f"(if this were a multiplier it would be {cost * 0.1:.4f})"
        )

    def test_returns_triple(self):
        """Return value is (return_margin_ok: bool, required_pct: float, cost_pct: float)."""
        pos = {"lat": 47.39, "lon": 8.54}
        result = estimate_battery_cost(pos, pos, 50.0, _REAL_MODEL, _CFG)
        assert len(result) == 3
        ok, required, cost = result
        assert isinstance(ok, bool)
        assert isinstance(required, float)
        assert isinstance(cost, float)
