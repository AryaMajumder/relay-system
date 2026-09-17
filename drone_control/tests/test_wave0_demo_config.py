"""
test_wave0_demo_config.py — Wave 0 gate tests for demo_config.py.

TEST_PROTOCOL: §5.1
HARNESS TYPE: A (pure data — call directly, assert values)

Every test here must pass before the Wave 0 gate opens.  The PROVEN column
below ties each test back to the buildspec rule it verifies.

  test_section_31_keys                -> BUILDSPEC §3.1, §3.2 (required keys present)
  test_section_32_keys / test_value   -> BUILDSPEC §3.2 (exact values; test_value is parametrized)
  test_severity_zero_gives_baseline   -> BUILDSPEC §3.1, §4.1 (noise formula lower endpoint)
  test_severity_one_gives_worst_case  -> BUILDSPEC §3.1, §4.1 (noise formula upper endpoint)
  TestPerModelSection                 -> BUILDSPEC §7.1 (placeholder floats; prior session resolved sentinels to concrete values)
"""

import math
import sys
import os
import pytest

# ── Path bootstrap ────────────────────────────────────────────────────────────
# Tests run from the repo root or the tests/ directory; either way we need the
# drone_control package on sys.path.
_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from drone_control.config.demo_config import DEMO_CONFIG, DRONE_MODELS, DRONE_MODEL_ASSIGNMENT


# ── §3.1 required keys ────────────────────────────────────────────────────────

_SECTION_31_KEYS = [
    "baseline_noise_dbm",
    "noise_range_db",
    "tx_power_dbm",
    "frequency_mhz",
    "radio_health_max_age_s",
    "position_bucket_m",
    "LINK_MARGINAL_QUALITY",
    "relay_position_tracker_hz",
    "geofence_polygon",
    "battery_reserve_pct",
    "bt_tick_period_s",
]

# ── §3.2 required keys and their exact values ─────────────────────────────────

_SECTION_32 = {
    "collection_window_s":          45,
    "rebroadcast_pause_s":          120,
    "relay_request_dedup_expiry_s": 600,
    "decline_dedup_expiry_s":       180,
    "boot_grace_window_s":          600,
    "tolerance_radius_m":           10.0,
    "authorization_validity_s":     1800,
    "reauth_response_timeout_s":    120,
    "return_margin_buffer_pct":     10.0,
}


class TestAllRequiredKeysPresent:
    """BUILDSPEC §3.1 and §3.2 — nothing downstream reads a missing key."""

    def test_section_31_keys(self):
        missing = [k for k in _SECTION_31_KEYS if k not in DEMO_CONFIG]
        assert missing == [], f"Missing §3.1 keys: {missing}"

    def test_section_32_keys(self):
        missing = [k for k in _SECTION_32 if k not in DEMO_CONFIG]
        assert missing == [], f"Missing §3.2 keys: {missing}"


class TestResolvedValuesExact:
    """BUILDSPEC §3.2 — exact resolved values, no rounding tolerance."""

    @pytest.mark.parametrize("key,expected", list(_SECTION_32.items()))
    def test_value(self, key, expected):
        assert DEMO_CONFIG[key] == expected, (
            f"DEMO_CONFIG['{key}'] = {DEMO_CONFIG[key]!r}, expected {expected!r}"
        )


class TestNoiseModelEndpoints:
    """BUILDSPEC §3.1, §4.1 — forward noise formula endpoints.

    Forward formula: noise_dbm = baseline_noise_dbm + severity * noise_range_db
    At severity 0.0 → -95 dBm (thermal noise floor, no jamming).
    At severity 1.0 → -55 dBm (fully jammed, -95 + 40).
    """

    def _noise(self, severity: float) -> float:
        return (
            DEMO_CONFIG["baseline_noise_dbm"]
            + severity * DEMO_CONFIG["noise_range_db"]
        )

    def test_severity_zero_gives_baseline(self):
        assert math.isclose(self._noise(0.0), -95.0, abs_tol=1e-9), (
            f"severity=0.0 should give -95.0 dBm, got {self._noise(0.0)}"
        )

    def test_severity_one_gives_worst_case(self):
        assert math.isclose(self._noise(1.0), -55.0, abs_tol=1e-9), (
            f"severity=1.0 should give -55.0 dBm, got {self._noise(1.0)}"
        )


class TestPerModelSection:
    """BUILDSPEC §7.1 — per-model constants are placeholder floats (resolved).

    Both consumption_rate_pct_per_s and cruise_speed_mps are now supplied
    as arbitrary placeholder values for the demo airframe.  They must be
    positive floats that support arithmetic without raising.
    """

    def _get_constant(self, key: str):
        model_name = DRONE_MODEL_ASSIGNMENT["drone-01"]
        return DRONE_MODELS[model_name][key]

    def test_drone_model_assignment_exists(self):
        assert "drone-01" in DRONE_MODEL_ASSIGNMENT
        assert "drone-02" in DRONE_MODEL_ASSIGNMENT

    def test_consumption_rate_is_positive_float(self):
        v = self._get_constant("consumption_rate_pct_per_s")
        assert isinstance(v, float) and v > 0, f"expected positive float, got {v!r}"

    def test_cruise_speed_is_positive_float(self):
        v = self._get_constant("cruise_speed_mps")
        assert isinstance(v, float) and v > 0, f"expected positive float, got {v!r}"

    def test_consumption_rate_supports_arithmetic(self):
        v = self._get_constant("consumption_rate_pct_per_s")
        result = 30.0 * v
        assert result > 0

    def test_cruise_speed_supports_arithmetic(self):
        v = self._get_constant("cruise_speed_mps")
        result = 500.0 / v
        assert result > 0
