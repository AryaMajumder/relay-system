"""
test_wave8_px4_agent.py — Wave 8 gate tests for px4_agent.py.

TEST_PROTOCOL: §5.13  Archetype F (fake MQTT/PX4 seam; capture setpoints)
BUILDSPEC:     §4.14

PROVEN column:
  test_ned_north_displacement       -> _global_to_ned x axis (North = positive x)
  test_ned_east_displacement        -> _global_to_ned y axis (East = positive y)
  test_ned_altitude_inverted        -> _global_to_ned z axis (NED z positive-down)
  test_ned_at_home                  -> _global_to_ned identity at home position
  test_ned_east_scales_with_cos_lat -> _global_to_ned cos(lat) East longitude scaling
  test_publish_drops_without_home   -> publish_setpoint: home_pos required
  test_publish_drops_without_mqtt   -> publish_setpoint: _mqtt_ready required
  test_publish_ned_when_ready       -> publish_setpoint: correct NED published
  test_zero_lat_zero_lon_rejected   -> _accept_home: (0,0) pre-GPS placeholder blocked
  test_none_rejected                -> _accept_home: None blocked
  test_valid_coords_accepted        -> _accept_home: real GPS fix accepted
  test_nonzero_lat_zero_lon_accepted -> _accept_home: nonzero lat, zero lon accepted
  test_zero_lat_nonzero_lon_accepted -> _accept_home: zero lat, nonzero lon accepted
"""

import json
import logging
import math
import os
import sys
import threading

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for _p in (_PKG_ROOT, os.path.join(_PKG_ROOT, "drone_control")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from drone_control.px4_agent import PX4Agent, _EARTH_R


# ── Stub helpers ──────────────────────────────────────────────────────────────

class _FakeMQTT:
    """Minimal MQTT client stub — captures publish() calls."""
    def __init__(self):
        self.published = []   # list of (topic, payload)

    def publish(self, topic, payload, qos=0):
        self.published.append((topic, payload))


def _make_agent():
    """
    Create a PX4Agent without starting any threads or connecting to MQTT.
    __new__ bypasses __init__ — the same pattern used for continuous_monitor tests.
    Instance state is set manually to match what __init__ would create.
    """
    agent = object.__new__(PX4Agent)
    agent._lock         = threading.Lock()
    agent._home_pos     = None
    agent._mqtt_ready   = False
    agent._mqtt_client  = _FakeMQTT()
    agent._log          = logging.getLogger("test.px4_agent")
    return agent


# ── _global_to_ned ───────────────────────────────────────────────────────────

class TestGlobalToNed:
    """
    Flat-earth approximation: valid within a few km of home.
    x = North (positive north), y = East (positive east), z = Down (positive down).
    """
    _HOME_LAT = 47.390
    _HOME_LON = 8.540
    _HOME_ALT = 0.0

    def _ned(self, lat, lon, alt):
        return PX4Agent._global_to_ned(
            lat, lon, alt,
            self._HOME_LAT, self._HOME_LON, self._HOME_ALT,
        )

    def test_ned_at_home(self):
        """Target == home → (0, 0, 0)."""
        x, y, z = self._ned(self._HOME_LAT, self._HOME_LON, self._HOME_ALT)
        assert abs(x) < 1e-6
        assert abs(y) < 1e-6
        assert abs(z) < 1e-6

    def test_ned_north_displacement(self):
        """Moving north → positive x, near-zero y."""
        x, y, z = self._ned(self._HOME_LAT + 0.001, self._HOME_LON, self._HOME_ALT)
        expected_x = _EARTH_R * math.radians(0.001)  # ≈ 111.2 m
        assert abs(x - expected_x) < 0.5, f"x={x:.2f}, expected≈{expected_x:.2f}"
        assert abs(y) < 0.01
        assert abs(z) < 1e-6

    def test_ned_east_displacement(self):
        """Moving east → positive y, near-zero x."""
        x, y, z = self._ned(self._HOME_LAT, self._HOME_LON + 0.001, self._HOME_ALT)
        # East is scaled by cos(lat) because longitude degrees narrow near poles.
        expected_y = _EARTH_R * math.cos(math.radians(self._HOME_LAT)) * math.radians(0.001)
        assert abs(y - expected_y) < 0.5, f"y={y:.2f}, expected≈{expected_y:.2f}"
        assert abs(x) < 0.01
        assert abs(z) < 1e-6

    def test_ned_altitude_inverted(self):
        """
        NED z is positive-down; altitude is positive-up.
        Target 100 m above home → z = -100.
        """
        x, y, z = self._ned(self._HOME_LAT, self._HOME_LON, 100.0)
        assert z == -100.0, f"Expected z=-100.0, got {z}"

    def test_ned_altitude_below_home(self):
        """Target below home altitude → z positive (NED down)."""
        x, y, z = self._ned(self._HOME_LAT, self._HOME_LON, -50.0)
        assert z == 50.0, f"Expected z=50.0, got {z}"

    def test_ned_east_scales_with_cos_lat(self):
        """
        Same longitude delta at different latitudes gives different East displacement.
        cos(60°) = 0.5, so a 1° East step at lat=60° is half the metres of lat=0°.
        This verifies the cos(lat) scaling is applied (not a constant).
        """
        _, y_equator, _ = PX4Agent._global_to_ned(
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.0,  # home at equator
        )
        _, y_60, _ = PX4Agent._global_to_ned(
            60.0, 0.001, 0.0,
            60.0, 0.0, 0.0,  # home at 60° latitude
        )
        ratio = y_equator / y_60
        # cos(0°)/cos(60°) = 1.0/0.5 = 2.0
        assert abs(ratio - 2.0) < 0.01, (
            f"East scaling ratio equator/60° = {ratio:.4f}, expected ≈ 2.0 "
            f"(cos(lat) shrinks East degrees near poles)"
        )


# ── publish_setpoint guards ───────────────────────────────────────────────────

class TestPublishSetpointGuards:

    def test_publish_drops_without_home(self):
        """
        publish_setpoint returns without publishing when home_pos is not known.
        Without home_pos, WGS-84→NED would be computed relative to (0,0,0) —
        the drone would be sent off-grid.
        """
        agent = _make_agent()
        agent._mqtt_ready = True
        # _home_pos deliberately None (default)
        agent.publish_setpoint(47.391, 8.541, 10.0)
        assert len(agent._mqtt_client.published) == 0, (
            "publish_setpoint must not publish when home_pos is unknown"
        )

    def test_publish_drops_without_mqtt(self):
        """publish_setpoint returns without publishing when MQTT is not ready."""
        agent = _make_agent()
        agent._home_pos = {"lat": 47.390, "lon": 8.540, "alt": 0.0}
        agent._mqtt_ready = False
        agent.publish_setpoint(47.391, 8.541, 10.0)
        assert len(agent._mqtt_client.published) == 0, (
            "publish_setpoint must not publish when MQTT is disconnected"
        )

    def test_publish_ned_when_ready(self):
        """When home and MQTT ready, publish_setpoint publishes correct NED setpoint."""
        agent = _make_agent()
        agent._home_pos = {"lat": 47.390, "lon": 8.540, "alt": 0.0}
        agent._mqtt_ready = True

        # Move 0.001° north of home.
        agent.publish_setpoint(47.391, 8.540, 0.0)

        assert len(agent._mqtt_client.published) == 1
        topic, payload = agent._mqtt_client.published[0]
        assert "setpoint" in topic

        sp = json.loads(payload)
        assert "x" in sp and "y" in sp and "z" in sp, "setpoint must have x, y, z fields"
        # North displacement → positive x.
        assert sp["x"] > 0, f"Expected positive x (north), got {sp['x']}"
        # No east offset → y near zero.
        assert abs(sp["y"]) < 1.0, f"Expected near-zero y, got {sp['y']}"

    def test_publish_ned_values_match_global_to_ned(self):
        """NED values in the published payload exactly match _global_to_ned output."""
        home_lat, home_lon = 47.390, 8.540
        tgt_lat,  tgt_lon  = 47.391, 8.541
        tgt_alt             = 20.0

        agent = _make_agent()
        agent._home_pos = {"lat": home_lat, "lon": home_lon, "alt": 0.0}
        agent._mqtt_ready = True
        agent.publish_setpoint(tgt_lat, tgt_lon, tgt_alt)

        topic, payload = agent._mqtt_client.published[0]
        sp = json.loads(payload)

        # publish_setpoint uses home_alt=0.0 (hardcoded); verify we match that.
        exp_x, exp_y, exp_z = PX4Agent._global_to_ned(
            tgt_lat, tgt_lon, tgt_alt,
            home_lat, home_lon, 0.0,
        )
        assert abs(sp["x"] - exp_x) < 1e-9
        assert abs(sp["y"] - exp_y) < 1e-9
        assert abs(sp["z"] - exp_z) < 1e-9


# ── _accept_home (home-position guard) ───────────────────────────────────────

class TestAcceptHome:
    """
    PX4 sends home_pos = {"lat": 0, "lon": 0} before GPS fix is acquired.
    Accepting that placeholder corrupts every NED conversion silently.
    _accept_home() is the guard that blocks it.
    """

    def test_zero_lat_zero_lon_rejected(self):
        """Integer zeros — the exact placeholder PX4 sends before GPS fix."""
        assert PX4Agent._accept_home({"lat": 0, "lon": 0}) is False

    def test_zero_float_lat_zero_float_lon_rejected(self):
        """Float zeros — same placeholder as floats."""
        assert PX4Agent._accept_home({"lat": 0.0, "lon": 0.0}) is False

    def test_none_rejected(self):
        """Missing home_pos (None) — not yet received."""
        assert PX4Agent._accept_home(None) is False

    def test_valid_coords_accepted(self):
        """Real GPS fix with nonzero lat and lon."""
        assert PX4Agent._accept_home({"lat": 47.390, "lon": 8.540}) is True

    def test_nonzero_lat_zero_lon_accepted(self):
        """
        Nonzero lat, zero lon — accepted because lat alone confirms a real fix.
        The prime meridian (lon=0) is a valid real-world location.
        """
        assert PX4Agent._accept_home({"lat": 47.390, "lon": 0.0}) is True

    def test_zero_lat_nonzero_lon_accepted(self):
        """Zero lat (equator), nonzero lon — accepted; equator is real."""
        assert PX4Agent._accept_home({"lat": 0.0, "lon": 8.540}) is True

    def test_negative_coords_accepted(self):
        """Negative lat/lon (southern/western hemisphere) — accepted."""
        assert PX4Agent._accept_home({"lat": -33.87, "lon": -70.67}) is True
