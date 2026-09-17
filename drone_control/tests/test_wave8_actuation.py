"""
test_wave8_actuation.py — Wave 8 gate tests for relay_mover.py / relay_position_tracker.py.

TEST_PROTOCOL: §5.13  Archetype F (actuation node — fake FCU seam)

PROVEN column:
  test_setpoint_rate              -> §4.14 / config (relay_position_tracker_hz)
  test_streams_in_both_roles      -> inventory (MOVING_TO_RELAY and RELAYING)
  test_target_change_immediate    -> inventory (reposition takes effect on next tick)
  test_never_writes_target        -> inventory (mover reads r_target; never computes it)
  test_tracker_four_guard_signals -> inventory (all four movement_status guard signals)
"""

import sys
import os

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for p in (
    _PKG_ROOT,
    os.path.join(_PKG_ROOT, "drone_control"),
    os.path.join(_PKG_ROOT, "drone_control", "relay_bt"),
):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.relay_mover import _RelayMoverCore
from drone_control.relay_position_tracker import _RelayPositionTrackerCore


# ── Helpers ───────────────────────────────────────────────────────────────────

_CFG = {"relay_position_tracker_hz": 3, "OFFBOARD_PRE_SECS": 1.5,
        "movement_acceptance_radius_m": 2.0}

_TARGET = {"lat": 47.3914, "lon": 8.5413, "alt_m": 10.0}


def _make_mover_core(clock_val=None):
    setpoints = []
    offboards = []
    commands  = []
    # List wraps a scalar so tests can mutate clock_ref[0] to advance time.
    clock_ref = [clock_val or 0.0]

    def clock(): return clock_ref[0]

    core = _RelayMoverCore(
        config=dict(_CFG),
        publish_setpoint_fn=lambda lat, lon, alt: setpoints.append((lat, lon, alt)),
        start_offboard_fn=lambda: offboards.append(1),
        send_command_fn=lambda cmd, params=None: commands.append((cmd, params)),
        clock=clock,
    )
    return core, setpoints, offboards, clock_ref


def _make_tracker_core():
    statuses  = []
    reached   = []
    confirmed = []
    # roles captures current_role publishes; added when relay_position_tracker was
    # given publish_role_fn to fix the RELAYING gap (no other node produces RELAYING).
    roles     = []
    # List wraps a scalar so tests can mutate clock_ref[0] to advance time (e.g.
    # to force a timeout without sleeping).
    clock_ref = [0.0]

    def clock(): return clock_ref[0]

    core = _RelayPositionTrackerCore(
        drone_id="drone-01",
        config=dict(_CFG),
        publish_status_fn=statuses.append,
        publish_pos_reached_fn=reached.append,
        publish_relay_confirmed_fn=confirmed.append,
        publish_role_fn=roles.append,
        clock=clock,
    )
    return core, statuses, reached, confirmed, roles, clock_ref


def _assignment(r_target=None):
    return {"drone_id": "drone-01", "r_target": r_target or dict(_TARGET), "eta_s": 10.0}


# ── §4.14: setpoint rate ──────────────────────────────────────────────────────

class TestSetpointRate:
    def test_setpoint_rate(self):
        """
        relay_mover must stream at relay_position_tracker_hz (3 Hz).
        TEST_PROTOCOL §5.13: assert on setpoint count over faked elapsed time.
        """
        core, setpoints, _, _ = _make_mover_core()
        core.on_role("MOVING_TO_RELAY")
        core.on_assignment(_assignment())

        # Drive 3 ticks — should produce exactly 3 setpoints.
        for _ in range(3):
            core.tick()

        assert len(setpoints) == 3, (
            f"Expected 3 setpoints at 3 Hz over 1s, got {len(setpoints)}"
        )

    def test_no_setpoints_when_inactive(self):
        """No setpoints are streamed when role is not MOVING_TO_RELAY or RELAYING."""
        core, setpoints, _, _ = _make_mover_core()
        core.on_assignment(_assignment())
        # No on_role call → _active stays False

        for _ in range(5):
            core.tick()

        assert len(setpoints) == 0, "Must not stream when inactive"

    def test_no_setpoints_without_target(self):
        """No setpoints are streamed before a relay_assignment is received."""
        core, setpoints, _, _ = _make_mover_core()
        core.on_role("MOVING_TO_RELAY")
        # No on_assignment call → _target stays None

        for _ in range(3):
            core.tick()

        assert len(setpoints) == 0, "Must not stream without a target"


# ── inventory: streams in both roles ─────────────────────────────────────────

class TestStreamsInBothRoles:
    def test_streams_in_both_roles(self):
        """
        Setpoints stream during MOVING_TO_RELAY AND during RELAYING.
        TEST_PROTOCOL §5.13: inventory — both active roles produce setpoints.
        """
        for role in ("MOVING_TO_RELAY", "RELAYING"):
            core, setpoints, _, _ = _make_mover_core()
            core.on_assignment(_assignment())
            core.on_role(role)
            core.tick()
            assert len(setpoints) == 1, (
                f"Role {role!r} must produce setpoints; got {len(setpoints)}"
            )

    def test_stops_on_non_active_role(self):
        """Setpoint stream stops when role leaves active states."""
        core, setpoints, _, _ = _make_mover_core()
        core.on_assignment(_assignment())
        core.on_role("MOVING_TO_RELAY")
        core.tick()
        assert len(setpoints) == 1

        core.on_role("OPEN_TO_RELAY")
        core.tick()
        assert len(setpoints) == 1, "Must not stream after role deactivated"


# ── inventory: target change takes effect immediately ─────────────────────────

class TestTargetChangeImmediate:
    def test_target_change_immediate(self):
        """
        When r_target changes (reposition), the next tick streams the new setpoint.
        TEST_PROTOCOL §5.13: inventory.
        """
        core, setpoints, _, _ = _make_mover_core()
        core.on_role("RELAYING")

        t1 = {"lat": 47.3900, "lon": 8.5400, "alt_m": 5.0}
        t2 = {"lat": 47.3950, "lon": 8.5450, "alt_m": 10.0}

        core.on_assignment({"drone_id": "drone-01", "r_target": t1, "eta_s": 5.0})
        core.tick()
        assert setpoints[-1] == (t1["lat"], t1["lon"], t1["alt_m"])

        core.on_assignment({"drone_id": "drone-01", "r_target": t2, "eta_s": 5.0})
        core.tick()
        assert setpoints[-1] == (t2["lat"], t2["lon"], t2["alt_m"]), (
            "New target must be used on the very next tick after assignment update"
        )


# ── inventory: mover never writes target ─────────────────────────────────────

class TestNeverWritesTarget:
    def test_never_writes_target(self):
        """
        relay_mover reads r_target from relay_assignment verbatim — never computes it.
        TEST_PROTOCOL §5.13: source inspection.
        BUILDSPEC §5.3 Decision 5: r_target is never recomputed downstream.
        """
        import inspect
        import drone_control.relay_mover as mod

        src = inspect.getsource(mod._RelayMoverCore.on_assignment)

        assert "bucket_position" not in src, (
            "relay_mover must not snap r_target (Decision 5)"
        )
        assert "haversine" not in src, (
            "relay_mover must not call haversine on r_target"
        )
        # Must read r_target (§2.7 field) not the old field names.
        assert '"r_target"' in src or "'r_target'" in src, (
            "relay_mover must read r_target (§2.7 key) from relay_assignment"
        )
        assert "current_relay_target" not in src, (
            "relay_mover must not fall back to old 'current_relay_target' key"
        )
        assert "relay_position" not in src, (
            "relay_mover must not fall back to old 'relay_position' key"
        )


# ── inventory: tracker publishes all four guard signals ───────────────────────

class TestTrackerFourGuardSignals:
    def test_tracker_four_guard_signals(self):
        """
        movement_status must contain all four guard signals consumed by capability_assessor.
        TEST_PROTOCOL §5.13: inventory — RelayCommandAccepted, RelayMovementProgressing,
        RelayModeHeld, RelayArrived.
        """
        core, statuses, _, _, _, _ = _make_tracker_core()

        core.on_assignment(_assignment())
        core.on_role("MOVING_TO_RELAY")
        core.on_drone_state({"position": {"lat": 47.3900, "lon": 8.5400, "alt_m": 0.0},
                             "flight_mode": "OFFBOARD"})
        core.tick()

        assert len(statuses) == 1
        s = statuses[0]

        required = {
            "last_command_ack",
            "distance_to_target_decreasing",
            "offboard_mode_held",
            "within_acceptance_radius",
        }
        missing = required - s.keys()
        assert not missing, (
            f"movement_status missing guard signal keys: {missing}"
        )

    def test_tracker_guard_signal_types(self):
        """Guard signal types: last_command_ack is a dict, booleans are booleans."""
        core, statuses, _, _, _, _ = _make_tracker_core()
        core.on_assignment(_assignment())
        core.on_role("RELAYING")
        core.on_drone_state({"position": {"lat": 47.3900, "lon": 8.5400, "alt_m": 0.0}})
        core.tick()

        s = statuses[0]
        assert isinstance(s["last_command_ack"], dict), "last_command_ack must be a dict"
        assert isinstance(s["distance_to_target_decreasing"], bool)
        assert isinstance(s["offboard_mode_held"], bool)
        assert isinstance(s["within_acceptance_radius"], bool)

    def test_tracker_reads_r_target_from_assignment(self):
        """relay_position_tracker reads r_target (§2.7) not old field names."""
        import inspect
        import drone_control.relay_position_tracker as mod

        src = inspect.getsource(mod._RelayPositionTrackerCore.on_assignment)
        assert '"r_target"' in src or "'r_target'" in src, (
            "relay_position_tracker must read 'r_target' from relay_assignment (§2.7)"
        )
        assert "current_relay_target" not in src
        assert "relay_position" not in src


# ── RELAYING role transition on confirmed arrival ─────────────────────────────

class TestRelayingRoleTransition:
    def test_confirmed_arrival_publishes_relaying(self):
        """
        On confirmed arrival relay_position_tracker publishes current_role=RELAYING.
        This is the only producer of the RELAYING state in the system.
        """
        core, _, _, confirmed, roles, _ = _make_tracker_core()

        # Place the drone right at the target (within 2m acceptance radius).
        target = {"lat": 47.3914, "lon": 8.5413, "alt_m": 10.0}
        core.on_assignment({"drone_id": "drone-01", "r_target": target, "eta_s": 5.0})
        core.on_role("MOVING_TO_RELAY")
        core.on_drone_state({"position": {"lat": 47.3914, "lon": 8.5413, "alt_m": 10.0}})

        core.tick()

        assert len(roles) == 1, (
            f"Confirmed arrival must publish current_role=RELAYING; got roles={roles}"
        )
        assert roles[0] == "RELAYING", (
            f"current_role must be 'RELAYING' on confirmed arrival, got {roles[0]!r}"
        )

    def test_relaying_published_exactly_once(self):
        """RELAYING is published once on arrival, not on every subsequent tick."""
        core, _, _, _, roles, _ = _make_tracker_core()

        target = {"lat": 47.3914, "lon": 8.5413, "alt_m": 10.0}
        core.on_assignment({"drone_id": "drone-01", "r_target": target, "eta_s": 5.0})
        core.on_role("MOVING_TO_RELAY")
        core.on_drone_state({"position": target})

        for _ in range(5):
            core.tick()

        assert len(roles) == 1, (
            f"RELAYING must be published exactly once, not {len(roles)} times"
        )

    def test_timeout_does_not_publish_relaying(self):
        """TIMEOUT relay_confirmed does not produce current_role=RELAYING."""
        core, _, _, confirmed, roles, clock_ref = _make_tracker_core()

        target = {"lat": 47.3914, "lon": 8.5413, "alt_m": 10.0}
        core.on_assignment({"drone_id": "drone-01", "r_target": target, "eta_s": 10.0})
        core.on_role("MOVING_TO_RELAY")
        # No drone_state → arrival never detected; force timeout by advancing clock.
        clock_ref[0] = 1000.0  # eta_s=10, factor=1.5 → timeout at 15s; jump far past it

        core.tick()

        timeout_confirms = [c for c in confirmed if c.get("status") == "TIMEOUT"]
        assert len(timeout_confirms) == 1, "Should have fired a TIMEOUT relay_confirmed"
        assert len(roles) == 0, (
            f"TIMEOUT must not publish RELAYING; got roles={roles}"
        )

    def test_relay_confirmed_status_confirmed_on_arrival(self):
        """relay_confirmed status is 'CONFIRMED' (not 'TIMEOUT') on genuine arrival."""
        core, _, _, confirmed, _, _ = _make_tracker_core()

        target = {"lat": 47.3914, "lon": 8.5413, "alt_m": 10.0}
        core.on_assignment({"drone_id": "drone-01", "r_target": target, "eta_s": 5.0})
        core.on_role("MOVING_TO_RELAY")
        core.on_drone_state({"position": target})
        core.tick()

        assert len(confirmed) == 1
        assert confirmed[0]["status"] == "CONFIRMED"
