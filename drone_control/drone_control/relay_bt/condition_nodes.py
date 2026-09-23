"""
condition_nodes.py — All BT condition/sensor nodes for relay capability assessment.

BUILDSPEC: §4.6
LAYER:     1 (pure blackboard reads — no network I/O, ever)
SUBSCRIBES: none
PUBLISHES:  none

Hard rules this file must satisfy (BUILDSPEC §4.6 / §5.5):
  - Exactly nine maintenance gates, no others (G8+G9 run in DIAG_SCAN, G1–G7 in ARBITER_SCAN)
    -> proven by test_exactly_nine_gates
  - GCLinkLossAcceptable removed
    -> proven by test_no_loss_gate
  - Gate 8 reads authorized position+timer from blackboard; writes reauth_requested_at
    on FAIL; never writes current_relay_target (no movement on FAIL)
    -> proven by test_gate8_*, test_gate8_failure_writes_no_movement
  - Gate 8 fails if EITHER outside radius OR timer expired (§5.4 OR, not AND)
    -> proven by test_gate8_timer_expired_inside_radius
  - ReauthResponseTimedOut: injectable clock; FAIL fires ProposeExitRelay
    -> proven by test_reauth_timeout_*
  - F_cap leader-reachability: never-seen and stale are distinct decline reasons
    -> proven by test_fcap_never_seen_vs_stale_distinct
  - F_cap: never-seen suppressed during boot_grace_window_s after construction
    -> proven by test_fcap_boot_grace_suppresses_never_seen
  - F_cap never dedups or suppresses its own output
    -> proven by test_fcap_no_self_suppression
"""

import logging
import time as _time

import py_trees

from .blackboard import TimestampedBlackboard
from .geometry import (
    effective_radio_range,
    relay_is_feasible,
    compute_relay_position,
    point_in_polygon,
    haversine,
    gap_distance,
    follower_reach_per_hop,
    band_bounds,
    band_center,
    predicted_inside_band,
    estimate_battery_cost,
)

log = logging.getLogger(__name__)

_S = py_trees.common.Status.SUCCESS
_F = py_trees.common.Status.FAILURE
_R = py_trees.common.Status.RUNNING

_ACCEPTABLE_FLIGHT_MODES = {"HOLD", "OFFBOARD", "POSCTL", "AUTO.LOITER", "AUTO.MISSION"}



# ── Base ──────────────────────────────────────────────────────────────────────

class ConditionNodeBase(py_trees.behaviour.Behaviour):
    """Base class — all condition nodes follow this pattern."""

    def __init__(self, bb: TimestampedBlackboard, config: dict,
                 name: str = None, clock=None):
        super().__init__(name=name or type(self).__name__)
        self.bb = bb
        self.config = config
        # Injectable clock: tests substitute FakeClock.now; production uses time.time.
        # Required for any node that reads now() for freshness or timer checks.
        self._clock = clock or _time.time

    def update(self) -> py_trees.common.Status:
        raise NotImplementedError

    def _set(self, status: py_trees.common.Status, msg: str) -> py_trees.common.Status:
        self.feedback_message = msg
        log.debug("[%s] %s — %s", self.name, status.name, msg)
        return status


# ═══════════════════════════════════════════════════════════════════════════════
# EXISTING ENTRY / CAPABILITY NODES
# ═══════════════════════════════════════════════════════════════════════════════

class LinkQualityAbove(ConditionNodeBase):
    """SUCCESS if link_state["quality"] > threshold."""

    def __init__(self, bb, config, threshold: float, name=None, clock=None):
        super().__init__(bb=bb, config=config,
                         name=name or f"LinkQualityAbove({threshold})", clock=clock)
        self.threshold = threshold

    def update(self):
        link_state = self.bb.get("link_state")
        if link_state is None:
            return self._set(_R, "link_state not yet available")
        quality = link_state.get("quality", 0.0)
        if quality > self.threshold:
            return self._set(_S, f"quality {quality:.2f} > {self.threshold}")
        return self._set(_F, f"quality {quality:.2f} <= {self.threshold}")


class LinkQualityBelow(ConditionNodeBase):
    """SUCCESS if link_state["quality"] < threshold."""

    def __init__(self, bb, config, threshold: float, name=None, clock=None):
        super().__init__(bb=bb, config=config,
                         name=name or f"LinkQualityBelow({threshold})", clock=clock)
        self.threshold = threshold

    def update(self):
        link_state = self.bb.get("link_state")
        if link_state is None:
            return self._set(_R, "link_state not yet available")
        quality = link_state.get("quality", 1.0)
        if quality < self.threshold:
            return self._set(_S, f"quality {quality:.2f} < {self.threshold}")
        return self._set(_F, f"quality {quality:.2f} >= {self.threshold}")


class GeometryFeasible(ConditionNodeBase):
    """SUCCESS if leader is within bridgeable range from GC (dual-range)."""

    def update(self):
        tasking = self.bb.get("relay_tasking_received")
        leader_pos = (
            (tasking or {}).get("leader_pos")
            or (self.bb.get("leader_state") or {}).get("position")
            or self.config.get("leader_pos")
        )
        if not leader_pos:
            return self._set(_R, "leader_pos not available (not in relay_tasking, bb, or config)")

        gc_pos = self.config["gc_pos"]
        gc_range = self.config.get("gc_radio_range_m", self.config.get("radio_range_m", 800))
        ldr_range = self.config.get("leader_radio_range_m", self.config.get("radio_range_m", 800))
        sev = _jamming_severity(self.bb)
        factor = self.config.get("jamming_severity_factor", 0.6)

        eff_gc = effective_radio_range(gc_range, sev, factor)
        eff_ldr = effective_radio_range(ldr_range, sev, factor)

        feasible, reason = relay_is_feasible(gc_pos, leader_pos, eff_gc, eff_ldr)
        return self._set(_S if feasible else _F, reason)


class DataFreshness(ConditionNodeBase):
    """
    SUCCESS if drone_state source timestamp is within its staleness window.
    Uses SOURCE timestamps (§5.6).
    BUILDSPEC §8 verification: signal_report was removed from the design; per-hop
    freshness is enforced by BandSensorNode via radio_health payloads, not here.
    """

    def update(self):
        windows = self.config["staleness_windows_s"]
        parts = []
        all_fresh = True
        now = self._clock()

        for key in ("drone_state",):
            max_age = windows[key]
            data = self.bb.get(key)
            if data is None:
                all_fresh = False
                parts.append(f"STALE:{key} never received")
                continue
            ts = data.get("timestamp")
            if ts is None:
                all_fresh = False
                parts.append(f"STALE:{key} missing timestamp")
                continue
            age = now - ts
            if age > max_age:
                all_fresh = False
                parts.append(f"STALE:{key} {age:.1f}s>{max_age}s")
            else:
                parts.append(f"{key} ok")

        if all_fresh:
            return self._set(_S, "all sources fresh")
        return self._set(_F, ", ".join(parts))


class NotAlreadyRelaying(ConditionNodeBase):
    """SUCCESS unless current_role is RELAYING or MOVING_TO_RELAY."""

    def update(self):
        role = self.bb.get("current_role") or "IDLE"
        if role in ("RELAYING", "MOVING_TO_RELAY"):
            return self._set(_F, f"current_role={role}")
        return self._set(_S, f"current_role={role}")


class GPSFixAdequate(ConditionNodeBase):
    """SUCCESS if gps_fix_type >= min_gps_fix_type."""

    def update(self):
        drone_state = self.bb.get("drone_state")
        if drone_state is None:
            return self._set(_R, "drone_state not yet available")
        fix = drone_state.get("gps_fix_type", 0)
        min_fix = self.config.get("min_gps_fix_type", 3)
        if fix >= min_fix:
            return self._set(_S, f"GPS fix type {fix}")
        return self._set(_F, f"GPS fix {fix} < {min_fix}")


class FlightModeAcceptable(ConditionNodeBase):
    """SUCCESS if flight_mode is in the acceptable set."""

    def update(self):
        drone_state = self.bb.get("drone_state")
        if drone_state is None:
            return self._set(_R, "drone_state not yet available")
        mode = drone_state.get("flight_mode", "UNKNOWN")
        if mode in _ACCEPTABLE_FLIGHT_MODES:
            return self._set(_S, f"mode={mode}")
        return self._set(_F, f"mode={mode} does not accept setpoints")


class BatteryAboveFloor(ConditionNodeBase):
    """SUCCESS if battery_pct > battery_reserve_pct."""

    def update(self):
        drone_state = self.bb.get("drone_state")
        if drone_state is None:
            return self._set(_R, "drone_state not yet available")
        battery = drone_state.get("battery_pct", 0.0)
        floor = self.config.get("battery_reserve_pct", 30)
        if battery > floor:
            return self._set(_S, f"{battery:.0f}% above floor {floor}%")
        return self._set(_F, f"battery {battery:.0f}% <= floor {floor}%")


class GCLinkSNRAdequate(ConditionNodeBase):
    """SUCCESS if gc_to_follower SNR > min_snr_db."""

    def update(self):
        snr = self.bb.get("follower_snr_db_gc_to_follower")
        if snr is None:
            return self._set(_R, "follower_snr_db_gc_to_follower not yet available")
        min_snr = self.config.get("min_snr_db", 8)
        if snr > min_snr:
            return self._set(_S, f"GC SNR {snr:.1f}dB > {min_snr}dB")
        return self._set(_F, f"GC SNR {snr:.1f}dB <= {min_snr}dB")


# DELIBERATELY ABSENT: GCLinkLossAcceptable — removed per BUILDSPEC §4.6.
# Loss is not a relay metric. Gate 7 (RelayLinkAdequate) is SNR-only.
# Do not re-add this class.


class LeaderLinkSNRAdequate(ConditionNodeBase):
    """SUCCESS if leader_to_follower SNR > min_leader_snr_db."""

    def update(self):
        snr = self.bb.get("follower_snr_db_leader_to_follower")
        if snr is None:
            return self._set(_R, "follower_snr_db_leader_to_follower not yet available")
        min_snr = self.config.get("min_leader_snr_db", self.config.get("min_snr_db", 8))
        if snr > min_snr:
            return self._set(_S, f"leader SNR {snr:.1f}dB > {min_snr}dB")
        return self._set(_F, f"leader SNR {snr:.1f}dB <= {min_snr}dB")


class GeofenceContainsRelayPos(ConditionNodeBase):
    """SUCCESS if the computed relay position is inside the geofence polygon."""

    def update(self):
        tasking = self.bb.get("relay_tasking_received")
        leader_pos = (
            (tasking or {}).get("leader_pos")
            or (self.bb.get("leader_state") or {}).get("position")
            or self.config.get("leader_pos")
        )
        if not leader_pos:
            return self._set(_R, "leader_pos not available (not in relay_tasking, bb, or config)")

        gc_pos = self.config["gc_pos"]
        gc_range = self.config.get("gc_radio_range_m", self.config.get("radio_range_m", 800))
        ldr_range = self.config.get("leader_radio_range_m", self.config.get("radio_range_m", 800))
        polygon = self.config.get("geofence_polygon", [])
        sev = _jamming_severity(self.bb)
        factor = self.config.get("jamming_severity_factor", 0.6)

        relay_pos = compute_relay_position(
            gc_pos, leader_pos,
            effective_radio_range(gc_range, sev, factor),
            effective_radio_range(ldr_range, sev, factor),
        )
        if not polygon:
            return self._set(_S, "no geofence configured")
        if point_in_polygon(relay_pos, polygon):
            return self._set(_S, f"relay ({relay_pos['lat']:.4f},{relay_pos['lon']:.4f}) inside geofence")
        return self._set(_F, f"relay ({relay_pos['lat']:.4f},{relay_pos['lon']:.4f}) outside geofence")


class BatterySufficientForReturn(ConditionNodeBase):
    """
    SUCCESS if battery covers the return leg from relay position with margin.
    Calls geometry.estimate_battery_cost() — raises §7.1 RuntimeError if
    model constants are unresolved.
    """

    def update(self):
        drone_state = self.bb.get("drone_state")
        if drone_state is None:
            return self._set(_R, "drone_state not yet available")

        battery  = drone_state.get("battery_pct", 0.0)
        home_pos = drone_state.get("home_pos")
        if not home_pos:
            return self._set(_R, "drone_state missing home_pos")

        tasking    = self.bb.get("relay_tasking_received")
        leader_pos = (
            (tasking or {}).get("leader_pos")
            or (self.bb.get("leader_state") or {}).get("position")
            or self.config.get("leader_pos")
        )
        if not leader_pos:
            return self._set(_R, "leader_pos not available (not in relay_tasking, bb, or config)")

        gc_pos    = self.config["gc_pos"]
        gc_range  = self.config.get("gc_radio_range_m", self.config.get("radio_range_m", 800))
        ldr_range = self.config.get("leader_radio_range_m", self.config.get("radio_range_m", 800))
        sev       = _jamming_severity(self.bb)
        factor    = self.config.get("jamming_severity_factor", 0.6)

        relay_pos = compute_relay_position(
            gc_pos, leader_pos,
            effective_radio_range(gc_range, sev, factor),
            effective_radio_range(ldr_range, sev, factor),
        )

        model_id  = self.config.get("drone_model_id", "generic")
        model_cfg = self.config.get("DRONE_MODELS", {}).get(model_id, {})
        ok, required, _ = estimate_battery_cost(relay_pos, home_pos, battery, model_cfg, self.config)

        if ok:
            return self._set(_S, f"need {required:.1f}%, have {battery:.0f}%")
        return self._set(_F, f"battery {battery:.0f}% insufficient: need {required:.1f}% for return")


class SingleFollowerSufficient(ConditionNodeBase):
    """Always SUCCESS for 2-drone demo."""

    def update(self):
        return self._set(_S, "single follower sufficient")


class ChainFeasible(ConditionNodeBase):
    """Always FAILURE for 2-drone demo (need 3+ drones)."""

    def update(self):
        return self._set(_F, "need 3+ drones for chain relay")


# ═══════════════════════════════════════════════════════════════════════════════
# ROLE GATES
# ═══════════════════════════════════════════════════════════════════════════════

class IsAlreadyRelaying(ConditionNodeBase):
    """SUCCESS when current_role is RELAYING or MOVING_TO_RELAY. Gates maintenance subtree."""

    def update(self):
        role = self.bb.get("current_role") or "IDLE"
        if role in ("RELAYING", "MOVING_TO_RELAY"):
            return self._set(_S, f"current_role={role}")
        return self._set(_F, f"current_role={role}")


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY GATES
# ═══════════════════════════════════════════════════════════════════════════════

class RelayRequestReceived(ConditionNodeBase):
    """
    SUCCESS when a relay_tasking payload is present on the blackboard.
    This is the SOLE relay-assessment entry trigger — no follower self-trigger.
    """

    def update(self):
        tasking = self.bb.get("relay_tasking_received")
        if tasking:
            return self._set(_S, f"relay_tasking present: leader={tasking.get('leader_id','?')}")
        return self._set(_F, "no relay_tasking received")


class TaskingIsValid(ConditionNodeBase):
    """
    Entry guard: validates the tasking payload.
    Primarily checks 'not me as leader' — a drone mis-tasked as relay when it is the leader.
    """

    def update(self):
        tasking = self.bb.get("relay_tasking_received")
        if not tasking:
            return self._set(_F, "no tasking to validate")
        # BUILDSPEC §2.4 relay_tasking is {round_id, timestamp, trigger} — no leader_id.
        # Fall back to the config's leader_id (populated from LEADER_ID env at startup)
        # so the "not me as leader" check still works.
        leader_id = tasking.get("leader_id") or self.config.get("leader_id", "")
        drone_id = self.config.get("drone_id", "")
        if drone_id and leader_id == drone_id:
            return self._set(_F, f"invalid: tasked as relay but I am the leader ({drone_id})")
        if not leader_id:
            return self._set(_F, "no leader_id in tasking or config")
        return self._set(_S, f"tasking valid: leader={leader_id}")


# ═══════════════════════════════════════════════════════════════════════════════
# F_CAP LEADER-REACHABILITY (§4.6 addition)
# ═══════════════════════════════════════════════════════════════════════════════

class LeaderReachabilityFresh(ConditionNodeBase):
    """
    F_cap addition — SUCCESS if leader radio_health is fresh on the blackboard.

    HARD RULE (BUILDSPEC §4.6): never-seen and stale are DISTINCT decline reasons.
    Never collapse them into a single code path or message.
    -> proven by test_fcap_never_seen_vs_stale_distinct

    Never-seen is suppressed during boot_grace_window_s after node construction,
    because the leader may simply not have transmitted yet.
    -> proven by test_fcap_boot_grace_suppresses_never_seen

    HARD RULE (BUILDSPEC §4.6): F_cap never dedups or suppresses its own output.
    No state is accumulated between update() calls.
    -> proven by test_fcap_no_self_suppression

    Blackboard key: 'leader_radio_health' — written by capability_assessor.
    Config keys: 'radio_health_max_age_s' (default 10.0s), 'boot_grace_window_s' (default 600s).
    """

    def __init__(self, bb, config, name=None, clock=None):
        super().__init__(bb=bb, config=config, name=name, clock=clock)
        # Startup time recorded once at construction for boot-grace calculation.
        self._startup_t = self._clock()

    def update(self):
        max_age = self.config.get("radio_health_max_age_s", 10.0)
        _, is_fresh, age = self.bb.get_with_freshness("leader_radio_health", max_age)

        if is_fresh:
            return self._set(_S, "leader_radio_health fresh")

        # age is None → key was never written (never_seen).
        # age is a float → key was written but is now stale.
        # HARD RULE: never collapse these — the GC needs to distinguish them.
        if age is None:
            boot_grace = self.config.get("boot_grace_window_s", 600)
            elapsed = self._clock() - self._startup_t
            if elapsed < boot_grace:
                return self._set(_S,
                    f"leader_radio_health: never_seen suppressed (boot grace {elapsed:.0f}s/{boot_grace}s)")
            return self._set(_F,
                "leader_radio_health: never_seen")

        return self._set(_F, f"leader_radio_health: stale ({age:.1f}s > {max_age}s)")


# ═══════════════════════════════════════════════════════════════════════════════
# BAND SENSOR NODE
# ═══════════════════════════════════════════════════════════════════════════════

class BandSensorNode(ConditionNodeBase):
    """
    Computes D, r_G/r_L, (t_lo, t_hi), fillable, R_target each tick and writes
    all to blackboard. Always returns SUCCESS — pure sensor, no gate logic.

    R_target written here is the single source of truth for relay position (Decision 5).
    Step A (v6.3): reads severity + range from blackboard; stale path degrades not inflates.
    """

    def update(self):
        cfg = self.config
        gc_pos = cfg["gc_pos"]
        max_rh_age = cfg.get("radio_health_max_age_s", 10.0)
        stale_sev = cfg.get("stale_severity_floor", 0.5)
        factor = cfg.get("jamming_severity_factor", 0.6)
        cap_follower = min(
            cfg.get("gc_radio_range_m", cfg.get("radio_range_m", 800)),
            cfg.get("leader_radio_range_m", cfg.get("radio_range_m", 800)),
        )

        tasking = self.bb.get("relay_tasking_received") or {}
        leader_pos = (tasking.get("leader_pos")
                      or (self.bb.get("leader_state") or {}).get("position")
                      or cfg.get("leader_pos"))
        if not leader_pos:
            self.bb.set("band_fillable", False)
            return self._set(_S, "leader_pos unknown — band not computable")

        drone_state = self.bb.get("drone_state") or {}
        follower_pos = drone_state.get("position")

        stale_flag = False

        gc_sev_val, gc_fresh, _ = self.bb.get_with_freshness("gc_severity", max_rh_age)
        if not gc_fresh:
            cap_gc = cfg.get("gc_radio_range_m", cfg.get("radio_range_m", 800))
            sev_gc = stale_sev
            stale_flag = True
        else:
            gc_range = (self.bb.get("gc_radio_range_m")
                        or cfg.get("gc_radio_range_m", cfg.get("radio_range_m", 800)))
            sev_gc = gc_sev_val
            cap_gc = effective_radio_range(gc_range, sev_gc, factor)

        ldr_sev_val, ldr_fresh, _ = self.bb.get_with_freshness("leader_severity", max_rh_age)
        if not ldr_fresh:
            cap_leader = cfg.get("leader_radio_range_m", cfg.get("radio_range_m", 800))
            sev_ldr = stale_sev
            stale_flag = True
        else:
            ldr_range = (self.bb.get("leader_radio_range_m")
                         or cfg.get("leader_radio_range_m", cfg.get("radio_range_m", 800)))
            sev_ldr = ldr_sev_val
            cap_leader = effective_radio_range(ldr_range, sev_ldr, factor)

        fol_sev_val, fol_fresh, _ = self.bb.get_with_freshness("follower_severity", max_rh_age)
        severity = fol_sev_val if fol_fresh else stale_sev
        self.bb.set("stale_radio_health", stale_flag)

        D = gap_distance(gc_pos, leader_pos)
        r_G, r_L = follower_reach_per_hop(cap_follower, cap_gc, cap_leader, severity, factor)
        t_lo, t_hi = band_bounds(D, r_G, r_L)
        fillable = t_lo <= t_hi

        self.bb.set("band_D", D)
        self.bb.set("band_r_G", r_G)
        self.bb.set("band_r_L", r_L)
        self.bb.set("band_t_lo", t_lo)
        self.bb.set("band_t_hi", t_hi)
        self.bb.set("band_fillable", fillable)

        if fillable:
            self.bb.set("R_target", band_center(t_lo, t_hi, gc_pos, leader_pos))

        if follower_pos and D > 0:
            t_now = haversine(gc_pos, follower_pos) / D
            self.bb.set("predicted_inside_band", predicted_inside_band(t_now, t_lo, t_hi))

        return self._set(_S,
            f"D={D:.0f}m r_G={r_G:.0f}m r_L={r_L:.0f}m "
            f"band=[{t_lo:.2f},{t_hi:.2f}] fillable={fillable} stale_rh={stale_flag}")


# ═══════════════════════════════════════════════════════════════════════════════
# MAINTENANCE GATES 1–9 (BUILDSPEC §4.6 — exactly nine, no others)
# ═══════════════════════════════════════════════════════════════════════════════

class FcuTelemetryFresh(ConditionNodeBase):
    """
    Gate 1 — SUCCESS if FCU telemetry (drone_state) is within fcu_telemetry_max_age_s.
    On FAILURE: writes lost_fc_intent=True to blackboard.
    On RECOVERY: clears lost_fc_intent (see safety note below).
    LOST_FC is NOT a commanded exit — PX4 onboard failsafe owns the airframe.

    Safety-critical (SESSION_LOG [2026-08-26]): lost_fc_intent is read by
    FollowerSafetyExit to suppress the RTL pending_command.  Previously the
    latch was NEVER cleared, so a single stale tick (e.g. a 2-second DDS
    hiccup) permanently disabled RTL for the process lifetime.  Recovery
    clears the latch so subsequent G2 battery / G3 offboard exits can still
    command RTL.  "Sustained loss owns airframe" semantic preserved — the
    latch is TRUE for the duration of the stale window.
    """

    def update(self):
        max_age = self.config.get("fcu_telemetry_max_age_s", 3.0)
        _, fresh, age = self.bb.get_with_freshness("drone_state", max_age)
        if fresh:
            # Recovery: clear stale latch if it was set by a prior tick.
            # See SESSION_LOG [2026-08-26] CRITICAL DISCOVERY.
            if self.bb.get("lost_fc_intent"):
                self.bb.set("lost_fc_intent", False)
                log.warning(
                    "[FcuTelemetryFresh] FCU telemetry RECOVERED after previous "
                    "loss — clearing lost_fc_intent latch.  Subsequent "
                    "FollowerSafetyExit calls will command RTL as normal."
                )
            return self._set(_S, f"FCU telemetry fresh ({age:.1f}s)")
        self.bb.set("lost_fc_intent", True)
        return self._set(_F,
            f"FCU telemetry stale ({age}s > {max_age}s) — intent logged, PX4 failsafe owns airframe")


class BatteryStillSufficientToRelay(ConditionNodeBase):
    """
    Gate 2 — stay predicate: battery_pct > return_from_relay_target + RESERVE.
    Forced-safety; not debounced.
    Calls geometry.estimate_battery_cost() — raises §7.1 RuntimeError if
    model constants are unresolved.
    """

    def update(self):
        drone_state = self.bb.get("drone_state")
        if drone_state is None:
            return self._set(_R, "drone_state not available")

        battery     = drone_state.get("battery_pct", 0.0)
        current_pos = drone_state.get("position")
        home_pos    = drone_state.get("home_pos")
        if not current_pos or not home_pos:
            return self._set(_R, "drone_state missing position or home_pos")

        reserve      = self.config.get("battery_reserve_pct", 30)
        relay_target = self.bb.get("current_relay_target") or current_pos
        model_id     = self.config.get("drone_model_id", "generic")
        model_cfg    = self.config.get("DRONE_MODELS", {}).get(model_id, {})

        _, _, cost_pct = estimate_battery_cost(relay_target, home_pos, battery, model_cfg, self.config)
        required = cost_pct + reserve

        if battery > required:
            return self._set(_S, f"battery {battery:.0f}% > return+reserve {required:.1f}%")
        return self._set(_F, f"battery {battery:.0f}% insufficient: need {required:.1f}% to return")


class OffboardModeHeld(ConditionNodeBase):
    """
    Gate 3 — SUCCESS if drone is in OFFBOARD mode.
    < recover_offboard_max_attempts bad ticks: RUNNING (recovery window).
    >= max attempts: FAILURE → FollowerSafetyExit.
    """

    def __init__(self, bb, config, name=None, clock=None):
        super().__init__(bb, config, name, clock)
        self._failed_ticks = 0

    def update(self):
        drone_state = self.bb.get("drone_state")
        if drone_state is None:
            return self._set(_R, "drone_state not available")

        mode = drone_state.get("flight_mode", "UNKNOWN")
        max_attempts = self.config.get("recover_offboard_max_attempts", 3)

        if mode == "OFFBOARD":
            self._failed_ticks = 0
            return self._set(_S, "OFFBOARD held")

        self._failed_ticks += 1
        if self._failed_ticks < max_attempts:
            return self._set(_R,
                f"OFFBOARD dropped, attempt {self._failed_ticks}/{max_attempts} (mode={mode})")
        return self._set(_F, f"OFFBOARD lost after {max_attempts} attempts (mode={mode})")


class PositionServiceable(ConditionNodeBase):
    """
    Gate 4 — SUCCESS if band is geometrically feasible and relay target is inside geofence.
    FAILURE triggers ProposeExitRelay.
    """

    def update(self):
        fillable = self.bb.get("band_fillable")
        if fillable is False:
            return self._set(_F, "band infeasible — relay geometry no longer possible")

        relay_target = self.bb.get("current_relay_target")
        if not relay_target:
            return self._set(_S, "no relay_target yet — position check deferred")

        polygon = self.config.get("geofence_polygon", [])
        if not polygon:
            return self._set(_S, "no geofence configured")

        if point_in_polygon(relay_target, polygon):
            return self._set(_S,
                f"relay target ({relay_target['lat']:.4f},{relay_target['lon']:.4f}) inside geofence")
        return self._set(_F,
            f"relay target ({relay_target['lat']:.4f},{relay_target['lon']:.4f}) outside geofence")


class RfLinkTelemetryFresh(ConditionNodeBase):
    """
    Gate 5 — SUCCESS if signal_report is within rf_link_max_age_s.
    Uses SOURCE timestamps (§5.6). Stale > max_age → FAILURE → forced exit.
    BUILDSPEC §4.13.
    """

    def update(self):
        max_age = self.config.get("rf_link_max_age_s", 5.0)
        problems = []
        _, is_fresh, age = self.bb.get_with_freshness("follower_severity", max_age)
        if age is None:
            problems.append("follower_severity:never_received")
        elif not is_fresh:
            problems.append(f"follower_severity:{age:.1f}s>{max_age}s")
        if not problems:
            return self._set(_S, f"RF-link data fresh (max_age={max_age}s)")
        return self._set(_F, "RF-link stale: " + ", ".join(problems))


class RelayStillNeeded(ConditionNodeBase):
    """
    Gate 6 — SUCCESS when the direct GC↔leader link is still poor.
    FAILURE when direct link recovers above relay_exit_quality_threshold (0.85).
    In v1 SITL, gc_leader_direct_quality is never set → always SUCCESS.
    """

    def update(self):
        threshold = self.config.get("relay_exit_quality_threshold", 0.85)
        quality = self.bb.get("gc_leader_direct_quality")
        if quality is None:
            return self._set(_S, "gc_leader_direct_quality not available — relay still needed")
        if quality >= threshold:
            return self._set(_F,
                f"direct link recovered: {quality:.2f} >= {threshold} — relay not needed")
        return self._set(_S, f"direct link still poor: {quality:.2f} < {threshold}")


class RelayLinkAdequate(ConditionNodeBase):
    """
    Gate 7 — PRIMARY trigger. SNR >= min_snr_db on BOTH GC↔follower and follower↔leader.
    N=3 consecutive-bad-tick debounce. Transient noise does not trigger.
    Cross-wiring guard: uses min_snr_db (8 dB), NOT reposition_improvement_threshold_db.
    """

    def __init__(self, bb, config, name=None, clock=None):
        super().__init__(bb, config, name, clock)
        self._bad_ticks = 0

    def _both_hops_adequate(self) -> tuple:
        min_snr = self.config.get("min_snr_db", 8)
        snr_gc  = self.bb.get("follower_snr_db_gc_to_follower")
        snr_ldr = self.bb.get("follower_snr_db_leader_to_follower")
        missing = [k for k, v in [("snr_gc", snr_gc), ("snr_ldr", snr_ldr)] if v is None]
        if missing:
            return False, f"missing data: {missing}"
        if snr_gc < min_snr:
            return False, f"GC SNR {snr_gc:.1f}dB < {min_snr}dB"
        if snr_ldr < min_snr:
            return False, f"leader SNR {snr_ldr:.1f}dB < {min_snr}dB"
        return True, f"GC SNR={snr_gc:.1f}dB leader SNR={snr_ldr:.1f}dB"

    def update(self):
        debounce_n = self.config.get("debounce_n", 3)
        adequate, reason = self._both_hops_adequate()
        if adequate:
            self._bad_ticks = 0
            return self._set(_S, reason)
        self._bad_ticks += 1
        if self._bad_ticks >= debounce_n:
            return self._set(_F,
                f"link bad {self._bad_ticks} ticks (N={debounce_n}): {reason}")
        return self._set(_S, f"debounce {self._bad_ticks}/{debounce_n}: {reason}")


class RelayActuallyImproved(ConditionNodeBase):
    """
    Gate 8 — authorization still valid: inside tolerance radius AND timer not expired.

    BUILDSPEC §4.6 full definition:
      r_target_now  = R_target from blackboard (BandSensorNode output this tick)
      inside_radius = haversine(r_target_now, current_relay_target) <= tolerance_radius_m
      timer_ok      = now() < authorization_valid_until
      PASS if (inside_radius AND timer_ok) else FAIL

    §5.4 — FAIL if EITHER condition fails (OR, not AND).
    On FAIL: writes reauth_requested_at to blackboard.
    HARD RULE: never writes current_relay_target — follower keeps streaming to authorized target.
    """

    def update(self):
        current_target = self.bb.get("current_relay_target")
        valid_until = self.bb.get("authorization_valid_until")

        # No assignment yet — nothing to validate, gate passes.
        if current_target is None or valid_until is None:
            return self._set(_S, "no relay assignment — gate 8 passes")

        tolerance_m = (self.bb.get("tolerance_radius_m")
                       or self.config.get("tolerance_radius_m", 10.0))
        r_target_now = self.bb.get("R_target")   # BandSensorNode writes this each tick

        now = self._clock()
        timer_ok = now < valid_until

        if r_target_now is not None:
            inside_radius = haversine(r_target_now, current_target) <= tolerance_m
        else:
            inside_radius = True   # can't compute — assume inside

        if inside_radius and timer_ok:
            return self._set(_S,
                f"auth valid: inside_radius=True timer_ok=True "
                f"({valid_until - now:.0f}s remaining)")

        # §5.4: FAIL on either condition. Write reauth intent; do NOT touch current_relay_target.
        # Only write if not already set — capability_assessor clears it on new authorization.
        # Overwriting would reset the 120s REAUTH_TIMEOUT clock on every tick.
        if self.bb.get("reauth_requested_at") is None:
            self.bb.set("reauth_requested_at", now)
        reasons = []
        if not inside_radius:
            reasons.append(f"outside_tolerance({tolerance_m}m)")
        if not timer_ok:
            reasons.append(f"timer_expired({now - valid_until:.0f}s_ago)")
        return self._set(_F, "reauth_triggered: " + ",".join(reasons))


class GpsHealthy(ConditionNodeBase):
    """
    Gate 9 — soft GPS check. Writes gps_health_advisory to blackboard.
    Non-blocking: wrapped in tree so it never stops the scan.
    """

    def update(self):
        drone_state = self.bb.get("drone_state")
        if drone_state is None:
            self.bb.set("gps_health_advisory", {"healthy": None, "reason": "no drone_state"})
            return self._set(_S, "gate9: no drone_state")

        fix = drone_state.get("gps_fix_type", 0)
        min_fix = self.config.get("min_gps_fix_type", 3)
        healthy = fix >= min_fix
        self.bb.set("gps_health_advisory", {"healthy": healthy, "fix_type": fix, "min_fix": min_fix})

        if healthy:
            return self._set(_S, f"gate9: GPS fix={fix} >= {min_fix}")
        return self._set(_F, f"gate9: GPS degraded fix={fix} < {min_fix} (alert only)")


# ── Gate registry ─────────────────────────────────────────────────────────────

# BUILDSPEC §4.6: exactly nine maintenance gates, in gate-number order.
# G1–G7 run in ARBITER_SCAN (priority Selector); G8–G9 run in DIAG_SCAN
# (unconditional Selector before ARBITER_SCAN) — see tree_builder.py.
# test_exactly_nine_gates asserts on this list.
MAINTENANCE_GATES = [
    FcuTelemetryFresh,             # Gate 1  — ARBITER_SCAN
    BatteryStillSufficientToRelay, # Gate 2  — ARBITER_SCAN
    OffboardModeHeld,              # Gate 3  — ARBITER_SCAN
    PositionServiceable,           # Gate 4  — ARBITER_SCAN
    RfLinkTelemetryFresh,          # Gate 5  — ARBITER_SCAN
    RelayStillNeeded,              # Gate 6  — ARBITER_SCAN
    RelayLinkAdequate,             # Gate 7  — ARBITER_SCAN
    RelayActuallyImproved,         # Gate 8  — DIAG_SCAN (unconditional)
    GpsHealthy,                    # Gate 9  — DIAG_SCAN (unconditional)
]


# ═══════════════════════════════════════════════════════════════════════════════
# REAUTH TIMEOUT CHECK (additional, wired into arbiter alongside the 9 gates)
# ═══════════════════════════════════════════════════════════════════════════════

class ReauthResponseTimedOut(ConditionNodeBase):
    """
    SUCCESS if no reauth is outstanding, or response still within window.
    FAILURE if reauth_requested_at is set and window has elapsed.
    FAILURE fires the existing ProposeExitRelay (no new action node).

    Config key: 'reauth_response_timeout_s' (default 120s, §3.2).
    Blackboard key read: 'reauth_requested_at' (float epoch or None).
    """

    def update(self):
        reauth_at = self.bb.get("reauth_requested_at")
        if reauth_at is None:
            return self._set(_S, "no reauth outstanding")

        timeout_s = self.config.get("reauth_response_timeout_s", 120)
        elapsed = self._clock() - reauth_at
        if elapsed < timeout_s:
            return self._set(_S, f"reauth pending: {elapsed:.0f}s/{timeout_s}s elapsed")
        return self._set(_F, f"reauth timed out: {elapsed:.0f}s > {timeout_s}s — triggering exit")


# ═══════════════════════════════════════════════════════════════════════════════
# REPOSITION / MOVEMENT NODES
# ═══════════════════════════════════════════════════════════════════════════════

class DriftedFromBand(ConditionNodeBase):
    """
    SUCCESS if the current commanded relay target has drifted outside the feasible band.
    Used on gate 7's FAILURE path to decide reposition vs exit.
    """

    def update(self):
        relay_target = self.bb.get("current_relay_target")
        if not relay_target:
            self.bb.set("drifted_from_band", False)
            return self._set(_F, "no current_relay_target — cannot check drift")

        fillable = self.bb.get("band_fillable")
        if not fillable:
            self.bb.set("drifted_from_band", True)
            return self._set(_S, "band infeasible — target is outside (drifted)")

        t_lo = self.bb.get("band_t_lo")
        t_hi = self.bb.get("band_t_hi")
        D = self.bb.get("band_D")
        if t_lo is None or t_hi is None or D is None:
            self.bb.set("drifted_from_band", False)
            return self._set(_F, "band data not available")

        gc_pos = self.config["gc_pos"]
        if D <= 0:
            self.bb.set("drifted_from_band", False)
            return self._set(_F, "D=0 — GC and leader co-located")

        t_target = haversine(gc_pos, relay_target) / D
        inside = t_lo <= t_target <= t_hi
        if inside:
            self.bb.set("drifted_from_band", False)
            return self._set(_F,
                f"target t={t_target:.2f} inside band [{t_lo:.2f},{t_hi:.2f}] — not drifted")
        self.bb.set("drifted_from_band", True)
        return self._set(_S,
            f"target t={t_target:.2f} outside band [{t_lo:.2f},{t_hi:.2f}] — drifted")


class RelayCommandAccepted(ConditionNodeBase):
    """SUCCESS if the last movement command was acknowledged."""

    def update(self):
        ack = self.bb.get("last_command_ack")
        if ack is None:
            return self._set(_R, "last_command_ack not yet received")
        accepted = ack.get("accepted", False)
        if accepted:
            return self._set(_S, f"command accepted: {ack.get('command_id','?')}")
        return self._set(_F, f"command not accepted: {ack.get('reason','unknown')}")


class RelayMovementProgressing(ConditionNodeBase):
    """SUCCESS if distance to relay target is decreasing over ticks."""

    def update(self):
        progressing = self.bb.get("distance_to_target_decreasing")
        if progressing is None:
            return self._set(_R, "movement progress not yet measured")
        if progressing:
            return self._set(_S, "movement progressing")
        return self._set(_F, "movement not progressing — distance not decreasing")


class RelayModeHeld(ConditionNodeBase):
    """SUCCESS if drone remains in a relay-compatible mode during movement."""

    def update(self):
        held = self.bb.get("offboard_mode_held")
        if held is None:
            return self._set(_R, "mode status not available")
        if held:
            return self._set(_S, "relay mode held")
        return self._set(_F, "relay mode dropped during movement")


class RelayArrived(ConditionNodeBase):
    """SUCCESS if drone is within movement_acceptance_radius_m of relay target."""

    def update(self):
        arrived = self.bb.get("within_acceptance_radius")
        if arrived is None:
            return self._set(_R, "arrival status not yet available")
        if arrived:
            return self._set(_S, "arrived at relay position")
        return self._set(_F, "not yet arrived at relay position")


# ── Private helpers ───────────────────────────────────────────────────────────

def _jamming_severity(bb: TimestampedBlackboard) -> float:
    """Read jamming severity from signal_report, defaulting to 0.0."""
    signal_report = bb.get("signal_report") or {}
    return (signal_report.get("jamming") or {}).get("severity", 0.0) or 0.0
