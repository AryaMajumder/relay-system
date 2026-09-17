"""
action_nodes.py — BT action nodes for relay strategy proposals and commands.

BUILDSPEC: §4.7
LAYER:     1 (pure blackboard reads/writes — no network I/O, ever)
SUBSCRIBES: none
PUBLISHES:  none

Hard rules this file must satisfy (BUILDSPEC §4.7 / §5.5):
  - No node performs network I/O — Layer 1 discipline
    -> proven by test_nodes_write_blackboard_only
  - ProposeExitRelay writes strategy == "EXIT_RELAY" to pending_proposal
    -> proven by test_propose_exit_writes_pending_proposal
  - FollowerSafetyExit writes alert_intent to blackboard (item 11);
    must not publish directly
    -> proven by test_safety_exit_writes_alert_intent
"""

import logging
import time
import uuid

import py_trees

from .blackboard import TimestampedBlackboard
from .geometry import haversine

log = logging.getLogger(__name__)


# ── Battery cost helper ───────────────────────────────────────────────────────

def _cost_pct(distance_m: float, speed_ms: float, endurance_s: float) -> float:
    """Battery percentage consumed flying distance_m at speed_ms with endurance_s total."""
    if speed_ms <= 0 or endurance_s <= 0:
        return 0.0
    return (distance_m / speed_ms) * (100.0 / endurance_s)


def _model_constants(config: dict) -> tuple:
    """
    Return (cruise_speed_mps, endurance_s) from the per-model config section.

    endurance_s is DERIVED as 100.0 / consumption_rate_pct_per_s — there is no
    standalone 'endurance_s' key.  Storing it independently would create a
    two-truths problem: endurance_s and consumption_rate_pct_per_s could disagree.
    One source of truth, derived.  §7.1.
    """
    model_id = config.get("drone_model_id", "generic")
    model    = (config.get("DRONE_MODELS") or {}).get(model_id, {})
    speed    = model.get("cruise_speed_mps")
    rate     = model.get("consumption_rate_pct_per_s")
    if speed is None or rate is None:
        raise KeyError(
            f"DRONE_MODELS[{model_id!r}] missing cruise_speed_mps or "
            "consumption_rate_pct_per_s — add airframe constants to config"
        )
    return float(speed), 100.0 / float(rate)


# ── Base ──────────────────────────────────────────────────────────────────────

class ActionNodeBase(py_trees.behaviour.Behaviour):

    def __init__(self, bb: TimestampedBlackboard, config: dict,
                 name: str = None, clock=None):
        super().__init__(name=name or type(self).__name__)
        self.bb = bb
        self.config = config
        self._clock = clock or time.time
        self.proposal_id = f"prop-{uuid.uuid4().hex[:12]}"

    def _set(self, status: py_trees.common.Status, msg: str) -> py_trees.common.Status:
        self.feedback_message = msg
        log.debug("[%s] %s - %s", self.name, status.name, msg)
        return status


# ── Helpers ───────────────────────────────────────────────────────────────────

def _cost_from_target(bb: TimestampedBlackboard, config: dict, R_target: dict) -> dict:
    """
    Battery cost and ETA for flying to R_target and returning home.
    R_target must be read from the blackboard by the caller (Decision 5).
    Uses drone telemetry speed/endurance, not §7.1 model constants.
    """
    drone_state = bb.get("drone_state") or {}
    current_pos = drone_state.get("position") or config.get("gc_pos", {})
    home_pos = drone_state.get("home_pos") or current_pos
    speed, endurance = _model_constants(config)

    d_to_relay = haversine(current_pos, R_target)
    d_return   = haversine(R_target, home_pos)
    cost = _cost_pct(d_to_relay + d_return, speed, endurance)
    eta  = (d_to_relay / speed) if speed > 0 else 0.0

    return {
        "repositioning_m":  round(d_to_relay, 1),
        "battery_cost_pct": round(cost, 1),
        "eta_seconds":      round(eta, 1),
    }


# ═══════════════════════════════════════════════════════════════════════════════
# ACTION NODES
# ═══════════════════════════════════════════════════════════════════════════════

class ProposeContinuousRelay(ActionNodeBase):
    """
    Builds CONTINUOUS_RELAY proposal and writes to bb["pending_proposal"].
    Reads R_target from blackboard (written by BandSensorNode — Decision 5).
    Does NOT compute the relay position.
    """

    def update(self) -> py_trees.common.Status:
        R_target = self.bb.get("R_target")
        if R_target is None:
            return self._set(py_trees.common.Status.FAILURE,
                             "R_target not on blackboard — BandSensorNode must run first")

        cost = _cost_from_target(self.bb, self.config, R_target)
        drone_state = self.bb.get("drone_state") or {}
        follower_id = drone_state.get("drone_id") or self.config.get("drone_id", "drone-02")

        # BUILDSPEC §4.9 assigns the §2.5 build (proposal_id via content-hash,
        # r_target snapping, capability_snapshot, trigger_context, round_id echo,
        # top-level eta_s) EXCLUSIVELY to relay_strategy_evaluator, which reads
        # this pending_proposal from the capability_report.  Keep this payload
        # minimal — the evaluator enriches to §2.5.  R_target passes through
        # unsnapped so the evaluator's _snap_position (the sole place bucketing
        # happens per Decision 5) runs.
        proposal = {
            "proposal_id":    self.proposal_id,
            "timestamp":      time.time(),
            "strategy":       "CONTINUOUS_RELAY",
            "relay_position": R_target,   # unsnapped; evaluator snaps + adds alt_m
            "cost": {
                "follower_id":      follower_id,
                "battery_cost_pct": cost["battery_cost_pct"],
                "repositioning_m":  cost["repositioning_m"],
                "eta_seconds":      cost["eta_seconds"],
            },
        }
        self.bb.set("pending_proposal", proposal)
        log.info(
            "[ProposeContinuousRelay] relay at (%.5f,%.5f) cost=%.1f%% eta=%.0fs",
            R_target["lat"], R_target["lon"],
            cost["battery_cost_pct"], cost["eta_seconds"],
        )
        return self._set(py_trees.common.Status.SUCCESS,
                         f"CONTINUOUS_RELAY proposed, eta {cost['eta_seconds']:.0f}s")


class ProposeChainRelay(ActionNodeBase):
    """
    Builds CHAIN_RELAY proposal. Reads R_target from blackboard (Decision 5).
    Unreachable in 2-drone demo (ChainFeasible always FAILURE).
    """

    def update(self) -> py_trees.common.Status:
        R_target = self.bb.get("R_target")
        if R_target is None:
            return self._set(py_trees.common.Status.FAILURE, "R_target not on blackboard")

        cost = _cost_from_target(self.bb, self.config, R_target)
        proposal = {
            "proposal_id":    self.proposal_id,
            "timestamp":      time.time(),
            "strategy":       "CHAIN_RELAY",
            "relay_position": R_target,
            "cost": {
                "battery_cost_pct": cost["battery_cost_pct"],
                "repositioning_m":  cost["repositioning_m"],
                "eta_seconds":      cost["eta_seconds"],
            },
        }
        self.bb.set("pending_proposal", proposal)
        log.info("[ProposeChainRelay] chain relay proposed, cost=%.1f%%",
                 cost["battery_cost_pct"])
        return self._set(py_trees.common.Status.SUCCESS, "CHAIN_RELAY proposed")


class ProposeLetLeaderIsolate(ActionNodeBase):
    """
    Explicit decline of GC tasking with reason; the GC decides what happens to the leader.
    Fallback — always succeeds. Fires at entry when capability checks fail.
    """

    def update(self) -> py_trees.common.Status:
        tasking = self.bb.get("relay_tasking_received") or {}
        proposal = {
            "proposal_id":  self.proposal_id,
            "timestamp":    time.time(),
            "strategy":     "LET_LEADER_ISOLATE",
            "tasking_id":   tasking.get("tasking_id"),
            "leader_id":    tasking.get("leader_id"),
        }
        self.bb.set("pending_proposal", proposal)
        log.info("[ProposeLetLeaderIsolate] entry capability fail — let leader isolate")
        return self._set(py_trees.common.Status.SUCCESS, "LET_LEADER_ISOLATE proposed")


class FollowerSafetyExit(ActionNodeBase):
    """
    Commands the follower's own RTL for airframe safety.
    Trigger scoped to follower FCU-state only: battery critical, GPS lost,
    OFFBOARD unrecoverable after max_attempts.

    BUILDSPEC §4.7 item 11: writes alert_intent to blackboard for capability_assessor
    to publish. Does NOT publish directly — Layer 1 discipline.
    -> proven by test_safety_exit_writes_alert_intent
    """

    def update(self) -> py_trees.common.Status:
        drone_state = self.bb.get("drone_state") or {}
        battery = drone_state.get("battery_pct", 0.0)
        mode = drone_state.get("flight_mode", "UNKNOWN")

        # Compute the ACTUAL trigger reason first, independent of the FCU
        # latch.  When the latch is set we still want to know what actually
        # triggered the safety exit, so downstream (RDA log, alert_intent
        # ring buffer) can see the misattribution instead of every safety
        # event appearing as "fcu_telemetry_lost."
        # SESSION_LOG [2026-08-26] CRITICAL DISCOVERY.
        if battery < self.config.get("abort_battery_pct", 30):
            actual_reason = f"battery_critical:{battery:.0f}pct"
        elif mode != "OFFBOARD":
            actual_reason = f"offboard_unrecoverable:mode={mode}"
        else:
            actual_reason = "follower_safety_exit"

        latched = bool(self.bb.get("lost_fc_intent"))
        reason  = "fcu_telemetry_lost" if latched else actual_reason

        log.warning(
            "[FollowerSafetyExit] safety exit — reason=%s battery=%.0f%% mode=%s",
            reason, battery, mode,
        )

        # §4.7 item 11: write alert intent to blackboard regardless of path.
        # capability_assessor reads this and publishes the alert.  Never publish here.
        # Include actual_reason + fcu_latched so RDA / operators can spot
        # when a latch has downgraded the reported reason (was invisible
        # pre-2026-08-26 — see SESSION_LOG DECISION for context).
        self.bb.set("alert_intent", {
            "type":          "FOLLOWER_SAFETY_EXIT",
            "reason":        reason,
            "actual_reason": actual_reason,
            "fcu_latched":   latched,
            "battery_pct":   battery,
            "flight_mode":   mode,
            "timestamp":     time.time(),
        })

        if latched:
            # Latch suppresses the RTL pending_command per §4.7: PX4 failsafe
            # owns the airframe when FCU comms are lost.  Log LOUDLY at ERROR
            # so a misattributed suppression is impossible to miss in the
            # journal — the pre-2026-08-26 code silently dropped RTL with no
            # trace beyond the (already-misleading) "fcu_telemetry_lost"
            # warning above.  If FCU telemetry has actually recovered but
            # this branch fires anyway, FcuTelemetryFresh must have not
            # re-executed to clear the latch — inspect the tick order.
            log.error(
                "[FollowerSafetyExit] RTL SUPPRESSED by lost_fc_intent latch — "
                "actual trigger was %r (battery=%.0f%% mode=%s).  If FCU telemetry "
                "has recovered, FcuTelemetryFresh must run to clear the latch; "
                "see SESSION_LOG [2026-08-26].",
                actual_reason, battery, mode,
            )
            return self._set(py_trees.common.Status.SUCCESS,
                             "FCU telemetry lost — PX4 failsafe owns airframe, alert intent written")

        self.bb.set("pending_command", {
            "timestamp":    time.time(),
            "command":      "RTL",
            "reason":       reason,
            "battery_pct":  battery,
            "flight_mode":  mode,
        })
        return self._set(py_trees.common.Status.SUCCESS, f"RTL commanded: {reason}")


class ProposeReposition(ActionNodeBase):
    """
    Fires on gate 7 FAILURE + target drifted from band.
    Reads R_target from blackboard (Decision 5). Does NOT compute relay position.

    Internal checks (returns FAILURE if either fails, Selector falls to ProposeExitRelay):
      1. Reposition battery predicate: battery > move + return + RESERVE.
      2. Gain headroom >= reposition_improvement_threshold_db (5 dB).

    Cross-wiring guard: uses reposition_improvement_threshold_db (5 dB) NOT
    relay_effective_snr_improvement_db (3 dB).
    Uses drone telemetry speed/endurance, not §7.1 model constants.
    """

    def update(self) -> py_trees.common.Status:
        R_target = self.bb.get("R_target")
        if R_target is None:
            return self._set(py_trees.common.Status.FAILURE, "R_target not on blackboard")

        drone_state = self.bb.get("drone_state") or {}
        battery = drone_state.get("battery_pct", 0.0)
        current_pos = drone_state.get("position")
        home_pos = drone_state.get("home_pos")
        if not current_pos or not home_pos:
            return self._set(py_trees.common.Status.FAILURE,
                             "drone_state missing position or home_pos")

        speed, endurance = _model_constants(self.config)
        reserve = self.config.get("battery_reserve_pct", 30)

        d_to_target = haversine(current_pos, R_target)
        d_return    = haversine(R_target, home_pos)
        cost = _cost_pct(d_to_target + d_return, speed, endurance)
        if battery <= cost + reserve:
            return self._set(py_trees.common.Status.FAILURE,
                             f"reposition battery: {battery:.0f}% <= {cost + reserve:.1f}%")

        threshold_db = self.config.get("reposition_improvement_threshold_db", 5)
        marginal_snr = self.config.get("marginal_snr_db", 13)
        signal_report = self.bb.get("signal_report") or {}
        snr_gc  = (signal_report.get("follower_to_gc") or {}).get("snr_db")
        snr_ldr = ((signal_report.get("leader_to_follower") or {}).get("snr_db")
                   or (signal_report.get("follower_to_leader") or {}).get("snr_db"))

        if snr_gc is not None and snr_ldr is not None:
            gain_headroom = marginal_snr - min(snr_gc, snr_ldr)
            if gain_headroom < threshold_db:
                return self._set(py_trees.common.Status.FAILURE,
                                 f"gain headroom {gain_headroom:.1f}dB < {threshold_db}dB")

        eta_s = (d_to_target / speed) if speed > 0 else 0.0
        proposal = {
            "proposal_id":    self.proposal_id,
            "timestamp":      time.time(),
            "strategy":       "REPOSITION_RELAY",
            "relay_position": R_target,
            "cost": {
                "battery_cost_pct": round(cost, 1),
                "repositioning_m":  round(d_to_target, 1),
                "eta_seconds":      round(eta_s, 1),
            },
        }
        self.bb.set("pending_proposal", proposal)
        log.info(
            "[ProposeReposition] REPOSITION_RELAY to (%.5f,%.5f) cost=%.1f%% eta=%.0fs",
            R_target["lat"], R_target["lon"], cost, eta_s,
        )
        return self._set(py_trees.common.Status.SUCCESS,
                         f"REPOSITION_RELAY proposed, eta {eta_s:.0f}s cost {cost:.1f}%")


class ProposeExitRelay(ActionNodeBase):
    """
    Proposes exit with diagnostic context; the GC decides next action.
    Reason is inferred from blackboard state at fire time.
    """

    def update(self) -> py_trees.common.Status:
        gate_8_advisory = self.bb.get("gate_8_advisory") or {}
        band_fillable   = self.bb.get("band_fillable")
        direct_quality  = self.bb.get("gc_leader_direct_quality")
        threshold = self.config.get("relay_exit_quality_threshold", 0.85)

        if direct_quality is not None and direct_quality >= threshold:
            reason, trigger = "direct_link_recovered", "gate_6"
        elif band_fillable is False:
            reason, trigger = "position_infeasible", "gate_4"
        elif self.bb.get("drifted_from_band"):
            reason, trigger = "reposition_declined", "gate_7"
        else:
            reason, trigger = "link_ineffective", "gate_7"

        proposal = {
            "proposal_id":       self.proposal_id,
            "timestamp":         time.time(),
            "strategy":          "EXIT_RELAY",
            "reason":            reason,
            "trigger":           trigger,
            "relay_vs_baseline": gate_8_advisory,
        }
        self.bb.set("pending_proposal", proposal)
        log.info("[ProposeExitRelay] EXIT_RELAY: reason=%s trigger=%s", reason, trigger)
        return self._set(py_trees.common.Status.SUCCESS,
                         f"EXIT_RELAY proposed: {reason} via {trigger}")
