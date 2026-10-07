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
import re
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


_LLI_MODE_RE = re.compile(r"\(([^)]+)\)\s*$")


class ProposeLetLeaderIsolate(ActionNodeBase):
    """
    Explicit decline of GC tasking; the GC decides what happens to the leader.
    Always succeeds. Two tree-wiring modes, identified by self.name:
      - ProposeLetLeaderIsolate(CapFail)    — a check in FULL_ENTRY failed.
        Reason is the name of that failing check, read from
        bb["last_capability_check"] (written by ConditionNodeBase._set).
      - ProposeLetLeaderIsolate(NoStrategy) — all FULL_ENTRY checks passed
        but neither SingleFollowerSufficient nor ChainFeasible selected a
        strategy. Reason is a fixed literal "no_strategy".
    """

    def update(self) -> py_trees.common.Status:
        tasking = self.bb.get("relay_tasking_received") or {}
        mode_match = _LLI_MODE_RE.search(self.name or "")
        mode = mode_match.group(1) if mode_match else "unknown"
        if mode == "NoStrategy":
            reason = "no_strategy"
        elif mode == "CapFail":
            reason = self.bb.get("last_capability_check") or "unknown_capability_check"
        else:
            reason = "unknown"
        proposal = {
            "proposal_id":  self.proposal_id,
            "timestamp":    time.time(),
            "strategy":     "LET_LEADER_ISOLATE",
            "reason":       reason,
            "trigger":      mode,
            "tasking_id":   tasking.get("tasking_id"),
            "leader_id":    tasking.get("leader_id"),
        }
        self.bb.set("pending_proposal", proposal)
        log.info("[ProposeLetLeaderIsolate] decline: mode=%s reason=%s", mode, reason)
        return self._set(py_trees.common.Status.SUCCESS,
                         f"LET_LEADER_ISOLATE proposed (mode={mode}, reason={reason})")


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


class ProposeIncumbentContinuousRelay(ActionNodeBase):
    """
    RELAYING-branch terminal: when all G1-G7 gates pass, the incumbent
    bids CONTINUOUS_RELAY for its live authorized relay point so that a
    reauth round can renew the incumbent instead of winning on EXIT_RELAY
    gate flaps.

    Reads current_relay_target (written by capability_assessor on
    relay_assignment arrival). If absent (e.g. the follower just entered
    RELAYING without having an assignment on the blackboard yet), this
    node writes no proposal and returns SUCCESS — the ARBITER_SCAN
    terminal must always succeed so RELAYING_BRANCH's outer Sequence
    completes and the follower keeps streaming to its target.

    SESSION_LOG 2026-10-06 DEVIATION — the pre-existing terminal was an
    AlwaysSucceed, meaning the incumbent submitted nothing on a reauth
    round's happy path. See the entry for discussion.
    """

    def update(self) -> py_trees.common.Status:
        # Bid for the LIVE R_target computed by BandSensorNode this tick, not
        # the authorized current_relay_target. When the leader has moved and
        # gate 8 has triggered a reauth round, the incumbent must bid for the
        # NEW R_target so the follow-on authorization moves the follower to
        # the shifted band center; bidding the authorized point would renew
        # the stale position. SESSION_LOG 2026-10-06 CORRECTION.
        R_target = self.bb.get("R_target")
        if not R_target:
            # BandSensorNode did not write R_target this tick (band not
            # fillable). Leave pending_proposal untouched. ARBITER_SCAN still
            # returns SUCCESS so RELAYING_BRANCH continues — the viability
            # gates (G4-G7) are the right place to react to infeasibility,
            # not the incumbent-bid terminal.
            return self._set(py_trees.common.Status.SUCCESS,
                             "no live R_target — no incumbent bid")

        cost = _cost_from_target(self.bb, self.config, R_target)
        drone_state = self.bb.get("drone_state") or {}
        follower_id = drone_state.get("drone_id") or self.config.get("drone_id", "drone-02")

        # Generate a fresh proposal_id per bid. ActionNodeBase.__init__ assigns
        # one proposal_id at construction time, which is fine for one-shot
        # proposers (ProposeContinuousRelay / ProposeChainRelay / ...), but
        # the incumbent fires every RELAYING tick — reusing the same id made
        # every bid look like the same proposal across rounds and across
        # process lifetimes. BUILDSPEC §4.9 content-hash dedup is independent
        # of this id; proposal_id is just an identifier.
        proposal_id = f"prop-{uuid.uuid4().hex[:12]}"

        proposal = {
            "proposal_id":    proposal_id,
            "timestamp":      time.time(),
            "strategy":       "CONTINUOUS_RELAY",
            "relay_position": R_target,
            "reason":         "incumbent_bid",
            "trigger":        "incumbent",
            "cost": {
                "follower_id":      follower_id,
                "battery_cost_pct": cost["battery_cost_pct"],
                "repositioning_m":  cost["repositioning_m"],
                "eta_seconds":      cost["eta_seconds"],
            },
        }
        self.bb.set("pending_proposal", proposal)
        log.info(
            "[ProposeIncumbentContinuousRelay] incumbent bid for current target "
            "(%.5f,%.5f) eta=%.0fs",
            R_target.get("lat", 0.0), R_target.get("lon", 0.0),
            cost["eta_seconds"],
        )
        return self._set(py_trees.common.Status.SUCCESS,
                         f"incumbent CONTINUOUS_RELAY bid, eta {cost['eta_seconds']:.0f}s")


_EXIT_GATE_RE = re.compile(r"\(([^)]+)\)\s*$")


class ProposeExitRelay(ActionNodeBase):
    """
    Proposes exit with diagnostic context; the GC decides next action.

    Gate attribution: the tree builder names each ProposeExitRelay instance
    with its gate in parentheses (e.g. "ProposeExitRelay(G4)",
    "ProposeExitRelay(ReauthTimeout)"). We extract that from self.name so
    both the log line and the proposal's trigger_context carry the actual
    gate that fired — not an inference from blackboard state (which can
    disagree, especially for G5 and REAUTH_TIMEOUT).
    """

    def update(self) -> py_trees.common.Status:
        band_fillable  = self.bb.get("band_fillable")
        direct_quality = self.bb.get("gc_leader_direct_quality")
        threshold = self.config.get("relay_exit_quality_threshold", 0.85)

        if direct_quality is not None and direct_quality >= threshold:
            reason = "direct_link_recovered"
        elif band_fillable is False:
            reason = "position_infeasible"
        else:
            reason = "link_ineffective"

        gate_match = _EXIT_GATE_RE.search(self.name or "")
        gate = gate_match.group(1) if gate_match else "unknown"

        proposal = {
            "proposal_id": self.proposal_id,
            "timestamp":   time.time(),
            "strategy":    "EXIT_RELAY",
            "reason":      reason,
            "trigger":     gate,
        }
        self.bb.set("pending_proposal", proposal)
        log.info("[ProposeExitRelay] EXIT_RELAY: gate=%s reason=%s", gate, reason)
        return self._set(py_trees.common.Status.SUCCESS,
                         f"EXIT_RELAY proposed: gate={gate} reason={reason}")
