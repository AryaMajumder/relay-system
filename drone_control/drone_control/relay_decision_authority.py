"""
relay_decision_authority.py — GC-side broadcast-and-collect authority (G_task + G_auth).

BUILDSPEC: §4.10
LAYER:     3  (network-facing decision layer, BUILDSPEC §5.5)
SUBSCRIBES: /gc/gc_link_quality
            /{drone_id}/relay_request      (per known drone from config)
            /{drone_id}/strategy_proposal  (per known drone from config)
            /{drone_id}/reauth_request     (per known drone from config)
            /{drone_id}/alert_intent       (per known drone; §4.11 item 11 → G_task)
PUBLISHES:  /relay_tasking              (shared topic, §2.4, §5.2)
            /{drone_id}/authorization   (per-drone topic, §2.6)

Hard rules this file must satisfy (BUILDSPEC §4.10):
  - relay_tasking to /relay_tasking (shared), not per-drone  -> test_relay_tasking_shared_topic
  - relay_tasking has no target drone field                   -> test_relay_tasking_has_no_target_id
  - authorization per-drone /{drone_id}/authorization         -> test_authorization_per_drone_topic
  - window opens on first proposal, not at broadcast          -> test_window_opens_on_first_proposal
  - window is fixed, late arrivals don't extend it            -> test_window_fixed_not_extended
  - collected set keyed by drone_id (replace, not append)     -> test_collected_set_keyed_by_drone
  - decline (LET_LEADER_ISOLATE) deletes the entry            -> test_decline_deletes_entry
  - winner picked: battery DESC, eta ASC, band DESC, gps DESC -> test_rank_* series
  - r_target verbatim from proposal (no recomputation)        -> test_r_target_verbatim
  - empty window → wait rebroadcast_pause_s → re-broadcast    -> test_empty_window_rebroadcasts
  - relay_request dedup table 1: (drone_id, snr_bucket) 600s  -> test_relay_request_dedup_*
  - decline dedup table 2: (drone_id, reason) tuple key, 180s -> test_decline_dedup_*
  - reauth_request ignored during active collection window     -> test_reauth_request_ignored_during_active_round
  - reauth_request starts new round when system is idle        -> test_reauth_request_starts_round
  - deduped declines still remove the collected entry          -> test_decline_still_processed_when_deduped
  - no MQTT import or client                                   -> test_no_mqtt_client
  - candidate selection reads proposals only (not gc_health)   -> test_no_gc_radio_health_for_selection

DELIBERATELY ABSENT (BUILDSPEC §4.10 stripped):
  No MQTT client. No Lambda forwarding. No circuit breaker. No retry queue.
  GC authority runs entirely local-process, in-memory.
"""

import json
import os
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# GC does not have a per-drone identity like followers do.
# It reads all drones from config to set up subscriptions.
GC_ID = os.environ.get("GC_ID", "gc")

# Quality threshold for triggering a broadcast round.
# BUILDSPEC §4.10: "TRIGGER: gc_link_quality degraded (initial)".
# Using the same threshold as the current gc_link_observer quality scale (0.0–1.0).
_GC_QUALITY_TRIGGER = float(os.environ.get("GC_QUALITY_TRIGGER", "0.5"))


def _load_config() -> dict:
    try:
        sys.path.insert(0, "/root/ros2_ws/src/drone_control/drone_control")
        from config.demo_config import DEMO_CONFIG
        return DEMO_CONFIG
    except Exception as e:
        raise RuntimeError(f"Cannot load demo_config: {e}")


def _ros_prefix(drone_id: str) -> str:
    return f"/{drone_id.replace('-', '_')}"


# ── Pure core — testable without ROS2 ────────────────────────────────────────

class _DecisionCore:
    """
    All state and logic lives here. The ROS2 node is a thin wrapper.

    Timing design (TEST_PROTOCOL §3.3):
      Every duration is compared against self._clock(). Inject a FakeClock
      to drive tests without sleeping. The ROS2 node calls check_timers() on
      a 1 Hz timer so the window and rebroadcast deadlines are checked.
    """

    def __init__(self, config: dict, publish_tasking_fn, publish_auth_fn,
                 clock=None, log_fn=None):
        self._config          = config
        self._publish_tasking = publish_tasking_fn   # (payload: dict) -> None
        self._publish_auth    = publish_auth_fn       # (drone_id: str, payload: dict) -> None
        self._clock           = clock or time.time
        self._log             = log_fn or (lambda msg: None)

        # ── Round state ───────────────────────────────────────────────────────
        self._current_round_id: str | None   = None
        self._collected:        dict         = {}    # drone_id → proposal
        self._window_start:     float | None = None  # set on first proposal
        self._window_closed:    bool         = False
        self._rebroadcast_at:   float | None = None  # set when window closes empty
        self._round_start_at:   float | None = None  # set when relay_tasking published

        # ── Tasking arm state ─────────────────────────────────────────────────
        # Armed: ready to fire a new round on quality drop.
        # Disarmed: round already in flight; re-arm when quality recovers.
        self._armed: bool = True

        # ── Dedup table 1 — relay_request ─────────────────────────────────────
        # Key: (drone_id, snr_bucket). Expiry: relay_request_dedup_expiry_s (600s).
        # Purpose: suppress repeated LOGGING of one continuous degradation episode.
        # HARD RULE: does NOT suppress the sender (L_det keeps publishing).
        # ASSUMPTION: snr_bucket = int(snr_db // 5), i.e. 5dB bins (session log).
        self._rr_dedup: dict = {}   # (drone_id, snr_bucket) → last_seen_at

        # ── Dedup table 2 — declines ──────────────────────────────────────────
        # Key: (drone_id, decline_reason) — DIRECT TUPLE, not a hash.
        # Expiry: decline_dedup_expiry_s (180s).
        # Purpose: avoid re-logging unchanged repeated decline reason.
        # HARD RULE: does NOT suppress the follower; entry is still removed.
        self._decline_dedup: dict = {}   # (drone_id, reason) → last_seen_at

        # ── Alert intents (§4.11 item 11 → G_task) ───────────────────────────
        # FollowerSafetyExit writes alert_intent to BB; capability_assessor
        # publishes /{drone_id}/alert_intent; RDA (this is G_task per §4.10
        # header) receives here.  In-memory ring buffer — no MQTT/Lambda/
        # cloud forwarding per §4.10 hard rule "GC authority runs entirely
        # local-process, in-memory."  Bounded to prevent unbounded growth
        # from a chattering follower; last N alerts retained per drone.
        self._alert_intents: dict = {}   # drone_id → list[payload]  (bounded)
        self._alert_ring_max = int(config.get("alert_intent_ring_max", 32))

    # ── Incoming events ───────────────────────────────────────────────────────

    def on_gc_link_quality(self, payload: dict) -> None:
        """
        PRIMARY TASKING trigger. Fire a broadcast round when quality drops.
        Re-arm when quality recovers so the next degradation can trigger again.
        BUILDSPEC §4.10 step 1 (initial trigger).
        """
        quality = payload.get("quality")
        if quality is None:
            return

        if quality >= _GC_QUALITY_TRIGGER:
            # Link acceptable; re-arm so next drop fires.
            if not self._armed:
                self._armed = True
                self._log(f"GC link recovered quality={quality:.3f} — tasking re-armed")
            return

        # Quality dropped below threshold.
        if not self._armed:
            return  # round already in flight for this degradation episode

        self._log(f"GC link degraded quality={quality:.3f} — starting broadcast round")
        self._start_round("initial")

    def on_relay_request(self, payload: dict) -> None:
        """
        Dedup table 1 — relay_request from a follower/leader.
        Log the first occurrence of each (drone_id, snr_bucket). Suppress repeat logging.
        HARD RULE: does NOT suppress the sender; all processing continues.
        BUILDSPEC §4.10 dedup table 1.
        """
        drone_id = payload.get("drone_id", "unknown")
        snr_db   = payload.get("snr_db") or 0.0
        # 5dB bucket (ASSUMPTION — session log entry).
        snr_bucket = int(snr_db // 5)
        key        = (drone_id, snr_bucket)

        now    = self._clock()
        expiry = self._config.get("relay_request_dedup_expiry_s", 600)

        last = self._rr_dedup.get(key)
        is_dup = (last is not None and now - last < expiry)
        self._rr_dedup[key] = now

        if not is_dup:
            self._log(
                f"relay_request: drone={drone_id} snr={snr_db:.1f}dB "
                f"bucket={snr_bucket} (new episode)"
            )
        # Duplicate relay_requests are intentionally not logged (dedup suppresses only logging).

    def on_reauth_request(self, payload: dict) -> None:
        """
        REAUTH TRIGGER: follower's authorization is expiring; start a new collection round.
        BUILDSPEC §4.10: "TRIGGER: follower reauth request (reauth)."

        Guard: if a round is already actively collecting proposals (_window_start is not None
        and window not yet closed), the active round will produce an authorization that serves
        as the reauth answer — do not reset the collected set mid-flight.
        """
        drone_id = payload.get("drone_id", "unknown")

        # Active round in flight: let it finish.  The resulting authorization is the answer.
        if self._current_round_id is not None and not self._window_closed:
            self._log(
                f"reauth_request from {drone_id} ignored — round "
                f"{self._current_round_id} already active"
            )
            return

        self._log(f"reauth_request from {drone_id} — starting reauth round")
        self._start_round("reauth")

    def on_alert_intent(self, payload: dict) -> None:
        """
        §4.11 item 11: FollowerSafetyExit → capability_assessor → this handler.
        The GC's role for these alerts is observability — record + log.  No
        control loop consumes this; the exit is already actioned by the
        follower's BT via the EXIT_RELAY strategy path.  Cloud forwarding is
        deferred per §4.10 (local-process, in-memory only).

        Ring-buffered per drone to bound memory in the face of a chattering
        follower; oldest evicted when self._alert_ring_max is exceeded.
        """
        drone_id = payload.get("drone_id") or "unknown"
        # drone_id isn't in the payload today (§4.11 item 11 marked 🔴 NEW).
        # Fall back to a "type + reason" summary for logging when absent.
        buf = self._alert_intents.setdefault(drone_id, [])
        buf.append(payload)
        if len(buf) > self._alert_ring_max:
            del buf[0:len(buf) - self._alert_ring_max]

        self._log(
            f"alert_intent from {drone_id}: "
            f"type={payload.get('type', '?')} "
            f"reason={payload.get('reason', '?')} "
            f"battery={payload.get('battery_pct', '?')}"
        )

    def on_strategy_proposal(self, payload: dict) -> None:
        """
        Receive a strategy_proposal from a follower.
        Steps 3 and 4 of the broadcast-and-collect round. BUILDSPEC §4.10.
        LET_LEADER_ISOLATE is treated as a decline (step 4 delete).
        """
        round_id = payload.get("round_id")
        drone_id = payload.get("drone_id", "unknown")
        strategy = payload.get("strategy", "UNKNOWN")

        # Discard proposals from superseded rounds.
        if round_id != self._current_round_id:
            self._log(
                f"strategy_proposal from {drone_id} discarded: "
                f"round_id={round_id!r} != current {self._current_round_id!r}"
            )
            return

        # First proposal in this round: start the collection window.
        # BUILDSPEC §4.10 step 3: "On first proposal arrival: start collection_window_s."
        # Window starts even for declines so that an all-decline round still
        # closes and schedules a rebroadcast instead of stalling indefinitely.
        if self._window_start is None:
            self._window_start  = self._clock()
            self._window_closed = False
            self._log(
                f"Collection window started: first proposal from {drone_id} "
                f"at t={self._window_start:.1f}"
            )

        # LET_LEADER_ISOLATE = decline: always remove, regardless of dedup.
        if strategy == "LET_LEADER_ISOLATE":
            self._handle_decline(drone_id, payload)
            return

        # Window is fixed — no extension on late arrivals.
        # BUILDSPEC §4.10: "fixed, not extended."
        if self._window_closed:
            self._log(
                f"strategy_proposal from {drone_id} arrived after window closed — discarded"
            )
            return

        now     = self._clock()
        elapsed = now - self._window_start
        window  = self._config.get("collection_window_s", 45)
        if elapsed >= window:
            # Window just expired on this arrival; close it before discarding.
            if not self._window_closed:
                self._close_window()
            self._log(
                f"strategy_proposal from {drone_id} arrived after window expired — discarded"
            )
            return

        # Accumulate: new entry or replace existing.
        # BUILDSPEC §4.10 step 4: "new proposal → insert; existing → replace."
        action = "replaced" if drone_id in self._collected else "inserted"
        self._collected[drone_id] = payload
        self._log(
            f"Collected {action}: drone={drone_id} strategy={strategy} "
            f"battery={payload.get('capability_snapshot', {}).get('battery_pct')}"
        )

    def check_timers(self) -> None:
        """
        Check whether the collection window or rebroadcast deadline has been reached.
        Call periodically (e.g. from a ROS2 1Hz timer, or from tests after advancing clock).
        """
        now = self._clock()

        # Collection window timeout.
        window = self._config.get("collection_window_s", 45)
        if (self._window_start is not None
                and not self._window_closed
                and self._current_round_id is not None):
            if now - self._window_start >= window:
                self._close_window()

        # If a round was started but no proposals arrived, the window never
        # opens (_window_start stays None).  Close the round as empty after
        # collection_window_s from the round start so rebroadcast is scheduled.
        elif (self._round_start_at is not None
                and self._window_start is None
                and not self._window_closed
                and self._current_round_id is not None):
            if now - self._round_start_at >= window:
                self._log(
                    f"Round {self._current_round_id} timed out with no proposals "
                    f"— closing empty"
                )
                self._close_window()

        # Rebroadcast deadline.
        if self._rebroadcast_at is not None and now >= self._rebroadcast_at:
            self._rebroadcast_at = None
            self._log("Rebroadcast pause elapsed — starting new broadcast round")
            self._start_round("initial")

    # ── Round management ─────────────────────────────────────────────────────

    def _start_round(self, trigger: str) -> None:
        """
        Steps 1–2 of the broadcast-and-collect round.
        Generate a fresh round_id, reset the collected set, publish relay_tasking.
        BUILDSPEC §4.10 steps 1 and 2.
        """
        self._armed            = False
        self._current_round_id = str(uuid.uuid4())
        self._collected        = {}
        self._window_start     = None
        self._window_closed    = False
        self._rebroadcast_at   = None
        self._round_start_at   = self._clock()

        # relay_tasking schema: §2.4. No target drone field — broadcast is intentional.
        payload = {
            "round_id":  self._current_round_id,
            "timestamp": self._clock(),
            "trigger":   trigger,
        }
        self._publish_tasking(payload)
        self._log(
            f"relay_tasking published: round_id={self._current_round_id} "
            f"trigger={trigger}"
        )

    def _close_window(self) -> None:
        """
        Step 5–6: close the collection window and either rebroadcast or grant authorization.
        BUILDSPEC §4.10 steps 5 and 6.
        """
        self._window_closed = True

        if not self._collected:
            # Step 5: empty window → schedule rebroadcast after pause.
            pause = self._config.get("rebroadcast_pause_s", 120)
            self._rebroadcast_at = self._clock() + pause
            self._log(
                f"Window closed empty — rebroadcasting in {pause}s at "
                f"t={self._rebroadcast_at:.1f}"
            )
            return

        # Step 6: pick winner and grant authorization.
        winner = self._pick_winner()
        self._log(
            f"Window closed with {len(self._collected)} candidate(s) — "
            f"winner={winner.get('drone_id')} "
            f"strategy={winner.get('strategy')}"
        )
        self._grant_authorization(winner)

    # ── Winner selection ─────────────────────────────────────────────────────

    def _pick_winner(self) -> dict:
        """
        Lexicographic comparison. BUILDSPEC §4.10:
          1. battery_pct  DESC  (higher wins)
          2. eta_s        ASC   (lower wins)
          3. t_hi - t_lo  DESC  (wider band wins)
          4. gps_fix_type DESC  (better fix wins)
        No weighting. No blended score. No thresholds. BUILDSPEC: "no weighting, no
        blended score, no thresholds on any field."

        SAFETY FILTER: Exclude leader drone from relay candidates. The leader is
        kept stable; only the follower (other drone) can become the relay point.
        """
        # Filter out leader: only follower can relay
        leader_id = self._config.get("leader_id")
        candidates = {
            did: prop for did, prop in self._collected.items()
            if did != leader_id
        }

        if not candidates:
            # All proposals are from leader (shouldn't happen in 2-drone system)
            # Fall back to original collected set
            candidates = self._collected

        def _sort_key(proposal: dict) -> tuple:
            cs   = proposal.get("capability_snapshot", {})
            t_lo = cs.get("t_lo", 0.0)
            t_hi = cs.get("t_hi", 0.0)
            return (
                -(cs.get("battery_pct") or 0.0),        # DESC → negate
                (proposal.get("eta_s") or float("inf")), # ASC
                -(t_hi - t_lo),                          # DESC → negate
                -(cs.get("gps_fix_type") or 0),          # DESC → negate
            )

        return min(candidates.values(), key=_sort_key)

    # ── Authorization grant ───────────────────────────────────────────────────

    def _grant_authorization(self, winner: dict) -> None:
        """
        Publish authorization to the winning drone. BUILDSPEC §4.10, §2.6.
        r_target is echoed VERBATIM from the proposal — never recomputed.
        BUILDSPEC §5.3 Decision 5: "never recompute a position that has been authorized."
        """
        now     = self._clock()
        cfg     = self._config

        authorization = {
            "proposal_id":        winner.get("proposal_id"),
            "drone_id":           winner.get("drone_id"),
            "round_id":           winner.get("round_id"),
            "strategy":           winner.get("strategy"),
            "r_target":           winner.get("r_target"),   # verbatim passthrough
            "tolerance_radius_m": cfg.get("tolerance_radius_m", 10.0),
            "valid_until":        now + cfg.get("authorization_validity_s", 1800),
            "timestamp":          now,
        }

        drone_id = winner.get("drone_id", "unknown")
        self._publish_auth(drone_id, authorization)
        self._log(
            f"Authorization granted: drone={drone_id} "
            f"strategy={authorization['strategy']} "
            f"proposal_id={authorization['proposal_id']} "
            f"valid_until={authorization['valid_until']:.1f}"
        )

        # Schedule next poll even when a winner was found. Without this, _armed
        # stays False and on_gc_link_quality can't fire a new round unless the
        # link recovers above _GC_QUALITY_TRIGGER — which never happens in a
        # persistent-jamming scenario (e.g. after EXIT_RELAY while GC link is
        # still degraded).
        pause = cfg.get("rebroadcast_pause_s", 120)
        self._rebroadcast_at = now + pause
        self._log(
            f"Next poll scheduled in {pause}s at t={self._rebroadcast_at:.1f}"
        )

    # ── Decline handling ─────────────────────────────────────────────────────

    def _handle_decline(self, drone_id: str, payload: dict) -> None:
        """
        Dedup table 2 — decline from a follower (strategy == "LET_LEADER_ISOLATE").
        Key: (drone_id, reason) DIRECT TUPLE. BUILDSPEC §4.10 dedup table 2:
        "a direct tuple key, not a hash."
        HARD RULE: dedup is LOGGING-ONLY. The collected entry is always removed.
        """
        reason     = (payload.get("trigger_context") or {}).get("reason", "")
        key        = (drone_id, reason)
        now        = self._clock()
        expiry     = self._config.get("decline_dedup_expiry_s", 180)

        last   = self._decline_dedup.get(key)
        is_dup = (last is not None and now - last < expiry)
        self._decline_dedup[key] = now

        if not is_dup:
            self._log(
                f"Decline received: drone={drone_id} reason={reason!r} (new)"
            )
        # Duplicate declines: suppress logging only — do NOT suppress removal.

        # HARD RULE (BUILDSPEC §4.10 step 4): decline → DELETE entry from collected.
        # This must happen even when the decline is deduped.
        removed = self._collected.pop(drone_id, None)
        if removed:
            self._log(f"Removed {drone_id} from collected set (decline)")


# ── ROS2 node ────────────────────────────────────────────────────────────────

class RelayDecisionAuthority(Node):

    def __init__(self):
        node_name = "relay_decision_authority"
        super().__init__(node_name)

        cfg = _load_config()

        # Publishers.
        # HARD RULE (BUILDSPEC §5.2): relay_tasking to ONE shared topic, not per-drone.
        self._tasking_pub = self.create_publisher(String, "/relay_tasking", 10)
        self._auth_pubs   = {}   # drone_id → Publisher

        # Build per-drone publishers from config's DRONE_MODEL_ASSIGNMENT.
        # In production this is the full set of known drones.
        drone_ids = list(cfg.get("DRONE_MODEL_ASSIGNMENT", {}).keys())
        for did in drone_ids:
            topic = f"{_ros_prefix(did)}/authorization"
            self._auth_pubs[did] = self.create_publisher(String, topic, 10)

        def _pub_tasking(payload: dict):
            msg      = String()
            msg.data = json.dumps(payload)
            self._tasking_pub.publish(msg)

        def _pub_auth(drone_id: str, payload: dict):
            pub = self._auth_pubs.get(drone_id)
            if pub:
                msg      = String()
                msg.data = json.dumps(payload)
                pub.publish(msg)
            else:
                self.get_logger().warning(
                    f"No publisher for drone_id={drone_id} — authorization dropped"
                )

        self._core = _DecisionCore(
            config=cfg,
            publish_tasking_fn=_pub_tasking,
            publish_auth_fn=_pub_auth,
            clock=time.time,
            log_fn=self.get_logger().info,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String, "/gc/gc_link_quality",
            self._on_gc_link_quality, 10)

        # Subscribe to each known drone's relay_request, strategy_proposal, and reauth_request.
        # rclpy does not support topic wildcards; enumerate from config.
        # BUILDSPEC §4.10: "does NOT subscribe gc_radio_health for candidate selection."
        for did in drone_ids:
            prefix = _ros_prefix(did)
            self.create_subscription(
                String, f"{prefix}/relay_request",
                self._on_relay_request, 10)
            self.create_subscription(
                String, f"{prefix}/strategy_proposal",
                self._on_strategy_proposal, 10)
            self.create_subscription(
                String, f"{prefix}/reauth_request",
                self._on_reauth_request, 10)
            # §4.11 item 11: capability_assessor publishes safety-exit alerts.
            # Prior to 2026-08-24 this had no subscriber — safety notifications
            # were silently dropped.  See SESSION_LOG DECISION entry.
            self.create_subscription(
                String, f"{prefix}/alert_intent",
                self._on_alert_intent, 10)

        # 1Hz timer drives window and rebroadcast deadline checks.
        self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            f"relay_decision_authority started: drones={drone_ids}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_gc_link_quality(self, msg: String):
        try:
            self._core.on_gc_link_quality(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on gc_link_quality: {e}")

    def _on_relay_request(self, msg: String):
        try:
            self._core.on_relay_request(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on relay_request: {e}")

    def _on_strategy_proposal(self, msg: String):
        try:
            self._core.on_strategy_proposal(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on strategy_proposal: {e}")

    def _on_reauth_request(self, msg: String):
        try:
            self._core.on_reauth_request(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on reauth_request: {e}")

    def _on_alert_intent(self, msg: String):
        try:
            self._core.on_alert_intent(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on alert_intent: {e}")

    def _on_timer(self):
        self._core.check_timers()


def main():
    rclpy.init()
    node = RelayDecisionAuthority()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
