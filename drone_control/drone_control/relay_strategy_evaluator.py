"""
relay_strategy_evaluator.py — Layer 3 strategy proposal publisher.

BUILDSPEC: §4.9
LAYER:     3  (network-facing decision layer, BUILDSPEC §5.5)
SUBSCRIBES: /{drone_id}/capability_report  (primary trigger)
            /relay_tasking                  (shared; tracks current round_id)
PUBLISHES:  /{drone_id}/strategy_proposal   (§2.5)

Hard rules this file must satisfy (BUILDSPEC §4.9):
  - proposal_id = 'prop-' + sha1(content_key)[:12]     -> proven by test_proposal_id_format
  - field named 'strategy', not 'strategy_type'         -> proven by test_field_named_strategy
  - no 'confidence_score' field                         -> proven by test_no_confidence_score
  - no 'band_range' field (consumer computes t_hi-t_lo) -> proven by test_no_band_range_field
  - r_target snapped to position_bucket_m grid          -> proven by test_r_target_snapped
  - content-hash dedup: same conditions -> no republish -> proven by test_content_hash_dedup
  - round_id echoed from prompting relay_tasking        -> proven by test_round_id_echoed

DELIBERATELY ABSENT (BUILDSPEC §7.4): no /link_state subscription.
  relay_bt_design_notes.docx described trigger_context from /link_state but no
  file publishes it. trigger_context uses BT-execution context shape (§2.5).
"""

import hashlib
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DRONE_ID = os.environ.get("DRONE_ID", "drone-01")


def _load_config() -> dict:
    try:
        sys.path.insert(0, "/root/ros2_ws/src/drone_control/drone_control")
        from config.demo_config import DEMO_CONFIG
        return DEMO_CONFIG
    except Exception as e:
        raise RuntimeError(f"Cannot load demo_config: {e}")


def _ros_prefix(drone_id: str) -> str:
    return f"/{drone_id.replace('-', '_')}"


def _check_pass(checks: dict, node_name: str) -> bool:
    """Extract pass/fail from a capability_report checks dict entry."""
    entry = checks.get(node_name)
    if entry is None:
        return False
    return bool(entry.get("pass", False))


# ── Pure core — testable without ROS2 ────────────────────────────────────────

class _StrategyEvaluatorCore:
    """
    All logic lives here. The ROS2 node is a thin wrapper that injects
    publish_fn and drives on_* callbacks from subscriptions.
    """

    def __init__(self, drone_id: str, config: dict, publish_fn, clock=None):
        self._drone_id     = drone_id
        self._config       = config
        self._publish      = publish_fn
        self._clock        = clock or time.time
        self._last_hash    = None   # content-hash dedup
        self._round_id     = None   # tracked from /relay_tasking

    def on_relay_tasking(self, payload: dict) -> None:
        # Track round_id so every proposal echoes it per §2.5.
        # Reset content-hash dedup so a new round always produces a proposal
        # even when battery/SNR conditions haven't changed since the last round.
        self._round_id  = payload.get("round_id")
        self._last_hash = None

    def on_capability_report(self, report: dict) -> None:
        pending = report.get("pending_proposal")

        # No pending_proposal means the BT failed before reaching any Propose*
        # node (e.g. RelayRequestReceived failed because the drone missed the
        # relay_tasking). If a round is active the drone is still implicitly
        # INCAPABLE — synthesize a decline so the RDA window can open and
        # eventually schedule a rebroadcast.
        if not pending:
            if self._round_id is None:
                return
            pending = {
                "strategy": "LET_LEADER_ISOLATE",
                "reason":   "incapable — no pending_proposal from BT",
            }

        strategy = pending.get("strategy", "UNKNOWN")
        inputs   = report.get("inputs", {})

        # Content-hash dedup: same strategy+conditions -> same id -> skip.
        # BUILDSPEC §4.9: "content-hash dedup" — position and timestamp excluded.
        content_hash = self._make_content_hash(strategy, inputs)
        if content_hash == self._last_hash:
            return

        self._last_hash = content_hash
        self._publish_proposal(pending, report, content_hash)

    # ── Content hash ─────────────────────────────────────────────────────────

    def _make_content_hash(self, strategy: str, inputs: dict) -> str:
        """
        proposal_id = 'prop-' + sha1(key)[:12]. BUILDSPEC §4.9.
        Bucket battery (20%), SNR (5dB). Position and timestamp excluded so
        R_target jitter doesn't produce repeated proposals for the same situation.
        """
        batt_b = int((inputs.get("battery_pct") or 0.0) // 20)
        snr_b  = int((inputs.get("gc_snr_db")   or 0.0) // 5)
        key    = f"{strategy}|batt={batt_b}|snr={snr_b}"
        return "prop-" + hashlib.sha1(key.encode()).hexdigest()[:12]

    # ── Proposal builder ─────────────────────────────────────────────────────

    def _publish_proposal(self, pending: dict, report: dict,
                          proposal_id: str) -> None:
        strategy = pending.get("strategy", "UNKNOWN")
        inputs   = report.get("inputs", {})
        checks   = report.get("checks", {})

        # Snap relay_position → r_target. BUILDSPEC §4.9, §5.3 Decision 5:
        # the bucketed position is what the GC authorizes; never recomputed downstream.
        raw_pos  = pending.get("relay_position")
        r_target = self._snap_position(raw_pos)

        # capability_snapshot — all §2.5 fields must be present.
        # cap_gc_m / cap_leader_m / cap_follower_m are BandSensorNode local vars
        # not written to the blackboard; set to 0.0 (ASSUMPTION — session log entry).
        capability_snapshot = {
            "battery_pct":      inputs.get("battery_pct") or 0.0,
            "gps_fix_type":     inputs.get("gps_fix_type") or 0,
            "cap_gc_m":         inputs.get("cap_gc_m") or 0.0,
            "cap_leader_m":     inputs.get("cap_leader_m") or 0.0,
            "cap_follower_m":   inputs.get("cap_follower_m") or 0.0,
            "band_feasible":    _check_pass(checks, "BandSensorNode"),
            "t_lo":             inputs.get("band_t_lo") or 0.0,
            "t_hi":             inputs.get("band_t_hi") or 0.0,
            "geofence_ok":      _check_pass(checks, "GeofenceContainsRelayPos"),
            "return_margin_ok": _check_pass(checks, "BatterySufficientForReturn"),
        }

        # trigger_context from BT-execution context. BUILDSPEC §2.5, §7.4:
        # use the BT execution shape, not /link_state (which doesn't exist).
        trigger_context = {
            "gate_fired": pending.get("trigger", ""),
            "reason":     pending.get("reason") or report.get("reason", ""),
            "source":     "capability_assessor",
        }

        # eta_s: informational, from proposal cost. BUILDSPEC §2.5 note.
        cost  = pending.get("cost", {})
        eta_s = float(cost.get("eta_seconds") or 0.0)

        proposal = {
            "proposal_id":        proposal_id,
            "drone_id":           self._drone_id,
            "round_id":           self._round_id,
            "strategy":           strategy,
            "r_target":           r_target,
            "eta_s":              eta_s,
            "capability_snapshot": capability_snapshot,
            "trigger_context":    trigger_context,
            "timestamp":          self._clock(),
        }

        # HARD RULE (BUILDSPEC §2.5): no 'confidence_score', no 'band_range',
        # no 'strategy_type'. Field set above is complete; do not extend.
        self._publish(proposal)

    # ── Position snapping ────────────────────────────────────────────────────

    def _snap_position(self, raw_pos) -> dict | None:
        """
        Snap relay_position to position_bucket_m grid → r_target.
        BUILDSPEC §4.9: snap before the position leaves this node.
        Returns None if raw_pos is None.
        """
        if not raw_pos:
            return None
        try:
            from relay_bt.geometry import bucket_position
            bucket_m = self._config.get("position_bucket_m", 5.0)
            snapped  = bucket_position(raw_pos, bucket_m)
            # §2.5/§2.7 r_target uses alt_m.  geometry.bucket_position returns
            # {lat, lon, alt} (internal convention).  Translate: prefer explicit
            # alt_m; else fall back to alt; else 0.  Never silently 0 out an
            # altitude that was present in the input.
            if snapped:
                snapped["alt_m"] = raw_pos.get("alt_m",
                                    raw_pos.get("alt", snapped.get("alt", 0.0)))
                snapped.pop("alt", None)   # keep the payload strictly §2.7
            return snapped
        except Exception:
            pos = dict(raw_pos)
            pos["alt_m"] = raw_pos.get("alt_m", raw_pos.get("alt", 0.0))
            pos.pop("alt", None)
            return pos


# ── ROS2 node ────────────────────────────────────────────────────────────────

class RelayStrategyEvaluator(Node):

    def __init__(self):
        node_name = f"relay_strategy_evaluator_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        cfg    = _load_config()
        prefix = _ros_prefix(DRONE_ID)

        published = []  # captured for diagnostics only

        def _publish(proposal: dict):
            msg      = String()
            msg.data = json.dumps(proposal)
            self._pub.publish(msg)
            published.append(proposal)
            self.get_logger().info(
                f"strategy_proposal published: "
                f"proposal_id={proposal['proposal_id']} "
                f"strategy={proposal['strategy']} "
                f"round_id={proposal['round_id']} "
                f"r_target={proposal.get('r_target')}"
            )

        self._core = _StrategyEvaluatorCore(DRONE_ID, cfg, _publish)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            String, f"{prefix}/strategy_proposal", 10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String, f"{prefix}/capability_report",
            self._on_capability_report, 10)

        # /relay_tasking is a shared topic (§2.4, §5.2) — every follower subscribes.
        # We track round_id here to echo it in every strategy_proposal (§2.5).
        self.create_subscription(
            String, "/relay_tasking",
            self._on_relay_tasking, 10)

        self.get_logger().info(
            f"relay_strategy_evaluator started: drone={DRONE_ID} "
            f"pub={prefix}/strategy_proposal"
        )

    def _on_relay_tasking(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.get_logger().info(f"relay_tasking received: round_id={payload.get('round_id')}")
            self._core.on_relay_tasking(payload)
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on relay_tasking: {e}")

    def _on_capability_report(self, msg: String):
        try:
            self._core.on_capability_report(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on capability_report: {e}")


def main():
    rclpy.init()
    node = RelayStrategyEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
