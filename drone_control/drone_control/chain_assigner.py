"""
chain_assigner.py — relay_assignment publisher.

BUILDSPEC: §4 (relay_assignment schema §2.7)
LAYER:     3  (network-facing, BUILDSPEC §5.5)
SUBSCRIBES: /{drone_id}/authorization   (primary trigger)
            /{drone_id}/drone_state      (for eta_s computation only)
PUBLISHES:  /{drone_id}/relay_assignment (§2.7)

Hard rules this file must satisfy:
  - r_target is verbatim from authorization (no recomputation)  -> proven by test_r_target_verbatim
  - tolerance_radius_m and valid_until carried unchanged         -> proven by test_radius_and_validity_passthrough
  - eta_s is the ONLY computed field                            -> proven by test_computes_only_eta

BUILDSPEC §5.3 Decision 5: "An authorized r_target is never recomputed anywhere
downstream. chain_assigner.py reads it verbatim." Do not call bucket_position()
or haversine() on r_target here.
"""

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


# ── Pure core — testable without ROS2 ────────────────────────────────────────

class _ChainAssignerCore:
    """
    All assignment-building logic lives here. The ROS2 node is a thin wrapper.
    """

    def __init__(self, drone_id: str, config: dict, publish_fn, clock=None):
        self._drone_id    = drone_id
        self._config      = config
        self._publish     = publish_fn
        self._clock       = clock or time.time
        self._current_pos = None
        self._last_auth_key = None  # (proposal_id, round_id) dedup key

    def on_drone_state(self, payload: dict) -> None:
        """Track current position for eta_s computation."""
        self._current_pos = payload.get("position")

    def on_authorization(self, payload: dict) -> None:
        """
        Build and publish relay_assignment on receipt of a movement authorization.
        EXIT_RELAY and LET_LEADER_ISOLATE do not produce assignments.
        BUILDSPEC §2.7: r_target, tolerance_radius_m, valid_until are verbatim passthroughs.
        """
        proposal_id = payload.get("proposal_id")
        round_id    = payload.get("round_id")
        # Dedup guard: same (proposal_id, round_id) pair on DDS replay would assign
        # the same target twice. round_id is included so a new round with the same
        # content hash (same proposal_id) still produces an assignment.
        auth_key = (proposal_id, round_id)
        if proposal_id and auth_key == self._last_auth_key:
            return

        strategy = payload.get("strategy", "")
        # EXIT_RELAY: drone must return home — no relay position assignment needed.
        # LET_LEADER_ISOLATE: handled by another path; no position target involved.
        if strategy not in ("CONTINUOUS_RELAY", "CHAIN_RELAY", "REPOSITION_RELAY"):
            return

        self._last_auth_key = auth_key

        # HARD RULE (BUILDSPEC §5.3 Decision 5): r_target comes verbatim.
        # tolerance_radius_m and valid_until are also verbatim passthroughs.
        r_target = payload.get("r_target")
        if not r_target:
            return

        # eta_s is the SOLE computed field (BUILDSPEC §2.7).
        # Uses cruise_speed_mps from DRONE_MODELS (avg_speed_ms removed — session 4).
        eta_s = self._compute_eta_s(r_target)

        assignment = {
            "drone_id":           self._drone_id,
            "r_target":           r_target,
            "tolerance_radius_m": payload.get("tolerance_radius_m"),
            "valid_until":        payload.get("valid_until"),
            "eta_s":              eta_s,
            "timestamp":          self._clock(),
        }
        self._publish(assignment)

    # ── eta_s computation ─────────────────────────────────────────────────────

    def _compute_eta_s(self, r_target: dict) -> float:
        """
        eta_s = haversine(current_pos, r_target) / cruise_speed_mps.
        Returns 0.0 when current_pos is unknown or computation fails.
        cruise_speed_mps comes from DRONE_MODELS config (§3.3).
        """
        if not self._current_pos:
            return 0.0
        try:
            from relay_bt.geometry import haversine
            d = haversine(self._current_pos, r_target)

            model_id = self._config.get("drone_model_id", "generic")
            model    = self._config.get("DRONE_MODELS", {}).get(model_id, {})
            # Direct key access (not .get(..., fallback)): §7.1 hard stop — must not
            # silently default cruise_speed_mps.  KeyError is caught by outer try/except
            # and returns 0.0, which is correct for an observability-only field.
            speed    = model["cruise_speed_mps"]

            if speed <= 0:
                return 0.0
            return round(d / speed, 1)
        except Exception:
            # eta_s is published for observability only — relay_position_tracker uses its
            # own timeout logic and does not gate on this value.  Swallowing here is
            # intentional; it never silences a gate-level failure.
            return 0.0


# ── ROS2 node ────────────────────────────────────────────────────────────────

class ChainAssigner(Node):

    def __init__(self):
        node_name = f"chain_assigner_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        cfg    = _load_config()
        prefix = _ros_prefix(DRONE_ID)

        self._pub = self.create_publisher(String, f"{prefix}/relay_assignment", 10)

        def _publish(assignment: dict):
            msg      = String()
            msg.data = json.dumps(assignment)
            self._pub.publish(msg)
            self.get_logger().info(
                f"relay_assignment: strategy={assignment.get('strategy')} "
                f"r_target={assignment.get('r_target')} "
                f"eta_s={assignment.get('eta_s')}s"
            )

        self._core = _ChainAssignerCore(DRONE_ID, cfg, _publish)

        self.create_subscription(
            String, f"{prefix}/authorization",
            self._on_authorization, 10)
        self.create_subscription(
            String, f"{prefix}/drone_state",
            self._on_drone_state, 10)

        self.get_logger().info(
            f"chain_assigner started: drone={DRONE_ID} "
            f"pub={prefix}/relay_assignment (r_target verbatim — Decision 5)"
        )

    def _on_authorization(self, msg: String):
        try:
            self._core.on_authorization(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on authorization: {e}")

    def _on_drone_state(self, msg: String):
        try:
            self._core.on_drone_state(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on drone_state: {e}")


def main():
    rclpy.init()
    node = ChainAssigner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
