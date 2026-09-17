"""
strategy_executor.py — Thin role-state machine.

BUILDSPEC: §4.8
LAYER:     3  (network-facing, BUILDSPEC §5.5)
SUBSCRIBES: /{drone_id}/authorization  (THE ONLY SUBSCRIPTION — §4.8 hard rule)
PUBLISHES:  /{drone_id}/current_role

Hard rules this file must satisfy (BUILDSPEC §4.8):
  - single subscription: authorization only           -> proven by test_single_subscription
  - no condition checking, no timers, no abort logic  -> proven by test_no_condition_logic
  - all four §4.8 strategy rows applied correctly     -> proven by test_mapping_complete
  - REPOSITION_RELAY → no role change                 -> proven by test_reposition_no_role_change
  - EXIT_RELAY source indistinguishable               -> proven by test_exit_sources_indistinguishable

Strategy → current_role mapping (BUILDSPEC §4.8):
  CONTINUOUS_RELAY | CHAIN_RELAY → MOVING_TO_RELAY
  REPOSITION_RELAY               → (no change)
  EXIT_RELAY                     → OPEN_TO_RELAY

DELIBERATELY ABSENT:
  - relay_confirmed subscription: MOVING_TO_RELAY → RELAYING is handled outside this
    file (§4.8 table has no relay_confirmed row; test_single_subscription enforces this).
  - LET_LEADER_ISOLATE handling: not in §4.8 mapping table.
  - Independent abort logic: every exit arrives as EXIT_RELAY through the pipeline.
    This file cannot and must not decide to exit on its own.
"""

import json
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# Late joiners (relay_mover after restart, capability_assessor, continuous_monitor,
# relay_position_tracker) receive the last-published current_role via TRANSIENT_LOCAL.
# All subscribers of /current_role MUST match this profile or messages are silently
# dropped. See relay_mover.py, capability_assessor.py, continuous_monitor.py,
# relay_position_tracker.py.
CURRENT_ROLE_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

DRONE_ID = os.environ.get("DRONE_ID", "drone-01")


def _ros_prefix(drone_id: str) -> str:
    return f"/{drone_id.replace('-', '_')}"


# ── Pure core — testable without ROS2 ────────────────────────────────────────

class _StrategyExecutorCore:
    """
    All role-mapping logic lives here. The ROS2 node is a thin wrapper.
    No timers. No condition checks. No abort logic.
    """

    def __init__(self, publish_role_fn):
        self._publish_role    = publish_role_fn
        self._last_proposal_id = None   # dedup: same proposal never processed twice

    def on_authorization(self, payload: dict) -> None:
        """
        Apply the §4.8 strategy → current_role mapping.
        Dedup on proposal_id (§2.6 field) so a resent authorization doesn't
        double-fire a role change. Unrecognized strategies are silently ignored.
        """
        proposal_id = payload.get("proposal_id")
        # Defensive dedup: §4.8 is silent on proposal_id handling, but ROS2 message
        # replay can deliver the same authorization twice.  Mirrors chain_assigner's
        # identical guard so a duplicate never triggers a second role change.
        if proposal_id and proposal_id == self._last_proposal_id:
            return
        self._last_proposal_id = proposal_id

        strategy = payload.get("strategy", "")

        if strategy in ("CONTINUOUS_RELAY", "CHAIN_RELAY"):
            # Step toward the relay position — chain_assigner supplies the setpoint,
            # relay_mover starts streaming, relay_position_tracker watches arrival.
            self._publish_role("MOVING_TO_RELAY")

        elif strategy == "REPOSITION_RELAY":
            # HARD RULE (BUILDSPEC §4.8): no role change.
            # chain_assigner updates the target; relay_mover picks it up on next tick.
            # The drone is already RELAYING; only the destination changes.
            pass

        elif strategy == "EXIT_RELAY":
            # HARD RULE (BUILDSPEC §4.8): cannot tell battery vs. timeout vs. link loss.
            # All EXIT_RELAY arrivals are identical here.
            # "OPEN_TO_RELAY" is the correct §2.9 enum value — "IDLE" is not valid.
            self._publish_role("OPEN_TO_RELAY")

        # All other values (including LET_LEADER_ISOLATE) — no action.


# ── ROS2 node ────────────────────────────────────────────────────────────────

class StrategyExecutor(Node):

    def __init__(self):
        node_name = f"strategy_executor_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        prefix = _ros_prefix(DRONE_ID)

        self._role_pub = self.create_publisher(String, f"{prefix}/current_role", CURRENT_ROLE_QOS)

        def _publish_role(role: str):
            msg      = String()
            msg.data = role
            self._role_pub.publish(msg)
            self.get_logger().info(f"current_role → {role}")

        self._core = _StrategyExecutorCore(_publish_role)

        # HARD RULE (BUILDSPEC §4.8): authorization is the ONLY subscription.
        # Do not add relay_confirmed, capability_report, drone_state, or any other.
        self.create_subscription(
            String, f"{prefix}/authorization",
            self._on_authorization, 10)

        self.get_logger().info(
            f"strategy_executor started: drone={DRONE_ID} "
            f"pub={prefix}/current_role"
        )

    def _on_authorization(self, msg: String):
        try:
            self._core.on_authorization(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on authorization: {e}")


def main():
    rclpy.init()
    node = StrategyExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
