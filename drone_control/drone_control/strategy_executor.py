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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# Startup-reconcile persistence (see SESSION_LOG 2026-10-01 DEVIATION).
# current_role is TRANSIENT_LOCAL; without reconcile, a restart would either
# (a) leave the previous session's stale role latched in DDS forever, or
# (b) blindly publish OPEN_TO_RELAY on startup and tear down a legitimately
# active relay. Persisting the last accepted authorization lets startup
# distinguish the two cases.
_STATE_DIR = os.environ.get(
    "STRATEGY_EXECUTOR_STATE_DIR", "/var/lib/drone-control"
)
_AUTH_MAX_AGE_S = float(
    os.environ.get("STRATEGY_EXECUTOR_AUTH_MAX_AGE_S", "300")
)

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

    def __init__(self, publish_role_fn, persist_fn=None):
        self._publish_role    = publish_role_fn
        self._persist         = persist_fn or (lambda _payload: None)
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
            self._persist(payload)

        elif strategy == "EXIT_RELAY":
            # HARD RULE (BUILDSPEC §4.8): cannot tell battery vs. timeout vs. link loss.
            # All EXIT_RELAY arrivals are identical here.
            # "OPEN_TO_RELAY" is the correct §2.9 enum value — "IDLE" is not valid.
            self._publish_role("OPEN_TO_RELAY")
            self._persist(payload)

        # All other values (including LET_LEADER_ISOLATE) — no action.


# ── ROS2 node ────────────────────────────────────────────────────────────────

def _state_path(drone_id: str) -> str:
    return os.path.join(_STATE_DIR, f"active_auth_{drone_id}.json")


def _persist_auth(drone_id: str, payload: dict, logger=None) -> None:
    """Write-then-rename so a crash mid-write cannot leave a half-file."""
    path = _state_path(drone_id)
    tmp  = path + ".tmp"
    try:
        os.makedirs(_STATE_DIR, exist_ok=True)
        envelope = {"ts": time.time(), "payload": payload}
        with open(tmp, "w") as f:
            json.dump(envelope, f)
        os.replace(tmp, path)
    except OSError as e:
        if logger:
            logger.warning(f"could not persist auth: {e}")


def _load_persisted_auth(drone_id: str, max_age_s: float, logger=None):
    path = _state_path(drone_id)
    try:
        with open(path) as f:
            envelope = json.load(f)
    except FileNotFoundError:
        return None
    except (OSError, json.JSONDecodeError) as e:
        if logger:
            logger.warning(f"could not read persisted auth ({e}); ignoring")
        return None
    age = time.time() - float(envelope.get("ts", 0))
    if age > max_age_s:
        if logger:
            logger.info(f"persisted auth is {age:.0f}s old (>{max_age_s:.0f}s) — discarding")
        return None
    return envelope.get("payload") or {}


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

        logger = self.get_logger()

        def _persist(payload: dict):
            _persist_auth(DRONE_ID, payload, logger)

        self._core = _StrategyExecutorCore(_publish_role, _persist)

        # HARD RULE (BUILDSPEC §4.8): authorization is the ONLY subscription.
        # Do not add relay_confirmed, capability_report, drone_state, or any other.
        self.create_subscription(
            String, f"{prefix}/authorization",
            self._on_authorization, 10)

        # Startup reconcile (see SESSION_LOG 2026-10-01 DEVIATION):
        # overwrite any stale TRANSIENT_LOCAL latch with a value that reflects
        # our actual last-known authorization — or OPEN_TO_RELAY if none/expired.
        persisted = _load_persisted_auth(DRONE_ID, _AUTH_MAX_AGE_S, logger)
        if persisted is None:
            logger.info("startup reconcile: no live auth → OPEN_TO_RELAY")
            _publish_role("OPEN_TO_RELAY")
        else:
            strat = persisted.get("strategy", "")
            if strat in ("CONTINUOUS_RELAY", "CHAIN_RELAY"):
                logger.info(f"startup reconcile: live auth ({strat}) → MOVING_TO_RELAY")
                _publish_role("MOVING_TO_RELAY")
            else:
                logger.info(f"startup reconcile: last auth was {strat!r} → OPEN_TO_RELAY")
                _publish_role("OPEN_TO_RELAY")
            # Re-seed dedup so a replay of this same proposal is still ignored.
            self._core._last_proposal_id = persisted.get("proposal_id")

        logger.info(
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
