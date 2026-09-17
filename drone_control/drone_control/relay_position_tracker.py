"""
relay_position_tracker.py — ROS2 node.

Arrival detector and BT fault surface.  Does NOT stream setpoints and does
NOT talk to PX4 — that is relay_mover's job.

Responsibilities:
  1. Detect arrival at current_relay_target via real haversine check and
     publish position_reached.
  2. Publish movement_status periodically with the four guard signals that
     capability_assessor writes to the blackboard for the BT movement-guard
     condition nodes (RelayCommandAccepted, RelayMovementProgressing,
     RelayModeHeld, RelayArrived).

Guard signals (published on /drone_NN/movement_status):
  last_command_ack              {accepted, command_id}
  distance_to_target_decreasing bool   — True when current haversine < previous tick
  offboard_mode_held            bool   — True when drone_state.flight_mode == OFFBOARD
  within_acceptance_radius      bool   — True when haversine < acceptance_radius_m

Arrival detection:
  haversine(current_pos, target) < movement_acceptance_radius_m (default 2.0 m).
  Requires a live current_pos from drone_state.  No elapsed-time simulation.

eta_s is carried from relay_assignment for observability only — it is logged
alongside the measured travel time at arrival but never used for timing.

By-design transport: Layer 2 (publishes ROS2 topics; consumed by
capability_assessor which writes them to the Layer-1 blackboard).
"""

import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# Shared /current_role QoS — must match strategy_executor and every subscriber
# (relay_mover, capability_assessor, continuous_monitor). See strategy_executor.py.
CURRENT_ROLE_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

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

class _RelayPositionTrackerCore:
    """
    Arrival detection and movement_status publishing logic. The ROS2 node is a thin wrapper.
    """

    def __init__(self, drone_id: str, config: dict, publish_status_fn,
                 publish_pos_reached_fn, publish_relay_confirmed_fn,
                 publish_role_fn, clock=None):
        self._drone_id               = drone_id
        self._config                 = config
        self._publish_status         = publish_status_fn
        self._publish_pos_reached    = publish_pos_reached_fn
        self._publish_relay_confirmed = publish_relay_confirmed_fn
        # publish_role_fn was added to close an architectural gap: nothing else in the
        # system produces current_role=RELAYING.  strategy_executor handles
        # MOVING_TO_RELAY and OPEN_TO_RELAY; relay_position_tracker is the sole producer
        # of RELAYING (published on confirmed arrival — see _do_publish_relay_confirmed).
        self._publish_role           = publish_role_fn
        self._clock                  = clock or time.time

        self._target           = None
        self._assignment_id    = None
        self._eta_s            = None
        self._move_start_time  = None
        self._arrived          = False
        self._timeout_fired    = False
        self._active           = False
        self._bb: dict         = {}
        self._prev_distance: float | None = None

    def on_assignment(self, payload: dict) -> None:
        # HARD RULE (BUILDSPEC §5.3 Decision 5): read r_target verbatim from §2.7.
        target = payload.get("r_target")
        if not target:
            return
        self._target          = target
        # relay_assignment (§2.7) has no explicit assignment_id field; drone_id is
        # used as a per-assignment label for logging and relay_confirmed payloads.
        self._assignment_id   = payload.get("drone_id", "?")
        self._arrived         = False
        self._timeout_fired   = False
        self._prev_distance   = None
        self._move_start_time = self._clock()
        self._eta_s           = payload.get("eta_s")

    def on_role(self, role: str) -> None:
        self._active = role in ("MOVING_TO_RELAY", "RELAYING")
        if not self._active:
            self._arrived         = False
            self._timeout_fired   = False
            self._prev_distance   = None
            self._move_start_time = None

    def on_drone_state(self, payload: dict) -> None:
        pos = payload.get("position")
        if pos:
            self._bb["current_pos"] = pos
        self._bb["offboard_mode_held"] = payload.get("flight_mode", "") == "OFFBOARD"

    def tick(self) -> None:
        if not self._active or self._target is None:
            return
        self._check_arrival()
        self._check_timeout()
        self._emit_status()

    def _check_arrival(self) -> None:
        current_pos = self._bb.get("current_pos")
        if self._arrived or current_pos is None:
            return
        from relay_bt.geometry import haversine
        dist   = haversine(current_pos, self._target)
        radius = self._config.get("movement_acceptance_radius_m", 2.0)
        if dist < radius:
            self._arrived       = True
            self._prev_distance = 0.0
            self._do_publish_pos_reached(dist)
        else:
            self._prev_distance = dist

    def _check_timeout(self) -> None:
        if self._arrived or self._timeout_fired or self._move_start_time is None:
            return
        elapsed = self._clock() - self._move_start_time
        # Safety factor > 1.0 gives grace beyond the eta_s estimate, which is
        # derived from cruise_speed_mps and ignores wind, traffic, and acceleration.
        # Falls back to 120 s when eta_s is unknown (current_pos was absent at
        # assignment time, so chain_assigner returned eta_s=0.0).
        factor  = self._config.get("movement_timeout_safety_factor", 1.5)
        timeout = (self._eta_s * factor) if self._eta_s is not None else 120.0
        if elapsed > timeout:
            self._timeout_fired = True
            self._do_publish_relay_confirmed("TIMEOUT")

    def _do_publish_pos_reached(self, final_dist_m: float) -> None:
        actual_s = None
        if self._move_start_time is not None:
            actual_s = round(self._clock() - self._move_start_time, 1)
        payload = {
            "assignment_id":   self._assignment_id,
            "drone_id":        self._drone_id,
            "timestamp":       self._clock(),
            "position":        self._target,
            "actual_travel_s": actual_s,
            "predicted_eta_s": self._eta_s,
        }
        self._publish_pos_reached(payload)
        self._do_publish_relay_confirmed("CONFIRMED")

    def _do_publish_relay_confirmed(self, status: str) -> None:
        payload = {
            "status":        status,
            "assignment_id": self._assignment_id,
            "drone_id":      self._drone_id,
            "timestamp":     self._clock(),
        }
        self._publish_relay_confirmed(payload)
        if status == "CONFIRMED":
            # CONFIRMED = drone physically arrived and is now serving as relay.
            # TIMEOUT = drone failed to arrive; publishing RELAYING would be incorrect
            # because the drone is not at the target and cannot act as relay.
            self._publish_role("RELAYING")

    def _emit_status(self) -> None:
        within     = self._arrived
        progressing = False
        current_pos = self._bb.get("current_pos")

        if current_pos is not None and self._target is not None and not self._arrived:
            from relay_bt.geometry import haversine
            dist = haversine(current_pos, self._target)
            if self._prev_distance is not None:
                progressing = dist < self._prev_distance
            self._prev_distance = dist

        status = {
            "timestamp":    self._clock(),
            "assignment_id": self._assignment_id,
            "last_command_ack": {
                "accepted":   self._target is not None,
                "command_id": self._assignment_id,
            },
            "distance_to_target_decreasing": progressing,
            "offboard_mode_held":            self._bb.get("offboard_mode_held", False),
            "within_acceptance_radius":      within,
        }
        self._publish_status(status)


class RelayPositionTracker(Node):

    def __init__(self):
        node_name = f"relay_position_tracker_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        cfg    = _load_config()
        prefix = _ros_prefix(DRONE_ID)

        self._pos_reached_pub     = self.create_publisher(String, f"{prefix}/position_reached", 10)
        self._relay_confirmed_pub = self.create_publisher(String, f"{prefix}/relay_confirmed", 10)
        self._status_pub          = self.create_publisher(String, f"{prefix}/movement_status", 10)
        self._role_pub            = self.create_publisher(String, f"{prefix}/current_role", CURRENT_ROLE_QOS)

        def _pub_status(d: dict):
            msg = String(); msg.data = json.dumps(d); self._status_pub.publish(msg)

        def _pub_pos_reached(d: dict):
            msg = String(); msg.data = json.dumps(d); self._pos_reached_pub.publish(msg)
            self.get_logger().info(f"position_reached: assign={d.get('assignment_id')}")

        def _pub_relay_confirmed(d: dict):
            msg = String(); msg.data = json.dumps(d); self._relay_confirmed_pub.publish(msg)
            self.get_logger().info(
                f"relay_confirmed: status={d.get('status')} assign={d.get('assignment_id')}"
            )

        def _pub_role(role: str):
            msg = String(); msg.data = role; self._role_pub.publish(msg)
            self.get_logger().info(f"current_role → {role}")

        self._core = _RelayPositionTrackerCore(
            drone_id=DRONE_ID,
            config=cfg,
            publish_status_fn=_pub_status,
            publish_pos_reached_fn=_pub_pos_reached,
            publish_relay_confirmed_fn=_pub_relay_confirmed,
            publish_role_fn=_pub_role,
        )

        self.create_subscription(
            String, f"{prefix}/relay_assignment", self._on_assignment, 10)
        self.create_subscription(
            String, f"{prefix}/current_role", self._on_role, CURRENT_ROLE_QOS)
        self.create_subscription(
            String, f"{prefix}/drone_state", self._on_drone_state, 10)

        hz = cfg.get("relay_position_tracker_hz", 3)
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f"relay_position_tracker started: drone={DRONE_ID} "
            f"status_hz={hz} "
            f"acceptance_radius={cfg.get('movement_acceptance_radius_m', 2.0)}m"
        )

    def _on_assignment(self, msg: String):
        try:
            self._core.on_assignment(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on relay_assignment: {e}")

    def _on_role(self, msg: String):
        self._core.on_role(msg.data.strip())

    def _on_drone_state(self, msg: String):
        try:
            self._core.on_drone_state(json.loads(msg.data))
        except Exception:
            pass

    def _tick(self):
        self._core.tick()


def main():
    rclpy.init()
    node = RelayPositionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
