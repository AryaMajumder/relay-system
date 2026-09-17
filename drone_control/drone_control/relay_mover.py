"""
relay_mover.py — ROS2 node.

Continuous setpoint streamer and sole owner of OFFBOARD keepalive.

Activation contract (BT boundary):
  The BT activates this node by publishing current_role=MOVING_TO_RELAY or
  RELAYING on the current_role topic.  That is the only BT dependency.
  The setpoint stream then runs at its own rate (setpoint_stream_hz, default
  10 Hz) in a dedicated callback group — completely independent of the BT
  tick rate.  The BT deactivates by publishing any other role.

Streaming runs at 10 Hz by default so OFFBOARD is established in ~0.3 s
(OFFBOARD_PRE_SECS setpoints), giving the BT's OffboardModeHeld grace window
(1.5 s) a 5× margin regardless of when the assessor starts.

OFFBOARD keepalive (load-bearing): PX4 drops OFFBOARD if the setpoint
stream stops for > 500 ms.  At 10 Hz the keepalive heartbeat is 100 ms —
well inside that limit.  There is no idle or paused state while active.

OFFBOARD activation: after OFFBOARD_PRE_SECS of continuous setpoint streaming,
px4_agent.start_offboard() is called once per activation to send START_LEAD to
the production px4_agent, which commands PX4 into OFFBOARD mode.

By-design transport: Layer 2 (calls px4_agent write path — intra-device).
"""

import json
import logging
import os
import sys
import time
import threading

log = logging.getLogger(__name__)

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# Late-joining subscription to /current_role — without TRANSIENT_LOCAL the mover
# misses the last-published role on restart (strategy_executor doesn't republish),
# so on_role never fires, _active stays False, and the entire OFFBOARD retry loop
# is dormant. Must match strategy_executor + relay_position_tracker publisher QoS.
CURRENT_ROLE_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

from drone_control.px4_agent import PX4Agent

DRONE_ID          = os.environ.get("DRONE_ID", "drone-01")
OFFBOARD_PRE_SECS = float(os.environ.get("OFFBOARD_PRE_SECS", "1.5"))


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

class _RelayMoverCore:
    """
    Setpoint streaming and OFFBOARD keepalive logic. The ROS2 node is a thin wrapper.

    Thread-safety: _lock guards _active, _target, _stream_start_t, _offboard_sent.
    tick() runs on the stream-timer thread; on_* run on the subscription thread.
    """

    def __init__(self, config: dict, publish_setpoint_fn, start_offboard_fn,
                 send_command_fn, clock=None, log_fn=None):
        self._config           = config
        self._publish_setpoint = publish_setpoint_fn
        self._start_offboard   = start_offboard_fn
        self._send_command     = send_command_fn
        self._clock            = clock or time.monotonic
        # log_fn: optional callable taking (msg: str) — used by the ROS node
        # wrapper to route rate-probe warnings to the node's rclcpp logger so
        # they reach journalctl.  Falls back to python stdlib logging (which
        # under rclpy silently drops WARNING-level messages unless configured).
        self._log_fn           = log_fn or (lambda msg: log.warning(msg))
        self._lock             = threading.Lock()
        self._target           = None
        self._active           = False
        self._stream_start_t   = None
        self._offboard_sent    = False
        self._offboard_sent_t  = None   # when START_LEAD was last fired
        self._hold_sent_t      = None   # when SET_MODE HOLD was sent for retry
        self._flight_mode      = "UNKNOWN"
        self._stream_count     = 0
        # Rate-health instrumentation.  _last_tick_t tracks the previous tick's
        # wall time so we can detect when the 10 Hz ROS timer is starved (any gap
        # > 500 ms will drop PX4 out of OFFBOARD).  Every 50 ticks (~5 s at 10 Hz)
        # we log the observed max gap plus the total window duration; if the ROS
        # timer is healthy the window will be ~5.0 s with max_gap ~0.1 s.  Gaps
        # significantly above 100 ms indicate the mover's executor is stalled
        # (subscription callback contention, GIL, paho publish blocking, …).
        self._last_tick_t      = None
        self._max_gap          = 0.0
        self._rate_window_t    = None
        self._rate_publish_t   = 0.0   # accumulated time inside publish_setpoint per window
        self._raw_tick_count   = 0     # counts ALL tick() invocations, not just active ones
        # G3-RTL rate-limit: forward the first offboard_unrecoverable RTL (so
        # PX4 actually gets the stop command), then suppress all further ones
        # after offboard_rtl_window_secs.  G1/G2 RTLs have different reasons
        # and are never rate-limited.
        self._rtl_first_sent_t = None

    def on_flight_mode(self, mode: str) -> None:
        with self._lock:
            self._flight_mode = mode
            if mode == "OFFBOARD":
                self._rtl_first_sent_t = None  # OFFBOARD achieved; allow fresh RTL if needed

    def on_assignment(self, payload: dict) -> None:
        # HARD RULE (BUILDSPEC §5.3 Decision 5): read r_target verbatim from §2.7.
        target = payload.get("r_target")
        if not target:
            return
        with self._lock:
            prev = self._target
            self._target = target
            # Reset the G3-RTL window only when the target coordinates actually
            # change (reposition).  chain_assigner re-publishes the same assignment
            # every tick; ignoring identical repeats prevents the window from
            # restarting continuously and blocking suppression.
            if (prev is None
                    or prev.get("lat") != target.get("lat")
                    or prev.get("lon") != target.get("lon")
                    or prev.get("alt_m") != target.get("alt_m")):
                self._rtl_first_sent_t = None

    def on_role(self, role: str) -> None:
        with self._lock:
            was_active = self._active
            self._active = role in ("MOVING_TO_RELAY", "RELAYING")
            if was_active and not self._active:
                # Deactivating: clear streaming state so next activation starts fresh.
                self._stream_start_t  = None
                self._offboard_sent   = False
                self._offboard_sent_t = None
                self._hold_sent_t     = None
                # _rtl_first_sent_t is intentionally NOT cleared here: the G3 RTL
                # window must survive role-cycling.  It is only reset on OFFBOARD
                # achieved (on_flight_mode) or on a new relay_assignment (on_assignment).

    def on_pending_command(self, payload: dict) -> None:
        action = payload.get("command", "")
        if not action:
            return
        # Rate-limit G3-sourced RTLs (reason="offboard_unrecoverable:*"): forward
        # the first one so PX4 actually receives the stop command, then suppress
        # all further ones once offboard_rtl_window_secs has elapsed.
        # G1 (battery) and G2 (FCU lost) RTLs use different reasons — never suppressed.
        if (action == "RTL"
                and payload.get("reason", "").startswith("offboard_unrecoverable")):
            with self._lock:
                now    = self._clock()
                window = self._config.get("offboard_rtl_window_secs", 5.0)
                if self._rtl_first_sent_t is None:
                    self._rtl_first_sent_t = now       # first RTL: record and allow
                elif now - self._rtl_first_sent_t > window:
                    return                             # window expired: suppress
        self._send_command(action)

    def tick(self) -> None:
        # Rate-health probe — measure gap since previous tick regardless of _active,
        # so we can distinguish "timer stalled" from "timer fine but drone not active".
        # Kept outside the main lock to avoid perturbing the measurement itself.
        _probe_now = self._clock()
        if self._last_tick_t is not None:
            _gap = _probe_now - self._last_tick_t
            if _gap > self._max_gap:
                self._max_gap = _gap
        self._last_tick_t = _probe_now
        if self._rate_window_t is None:
            self._rate_window_t = _probe_now
        self._raw_tick_count += 1

        # Unconditional probe — fires every 50 raw ticks (~5 s at 10 Hz) whether
        # or not the mover is _active.  This lets us distinguish two failure modes:
        # (a) ROS timer starved — probe stops firing entirely
        # (b) ROS timer fine but pipeline dormant — probe fires with _active=False
        # Also reports the mover's activation state so we can correlate with any
        # daemon-side stalls when the mover thinks it's streaming.
        if self._raw_tick_count % 50 == 0:
            _window = _probe_now - self._rate_window_t
            self._log_fn(
                f"rate_probe: raw_ticks={self._raw_tick_count} "
                f"window={_window:.2f}s max_gap={self._max_gap:.3f}s "
                f"active={self._active} target={'set' if self._target else 'None'} "
                f"stream_count={self._stream_count} "
                f"pub_total={self._rate_publish_t:.3f}s "
                f"(expected window~5.0s max_gap~0.10s)"
            )
            self._max_gap        = 0.0
            self._rate_window_t  = _probe_now
            self._rate_publish_t = 0.0

        with self._lock:
            if not self._active or self._target is None:
                return
            # Start the pre-stream clock on the first tick where both active and
            # target are known — OFFBOARD_PRE_SECS counts from actual streaming,
            # not from role activation (which may precede relay_assignment arrival).
            if self._stream_start_t is None:
                self._stream_start_t = self._clock()
            lat = self._target["lat"]
            lon = self._target["lon"]
            alt = self._target["alt_m"]
            stream_start_t = self._stream_start_t
            offboard_sent  = self._offboard_sent
            offboard_pre   = self._config.get("OFFBOARD_PRE_SECS", OFFBOARD_PRE_SECS)
            retry_secs     = self._config.get("offboard_retry_secs", 5.0)
            hold_settle    = self._config.get("hold_settle_secs", 2.0)
            flight_mode    = self._flight_mode
            self._stream_count += 1
            now = self._clock()

            fire_offboard = False
            fire_hold     = False

            if not offboard_sent and now - stream_start_t >= offboard_pre:
                # Gate: wait for HOLD to settle before retrying START_LEAD.
                if (self._hold_sent_t is None
                        or now - self._hold_sent_t >= hold_settle):
                    self._offboard_sent   = True
                    self._offboard_sent_t = now
                    fire_offboard = True
            elif (offboard_sent
                  and flight_mode != "OFFBOARD"
                  and self._offboard_sent_t is not None
                  and now - self._offboard_sent_t >= retry_secs):
                # START_LEAD fired but OFFBOARD not achieved — switch to HOLD
                # (a mode PX4 accepts OFFBOARD from) then retry START_LEAD.
                fire_hold             = True
                self._hold_sent_t     = now
                self._offboard_sent   = False
                self._offboard_sent_t = None
                self._stream_start_t  = now  # restart pre-stream countdown

        # I/O outside the lock so publish_setpoint / start_offboard don't block
        # subscription callbacks.
        _pub_t0 = self._clock()
        self._publish_setpoint(lat, lon, alt)
        _pub_elapsed = self._clock() - _pub_t0
        self._rate_publish_t += _pub_elapsed

        if fire_hold:
            self._send_command("SET_MODE", {"mode": "HOLD"})
        if fire_offboard:
            self._start_offboard()


class RelayMover(Node):

    def __init__(self, px4: PX4Agent):
        node_name = f"relay_mover_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        cfg    = _load_config()
        self._px4 = px4
        prefix = _ros_prefix(DRONE_ID)

        self._core = _RelayMoverCore(
            config=cfg,
            publish_setpoint_fn=px4.publish_setpoint,
            start_offboard_fn=px4.start_offboard,
            send_command_fn=px4.send_command,
            log_fn=lambda msg: self.get_logger().warning(msg),
        )

        # Dedicated callback group for the setpoint stream timer so it runs on
        # its own executor thread, independent of subscription callbacks.
        # This guarantees the stream fires at its configured rate even if a
        # subscription callback is slow.
        stream_group = MutuallyExclusiveCallbackGroup()
        hz = cfg.get("setpoint_stream_hz", 10)
        self.create_timer(1.0 / hz, self._stream_tick, callback_group=stream_group)

        # Subscriptions use the default callback group (also MutuallyExclusive).
        # BT publishes current_role to activate/deactivate; all other state
        # (assignment, pending commands) arrives here too.
        self.create_subscription(
            String, f"{prefix}/relay_assignment", self._on_assignment, 10)
        self.create_subscription(
            String, f"{prefix}/current_role", self._on_role, CURRENT_ROLE_QOS)
        self.create_subscription(
            String, f"{prefix}/pending_command", self._on_pending_command, 10)
        self.create_subscription(
            String, f"{prefix}/drone_state", self._on_drone_state, 10)

        self.get_logger().info(
            f"relay_mover started: drone={DRONE_ID} stream_hz={hz} "
            f"(OFFBOARD keepalive owner; stream independent of BT tick rate)"
        )

    # ── Subscriptions ─────────────────────────────────────────────────────────

    def _on_assignment(self, msg: String):
        try:
            self._core.on_assignment(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on relay_assignment: {e}")

    def _on_role(self, msg: String):
        self._core.on_role(msg.data.strip())

    def _on_pending_command(self, msg: String):
        try:
            self._core.on_pending_command(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on pending_command: {e}")

    def _on_drone_state(self, msg: String):
        try:
            state = json.loads(msg.data)
            self._core.on_flight_mode(state.get("flight_mode", "UNKNOWN"))
        except Exception as e:
            self.get_logger().warning(f"Bad JSON on drone_state: {e}")

    # ── Stream tick ───────────────────────────────────────────────────────────

    def _stream_tick(self):
        self._core.tick()


def main():
    rclpy.init()
    px4 = PX4Agent()
    node = RelayMover(px4)
    # MultiThreadedExecutor so the stream-timer callback group gets its own
    # thread and is never blocked by subscription callbacks.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    px4._log = node.get_logger()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
