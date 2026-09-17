"""
leader_radio_health_publisher.py — L_radio node (gc_build_component_v2.md §5).

Always-on publisher of leader-side radio health to /{DRONE_ID}/radio_health
(DDS-peer). Reads the leader's own observation of the GC↔leader link from its
own signal_report and derives:

  leader_severity      — leader's own noise-floor elevation on this hop,
                         inverted from gc_to_leader.noise_dbm using the shared
                         SITL noise model (noise_baseline_dbm / noise_range_db
                         from demo_config.py — same constants signal_faker.py
                         and gc_radio_health_publisher.py use).
                         Own physics only — does NOT cross-guess the GC's
                         noise floor and does NOT compute leader_cap_m.

  leader_radio_range_m — leader's own radio nominal range (own hardware
                         constant, published not config-assumed —
                         build-spec §5 Rule 1a).

Does NOT publish:
  - cap_m              (BandSensorNode computes that from severity + range)
  - gc_severity          (GC publishes its own via G_radio)

Transport: ROS2 / DDS-peer — /{DRONE_ID}/radio_health.
Previous MQTT transport removed: radio_health is relay-coordination traffic
(drone↔drone), not cloud C2 — DDS is correct per build-spec §0.

Input:
  /{DRONE_ID}/signal_report — gc_to_leader.noise_dbm (leader is the receiver
                               on that direction, so this is the leader's own
                               noise-floor reading).
"""

import json   # parse signal_report JSON string into a dict
import os     # read DRONE_ID env var
import sys    # manipulate Python path so demo_config can be imported by path
import time   # stamp each outbound payload with wall-clock time

import rclpy                        # ROS2 Python client library — init, spin, shutdown
from rclpy.node import Node         # base class for all ROS2 nodes
from std_msgs.msg import String     # ROS2 message type used for all JSON topics in this system

# ── Identity ──────────────────────────────────────────────────────────────────

# Which drone this process represents — controls topic names.
DRONE_ID = os.environ.get("DRONE_ID", "drone-01")


# ── Config loader ─────────────────────────────────────────────────────────────

def _load_config() -> dict:
    # demo_config.py holds scenario parameters including the shared noise model
    # constants (noise_baseline_dbm / noise_range_db).
    # Falls back to safe defaults rather than hard-failing — the leader can still
    # fly and publish a conservative severity=0.0 if config is unavailable,
    # unlike gc_radio_health_publisher which hard-fails on missing range.
    try:
        sys.path.insert(0, "/root/ros2_ws/src/drone_control/drone_control")
        from config.demo_config import DEMO_CONFIG
        return DEMO_CONFIG
    except Exception as e:
        import logging
        logging.getLogger(__name__).warning(
            "Could not load demo_config (%s), using defaults", e)
        return {"radio_range_m": 800}


# Load config once at module level — same pattern as signal_faker.py,
# follower_signal_faker.py, and gc_radio_health_publisher.py.
_config = _load_config()

# ── Shared SITL noise model constants ────────────────────────────────────────
#
# Single source of truth: demo_config.py keys noise_baseline_dbm / noise_range_db.
# signal_faker.py, follower_signal_faker.py, and gc_radio_health_publisher.py
# all derive the same constants from the same config keys — a retune in
# demo_config.py propagates to all four files without file-by-file edits.
#
# Forward model (signal_faker injects):
#   noise_dbm = _BASELINE_NOISE_DBM + severity * _NOISE_RANGE_DB
#
# Inverse model (this file recovers severity from a measured noise reading):
#   severity  = (noise_dbm - _BASELINE_NOISE_DBM) / _NOISE_RANGE_DB
#
_BASELINE_NOISE_DBM = float(_config.get("noise_baseline_dbm", -95.0))
_NOISE_RANGE_DB     = float(_config.get("noise_range_db", 40.0))


# ── ROS2 topic helper ─────────────────────────────────────────────────────────

def _ros_prefix(drone_id: str) -> str:
    # ROS2 topic names use underscores; drone IDs use hyphens.
    # e.g. "drone-01" → "/drone_01"
    return f"/{drone_id.replace('-', '_')}"


# ── Physics: noise floor → severity ──────────────────────────────────────────

def _severity_from_noise(noise_dbm: float) -> float:
    # Inverts the SITL noise model to recover jamming severity [0, 1]
    # from a measured noise floor reading.
    #
    # Clamped to [0, 1]:
    #   Below _BASELINE_NOISE_DBM (-95 dBm) → clamps to 0.0 (no jamming).
    #   Above _BASELINE_NOISE_DBM + _NOISE_RANGE_DB (-55 dBm) → clamps to 1.0
    #   (full-severity saturation — further noise rise can't be distinguished).
    return max(0.0, min(1.0, (noise_dbm - _BASELINE_NOISE_DBM) / _NOISE_RANGE_DB))


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class LeaderRadioHealthPublisher(Node):

    def __init__(self):
        # Node name includes DRONE_ID so multiple instances on the same host
        # don't collide (e.g. in a multi-drone SITL run).
        node_name = f"leader_radio_health_publisher_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        # leader_radio_range_m — leader's own hardware nominal range.
        # Read from the already-loaded module-level _config (not re-loaded here).
        # Falls back through leader_radio_range_m → radio_range_m → 800 m.
        self._leader_radio_range_m = float(
            _config.get("leader_radio_range_m",
                        _config.get("radio_range_m", 800))
        )

        # Holds the most recent signal_report from this drone's own pipeline.
        # None until the first message arrives — publish tick skips while None.
        self._last_signal_report = None

        prefix    = _ros_prefix(DRONE_ID)      # e.g. "/drone_01"
        pub_topic = f"{prefix}/radio_health"   # e.g. "/drone_01/radio_health"

        # ── Publisher ─────────────────────────────────────────────────────────
        # Publishes {leader_severity, leader_radio_range_m} to
        # /{DRONE_ID}/radio_health (DDS-peer).
        # BandSensorNode on the follower subscribes here to get the leader's
        # severity and range, then computes leader_cap_m itself (Step A).
        self._pub = self.create_publisher(String, pub_topic, 10)

        # ── Subscription ──────────────────────────────────────────────────────
        # The leader observes the GC↔leader link from its own receive side.
        # "gc_to_leader" means GC transmits, leader receives — so noise_dbm
        # under that key is the leader's own noise floor reading.
        # signal_faker.py publishes gc_to_leader alongside leader_to_gc on
        # the same raw_signal_inject payload; signal_reader passes both through
        # to the ROS2 signal_report topic.
        self.create_subscription(
            String, f"{prefix}/signal_report",
            self._on_signal_report, 10)

        # ── Publish timer ─────────────────────────────────────────────────────
        # Fires at leader_radio_health_hz (default 1 Hz).
        # The last known severity is republished each tick even if no new
        # signal_report has arrived — staleness is handled by BandSensorNode's
        # per-peer freshness guard (radio_health_max_age_s), not here.
        hz = _config.get("leader_radio_health_hz",
                         _config.get("radio_health_reader_hz", 1))
        self.create_timer(1.0 / hz, self._publish_tick)

        self.get_logger().info(
            "leader_radio_health_publisher started: drone=%s "
            "range=%.0fm pub=%s rate=%dHz",
            DRONE_ID, self._leader_radio_range_m, pub_topic, hz,
        )

    # ── Subscription callback ─────────────────────────────────────────────────

    def _on_signal_report(self, msg: String):
        # Fires whenever signal_reader publishes a new signal_report for this drone.
        # Decodes the JSON string and caches the full dict.
        # Only gc_to_leader.noise_dbm is used in _publish_tick, but caching
        # the full dict avoids re-parsing on every tick.
        try:
            self._last_signal_report = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warning("Bad JSON on signal_report: %s", e)

    # ── Publish tick ──────────────────────────────────────────────────────────

    def _publish_tick(self):
        sr = self._last_signal_report
        if sr is None:
            # No signal_report has arrived yet — nothing to base severity on.
            # Hold; do not publish a zero-severity placeholder that would make
            # BandSensorNode think the link is clean when it's actually unknown.
            return

        # Extract the leader's own noise floor from the gc_to_leader link dict.
        # "gc_to_leader" means GC transmits, leader receives — so noise_dbm
        # here describes the noise the leader's own receiver is seeing.
        # The (.get("gc_to_leader") or {}) guard handles a missing key without
        # raising; the outer .get("noise_dbm") returns None if absent.
        noise_dbm = (sr.get("gc_to_leader") or {}).get("noise_dbm")

        # Invert the noise floor back to a severity value via _severity_from_noise.
        # If noise_dbm is missing from the payload, default to 0.0 (no jamming)
        # rather than crashing — a degraded observation is better than no publish.
        leader_sev = round(_severity_from_noise(noise_dbm), 4) if noise_dbm is not None else 0.0

        # Build the outbound payload.
        # leader_severity and leader_radio_range_m are the two values
        # BandSensorNode needs from the leader. Nothing else belongs here —
        # cap_m is computed on the follower (Step A), and loss is removed
        # as a metric (v6.3).
        payload = {
            "source":               "leader_radio",          # identifies publisher for logging/debugging
            "timestamp":            time.time(),             # wall-clock at publish time; BandSensorNode uses this for its per-peer freshness guard
            "leader_severity":      leader_sev,              # leader's own noise-floor-derived jamming severity [0, 1]; inverted from gc_to_leader.noise_dbm
            "leader_radio_range_m": self._leader_radio_range_m,  # leader's nominal radio range (m); BandSensorNode applies effective_radio_range() to this alongside leader_severity
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

        self.get_logger().debug(
            "leader_radio_health: leader_sev=%.3f range=%.0fm noise=%.1fdBm",
            leader_sev, self._leader_radio_range_m,
            noise_dbm if noise_dbm is not None else float("nan"),
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()                             # initialise the ROS2 runtime
    node = LeaderRadioHealthPublisher()      # construct the node (sets up pub, sub, timer)
    try:
        rclpy.spin(node)                     # hand control to the ROS2 executor; blocks until shutdown
    except KeyboardInterrupt:
        pass                                 # clean Ctrl-C exit
    finally:
        node.destroy_node()                  # release ROS2 resources
        rclpy.shutdown()                     # shut down the ROS2 runtime


if __name__ == "__main__":
    main()
