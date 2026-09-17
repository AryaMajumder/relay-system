"""
gc_radio_health_publisher.py — G_radio node (gc_build_component_v2.md §5).

Always-on publisher of GC-side radio health to /gc/radio_health (DDS).
Reads the GC's own observation of the GC↔leader link from the leader's
signal_report and derives:

  gc_severity      — GC's own noise-floor elevation on this hop,
                     inverted from leader_to_gc.noise_dbm using the shared
                     SITL noise model (noise_baseline_dbm / noise_range_db
                     from demo_config.py — same constants signal_faker.py
                     and follower_signal_faker.py use to inject noise).
                     Own physics only — does NOT cross-guess the leader's
                     noise floor and does NOT compute gc_cap_m.

  gc_radio_range_m — GC's own radio nominal range (own hardware constant,
                     published not config-assumed — build-spec §5 Rule 1a).

Does NOT publish:
  - cap_m           (BandSensorNode computes that from severity + range)
  - leader_severity   (leader publishes its own via L_radio)

Transport: ROS2 / DDS — /gc/radio_health.

Input:
  /{LEADER_ID}/signal_report — leader_to_gc.noise_dbm (GC is the receiver
                                on that direction, so this is the GC's own
                                noise-floor reading).
"""

import json   # parse signal_report JSON string into a dict
import os     # read LEADER_ID env var
import sys    # manipulate Python path so demo_config can be imported by path
import time   # stamp each outbound payload with wall-clock time

import rclpy                        # ROS2 Python client library — init, spin, shutdown
from rclpy.node import Node         # base class for all ROS2 nodes
from std_msgs.msg import String     # ROS2 message type used for all JSON topics in this system

# ── Identity ──────────────────────────────────────────────────────────────────

# Which leader drone this GC is observing.  The GC subscribes to the leader's
# own signal_report to get its view of the GC↔leader link.
LEADER_ID = os.environ.get("LEADER_ID", "drone-01")


# ── Config loader ─────────────────────────────────────────────────────────────

def _load_config() -> dict:
    # demo_config.py holds scenario parameters (rates, ranges, jamming setup,
    # and the shared noise model constants).
    # Hard-fail if it can't be loaded — this node has no safe defaults for
    # gc_radio_range_m or the noise model constants; running with wrong values
    # would silently corrupt the band geometry every downstream consumer relies on.
    try:
        sys.path.insert(0, "/root/ros2_ws/src/drone_control/drone_control")
        from config.demo_config import DEMO_CONFIG
        return DEMO_CONFIG
    except Exception as e:
        raise RuntimeError(f"Cannot load demo_config: {e}")


# Load config once at module level — same pattern as signal_faker.py and
# follower_signal_faker.py — so constants are available to module-level
# helpers before the ROS2 node is instantiated.
_config = _load_config()

# ── Shared SITL noise model constants ────────────────────────────────────────
#
# Single source of truth: demo_config.py keys noise_baseline_dbm / noise_range_db.
# signal_faker.py and follower_signal_faker.py import the same values, ensuring
# the forward model (noise injection) and inverse model (severity recovery here)
# always stay in sync — a retune in demo_config propagates to all three files
# without any file-by-file edits.
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

class GcRadioHealthPublisher(Node):

    def __init__(self):
        # Register this process as a ROS2 node named "gc_radio_health_publisher".
        # Node name is fixed (not parameterised by LEADER_ID) — only one GC
        # exists in this topology.
        super().__init__("gc_radio_health_publisher")

        # gc_radio_range_m — GC's own hardware nominal range.
        # Read from the already-loaded module-level _config (not re-loaded here).
        # Falls back through gc_radio_range_m → radio_range_m → 800 m.
        # Stored as float so it serialises cleanly to JSON.
        self._gc_radio_range_m = float(
            _config.get("gc_radio_range_m",
                        _config.get("radio_range_m", 800))
        )

        # Holds the most recent signal_report from the leader.
        # None until the first message arrives — publish tick skips while None.
        self._last_signal_report = None

        leader_prefix = _ros_prefix(LEADER_ID)   # e.g. "/drone_01"

        # ── Publisher ─────────────────────────────────────────────────────────
        # Publishes {gc_severity, gc_radio_range_m} to /gc/radio_health.
        # BandSensorNode on the follower subscribes here to get the GC's
        # severity and range, then computes gc_cap_m itself (Step A).
        self._pub = self.create_publisher(String, "/gc/radio_health", 10)

        # ── Subscription ──────────────────────────────────────────────────────
        # The GC observes the GC↔leader link from its own receive side.
        # In SITL, that observation lives in the leader's signal_report under
        # the "leader_to_gc" key — because signal_faker.py computes that link
        # from the leader's perspective, and the GC is the receiver on it.
        # noise_dbm in "leader_to_gc" is therefore the GC's own noise floor.
        self.create_subscription(
            String, f"{leader_prefix}/signal_report",
            self._on_signal_report, 10)

        # ── Publish timer ─────────────────────────────────────────────────────
        # Fires at gc_radio_health_hz (default 1 Hz) to push a fresh payload.
        # Rate is independent of the signal_report arrival cadence — the last
        # known value is republished each tick even if no new signal_report
        # has arrived (staleness is handled by BandSensorNode's freshness guard,
        # not here).
        hz = _config.get("gc_radio_health_hz", 1)
        self.create_timer(1.0 / hz, self._publish_tick)

        self.get_logger().info(
            "gc_radio_health_publisher started: leader=%s "
            "range=%.0fm pub=/gc/radio_health rate=%dHz",
            LEADER_ID, self._gc_radio_range_m, hz,
        )

    # ── Subscription callback ─────────────────────────────────────────────────

    def _on_signal_report(self, msg: String):
        # Fires whenever the leader publishes a new signal_report.
        # Decodes the JSON string and caches the full dict.
        # Only noise_dbm is used in _publish_tick, but caching the full dict
        # avoids re-parsing on every tick and keeps this callback minimal.
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

        # Extract the GC's own noise floor from the leader_to_gc link dict.
        # "leader_to_gc" means leader transmits, GC receives — so noise_dbm
        # here describes the noise the GC's own receiver is seeing.
        # The (.get("leader_to_gc") or {}) guard handles a missing key without
        # raising; the outer .get("noise_dbm") returns None if absent.
        noise_dbm = (sr.get("leader_to_gc") or {}).get("noise_dbm")

        # Invert the noise floor back to a severity value via _severity_from_noise.
        # If noise_dbm is missing from the payload, default to 0.0 (no jamming)
        # rather than crashing — a degraded observation is better than no publish.
        gc_sev = round(_severity_from_noise(noise_dbm), 4) if noise_dbm is not None else 0.0

        # Build the outbound payload.
        # gc_severity and gc_radio_range_m are the two values BandSensorNode
        # needs from the GC.  Nothing else belongs here — cap_m is computed
        # on the follower (Step A), and loss is removed as a metric (v6.3).
        payload = {
            "source":           "gc_radio",             # identifies publisher for logging/debugging
            "timestamp":        time.time(),            # wall-clock at publish time; BandSensorNode uses this for its per-peer freshness guard
            "gc_severity":      gc_sev,                 # GC's own noise-floor-derived jamming severity [0, 1]; inverted from leader_to_gc.noise_dbm
            "gc_radio_range_m": self._gc_radio_range_m, # GC's nominal radio range (m); BandSensorNode applies effective_radio_range() to this alongside gc_severity
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

        # Debug log — only emitted at DEBUG level so it doesn't spam at INFO.
        self.get_logger().debug(
            "gc_radio_health: gc_sev=%.3f range=%.0fm noise=%.1fdBm",
            gc_sev, self._gc_radio_range_m,
            noise_dbm if noise_dbm is not None else float("nan"),
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()                          # initialise the ROS2 runtime
    node = GcRadioHealthPublisher()       # construct the node (sets up pub, sub, timer)
    try:
        rclpy.spin(node)                  # hand control to the ROS2 executor; blocks until shutdown
    except KeyboardInterrupt:
        pass                              # clean Ctrl-C exit
    finally:
        node.destroy_node()               # release ROS2 resources
        rclpy.shutdown()                  # shut down the ROS2 runtime


if __name__ == "__main__":
    main()
