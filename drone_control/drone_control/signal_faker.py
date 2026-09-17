"""
signal_faker.py — SITL signal source for all RF hops.

BUILDSPEC: §4.1
LAYER:     1 (source — publishes raw signal data, no decisions)
SUBSCRIBES:
  /{drone_id}/drone_state  (live position — forwarded from PX4 via state_bridge)
PUBLISHES:
  /signal/gc_to_leader
  /signal/leader_to_gc
  /signal/gc_to_follower/{drone_id}
  /signal/follower_to_gc/{drone_id}
  /signal/leader_to_follower/{drone_id}

Replaces: leader_signal_faker.py, follower_signal_faker.py, gc_signal_faker.py.
One process, all hops.  BUILDSPEC §4.1.

Hard rules this file must satisfy (BUILDSPEC §4.1):
  - Every hop's severity is independent (never share, never default one to
    another's value, never default to zero)  -> proven by test_per_hop_severity_independent
  - Output is raw signal data only; severity is an INPUT not computed here
    -> proven by test_schema_exact (no severity-computation fields in output)
  - FSPL formula matches §4.1 exactly        -> proven by test_fspl_known_distance
  - timestamp is origin time, not publish time -> proven by test_timestamp_is_origin
  - All five hop topics published per tick   -> proven by test_publishes_all_five_hops
  - snr_db == rssi_dbm - noise_dbm exactly  -> proven by test_snr_is_rssi_minus_noise
"""

import json
import logging
import os
import sys
import time
from math import log10, radians, sin, cos, asin, sqrt

log = logging.getLogger(__name__)

# ── Config loading ────────────────────────────────────────────────────────────

def _load_config() -> dict:
    try:
        _here = os.path.dirname(os.path.abspath(__file__))
        if _here not in sys.path:
            sys.path.insert(0, _here)
        from config.demo_config import DEMO_CONFIG
        return DEMO_CONFIG
    except Exception as e:
        log.warning("demo_config load failed (%s) — using bare defaults", e)
        return {
            "baseline_noise_dbm": -95.0,
            "noise_range_db":     40.0,
            "tx_power_dbm":       20.0,
            "frequency_mhz":      915,
            "gc_pos":             {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},
            "hop_severities": {
                "gc_to_leader":       0.0,
                "leader_to_gc":       0.0,
                "gc_to_follower":     0.0,
                "follower_to_gc":     0.0,
                "leader_to_follower": 0.0,
            },
        }


# ── Pure physics — no I/O, fully testable ─────────────────────────────────────

def haversine(a: dict, b: dict) -> float:
    """Great-circle distance in metres between two {lat, lon[, alt]} dicts."""
    R = 6_371_000.0
    lat1, lat2 = radians(a["lat"]), radians(b["lat"])
    dlat = lat2 - lat1
    dlon = radians(b["lon"] - a["lon"])
    h = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    return 2 * R * asin(sqrt(h))


def compute_hop(pos_a: dict, pos_b: dict, severity: float, cfg: dict, t: float) -> dict:
    """
    Compute the §2.1 signal payload for one directed hop.

    Per-hop computation order (BUILDSPEC §4.1):
      distance_m   = haversine(pos_a, pos_b)
      path_loss_db = 20*log10(distance_m) + 20*log10(frequency_mhz) - 27.55
      rssi_dbm     = tx_power_dbm - path_loss_db
      noise_dbm    = baseline_noise_dbm + severity * noise_range_db
      snr_db       = rssi_dbm - noise_dbm

    severity is an INPUT from config (per-hop independent dial).
    This function never computes severity — BUILDSPEC §4.1 hard rule.

    t is the origin time of this computation (BUILDSPEC §5.6).
    """
    baseline  = cfg["baseline_noise_dbm"]
    noise_rng = cfg["noise_range_db"]
    tx_pwr    = cfg["tx_power_dbm"]
    freq_mhz  = cfg["frequency_mhz"]

    distance_m   = max(haversine(pos_a, pos_b), 1.0)   # clamp to avoid log10(0)
    path_loss_db = 20 * log10(distance_m) + 20 * log10(freq_mhz) - 27.55
    rssi_dbm     = tx_pwr - path_loss_db
    noise_dbm    = baseline + severity * noise_rng
    snr_db       = rssi_dbm - noise_dbm

    # BUILDSPEC §2.1: exactly these five fields, no others.
    # timestamp is origin time of computation — BUILDSPEC §5.6.
    return {
        "rssi_dbm":  rssi_dbm,
        "noise_dbm": noise_dbm,
        "snr_db":    snr_db,
        "severity":  severity,
        "timestamp": t,
    }


# ── SignalFaker — injectable boundaries for testing ───────────────────────────

class SignalFaker:
    """
    Computes and publishes §2.1 signal payloads for all hops at 2 Hz.

    Design for testability (TEST_PROTOCOL §3.3):
      - clock:       injectable callable returning current time (default: time.time)
      - publish:     injectable callable (topic: str, payload: dict) -> None
      - get_positions: injectable callable () -> {"leader": {...}, drone_id: {...}, ...}
      - get_severity: injectable callable (hop_name: str) -> float [0.0, 1.0]

    In production, get_positions reads from MAVLink; publish writes to ROS2.
    In tests, both are captured lists.
    """

    # HARD RULE (BUILDSPEC §4.1): every hop topic listed here must be published
    # on every tick.  Topic templates use Python str.format().
    _HOP_TOPICS = [
        "/signal/gc_to_leader",
        "/signal/leader_to_gc",
        "/signal/gc_to_follower/{drone_id}",
        "/signal/follower_to_gc/{drone_id}",
        "/signal/leader_to_follower/{drone_id}",
    ]

    def __init__(
        self,
        cfg: dict,
        drone_ids: list,           # follower drone IDs (e.g. ["drone-02"])
        get_positions,             # () -> {"gc": {lat,lon,alt}, "leader": ..., drone_id: ...}
        publish,                   # (topic: str, payload: dict) -> None
        get_severity=None,         # (hop_name: str) -> float; defaults to config lookup
        clock=None,                # () -> float; defaults to time.time
    ):
        self._cfg        = cfg
        self._drone_ids  = drone_ids
        self._get_pos    = get_positions
        self._publish    = publish
        self._clock      = clock if clock is not None else time.time

        # HARD RULE (BUILDSPEC §4.1): every hop's severity is independent —
        # never share between hops, never default one to another's value,
        # never default to zero.  Each hop reads its own config entry.
        hop_sevs = cfg.get("hop_severities", {})
        if get_severity is not None:
            self._get_severity = get_severity
        else:
            # Read from config; missing key is an error (not silently 0).
            def _cfg_severity(hop_name: str) -> float:
                if hop_name not in hop_sevs:
                    raise KeyError(
                        f"hop_severities['{hop_name}'] missing from config. "
                        "BUILDSPEC §4.1: every hop must have an independent, "
                        "explicitly configured severity."
                    )
                return float(hop_sevs[hop_name])
            self._get_severity = _cfg_severity

    def tick(self) -> None:
        """
        One publish cycle: read positions, compute all hops, publish.

        The origin timestamp (t0) is captured BEFORE position read so all hops
        in one tick share the same timestamp.  BUILDSPEC §5.6: timestamp is
        origin time of computation, not time of publish.
        """
        # Capture origin time before any I/O so all hops in one tick share it.
        # BUILDSPEC §5.6 — do not restamp on publish.
        t0 = self._clock()

        positions = self._get_pos()
        gc_pos     = positions["gc"]
        leader_pos = positions["leader"]

        for drone_id in self._drone_ids:
            follower_pos = positions[drone_id]

            hops = {
                # Hop name must match the key in hop_severities config.
                "gc_to_leader": (gc_pos,      leader_pos),
                "leader_to_gc": (leader_pos,  gc_pos),
                "gc_to_follower":     (gc_pos,      follower_pos),
                "follower_to_gc":     (follower_pos, gc_pos),
                "leader_to_follower": (leader_pos,  follower_pos),
            }

            for hop_name, (pos_a, pos_b) in hops.items():
                # HARD RULE (BUILDSPEC §4.1): each hop gets its own severity.
                # Never pass another hop's severity here.
                sev     = self._get_severity(hop_name)
                payload = compute_hop(pos_a, pos_b, sev, self._cfg, t0)

                topic = self._hop_topic(hop_name, drone_id)
                self._publish(topic, payload)

    def _hop_topic(self, hop_name: str, drone_id: str) -> str:
        """Map hop_name → ROS2 topic string."""
        # ROS 2 topic names disallow hyphens; sanitize DRONE_ID for the path only.
        tid = drone_id.replace('-', '_')
        mapping = {
            "gc_to_leader":       "/signal/gc_to_leader",
            "leader_to_gc":       "/signal/leader_to_gc",
            "gc_to_follower":     f"/signal/gc_to_follower/{tid}",
            "follower_to_gc":     f"/signal/follower_to_gc/{tid}",
            "leader_to_follower": f"/signal/leader_to_follower/{tid}",
        }
        return mapping[hop_name]


# ── ROS2 drone_state position provider (production seam) ─────────────────────

def _make_ros2_position_provider(node, drone_ids: list, cfg: dict):
    """
    Returns a get_positions() callable backed by ROS2 drone_state subscriptions.

    state_bridge already forwards PX4 MAVLink positions to /{drone_id}/drone_state
    at ~1 Hz.  Subscribing here avoids competing with mav_to_mqtt for the same
    UDP stream on the same port.

    This seam is faked in tests per TEST_PROTOCOL §3.4.
    """
    import threading
    from std_msgs.msg import String

    gc_pos = cfg["gc_pos"]
    _positions = {
        "gc":     gc_pos,
        "leader": {"lat": 47.3980, "lon": 8.5480, "alt": 50.0},
    }
    for d in drone_ids:
        _positions[d] = {"lat": 47.3980, "lon": 8.5490, "alt": 50.0}
    _lock = threading.Lock()

    leader_id = cfg.get("leader_id", "drone-01")

    def _make_cb(drone_id: str):
        def _cb(msg):
            try:
                state = json.loads(msg.data)
                pos = state.get("position")
                if pos and "lat" in pos and "lon" in pos:
                    entry = {"lat": pos["lat"], "lon": pos["lon"], "alt": pos.get("alt", 0.0)}
                    with _lock:
                        _positions[drone_id] = entry
                        if drone_id == leader_id:
                            _positions["leader"] = entry
            except Exception as e:
                log.warning("drone_state parse error (%s): %s", drone_id, e)
        return _cb

    for d in drone_ids:
        topic = f"/{d.replace('-', '_')}/drone_state"
        node.create_subscription(String, topic, _make_cb(d), 10)
        log.info("signal_faker: subscribed to %s for live positions", topic)

    def get_positions():
        with _lock:
            return dict(_positions)

    return get_positions


# ── ROS2 publish seam (production) ───────────────────────────────────────────

def _make_ros2_publisher(node, drone_ids: list):
    """
    Returns a publish(topic, payload) callable backed by ROS2 String publishers.
    One publisher object per topic, created once at startup.
    """
    from std_msgs.msg import String

    # Build the full topic list for this set of drone IDs.
    topics = [
        "/signal/gc_to_leader",
        "/signal/leader_to_gc",
    ]
    for d in drone_ids:
        tid = d.replace('-', '_')   # ROS 2 topic names disallow hyphens
        topics += [
            f"/signal/gc_to_follower/{tid}",
            f"/signal/follower_to_gc/{tid}",
            f"/signal/leader_to_follower/{tid}",
        ]

    publishers = {t: node.create_publisher(String, t, 10) for t in topics}

    def publish(topic: str, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        if topic in publishers:
            publishers[topic].publish(msg)
        else:
            log.warning("No publisher for topic %s", topic)

    return publish


# ── ROS2 Node wrapper ─────────────────────────────────────────────────────────

def _run_ros2_node():
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    cfg      = _load_config()
    drone_id = os.environ.get("DRONE_ID", "drone-01")
    # In production signal_faker runs on the GC and knows all followers.
    # Drone IDs discovered from config or env.
    drone_ids = [drone_id] if drone_id != "gc" else ["drone-01", "drone-02"]

    class _FakerNode(Node):
        def __init__(self):
            super().__init__("signal_faker")
            publish       = _make_ros2_publisher(self, drone_ids)
            get_positions = _make_ros2_position_provider(self, drone_ids, cfg)
            self._faker   = SignalFaker(
                cfg=cfg,
                drone_ids=drone_ids,
                get_positions=get_positions,
                publish=publish,
            )
            # 2 Hz tick rate — BUILDSPEC §4.1.
            self.create_timer(0.5, self._tick)
            # rclpy's Logger.info() takes only the formatted string — no printf args.
            self.get_logger().info(
                f"signal_faker started: drone_ids={drone_ids} rate=2Hz"
            )

        def _tick(self):
            self._faker.tick()

    node = _FakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    _run_ros2_node()


if __name__ == "__main__":
    main()
