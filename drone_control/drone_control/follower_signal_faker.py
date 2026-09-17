"""
follower_signal_faker.py — SITL only, follower-side hardware stand-in.

Stand-in for the follower's physical radio hardware. In production the follower's
radios measure follower_to_gc and leader_to_follower directly. In SITL this node
computes those two links from GPS positions using FSPL and publishes them to MQTT
where signal_reader picks them up.

Does NOT compute leader_to_gc — that link is between the leader and GC; the
follower has no observation of it. signal_faker on drone-01 is the sole source.

ROS2 + MQTT hybrid: subscribes to drone_state topics on ROS2 domain 42 for
GPS positions (no cross-broker MQTT needed), publishes to MQTT port 1885.

Signal model:
  RSSI  = 20 - FSPL(dist_m, 915 MHz)
  noise = -95 + severity * 40    (jamming raises noise on affected link)
  SNR   = RSSI - noise
"""

import json
import logging
import os
import sys
import threading
import time
from math import log10, radians, sin, cos, asin, sqrt

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_LOG_LEVEL = os.environ.get("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, _LOG_LEVEL, logging.INFO),
    format="%(asctime)s %(levelname)s [follower_signal_faker] %(message)s",
)
log = logging.getLogger(__name__)

DRONE_ID  = os.environ.get("DRONE_ID",  "drone-02")
LEADER_ID = os.environ.get("LEADER_ID", "drone-01")

_PORT_MAP   = {"drone-01": 1884, "drone-02": 1885}
_MQTT_PORT  = _PORT_MAP.get(DRONE_ID, 1885)
_MQTT_HOST  = "127.0.0.1"
_MQTT_USER  = "drone"
_PUBLISH_HZ = 2.0

_DEFAULT_GC_POS       = {"lat": 47.3900, "lon": 8.5400, "alt":  0.0}
_DEFAULT_FOLLOWER_POS = {"lat": 47.3977, "lon": 8.5456, "alt": 50.0}
_DEFAULT_LEADER_POS   = {"lat": 47.4100, "lon": 8.5600, "alt": 50.0}


def _ros_prefix(drone_id: str) -> str:
    return f"/{drone_id.replace('-', '_')}"


def _read_password() -> str:
    for path in (
        "/etc/drone-pub/mqtt_pass.txt",
        "/etc/mqtt-creds/mosquitto_drone_credentials.txt",
    ):
        try:
            return open(path).read().strip()
        except OSError:
            pass
    return os.environ.get("MQTT_PASS", "")


def _haversine(a: dict, b: dict) -> float:
    R = 6_371_000.0
    lat1, lat2 = radians(a["lat"]), radians(b["lat"])
    dlat = lat2 - lat1
    dlon = radians(b["lon"] - a["lon"])
    h = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    return 2 * R * asin(sqrt(h))


def _fspl_rssi(dist_m: float) -> float:
    dist_m = max(dist_m, 1.0)
    return 20.0 - (20.0 * log10(dist_m) + 20.0 * log10(915.0) - 27.55)


def _noise_dbm(jammed: bool, severity: float) -> float:
    # Constants sourced from demo_config.py (noise_baseline_dbm / noise_range_db).
    return (_BASELINE_NOISE_DBM + severity * _NOISE_RANGE_DB) if jammed else _BASELINE_NOISE_DBM


def _compute_link(pos_a: dict, pos_b: dict, jammed: bool, severity: float) -> dict:
    dist  = _haversine(pos_a, pos_b)
    rssi  = _fspl_rssi(dist)
    noise = _noise_dbm(jammed, severity)
    return {
        "dist_m":    round(dist, 2),
        "rssi_dbm":  round(rssi, 2),
        "noise_dbm": round(noise, 2),
        "snr_db":    round(rssi - noise, 2),
    }


def _load_config() -> dict:
    try:
        sys.path.insert(0, "/root/ros2_ws/src/drone_control/drone_control")
        from config.demo_config import DEMO_CONFIG
        return DEMO_CONFIG
    except Exception as e:
        log.warning("Could not load demo_config (%s), using defaults", e)
        return {
            "jamming_mode":         "gradual",
            "jamming_ramp_s":       30,
            "jamming_target_link":  "leader_to_gc",
            "jamming_max_severity": 1.0,
            "gc_pos":               _DEFAULT_GC_POS,
        }


_config     = _load_config()
_jam_mode   = _config.get("jamming_mode", "gradual")
_jam_ramp_s = float(_config.get("jamming_ramp_s", 30))
_jam_target = _config.get("jamming_target_link", "leader_to_gc")
_jam_max    = float(_config.get("jamming_max_severity", 1.0))
_gc_pos     = _config.get("gc_pos", _DEFAULT_GC_POS)
_start_time = time.time()

# Shared noise model constants — single source of truth in demo_config.py.
# Must match signal_faker.py and gc_radio_health_publisher.py exactly.
_BASELINE_NOISE_DBM = float(_config.get("noise_baseline_dbm", -95.0))
_NOISE_RANGE_DB     = float(_config.get("noise_range_db", 40.0))


def _current_severity() -> float:
    if _jam_mode == "clean":
        return 0.0
    if _jam_mode == "sudden":
        return _jam_max
    elapsed = time.time() - _start_time
    frac = min(elapsed / _jam_ramp_s, 1.0) if _jam_ramp_s > 0 else 1.0
    return round(_jam_max * frac, 4)


class FollowerSignalFaker(Node):

    def __init__(self, mqtt_client):
        node_name = f"follower_signal_faker_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        self._mqtt         = mqtt_client
        self._lock         = threading.Lock()
        self._follower_pos = dict(_DEFAULT_FOLLOWER_POS)
        self._leader_pos   = dict(_DEFAULT_LEADER_POS)
        self._tick         = 0

        follower_prefix = _ros_prefix(DRONE_ID)
        leader_prefix   = _ros_prefix(LEADER_ID)

        self.create_subscription(
            String, f"{follower_prefix}/drone_state", self._on_follower_state, 10)
        self.create_subscription(
            String, f"{leader_prefix}/drone_state",   self._on_leader_state,   10)

        self.create_timer(1.0 / _PUBLISH_HZ, self._publish_tick)

        self.get_logger().info(
            "follower_signal_faker started: drone=%s leader=%s "
            "jam_mode=%s target=%s max=%.2f ramp=%.1fs",
            DRONE_ID, LEADER_ID, _jam_mode, _jam_target, _jam_max, _jam_ramp_s,
        )

    def _on_follower_state(self, msg: String):
        try:
            pos = json.loads(msg.data).get("position")
            if pos:
                with self._lock:
                    self._follower_pos = pos
        except Exception:
            pass

    def _on_leader_state(self, msg: String):
        try:
            pos = json.loads(msg.data).get("position")
            if pos:
                with self._lock:
                    self._leader_pos = pos
        except Exception:
            pass

    def _publish_tick(self):
        with self._lock:
            flw = dict(self._follower_pos)
            ldr = dict(self._leader_pos)

        sev    = _current_severity()
        active = sev > 0.0

        ftg = _compute_link(flw, _gc_pos,
                            _jam_target == "follower_to_gc"     and active, sev)
        ltf = _compute_link(ldr, flw,
                            _jam_target == "leader_to_follower" and active, sev)

        payload = {
            "timestamp":          time.time(),
            "follower_to_gc":     ftg,
            "leader_to_follower": ltf,
            "jamming": {
                "severity":      sev,
                "mode":          _jam_mode,
                "affected_link": _jam_target,
            },
        }

        self._mqtt.publish(
            f"drone/{DRONE_ID}/raw_signal_inject",
            json.dumps(payload),
            qos=0,
        )

        self._tick += 1
        if self._tick % 20 == 0:
            self.get_logger().info(
                "tick=%d sev=%.3f snr: ftg=%.1f ltf=%.1f",
                self._tick, sev, ftg["snr_db"], ltf["snr_db"],
            )


def main():
    rclpy.init()

    password = _read_password()
    import paho.mqtt.client as _mqtt
    try:
        from paho.mqtt.client import CallbackAPIVersion
        client = _mqtt.Client(CallbackAPIVersion.VERSION1,
                              client_id=f"follower_signal_faker_{DRONE_ID}")
    except (ImportError, AttributeError):
        client = _mqtt.Client(client_id=f"follower_signal_faker_{DRONE_ID}")

    client.username_pw_set(_MQTT_USER, password)
    client.connect(_MQTT_HOST, _MQTT_PORT, keepalive=60)
    client.loop_start()
    log.info("MQTT client connected to %s:%d", _MQTT_HOST, _MQTT_PORT)

    node = FollowerSignalFaker(client)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        client.loop_stop()


if __name__ == "__main__":
    main()
