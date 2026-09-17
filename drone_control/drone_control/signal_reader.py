"""
signal_reader.py — ROS2 node, always running.

Bridges a signal source into ROS2 /drone_NN/signal_report at 2Hz.

Source modes (SIGNAL_SOURCE env var, default: mqtt_inject):
  mqtt_inject — subscribes drone/<id>/raw_signal_inject via MQTT (SITL)
  mavlink     — production stub, raises NotImplementedError

Critical behavior: republishes last-known payload using the SOURCE's
original timestamp on every tick, even when source is silent.
This lets DataFreshness detect staleness from timestamp age.
Before the first source message, nothing is published.
"""

import json
import logging
import os
import sys
import threading
import time

# ── ROS2 setup ────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DRONE_ID      = os.environ.get("DRONE_ID", "drone-01")
SIGNAL_SOURCE = os.environ.get("SIGNAL_SOURCE", "mqtt_inject")
STALE_WARN_S  = 5.0

_LOG_LEVEL = os.environ.get("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, _LOG_LEVEL, logging.INFO),
    format="%(asctime)s %(levelname)s [signal_reader] %(message)s",
)
log = logging.getLogger(__name__)

_PORT_MAP = {"drone-01": 1884, "drone-02": 1885}

# ── Helpers ───────────────────────────────────────────────────────────────────

def _ros_topic(drone_id: str) -> str:
    return f"/{drone_id.replace('-', '_')}/signal_report"


def _link_subset(link: dict) -> dict:
    return {k: link[k] for k in ("rssi_dbm", "noise_dbm", "snr_db") if k in link}


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


def _build_mqtt_client(client_id: str):
    import paho.mqtt.client as mqtt
    try:
        from paho.mqtt.client import CallbackAPIVersion
        client = mqtt.Client(CallbackAPIVersion.VERSION1, client_id=client_id)
    except (ImportError, AttributeError):
        client = mqtt.Client(client_id=client_id)
    return client

# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class SignalReader(Node):

    def __init__(self):
        node_name = f"signal_reader_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        self._pub = self.create_publisher(String, _ros_topic(DRONE_ID), 10)
        self._lock = threading.Lock()
        self._last_payload = None
        self._last_recv_ts = 0.0
        self._stale_logged = False

        if SIGNAL_SOURCE == "mavlink":
            raise NotImplementedError(
                "SIGNAL_SOURCE=mavlink not yet implemented"
            )
        elif SIGNAL_SOURCE != "mqtt_inject":
            raise ValueError(f"Unknown SIGNAL_SOURCE={SIGNAL_SOURCE!r}")

        self._start_mqtt()
        self.create_timer(1.0 / 2.0, self._republish_tick)

        self.get_logger().info(
            f"signal_reader started: source={SIGNAL_SOURCE} topic={_ros_topic(DRONE_ID)} rate=2Hz"
        )

    # ── MQTT source ───────────────────────────────────────────────────────────

    def _start_mqtt(self):
        port = _PORT_MAP.get(DRONE_ID, 1884)
        password = _read_password()

        self._mqtt = _build_mqtt_client(f"signal_reader_{DRONE_ID}")
        self._mqtt.username_pw_set("drone", password)
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_message = self._on_mqtt_message
        self._mqtt.connect("127.0.0.1", port, keepalive=60)
        self._mqtt.loop_start()

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            topic = f"drone/{DRONE_ID}/raw_signal_inject"
            client.subscribe(topic)
            self.get_logger().info(f"MQTT connected, subscribed {topic}")
        else:
            self.get_logger().error(f"MQTT connect failed rc={rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            raw = json.loads(msg.payload.decode())
        except Exception:
            return

        jam_raw = raw.get("jamming", {})
        severity = jam_raw.get("severity", 0) or 0

        out = {
            "drone_id":  DRONE_ID,
            "timestamp": raw.get("timestamp", time.time()),
            "leader_to_gc":       _link_subset(raw.get("leader_to_gc", {})),
            "leader_to_follower": _link_subset(raw.get("leader_to_follower", {})),
            "follower_to_gc":     _link_subset(raw.get("follower_to_gc", {})),
            "jamming": {
                "active":        severity > 0,
                "severity":      severity,
                "affected_link": jam_raw.get("affected_link", ""),
            },
        }

        now = time.time()
        with self._lock:
            self._last_payload = out
            self._last_recv_ts = now
            if self._stale_logged:
                self._stale_logged = False
                self.get_logger().info("signal source recovered")

    # ── 2Hz publish tick ──────────────────────────────────────────────────────

    def _republish_tick(self):
        with self._lock:
            payload = self._last_payload
            recv_ts = self._last_recv_ts

        if payload is None:
            return

        silence_s = time.time() - recv_ts
        if silence_s > STALE_WARN_S and not self._stale_logged:
            ts_age = time.time() - payload["timestamp"]
            self.get_logger().warning(
                f"signal source silent for {silence_s:.1f}s (ts age={ts_age:.1f}s)"
            )
            with self._lock:
                self._stale_logged = True

        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = SignalReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
