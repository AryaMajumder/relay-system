"""
state_bridge.py — ROS2 node, always running.

Bridges MQTT drone/<id>/state → ROS2 /<id>/drone_state at 1Hz.

Source is always px4_agent via MQTT — no source-mode abstraction needed.

Critical behavior: republishes last-known payload using the SOURCE's
original timestamp on every tick, even when source is silent.
Before the first source message, nothing is published.
"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DRONE_ID     = os.environ.get("DRONE_ID", "drone-01")
# 5s without a new MQTT message triggers a stale warning; not a gate, just observability.
STALE_WARN_S = 5.0

# Each drone's px4_agent publishes its state on a different MQTT broker port so
# multiple drones can co-exist on one machine without topic collisions.
_PORT_MAP = {"drone-01": 1884, "drone-02": 1885}


def _ros_topic(drone_id: str) -> str:
    return f"/{drone_id.replace('-', '_')}/drone_state"


def _read_password() -> str:
    # Try two credential file locations in order: production path first, dev path second.
    # Fall back to MQTT_PASS env var so tests and CI can run without credential files.
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
    # paho-mqtt ≥2.0 requires CallbackAPIVersion; older paho uses positional args.
    # The try/except handles both so the same code works across paho versions.
    import paho.mqtt.client as mqtt
    try:
        from paho.mqtt.client import CallbackAPIVersion
        client = mqtt.Client(CallbackAPIVersion.VERSION1, client_id=client_id)
    except (ImportError, AttributeError):
        client = mqtt.Client(client_id=client_id)
    return client


class StateBridge(Node):

    def __init__(self):
        node_name = f"state_bridge_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        self._pub = self.create_publisher(String, _ros_topic(DRONE_ID), 10)
        # _lock guards _last_payload and _last_recv_ts which are written by the
        # MQTT thread and read by the ROS2 timer callback — two different threads.
        self._lock = threading.Lock()
        self._last_payload = None   # None means "no message received yet" — gate in _republish_tick
        self._last_recv_ts = 0.0
        # One-shot flag so the "source silent" warning fires once, not every tick.
        self._stale_logged = False
        self._publish_count = 0

        self._start_mqtt()
        # 1 Hz republish timer — downstream nodes expect a steady 1 Hz state feed.
        self.create_timer(1.0, self._republish_tick)

        self.get_logger().info(
            f"state_bridge started: topic={_ros_topic(DRONE_ID)} rate=1Hz"
        )

    def _start_mqtt(self):
        port = _PORT_MAP.get(DRONE_ID, 1884)
        password = _read_password()

        self._mqtt = _build_mqtt_client(f"state_bridge_{DRONE_ID}")
        self._mqtt.username_pw_set("drone", password)
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_message = self._on_mqtt_message
        # keepalive=60: broker drops client after 1.5× this without a PING — 60s is generous.
        self._mqtt.connect("127.0.0.1", port, keepalive=60)
        # loop_start() spawns a background thread; avoids blocking the ROS spin thread.
        self._mqtt.loop_start()

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            topic = f"drone/{DRONE_ID}/state"
            client.subscribe(topic)
            self.get_logger().info(f"MQTT connected, subscribed {topic}")
        else:
            self.get_logger().error(f"MQTT connect failed rc={rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            raw = json.loads(msg.payload.decode())
        except Exception:
            return

        # Only add timestamp when the source omitted it; never overwrite an existing one.
        # This is the sole exception to the "don't restamp" rule (§5.6): we're stamping
        # a message that arrived without any timestamp at all, not restamping an existing one.
        if "timestamp" not in raw:
            raw["timestamp"] = time.time()

        now = time.time()
        with self._lock:
            # Capture is_first while holding the lock so the check and the write are atomic.
            is_first = self._last_payload is None
            self._last_payload = raw
            self._last_recv_ts = now
            # Reset the stale flag inside the lock so the warning can fire again if source goes silent again.
            if self._stale_logged:
                self._stale_logged = False
                self.get_logger().info("drone_state source recovered")

        # Log outside the lock — logging can be slow and we don't want to hold _lock.
        if is_first:
            self.get_logger().info(
                f"drone_state first MQTT message received — mode={raw.get('flight_mode')}"
            )

    def _republish_tick(self):
        # Copy under lock, then release before any slow operations.
        with self._lock:
            payload = self._last_payload
            recv_ts = self._last_recv_ts

        # Gate: don't publish until at least one MQTT message has been received.
        # Prevents downstream freshness gates from seeing a stale zero-timestamp payload.
        if payload is None:
            return

        silence_s = time.time() - recv_ts
        if silence_s > STALE_WARN_S and not self._stale_logged:
            ts_age = time.time() - payload["timestamp"]
            self.get_logger().warning(
                f"drone_state source silent for {silence_s:.1f}s (ts age={ts_age:.1f}s)"
            )
            # Set under lock to prevent a race where two ticks both see stale_logged=False.
            with self._lock:
                self._stale_logged = True

        msg = String()
        # Republish last-known payload with its original timestamp — not the current time.
        # Downstream freshness guards (condition_nodes) measure age from this timestamp,
        # so they will correctly detect that the source has been silent.
        msg.data = json.dumps(payload)
        try:
            self._pub.publish(msg)
            self._publish_count += 1
            # Heartbeat log every 10 publishes (~10 s) to confirm liveness without flooding.
            if self._publish_count % 10 == 0:
                self.get_logger().info(
                    f"drone_state publish #{self._publish_count} "
                    f"silence={silence_s:.1f}s mode={payload.get('flight_mode')}"
                )
        except Exception as e:
            self.get_logger().error(f"_republish_tick publish FAILED: {e}")


def main():
    rclpy.init()
    node = StateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
