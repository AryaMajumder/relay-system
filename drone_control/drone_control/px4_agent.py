"""
px4_agent.py — MQTT client to the production px4_agent daemon.

Write path only: publish_setpoint(lat, lon, alt), start_offboard(), send_command().
  Converts WGS-84 → LOCAL NED using home_pos cached from drone/{DRONE_ID}/state,
  then publishes to drone/{DRONE_ID}/setpoint (QoS 0).  The production agent's
  _offboard_sender (20 Hz) forwards to PX4 via SET_POSITION_TARGET_LOCAL_NED.
  Home position is cached from the first drone_state message; setpoints are
  silently dropped until home is known.

There is no direct MAVLink read path here.  Position and OFFBOARD mode state
reach the BT via the production MQTT chain (drone_state) → relay_position_tracker
→ capability_assessor → BT blackboard.
"""

import json
import logging
import math
import os
import threading
import time
import uuid

log = logging.getLogger(__name__)

DRONE_ID   = os.environ.get("DRONE_ID", "drone-01")

# Per-drone MQTT broker ports: each drone's px4_agent runs its own broker so
# multiple drones can co-exist on one machine without topic collisions.
_BROKER_PORT = {"drone-01": 1884, "drone-02": 1885}
_EARTH_R     = 6371000.0   # WGS-84 mean radius for NED conversion (metres)


def _read_mqtt_pass() -> str:
    # Try production credential file first, dev path second, env var as fallback.
    # Same search order as state_bridge._read_password().
    for path in ("/etc/drone-pub/mqtt_pass.txt",
                 "/etc/mqtt-creds/mosquitto_drone_credentials.txt"):
        try:
            return open(path).read().strip()
        except OSError:
            pass
    return os.environ.get("MQTT_PASS", "")


def _build_mqtt_client(client_id: str):
    # Same paho version-compat pattern as state_bridge._build_mqtt_client.
    import paho.mqtt.client as mqtt
    try:
        from paho.mqtt.client import CallbackAPIVersion
        return mqtt.Client(CallbackAPIVersion.VERSION1, client_id=client_id)
    except (ImportError, AttributeError):
        return mqtt.Client(client_id=client_id)


class PX4Agent:
    """
    MQTT write-path client to the production px4_agent daemon.
    publish_setpoint() / start_offboard() / send_command() are the only public methods.
    home_pos is cached from the drone_state subscription to enable WGS-84→NED conversion.
    """

    def __init__(self, logger=None):
        self._log = logger or log
        self._lock = threading.Lock()

        # home_pos is cached here under _lock; written by MQTT on_message, read by publish_setpoint.
        self._home_pos: dict | None = None

        self._mqtt_client = None
        # _mqtt_ready gates write-path calls so setpoints are not sent when disconnected.
        self._mqtt_ready  = False

        self._start_mqtt()

    # ── Write path — MQTT setup ───────────────────────────────────────────────

    def _start_mqtt(self):
        port   = _BROKER_PORT.get(DRONE_ID, 1884)
        pw     = _read_mqtt_pass()
        client = _build_mqtt_client(f"px4agent-bt-{DRONE_ID}")
        client.username_pw_set("drone", pw)

        def on_connect(c, userdata, flags, rc):
            if rc == 0:
                c.subscribe(f"drone/{DRONE_ID}/state", qos=0)
                self._mqtt_ready = True
                self._log.info(
                    f"px4_agent MQTT connected :{port} — subscribed to drone/{DRONE_ID}/state"
                )
            else:
                self._log.warning(f"px4_agent MQTT connect failed rc={rc}")

        def on_disconnect(c, userdata, rc):
            self._mqtt_ready = False

        def on_message(c, userdata, msg):
            try:
                data = json.loads(msg.payload)
                home = data.get("home_pos")
                if PX4Agent._accept_home(home):
                    with self._lock:
                        self._home_pos = home
            except Exception as e:
                self._log.debug(f"px4_agent MQTT state parse error: {e}")

        client.on_connect    = on_connect
        client.on_disconnect = on_disconnect
        client.on_message    = on_message
        self._mqtt_client    = client

        def _run():
            # Infinite retry loop: MQTT reconnects automatically after broker restarts.
            # A 5 s delay prevents busy-looping if the broker is persistently down.
            while True:
                try:
                    client.connect("127.0.0.1", port, keepalive=30)
                    client.loop_forever()
                except Exception as e:
                    self._mqtt_ready = False
                    self._log.warning(f"px4_agent MQTT error: {e} — retrying in 5s")
                    time.sleep(5)

        # Daemon thread: process can exit even if this thread is blocked in connect().
        threading.Thread(target=_run, daemon=True, name="px4-mqtt").start()

    @staticmethod
    def _accept_home(home) -> bool:
        """
        Return True if home is a valid GPS fix that should be cached.
        PX4 sends home_pos as {"lat": 0, "lon": 0} before it acquires a GPS fix.
        Accepting that placeholder would make every subsequent NED conversion silently
        wrong — the drone would be sent relative to the origin (0, 0) instead of the
        real launch point.  Reject the message when BOTH lat and lon are zero/falsy.
        """
        return bool(home and (home.get("lat") or home.get("lon")))

    @staticmethod
    def _global_to_ned(lat, lon, alt, home_lat, home_lon, home_alt):
        """WGS-84 → LOCAL NED (metres). x=North, y=East, z=Down (altitude inverted)."""
        # Small-angle flat-earth approximation — valid within a few km of home.
        x = _EARTH_R * math.radians(lat - home_lat)
        # East displacement scales by cos(lat) because longitude degrees shrink near poles.
        y = _EARTH_R * math.cos(math.radians(home_lat)) * math.radians(lon - home_lon)
        # NED z is positive-down; altitude is positive-up, so negate.
        z = -(alt - home_alt)
        return x, y, z

    # ── Write path ────────────────────────────────────────────────────────────

    def publish_setpoint(self, lat: float, lon: float, alt: float) -> None:
        """
        Convert WGS-84 → LOCAL NED and forward to the production px4_agent via
        drone/{DRONE_ID}/setpoint MQTT.  The production agent's _offboard_sender
        (20 Hz) picks it up and sends SET_POSITION_TARGET_LOCAL_NED to PX4.

        Silently drops the setpoint if home_pos is not yet known or MQTT is not
        connected — relay_mover will retry on the next tick (3 Hz).
        """
        with self._lock:
            home = self._home_pos

        # home_pos is required for WGS-84→NED conversion.  Without it the setpoint
        # would be computed relative to (0, 0, 0) and would send the drone off-grid.
        # relay_mover retries on the next tick (3 Hz) so dropping one setpoint is safe.
        if home is None:
            self._log.warning("publish_setpoint: home_pos not yet known — skipping")
            return

        if not self._mqtt_ready:
            self._log.warning("publish_setpoint: MQTT not ready — skipping")
            return

        x, y, z = self._global_to_ned(
            lat, lon, alt,
            home["lat"], home["lon"], home.get("alt", 0.0),
        )
        sp = {"x": x, "y": y, "z": z, "yaw": 0.0}

        try:
            self._mqtt_client.publish(
                f"drone/{DRONE_ID}/setpoint", json.dumps(sp), qos=0
            )
            self._log.debug(
                f"setpoint → NED x={x:.1f} y={y:.1f} z={z:.1f} "
                f"(lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m)"
            )
        except Exception as e:
            self._log.warning(f"publish_setpoint MQTT publish error: {e}")

    def send_command(self, cmd: str, params: dict = None) -> None:
        """
        Send an arbitrary command to the production px4_agent via
        drone/{DRONE_ID}/cmd MQTT (QoS 1).  Used by relay_mover to
        action pending_command payloads written by FollowerSafetyExit.
        """
        if not self._mqtt_ready:
            self._log.warning(f"send_command({cmd}): MQTT not ready — dropped")
            return

        payload = {
            "cmd_id":   f"relay-cmd-{uuid.uuid4().hex[:8]}",
            "drone_id": DRONE_ID,
            "cmd":      cmd,
            "params":   params or {},
        }
        try:
            self._mqtt_client.publish(
                f"drone/{DRONE_ID}/cmd", json.dumps(payload), qos=1
            )
            self._log.warning(
                f"cmd sent → drone/{DRONE_ID}/cmd  cmd={cmd}  "
                f"cmd_id={payload['cmd_id']}"
            )
        except Exception as e:
            self._log.warning(f"send_command MQTT publish error: {e}")

    def start_offboard(self) -> None:
        """
        Send START_LEAD to the production px4_agent via drone/{DRONE_ID}/cmd MQTT.
        The production agent verifies the setpoint stream has been flowing for
        OFFBOARD_PRE_SECS, then commands PX4 into OFFBOARD mode.
        Called once by relay_mover after the pre-stream window has elapsed.
        """
        if not self._mqtt_ready:
            self._log.warning("start_offboard: MQTT not ready — cannot send START_LEAD")
            return

        cmd = {
            "cmd_id":   f"relay-offboard-{uuid.uuid4().hex[:8]}",
            "drone_id": DRONE_ID,
            "cmd":      "START_LEAD",
            "params":   {},
        }
        try:
            self._mqtt_client.publish(
                f"drone/{DRONE_ID}/cmd", json.dumps(cmd), qos=1
            )
            self._log.info(
                f"START_LEAD sent → drone/{DRONE_ID}/cmd  cmd_id={cmd['cmd_id']}"
            )
        except Exception as e:
            self._log.warning(f"start_offboard MQTT publish error: {e}")
