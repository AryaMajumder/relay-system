#!/usr/bin/env python3
"""
fake_drone_state_broadcaster.py

Publishes synthetic drone_state MQTT messages for drone-01 (leader) and drone-02
(follower) at 1 Hz so state_bridge -> capability_assessor sees positions/battery/mode
without a live PX4 SITL.

State values are picked to satisfy every relay-BT gate:
  - fresh timestamp (G1 FcuTelemetryFresh)
  - battery_pct=95           (G2 BatteryStillSufficientToRelay)
  - flight_mode="OFFBOARD"   (G3 OffboardModeHeld)
  - real GPS coords + home   (G4 PositionServiceable, G5+ position math)

Leader is placed near a Zurich reference point; follower is 500 m south (well within
comms range so relay computations produce sane numbers).

Run:
  DRONE=drone-01 python3 fake_drone_state_broadcaster.py &
  DRONE=drone-02 python3 fake_drone_state_broadcaster.py &

Or run without env to broadcast for BOTH drones (default).
"""

import json
import os
import sys
import time
import threading
import paho.mqtt.client as mqtt

MQTT_USER = "drone"
MQTT_PASS = open("/etc/mqtt-creds/mosquitto_drone_credentials.txt").read().strip().split(":")[1]

# Per-drone broker port (SITL convention)
BROKER_PORT = {"drone-01": 1884, "drone-02": 1885}

# Positions (WGS-84). Leader at Zurich reference, follower 500m south.
STATE = {
    "drone-01": {
        "lat":  47.3980,
        "lon":   8.5480,
        "alt":  50.0,
        "home_lat": 47.3980,
        "home_lon":  8.5480,
        "home_alt": 50.0,
    },
    "drone-02": {
        "lat":  47.3935,   # ~500m south of leader (~0.0045 deg lat)
        "lon":   8.5480,
        "alt":  50.0,
        "home_lat": 47.3935,
        "home_lon":  8.5480,
        "home_alt": 50.0,
    },
}


def build_state(drone_id: str) -> dict:
    s = STATE[drone_id]
    return {
        "drone_id":     drone_id,
        "timestamp":    time.time(),
        "battery_pct":  95,           # G2 pass
        "gps_fix_type": 3,            # 3D fix
        "flight_mode":  "OFFBOARD",   # G3 pass
        "position":     {"lat": s["lat"], "lon": s["lon"], "alt": s["alt"]},
        "home_pos":     {"lat": s["home_lat"], "lon": s["home_lon"], "alt": s["home_alt"]},
        # legacy fields still emitted by the production px4_agent
        "avg_speed_ms": 8.0,
        "endurance_s":  1200,
    }


def broadcast(drone_id: str):
    port = BROKER_PORT[drone_id]
    client = mqtt.Client(client_id=f"fake-state-{drone_id}")
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.connect("127.0.0.1", port, keepalive=30)
    client.loop_start()
    topic = f"drone/{drone_id}/state"
    print(f"[{drone_id}] connected :{port}  publishing to {topic}", flush=True)
    n = 0
    while True:
        payload = json.dumps(build_state(drone_id))
        client.publish(topic, payload, qos=0)
        n += 1
        if n % 10 == 1:
            print(f"[{drone_id}] published #{n}: {payload[:80]}...", flush=True)
        time.sleep(1.0)


def main():
    drones = [os.environ["DRONE"]] if "DRONE" in os.environ else ["drone-01", "drone-02"]
    threads = [threading.Thread(target=broadcast, args=(d,), daemon=True) for d in drones]
    for t in threads:
        t.start()
    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        print("shutting down", file=sys.stderr)


if __name__ == "__main__":
    main()
