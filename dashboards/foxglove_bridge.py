#!/usr/bin/env python3
"""
foxglove_bridge.py — MQTT → Foxglove WebSocket bridge for the two-drone
relay demo.

Subscribes to `drone/{drone_id}/state` on both per-drone MQTT brokers.
For each drone, republishes into two Foxglove channels:

  /{drone-id}/location   — foxglove.LocationFix schema (map + 3D scene)
  /{drone-id}/state      — raw state JSON (plots, indicators, raw messages)

Also publishes a single retained LocationFix for the GC point so it shows
up as a static marker on the Map panel.

Run:
    pip install --user foxglove-websocket
    python3 /root/foxglove_bridge.py     # binds ws://0.0.0.0:8765

On Windows, open Foxglove Studio → Data Source → Foxglove WebSocket
→ ws://localhost:8765
"""

import asyncio
import json
import threading
import time
from queue import Empty, SimpleQueue

import paho.mqtt.client as mqtt
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer

# ── Config ────────────────────────────────────────────────────────────────────

MQTT_USER, MQTT_PASSWORD = "drone", "cBuKdY6JmwsBR6MICiHA"
BROKERS = {
    "drone-01": ("127.0.0.1", 1884),
    "drone-02": ("127.0.0.1", 1885),
}
GC_LAT, GC_LON, GC_ALT = 47.390, 8.540, 488.0     # matches drone home_alt frame
WS_HOST, WS_PORT = "0.0.0.0", 8765

# Foxglove standard schemas (JSON-schema strings).
LOCATION_FIX_SCHEMA = json.dumps({
    "title": "foxglove.LocationFix",
    "type": "object",
    "properties": {
        "timestamp":   {"type": "object",
                        "properties": {"sec": {"type": "integer"},
                                       "nsec": {"type": "integer"}}},
        "frame_id":    {"type": "string"},
        "latitude":    {"type": "number"},
        "longitude":   {"type": "number"},
        "altitude":    {"type": "number"},
        "position_covariance":      {"type": "array", "items": {"type": "number"}},
        "position_covariance_type": {"type": "integer"},
    },
})

STATE_SCHEMA = json.dumps({
    "title": "drone.State",
    "type":  "object",
    "properties": {
        "drone_id":       {"type": "string"},
        "flight_mode":    {"type": "string"},
        "battery_pct":    {"type": "number"},
        "gps_fix_type":   {"type": "integer"},
        "altitude_agl_m": {"type": "number"},
        "position": {"type": "object",
                     "properties": {"lat":{"type":"number"},"lon":{"type":"number"},
                                    "alt":{"type":"number"}}},
        "home_pos": {"type": "object",
                     "properties": {"lat":{"type":"number"},"lon":{"type":"number"},
                                    "alt":{"type":"number"}}},
        "timestamp":      {"type": "number"},
    },
})

# ── MQTT → asyncio bridge queue ───────────────────────────────────────────────

_queue: SimpleQueue = SimpleQueue()    # (drone_id, state_dict, ts_ns)


def _mqtt_worker(drone_id: str, host: str, port: int):
    """Runs a paho client in a daemon thread, forwards state msgs to the queue."""
    def on_message(client, _u, msg):
        try:
            d = json.loads(msg.payload)
        except Exception:
            return
        ts_ns = int((d.get("timestamp") or time.time()) * 1e9)
        _queue.put((drone_id, d, ts_ns))

    def run():
        while True:
            try:
                c = mqtt.Client(client_id=f"fox-{drone_id}")
                c.username_pw_set(MQTT_USER, MQTT_PASSWORD)
                c.on_connect = lambda cli, u, f, rc: cli.subscribe(f"drone/{drone_id}/state", qos=0)
                c.on_message = on_message
                c.connect(host, port, keepalive=30)
                c.loop_forever()
            except Exception as e:
                print(f"[{drone_id}] MQTT err {e} — retry 5 s")
                time.sleep(5)
    threading.Thread(target=run, daemon=True, name=f"mqtt-{drone_id}").start()


for did, (host, port) in BROKERS.items():
    _mqtt_worker(did, host, port)


# ── Foxglove server ───────────────────────────────────────────────────────────

async def main():
    async with FoxgloveServer(WS_HOST, WS_PORT, "relay-demo") as server:
        # One LocationFix channel per drone + one for GC.
        # One raw state channel per drone.
        chans = {}
        for did in BROKERS:
            chans[(did, "loc")] = await server.add_channel({
                "topic":       f"/{did}/location",
                "encoding":    "json",
                "schemaName":  "foxglove.LocationFix",
                "schema":      LOCATION_FIX_SCHEMA,
            })
            chans[(did, "state")] = await server.add_channel({
                "topic":       f"/{did}/state",
                "encoding":    "json",
                "schemaName":  "drone.State",
                "schema":      STATE_SCHEMA,
            })
        gc_chan = await server.add_channel({
            "topic":       "/gc/location",
            "encoding":    "json",
            "schemaName":  "foxglove.LocationFix",
            "schema":      LOCATION_FIX_SCHEMA,
        })

        print(f"Foxglove WS server: ws://{WS_HOST}:{WS_PORT}")
        print(f"Channels: {[c[0]+'/'+c[1] for c in chans]} + /gc/location")

        # Push GC location periodically (2 Hz) so Map panel always has it.
        async def gc_beacon():
            while True:
                now = time.time()
                loc = {
                    "timestamp": {"sec": int(now), "nsec": int((now % 1) * 1e9)},
                    "frame_id":  "map",
                    "latitude":  GC_LAT,
                    "longitude": GC_LON,
                    "altitude":  GC_ALT,
                    "position_covariance":      [0]*9,
                    "position_covariance_type": 0,
                }
                await server.send_message(gc_chan, int(now * 1e9), json.dumps(loc).encode())
                await asyncio.sleep(0.5)
        asyncio.create_task(gc_beacon())

        # Drain the MQTT queue and forward.
        loop = asyncio.get_event_loop()
        while True:
            # Blocking get on a background thread executor so asyncio stays responsive.
            drone_id, d, ts_ns = await loop.run_in_executor(None, _queue.get)
            p = d.get("position") or {}
            h = d.get("home_pos") or {}
            lat, lon, alt = p.get("lat"), p.get("lon"), p.get("alt", 0)
            if lat is None or lon is None:
                continue
            alt_agl = (alt or 0) - (h.get("alt") or 0)

            loc = {
                "timestamp": {"sec": ts_ns // 1_000_000_000,
                              "nsec": ts_ns %  1_000_000_000},
                "frame_id":  "map",
                "latitude":  lat,
                "longitude": lon,
                "altitude":  alt,
                "position_covariance":      [0]*9,
                "position_covariance_type": 0,
            }
            state = {
                "drone_id":       drone_id,
                "flight_mode":    d.get("flight_mode", "?"),
                "battery_pct":    float(d.get("battery_pct") or 0),
                "gps_fix_type":   int(d.get("gps_fix_type") or 0),
                "altitude_agl_m": round(alt_agl, 2),
                "position":       {"lat": lat, "lon": lon, "alt": alt or 0},
                "home_pos":       {"lat": h.get("lat") or 0,
                                   "lon": h.get("lon") or 0,
                                   "alt": h.get("alt") or 0},
                "timestamp":      float(d.get("timestamp") or time.time()),
            }
            await server.send_message(chans[(drone_id, "loc")],   ts_ns, json.dumps(loc).encode())
            await server.send_message(chans[(drone_id, "state")], ts_ns, json.dumps(state).encode())


if __name__ == "__main__":
    run_cancellable(main())
