#!/usr/bin/env python3
"""
dashboard.py — Leaflet map dashboard for the two-drone relay demo.

Subscribes to MQTT `drone/{drone_id}/state` on both per-drone brokers,
caches the latest state, and serves a browser dashboard showing:

  - GC ground control point (static marker)
  - drone-01 (leader) and drone-02 (follower) live positions
  - trails (recent N samples per drone)
  - relay-line overlay when follower is in OFFBOARD
  - readout of mode, altitude AGL, battery, distance-from-GC per drone

Run:
    python3 /root/dashboard.py          # requires paho-mqtt only

Then open http://<host>:5000 in a browser.

Zero relay-pipeline dependency — reads only what state_bridge already
publishes to MQTT.
"""

import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import paho.mqtt.client as mqtt

# ── Config ────────────────────────────────────────────────────────────────────

MQTT_PASSWORD = "cBuKdY6JmwsBR6MICiHA"
MQTT_USER     = "drone"
BROKERS = {
    "drone-01": dict(host="127.0.0.1", port=1884),
    "drone-02": dict(host="127.0.0.1", port=1885),
}
GC_LAT, GC_LON = 47.390, 8.540
TRAIL_MAX = 240                 # ~4 min at 1 Hz state updates
HTTP_PORT = 5000

# ── State cache (thread-safe) ─────────────────────────────────────────────────

_state_lock = threading.Lock()
_latest    = {did: None for did in BROKERS}
_trails    = {did: deque(maxlen=TRAIL_MAX) for did in BROKERS}


def _on_message(client, userdata, msg):
    did = userdata
    try:
        d = json.loads(msg.payload)
    except Exception:
        return
    p, h = d.get("position") or {}, d.get("home_pos") or {}
    lat, lon = p.get("lat"), p.get("lon")
    if lat is None or lon is None:
        return
    alt_r = (p.get("alt") or 0) - (h.get("alt") or 0)
    entry = {
        "lat":        lat,
        "lon":        lon,
        "alt_agl":    round(alt_r, 1),
        "mode":       d.get("flight_mode", "?"),
        "battery":    d.get("battery_pct"),
        "gps_fix":    d.get("gps_fix_type"),
        "home":       {"lat": h.get("lat"), "lon": h.get("lon"),
                       "alt": h.get("alt")},
        "ts":         d.get("timestamp") or time.time(),
        "received":   time.time(),
    }
    with _state_lock:
        _latest[did] = entry
        _trails[did].append([lat, lon])


def _mqtt_client(did: str, host: str, port: int):
    """Run a paho client in a daemon thread; reconnect indefinitely."""
    def _run():
        while True:
            try:
                c = mqtt.Client(client_id=f"dashboard-{did}", userdata=did)
                c.username_pw_set(MQTT_USER, MQTT_PASSWORD)
                c.on_message = _on_message
                c.on_connect = lambda cli, u, f, rc: cli.subscribe(f"drone/{did}/state", qos=0)
                c.connect(host, port, keepalive=30)
                c.loop_forever()
            except Exception as e:
                print(f"[{did}] MQTT error: {e} — retry in 5s")
                time.sleep(5)
    t = threading.Thread(target=_run, daemon=True, name=f"mqtt-{did}")
    t.start()


for did, cfg in BROKERS.items():
    _mqtt_client(did, cfg["host"], cfg["port"])


# ── HTTP + HTML ───────────────────────────────────────────────────────────────

INDEX_HTML = """<!doctype html>
<html>
<head>
<title>Relay demo dashboard</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  body { margin: 0; font-family: system-ui, sans-serif; }
  #map { height: 100vh; width: 100%; }
  .panel {
    position: absolute; top: 10px; right: 10px; z-index: 500;
    background: rgba(255,255,255,0.94); padding: 10px 14px; border-radius: 6px;
    box-shadow: 0 2px 8px rgba(0,0,0,0.2); font-size: 13px;
    min-width: 250px;
  }
  .panel h3 { margin: 0 0 6px 0; font-size: 14px; }
  .row      { display: flex; justify-content: space-between; }
  .row span:last-child { font-family: monospace; }
  .mode-OFFBOARD    { color: #0a0; font-weight: bold; }
  .mode-AUTO\\.MISSION { color: #06f; font-weight: bold; }
  .mode-HOLD        { color: #d80; }
  .mode-RTL, .mode-AUTO\\.RTL { color: #d00; }
  .stale { color: #999; font-style: italic; }
  .drone-01 { border-left: 4px solid #06f; padding-left: 8px; }
  .drone-02 { border-left: 4px solid #0a0; padding-left: 8px; }
  hr.sep { margin: 8px 0; border: none; border-top: 1px solid #eee; }
  .footer { position: absolute; bottom: 6px; left: 10px; z-index: 500;
            font-size: 11px; color: #666; background: rgba(255,255,255,0.85);
            padding: 3px 8px; border-radius: 4px; }
</style>
</head>
<body>
<div id="map"></div>
<div class="panel" id="panel"><em>connecting…</em></div>
<div class="footer">GC: {{gc_lat}}, {{gc_lon}}  ·  updates 1 Hz</div>

<script>
const GC = [{{gc_lat}}, {{gc_lon}}];
const map = L.map('map').setView(GC, 16);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '© OpenStreetMap'
}).addTo(map);

// GC marker (red)
const gcMarker = L.circleMarker(GC, {
  radius: 10, color: '#c00', fillColor: '#c00', fillOpacity: 0.75, weight: 2
}).addTo(map).bindTooltip('GC (Ground Control)', {permanent: true, direction: 'right', offset: [10, 0]});

// Drone markers + trails
const droneStyle = {
  'drone-01': { color: '#06f', label: 'Leader (drone-01)' },
  'drone-02': { color: '#0a0', label: 'Follower (drone-02)' }
};
const droneMarkers = {}, droneTrails = {};
for (const [id, s] of Object.entries(droneStyle)) {
  droneMarkers[id] = L.circleMarker(GC, {
    radius: 8, color: s.color, fillColor: s.color, fillOpacity: 0.85, weight: 2
  }).addTo(map).bindTooltip(s.label, {permanent: true, direction: 'right', offset: [10, 0]});
  droneTrails[id] = L.polyline([], { color: s.color, weight: 2, opacity: 0.6, dashArray: '4,6' }).addTo(map);
}

// Relay line: GC ↔ follower ↔ leader when follower is OFFBOARD (has active authorization).
const relayLine = L.polyline([], {
  color: '#d80', weight: 3, opacity: 0.85, dashArray: '8,6'
}).addTo(map);

function fmt(v, unit='', fixed=1) {
  if (v === null || v === undefined) return '–';
  return typeof v === 'number' ? v.toFixed(fixed) + unit : v + unit;
}

function haversine(a, b) {
  const R = 6371000;
  const dl = (b[0]-a[0])*Math.PI/180, dm = (b[1]-a[1])*Math.PI/180;
  const x = Math.sin(dl/2)**2 + Math.cos(a[0]*Math.PI/180)*Math.cos(b[0]*Math.PI/180)*Math.sin(dm/2)**2;
  return 2*R*Math.asin(Math.sqrt(x));
}

async function tick() {
  try {
    const r = await fetch('/state', {cache: 'no-store'});
    const data = await r.json();
    const now  = data.now;
    const parts = [];
    let followerOffboard = false, leaderPos = null, followerPos = null;

    for (const [id, s] of Object.entries(droneStyle)) {
      const st = data.drones[id];
      let block = `<div class="${id}"><h3>${s.label}</h3>`;
      if (!st) {
        block += '<em class="stale">no data</em></div>';
        parts.push(block);
        continue;
      }
      const pos = [st.lat, st.lon];
      const age = now - st.received;
      droneMarkers[id].setLatLng(pos);
      const trail = data.trails[id] || [];
      droneTrails[id].setLatLngs(trail);
      const d_gc = haversine(GC, pos);
      const modeClass = 'mode-' + (st.mode||'').replace(/\\./g, '\\\\.');
      block += `<div class="row"><span>Mode</span><span class="${modeClass}">${st.mode}</span></div>`;
      block += `<div class="row"><span>Alt AGL</span><span>${fmt(st.alt_agl,' m')}</span></div>`;
      block += `<div class="row"><span>Battery</span><span>${fmt(st.battery,'%',0)}</span></div>`;
      block += `<div class="row"><span>d_GC</span><span>${d_gc.toFixed(0)} m</span></div>`;
      block += `<div class="row"><span>Data age</span><span class="${age>3?'stale':''}">${age.toFixed(1)} s</span></div>`;
      block += `</div>`;
      parts.push(block);

      if (id === 'drone-01') leaderPos = pos;
      if (id === 'drone-02') { followerPos = pos; followerOffboard = (st.mode === 'OFFBOARD'); }
    }

    if (followerOffboard && leaderPos && followerPos) {
      relayLine.setLatLngs([GC, followerPos, leaderPos]);
      relayLine.setStyle({opacity: 0.85});
    } else {
      relayLine.setStyle({opacity: 0});
    }

    document.getElementById('panel').innerHTML =
      parts.join('<hr class="sep">') +
      (followerOffboard ? '<hr class="sep"><em>Relay ACTIVE (follower in OFFBOARD)</em>' : '');
  } catch (e) {
    document.getElementById('panel').innerHTML = '<em class="stale">fetch error: '+e+'</em>';
  }
}
tick(); setInterval(tick, 1000);
</script>
</body>
</html>
"""


def _render_index() -> bytes:
    return (INDEX_HTML
            .replace("{{gc_lat}}", str(GC_LAT))
            .replace("{{gc_lon}}", str(GC_LON))
            .encode("utf-8"))


def _snapshot_json() -> bytes:
    with _state_lock:
        payload = {
            "now":    time.time(),
            "drones": {did: (_latest[did] or None) for did in BROKERS},
            "trails": {did: list(_trails[did]) for did in BROKERS},
        }
    return json.dumps(payload).encode("utf-8")


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            body = _render_index()
            ctype = "text/html; charset=utf-8"
        elif self.path == "/state":
            body = _snapshot_json()
            ctype = "application/json"
        else:
            self.send_response(404); self.end_headers(); return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    # Quiet the default per-request stderr log — dashboards poll a lot.
    def log_message(self, format, *args): pass


if __name__ == "__main__":
    print(f"Dashboard on http://0.0.0.0:{HTTP_PORT}")
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), _Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()
