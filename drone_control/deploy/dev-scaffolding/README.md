# deploy/dev-scaffolding — SITL-only stand-ins

Utilities that substitute for hardware or upstream services during dev/SITL
work. **NOT part of any deployment target.** No systemd unit files ship for
these; run them by hand under `systemd-run --unit=...` when needed. On a real
fleet these are replaced by the actual data sources.

## `fake_drone_state_broadcaster.py`

**Substitutes for:** the production `px4_agent.service` publishing MQTT
`drone/{drone_id}/state` from live MAVLink telemetry.

**Why it exists:** the relay BT needs `drone_state` (battery, GPS fix, flight
mode, position, home) to progress past G1/G2/G3/G4 gates. When no PX4 SITL is
running, px4_agent has nothing to publish. This tool emits synthetic messages
with values chosen to satisfy every data-freshness / capability gate:

| Field | Value | Gate it enables |
|-------|-------|-----------------|
| `battery_pct` | 95 | G2 BatteryStillSufficientToRelay, F_cap BatteryAboveFloor |
| `gps_fix_type` | 3 | GPSFixAdequate (3D fix) |
| `flight_mode` | "OFFBOARD" | G3 OffboardModeHeld, FlightModeAcceptable |
| `position` | Zurich reference | PositionServiceable |
| `home_pos` | Same as position | `_accept_home()` (nonzero lat/lon) |
| `avg_speed_ms`, `endurance_s` | legacy fields | back-compat with prior consumers |

Positions: drone-01 at 47.3980/8.5480 (Zurich), drone-02 ~500 m south — both
inside the demo geofence, well within radio range so `GeometryFeasible` and
BandSensor gates pass.

**Usage** (transient systemd unit — survives shell exit, own journal stream):
```bash
systemd-run --unit=fake-drone-state-broadcaster \
  --description="fake drone_state MQTT broadcaster (SITL, no PX4)" \
  /usr/bin/python3 /root/ros2_ws/src/drone_control/deploy/dev-scaffolding/fake_drone_state_broadcaster.py

# tail logs
journalctl -u fake-drone-state-broadcaster.service -f

# stop
systemctl stop fake-drone-state-broadcaster.service
```

**Or per-drone:**
```bash
DRONE=drone-01 python3 fake_drone_state_broadcaster.py &
DRONE=drone-02 python3 fake_drone_state_broadcaster.py &
```

**Broker ports (SITL convention):** drone-01 → 1884, drone-02 → 1885. Reads
MQTT password from `/etc/mqtt-creds/mosquitto_drone_credentials.txt`. Same
setup the production `px4_agent.service` uses.

## When you no longer need it

Real fleet: never enable. The production `px4_agent.service` provides real
telemetry from PX4.

SITL rig running PX4 SITL: also never enable — px4_agent will publish live
data from the SITL flight controller. `fake_drone_state_broadcaster` only
plugs the gap when there is NO PX4 (SITL or real) at all.

## Design intent

Every file in this directory:
- Must be manually invoked (no `[Install]` section, no target dependency)
- Documents the class of data it substitutes for
- Documents when it must NOT be enabled
- Lives out of the shipping deployment tree (`../systemd/`, `../sitl/`) so
  `install/`, `systemctl enable`, and CI can't pick it up by accident
