#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
px4_agent.py

State model:
  RECEIVED  -- command persisted + acknowledged
  EXECUTED  -- PX4 confirmed
  FAILED    -- sent to PX4, no confirmation or PX4 rejected
  REJECTED  -- not sent to PX4 at all

Supported commands: RTL, LAND, LOITER, HOLD, ARM, DISARM, TAKEOFF, SET_MODE,
                    UPLOAD_MISSION, START_MISSION, START_LEAD, STOP_LEAD

Payload formats accepted:
  Legacy: {"cmd_id": "...", "drone_id": "...", "cmd": "RTL", "params": {}}
  Lambda: {"command_id": "...", "target_id": "...", "action": "rtl", "params": {}}

OFFBOARD/FOLLOW notes:
  The follower_node publishes setpoints to drone/{DRONE_ID}/setpoint at 20 Hz.
  The agent receives them, stores _latest_setpoint, and _offboard_sender
  forwards them to PX4 at 20 Hz as SET_POSITION_TARGET_LOCAL_NED.
  Setpoints must flow for OFFBOARD_PRE_SECS before START_LEAD will succeed.
  If setpoints go stale, PX4 exits OFFBOARD automatically (safety feature).
"""

import os
import re
import time
import json
import queue
import sqlite3
import threading
import logging
from datetime import datetime
import paho.mqtt.client as mqtt
from pymavlink import mavutil

# -----------------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------------
DRONE_ID    = os.environ.get("DRONE_ID",    "drone-01")
MQTT_HOST   = os.environ.get("MQTT_HOST",   "127.0.0.1")
MQTT_PORT   = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_USER   = os.environ.get("MQTT_USER",   "drone")
MQTT_PASS   = os.environ.get("MQTT_PASS",   "")
PX4_URL     = os.environ.get("PX4_URL",     "udpin:0.0.0.0:14556")
PX4_RX_URL  = os.environ.get("PX4_RX_URL",  "")  # separate telemetry listener if needed
PX4_SYS_ID  = int(os.environ.get("PX4_SYS_ID", "0"))  # hard-override for target_system (0 = auto)
DB_PATH     = os.environ.get("DB_PATH",     "/var/lib/px4_agent/agent.db")
HEARTBEAT_TIMEOUT       = int(os.environ.get("HEARTBEAT_TIMEOUT", "10"))
LOG_LEVEL               = os.environ.get("LOG_LEVEL", "INFO")
ENABLE_CLOUD_LINK_CHECK = os.environ.get(
    "ENABLE_CLOUD_LINK_CHECK", "false").lower() in ("true", "1", "yes")
MISSION_ITEM_TIMEOUT    = float(os.environ.get("MISSION_ITEM_TIMEOUT", "5"))
MISSION_ITEM_RETRIES    = int(os.environ.get("MISSION_ITEM_RETRIES",   "3"))

# ADDED: OFFBOARD tuning
OFFBOARD_PRE_SECS = float(os.environ.get("OFFBOARD_PRE_SECS", "1.5"))
SETPOINT_STALE_S  = float(os.environ.get("SETPOINT_STALE_S",  "1.0"))

# Read MQTT password from credential file if not in env
if not MQTT_PASS:
    try:
        for path in [
            '/etc/mqtt-creds/mosquitto_drone_credentials.txt',
            '/root/mosquitto_drone_credentials.txt'
        ]:
            if os.path.exists(path):
                with open(path, 'r') as f:
                    for line in f:
                        if line.startswith(f'{MQTT_USER}:'):
                            MQTT_PASS = line.strip().split(':', 1)[1]
                            break
                if MQTT_PASS:
                    break
    except Exception:
        pass

# -----------------------------------------------------------------------------
# Logging
# -----------------------------------------------------------------------------
logging.basicConfig(level=LOG_LEVEL,
                    format="%(asctime)s %(levelname)s: %(message)s")
log = logging.getLogger("px4_agent")

log.info("=" * 70)
log.info("PX4 Agent -- Merged (cloud + PX4 fixes)")
log.info("Cloud link check: %s", "ENABLED" if ENABLE_CLOUD_LINK_CHECK else "DISABLED")
log.info("=" * 70)
log.info("DRONE_ID  : %s", DRONE_ID)
log.info("MQTT      : %s:%s  user=%s  pass=%s",
         MQTT_HOST, MQTT_PORT, MQTT_USER, "***" if MQTT_PASS else "NOT SET")
log.info("PX4_URL   : %s", PX4_URL)
log.info("DB_PATH   : %s", DB_PATH)
log.info("=" * 70)

# -----------------------------------------------------------------------------
# SQLite -- per-call connections, thread-safe via lock
# -----------------------------------------------------------------------------
os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
db_lock = threading.Lock()

def _db():
    conn = sqlite3.connect(DB_PATH, timeout=10.0)
    conn.execute("PRAGMA journal_mode=WAL")
    return conn

def _init_db():
    with db_lock:
        conn = _db()
        try:
            c = conn.cursor()
            c.execute("""CREATE TABLE IF NOT EXISTS commands(
                cmd_id             TEXT PRIMARY KEY,
                drone_id           TEXT,
                payload            TEXT,
                status             TEXT,
                reason             TEXT,
                exec_result        TEXT,
                created_at         TEXT,
                updated_at         TEXT,
                cloud_confirmed    INTEGER DEFAULT 0,
                cloud_confirmed_at TEXT
            )""")
            c.execute("""CREATE TABLE IF NOT EXISTS events(
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                level      TEXT,
                msg        TEXT,
                meta       TEXT,
                created_at TEXT DEFAULT (datetime('now'))
            )""")
            c.execute("""CREATE INDEX IF NOT EXISTS idx_status
                ON commands(status, created_at)""")
            c.execute("""CREATE INDEX IF NOT EXISTS idx_replay
                ON commands(status, cloud_confirmed, created_at)""")
            conn.commit()

            existing = {row[1] for row in c.execute("PRAGMA table_info(commands)")}
            for col, defn in [
                ("reason",             "TEXT"),
                ("exec_result",        "TEXT"),
                ("cloud_confirmed",    "INTEGER DEFAULT 0"),
                ("cloud_confirmed_at", "TEXT"),
            ]:
                if col not in existing:
                    c.execute(f"ALTER TABLE commands ADD COLUMN {col} {defn}")
                    log.info("DB migration: added column %s", col)
            conn.commit()
            log.info("DB initialised")
        finally:
            conn.close()

_init_db()

def command_exists(cmd_id):
    with db_lock:
        conn = _db()
        try:
            row = conn.execute(
                "SELECT 1 FROM commands WHERE cmd_id=? LIMIT 1", (cmd_id,)
            ).fetchone()
            if row:
                log.warning("DUPLICATE command: %s", cmd_id)
            return row is not None
        finally:
            conn.close()

def db_insert_command(cmd_id, drone_id, payload_json, status="RECEIVED"):
    now = datetime.utcnow().isoformat() + "Z"
    with db_lock:
        conn = _db()
        try:
            conn.execute(
                "INSERT INTO commands(cmd_id,drone_id,payload,status,created_at,updated_at)"
                " VALUES (?,?,?,?,?,?)",
                (cmd_id, drone_id, json.dumps(payload_json), status, now, now)
            )
            conn.commit()
            log.info("DB insert: %s  status=%s", cmd_id, status)
            return True
        except sqlite3.IntegrityError:
            log.warning("DB insert blocked (already exists): %s", cmd_id)
            return False
        finally:
            conn.close()

def db_update_status(cmd_id, status, reason=None, exec_result=None):
    now = datetime.utcnow().isoformat() + "Z"
    with db_lock:
        conn = _db()
        try:
            conn.execute(
                "UPDATE commands SET status=?, reason=?, exec_result=?, updated_at=?"
                " WHERE cmd_id=?",
                (
                    status,
                    reason,
                    json.dumps(exec_result) if exec_result is not None else None,
                    now,
                    cmd_id,
                )
            )
            conn.commit()
            log.info("DB update: %s -> %s", cmd_id, status)
        finally:
            conn.close()

def db_get_status(cmd_id):
    with db_lock:
        conn = _db()
        try:
            row = conn.execute(
                "SELECT status FROM commands WHERE cmd_id=?", (cmd_id,)
            ).fetchone()
            return row[0] if row else None
        finally:
            conn.close()

def db_log_event(level, msg, meta=None):
    with db_lock:
        conn = _db()
        try:
            conn.execute(
                "INSERT INTO events(level,msg,meta) VALUES (?,?,?)",
                (level, msg, json.dumps(meta or {}))
            )
            conn.commit()
        finally:
            conn.close()

with db_lock:
    conn = _db()
    try:
        rows = conn.execute(
            "SELECT status, COUNT(*) FROM commands GROUP BY status"
        ).fetchall()
        log.info("DB stats: %s", {r[0]: r[1] for r in rows})
    finally:
        conn.close()

# -----------------------------------------------------------------------------
# MAVLink connection
# -----------------------------------------------------------------------------
log.info("Connecting to PX4 at %s ...", PX4_URL)
mav = mavutil.mavlink_connection(
    PX4_URL,
    source_system=255,
    source_component=190
)
log.info("Waiting for PX4 heartbeat ...")
try:
    mav.wait_heartbeat(timeout=15)
except Exception as e:
    log.warning("Heartbeat timeout or error (expected for some configs): %s", e)
log.info("Heartbeat OK -- target_system=%s target_component=%s",
         mav.target_system, mav.target_component)

if PX4_SYS_ID:
    log.info("target_system override via PX4_SYS_ID=%d (was %d)", PX4_SYS_ID, mav.target_system)
    mav.target_system = PX4_SYS_ID
elif mav.target_system == 0:
    log.warning("target_system=0, forcing to 1")
    mav.target_system = 1
if mav.target_component == 0:
    mav.target_component = 1

# Optional secondary telemetry connection (e.g., separate RX port for multi-port MAVLink)
mav_rx = None
if PX4_RX_URL:
    try:
        log.info("Connecting to PX4 telemetry at %s ...", PX4_RX_URL)
        mav_rx = mavutil.mavlink_connection(PX4_RX_URL, source_system=255, source_component=190)
        mav_rx.wait_heartbeat(timeout=2)
        log.info("PX4 telemetry OK (secondary connection)")
        # Trust the telemetry stream's system ID over the command port's system ID.
        # The command port (udpout) may return sysid=0; the telemetry stream is authoritative.
        if mav_rx.target_system and mav_rx.target_system != mav.target_system:
            log.info("Updating target_system from telemetry: %d → %d",
                     mav.target_system, mav_rx.target_system)
            mav.target_system   = mav_rx.target_system
            mav.target_component = mav_rx.target_component or 1
    except Exception as e:
        log.warning("Failed to establish secondary telemetry connection (%s), will use primary only: %s",
                    PX4_RX_URL, e)
        mav_rx = None

# -----------------------------------------------------------------------------
# Single MAVLink sender thread
# ALL mav.mav.* calls must be enqueued here to avoid C-level race conditions
# -----------------------------------------------------------------------------
_mav_queue = queue.Queue()

# -----------------------------------------------------------------------------
# MAVLink receiver thread — drains the UDP socket continuously so the kernel
# buffer never fills up (213 KB default) and COMMAND_ACK packets are not
# dropped while the sender thread is waiting for an ACK.
# -----------------------------------------------------------------------------
_ack_queue     = queue.Queue()
_home_queue    = queue.Queue()
_mission_queue = queue.Queue()

# RELAY_BT: shared drone state populated by _mav_receiver, published at 1Hz
_state_lock          = threading.Lock()
_state_home_pos      = None   # {lat, lon, alt}
_state_position      = None   # {lat, lon, alt}
_state_battery_pct   = -1     # -1 = unknown
_state_gps_fix_type  = 0
_state_flight_mode   = "UNKNOWN"
_state_first_home_logged = False

_offboard_lock    = threading.Lock()
_offboard_active  = False
_latest_setpoint  = None
_setpoint_first_t = 0.0
_setpoint_last_t  = 0.0


def decode_flight_mode(custom_mode: int) -> str:
    """Decode PX4 custom_mode bits into a human-readable flight mode string."""
    main = (custom_mode >> 16) & 0xFF
    sub  = (custom_mode >> 24) & 0xFF
    table = {
        (1, 0): "MANUAL",
        (2, 0): "ALTCTL",
        (3, 0): "POSCTL",
        (4, 1): "AUTO.READY",
        (4, 2): "AUTO.TAKEOFF",
        (4, 3): "HOLD",
        (4, 4): "AUTO.MISSION",
        (4, 5): "AUTO.RTL",
        (4, 6): "AUTO.LAND",
        (4, 7): "AUTO.RTGS",
        (4, 8): "AUTO.FOLLOW",
        (5, 0): "ACRO",
        (6, 0): "OFFBOARD",
        (7, 0): "STABILIZED",
        (8, 0): "RATTITUDE",
    }
    return table.get((main, sub), f"UNKNOWN({main},{sub})")


def _accept_home_raw(lat_scaled_int, lon_scaled_int) -> bool:
    """
    Return True if the HOME_POSITION message reports a real GPS fix.
    PX4 emits HOME_POSITION with lat=0, lon=0 (both as 1e7-scaled ints) before it
    acquires GPS. Caching that placeholder makes every downstream NED conversion
    silently wrong — the drone is positioned relative to (0, 0) on the equator
    instead of its real launch point. Reject when BOTH lat and lon are zero.
    """
    return bool(lat_scaled_int or lon_scaled_int)


_home_reject_logged = False


def _mav_receiver():
    global _state_home_pos, _state_position, _state_battery_pct
    global _state_gps_fix_type, _state_flight_mode, _state_first_home_logged
    global _home_reject_logged, _offboard_active
    while True:
        try:
            # Always drain one message from primary connection first so ACKs
            # (which PX4 returns to our sending address, not to mav_rx) are
            # never starved by the high-rate mav_rx telemetry stream.
            # Also process SYS_STATUS and HEARTBEAT here since they arrive on
            # the primary connection but not on the telemetry-only mav_rx stream.
            _prim = mav.recv_match(blocking=False)
            if _prim:
                _pt = _prim.get_type()
                if _pt == "COMMAND_ACK":
                    _ack_queue.put(_prim)
                elif _pt == "HOME_POSITION":
                    _home_queue.put(_prim)
                elif _pt == "SYS_STATUS":
                    with _state_lock:
                        _state_battery_pct = int(_prim.battery_remaining)
                elif _pt == "HEARTBEAT":
                    mode_str = decode_flight_mode(int(_prim.custom_mode))
                    with _state_lock:
                        _state_flight_mode = mode_str
                    if mode_str != "OFFBOARD":
                        with _offboard_lock:
                            if _offboard_active:
                                _offboard_active = False
                                log.info("PX4 left OFFBOARD (now %s) — _offboard_active cleared", mode_str)

            # Read telemetry from secondary connection (port 15103) or primary.
            msg = None
            if mav_rx:
                msg = mav_rx.recv_match(blocking=False, timeout=0.1)
            if msg is None:
                msg = mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == "COMMAND_ACK":
                _ack_queue.put(msg)
            elif mt in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"):
                _mission_queue.put(msg)
            elif mt == "HOME_POSITION":
                if not _accept_home_raw(msg.latitude, msg.longitude):
                    if not _home_reject_logged:
                        _home_reject_logged = True
                        log.warning("HOME_POSITION with lat=lon=0 rejected (pre-GPS-fix placeholder)")
                    continue
                _home_queue.put(msg)
                home = {
                    "lat": msg.latitude  / 1e7,
                    "lon": msg.longitude / 1e7,
                    "alt": msg.altitude  / 1000.0,
                }
                with _state_lock:
                    _state_home_pos = home
                    if not _state_first_home_logged:
                        _state_first_home_logged = True
                        log.info("HOME_POSITION first received: %s", home)
            elif mt == "GPS_RAW_INT":
                with _state_lock:
                    _state_gps_fix_type = int(msg.fix_type)
            elif mt == "GLOBAL_POSITION_INT":
                with _state_lock:
                    _state_position = {
                        "lat": msg.lat / 1e7,
                        "lon": msg.lon / 1e7,
                        "alt": msg.alt / 1000.0,
                    }
            elif mt == "SYS_STATUS":
                with _state_lock:
                    _state_battery_pct = int(msg.battery_remaining)
            elif mt == "HEARTBEAT":
                mode_str = decode_flight_mode(int(msg.custom_mode))
                with _state_lock:
                    _state_flight_mode = mode_str
                if mode_str != "OFFBOARD":
                    with _offboard_lock:
                        if _offboard_active:
                            _offboard_active = False
                            log.info("PX4 left OFFBOARD (now %s) — _offboard_active cleared", mode_str)
        except Exception as e:
            log.error("mav_receiver error: %s", e)

threading.Thread(target=_mav_receiver, daemon=True, name="mav-receiver").start()

def _mav_sender():
    while True:
        fn = _mav_queue.get()
        try:
            fn()
        except Exception as e:
            log.error("mav_sender error: %s", e)

threading.Thread(target=_mav_sender, daemon=True, name="mav-sender").start()

def mav_send(fn):
    _mav_queue.put(fn)

def mav_send_sync(fn, timeout=5.0):
    done = threading.Event()
    def wrapped():
        try:
            fn()
        finally:
            done.set()
    _mav_queue.put(wrapped)
    if not done.wait(timeout=timeout):
        log.error("mav_send_sync timed out after %.1fs", timeout)

# -----------------------------------------------------------------------------
# ADDED: GCS heartbeat thread
# PX4 requires a GCS heartbeat to permit and maintain OFFBOARD mode.
# -----------------------------------------------------------------------------
def _gcs_heartbeat():
    while True:
        mav_send(lambda: mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        ))
        time.sleep(1.0)

threading.Thread(target=_gcs_heartbeat, daemon=True, name="gcs-hb").start()
log.info("GCS heartbeat thread started (1 Hz)")

# -----------------------------------------------------------------------------
# ADDED: OFFBOARD setpoint state
# follower_node publishes setpoints to drone/{DRONE_ID}/setpoint at 20 Hz.
# _offboard_sender forwards them to PX4 as SET_POSITION_TARGET_LOCAL_NED.
#
# type_mask 1528 = ignore velocity, acceleration, yaw_rate; use position + yaw.
# PX4 NED: x=North, y=East, z=Down (negative z = above home).
# -----------------------------------------------------------------------------
_SP_TYPE_MASK = 0b10111111000  # 1528: position + yaw, ignore vel/accel/yaw_rate

def _offboard_sender():
    while True:
        time.sleep(0.05)  # 20 Hz
        with _offboard_lock:
            active = _offboard_active
            sp     = _latest_setpoint
            stale  = (time.time() - _setpoint_last_t) > SETPOINT_STALE_S

        if sp is None or stale:
            if active and stale:
                log.warning("OFFBOARD: setpoints stale (%.1fs) -- pausing",
                            time.time() - _setpoint_last_t)
            continue

        x   = float(sp.get("x",   0.0))
        y   = float(sp.get("y",   0.0))
        z   = float(sp.get("z",  -5.0))
        yaw = float(sp.get("yaw", 0.0))

        mav_send(lambda x=x, y=y, z=z, yaw=yaw: mav.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _SP_TYPE_MASK,
            x, y, z,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            yaw, 0.0
        ))

threading.Thread(target=_offboard_sender, daemon=True, name="offboard-sp").start()
log.info("OFFBOARD setpoint sender thread started (20 Hz, inactive until START_LEAD)")

# -----------------------------------------------------------------------------
# MAVLink helpers
# -----------------------------------------------------------------------------
MAV_RESULT = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
}

def wait_ack(label, target_cmd=None, timeout=10):
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining = max(0.0, deadline - time.time())
        try:
            msg = _ack_queue.get(timeout=min(0.5, remaining))
        except queue.Empty:
            continue
        cmd    = int(msg.command)
        result = int(msg.result)
        text   = MAV_RESULT.get(result, f"UNKNOWN_{result}")
        log.info("ACK cmd=%s result=%s(%s)", cmd, result, text)
        if target_cmd is None or cmd == target_cmd:
            return result, text
        log.debug("(ACK for cmd=%s, still waiting for cmd=%s)", cmd, target_cmd)
    log.warning("ACK TIMEOUT for '%s' after %ss", label, timeout)
    return None, "TIMEOUT"

def get_home_alt_amsl(timeout=5.0):
    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0
    ))
    start = time.time()
    while time.time() - start < timeout:
        remaining = max(0.0, timeout - (time.time() - start))
        try:
            msg = _home_queue.get(timeout=min(0.5, remaining))
            return msg.altitude / 1000.0
        except queue.Empty:
            continue
    return None

# -----------------------------------------------------------------------------
# Command implementations (unchanged from working agent)
# -----------------------------------------------------------------------------

def _cmd_rtl():
    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    ))
    res, text = wait_ack("RTL", mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
    # Safety: deactivate OFFBOARD if active
    with _offboard_lock:
        global _offboard_active
        if _offboard_active:
            _offboard_active = False
            log.info("RTL: OFFBOARD deactivated")
    return {"success": res == 0, "detail": f"RTL {text}"}


def _cmd_land():
    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, float('nan'), 0, 0, 0
    ))
    res, text = wait_ack("LAND", mavutil.mavlink.MAV_CMD_NAV_LAND)
    with _offboard_lock:
        global _offboard_active
        if _offboard_active:
            _offboard_active = False
            log.info("LAND: OFFBOARD deactivated")
    return {"success": res == 0, "detail": f"LAND {text}"}


def _cmd_arm():
    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    ))
    res, text = wait_ack("ARM", mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    return {"success": res == 0, "detail": f"ARM {text}"}


def _cmd_disarm():
    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    ))
    res, text = wait_ack("DISARM", mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    return {"success": res == 0, "detail": f"DISARM {text}"}


def _cmd_loiter():
    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0,
        -1,
        0,
        0,
        float('nan'),
        float('nan'),
        float('nan'),
        float('nan'),
    ))
    res, text = wait_ack("LOITER", mavutil.mavlink.MAV_CMD_DO_REPOSITION)
    with _offboard_lock:
        global _offboard_active
        if _offboard_active:
            _offboard_active = False
            log.info("LOITER: OFFBOARD deactivated")
    return {"success": res == 0, "detail": f"LOITER {text}"}


def _cmd_takeoff(params):
    alt = float((params or {}).get("altitude", 10))
    home_amsl = get_home_alt_amsl()
    if home_amsl is None:
        log.warning("Could not get home AMSL, using relative alt")
        target_amsl = alt
    else:
        target_amsl = home_amsl + alt

    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    ))
    wait_ack("ARM", mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

    # NAV_TAKEOFF alone triggers both the mode switch (via commander) and populates
    # the takeoff triplet (via navigator) in the same navigator loop iteration.
    # A prior SET_MODE to AUTO.TAKEOFF causes on_activation() to fire before the
    # triplet is populated, falling back to the 2.5m MIS_TAKEOFF_ALT default.
    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, float('nan'), 0, 0, target_amsl
    ))
    res, text = wait_ack("NAV_TAKEOFF", mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=10)
    return {
        "success": res == 0,
        "detail": f"TAKEOFF climbing to {target_amsl:.1f}m AMSL" if res == 0
                  else f"NAV_TAKEOFF {text} (code={res})"
    }


def _cmd_set_mode(params):
    mode = ((params or {}).get("mode") or "").upper()
    mode_map = {
        "MANUAL":       (1, 0),
        "POSCTL":       (3, 0),
        "OFFBOARD":     (6, 0),
        "HOLD":         (4, 3),
        "AUTO.MISSION": (4, 4),
        "AUTO.RTL":     (4, 5),
        "AUTO.LAND":    (4, 6),
    }
    if mode not in mode_map:
        return {"success": False, "detail": f"unknown mode '{mode}'"}
    main, sub = mode_map[mode]
    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        main, sub, 0, 0, 0, 0
    ))
    res, text = wait_ack(f"SET_MODE {mode}", mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    return {"success": res == 0, "detail": f"SET_MODE {mode} {text}"}


def _cmd_start_mission():
    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    ))
    res, text = wait_ack("START_MISSION", mavutil.mavlink.MAV_CMD_MISSION_START)
    return {"success": res == 0, "detail": f"START_MISSION {text}"}


# -----------------------------------------------------------------------------
# ADDED: START_LEAD -- verify setpoint stream then switch to OFFBOARD
# -----------------------------------------------------------------------------

def _cmd_start_lead():
    global _offboard_active, _state_flight_mode

    with _offboard_lock:
        sp         = _latest_setpoint
        first_t    = _setpoint_first_t
        last_t     = _setpoint_last_t
        already_on = _offboard_active
    log.info("START_LEAD check: sp=%s first_t=%s last_t=%s age=%.2fs duration=%.2fs",
             sp, first_t, last_t,
             time.time() - last_t if last_t else -1,
             (last_t - first_t) if last_t and first_t else -1) 

    if already_on:
        return {"success": True, "detail": "OFFBOARD already active"}

    if sp is None:
        return {
            "success": False,
            "detail": (
                "No setpoints received on drone/{}/setpoint. "
                "Start follower_node first: "
                "DRONE_ID={} LEADER_ID=<other> ros2 run drone_control follower_node"
                .format(DRONE_ID, DRONE_ID)
            )
        }

    stream_age = time.time() - last_t
    if stream_age > SETPOINT_STALE_S:
        return {
            "success": False,
            "detail": "Setpoints stale ({:.1f}s). follower_node must publish continuously.".format(stream_age)
        }

    stream_duration = last_t - first_t
    if stream_duration < OFFBOARD_PRE_SECS:
        return {
            "success": False,
            "detail": "Setpoint stream too short ({:.2f}s, need {:.1f}s). Wait and retry.".format(
                stream_duration, OFFBOARD_PRE_SECS)
        }

    log.info("Setpoints OK: stream=%.2fs  last=%.2fs ago -- arming then switching OFFBOARD",
             stream_duration, stream_age)

    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    ))
    res_arm, text_arm = wait_ack("ARM", mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=5)
    if res_arm != 0:
        log.warning("ARM failed (%s) — retrying with force-arm (SITL/pre-arm bypass)", text_arm)
        mav_send(lambda: mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0  # param2=21196 bypasses pre-arm checks
        ))
        res_arm, text_arm = wait_ack("FORCE_ARM", mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=5)
        if res_arm != 0:
            log.warning("FORCE_ARM also failed: %s (proceeding anyway — may already be armed)", text_arm)

    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6, 0, 0, 0, 0, 0
    ))
    res, text = wait_ack("OFFBOARD", mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=5)

    if res == 0:
        with _offboard_lock:
            _offboard_active = True
        with _state_lock:
            _state_flight_mode = "OFFBOARD"
        log.info("OFFBOARD mode ACTIVE")
        db_log_event("INFO", "offboard_started", {"sp": sp})
        return {"success": True, "detail": "OFFBOARD active: {}".format(text)}
    else:
        log.error("OFFBOARD switch FAILED: %s", text)
        return {
            "success": False,
            "detail": "OFFBOARD mode switch failed: {}. Drone must be armed and airborne.".format(text)
        }


# -----------------------------------------------------------------------------
# ADDED: STOP_LEAD -- deactivate OFFBOARD, switch to HOLD
# -----------------------------------------------------------------------------

def _cmd_stop_lead():
    global _offboard_active

    with _offboard_lock:
        was_active       = _offboard_active
        _offboard_active = False

    log.info("STOP_LEAD: was_active=%s -- switching to HOLD", was_active)

    mav_send(lambda: mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4, 3, 0, 0, 0, 0
    ))
    res, text = wait_ack("HOLD", mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=5)
    db_log_event("INFO", "offboard_stopped", {"was_active": was_active})
    return {
        "success": res == 0,
        "detail": "OFFBOARD stopped (was_active={}), HOLD: {}".format(was_active, text)
    }


# -----------------------------------------------------------------------------
# ADDED: setpoint handler -- called from on_message for /setpoint topic
# -----------------------------------------------------------------------------

def _handle_setpoint(sp):
    global _latest_setpoint, _setpoint_last_t, _setpoint_first_t
    with _offboard_lock:
        if _latest_setpoint is None:
            _setpoint_first_t = time.time()
            log.info("First setpoint received -- stream started")
        _latest_setpoint = sp
        _setpoint_last_t = time.time()


# -----------------------------------------------------------------------------
# UPLOAD_MISSION (unchanged from working agent)
# -----------------------------------------------------------------------------

def _parse_mission_items(content, fmt):
    fmt = (fmt or "").lower().strip()
    if fmt in ("qgc_plan", "plan"):
        return _parse_qgc_plan(content)
    elif fmt in ("qgc_waypoints", "waypoints"):
        return _parse_qgc_waypoints(content)
    elif fmt in ("mavlink_json", "json"):
        return _parse_mavlink_json(content)
    else:
        stripped = content.strip()
        if stripped.startswith("{"):
            try:
                return _parse_qgc_plan(content)
            except Exception:
                return _parse_mavlink_json(content)
        elif stripped.startswith("QGC"):
            return _parse_qgc_waypoints(content)
        elif stripped.startswith("["):
            return _parse_mavlink_json(content)
        raise ValueError("Cannot auto-detect mission format. Set params.format.")


def _parse_qgc_plan(content):
    data    = json.loads(content)
    mission = data.get("mission", data)
    items   = []
    for i, raw in enumerate(mission.get("items", [])):
        p = raw.get("params", [0, 0, 0, 0, 0, 0, 0])
        while len(p) < 7:
            p.append(0)
        lat = float(p[4] or 0)
        lon = float(p[5] or 0)
        alt = float(p[6] or 0)
        items.append({
            "seq":          i,
            "frame":        int(raw.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)),
            "command":      int(raw.get("command", 16)),
            "current":      1 if i == 0 else 0,
            "autocontinue": int(raw.get("autoContinue", 1)),
            "param1":       float(p[0] or 0),
            "param2":       float(p[1] or 0),
            "param3":       float(p[2] or 0),
            "param4":       float(p[3]) if p[3] is not None else float('nan'),
            "x":            int(lat * 1e7),
            "y":            int(lon * 1e7),
            "z":            alt,
        })
    return items


def _parse_qgc_waypoints(content):
    items = []
    for line in content.strip().splitlines():
        line = line.strip()
        if not line or line.startswith("QGC") or line.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) < 12:
            parts = re.split(r'\s+', line)
        if len(parts) < 12:
            log.warning("Skipping malformed waypoint line: %s", line)
            continue
        try:
            seq = int(parts[0])
            items.append({
                "seq":          seq,
                "frame":        int(parts[2]),
                "command":      int(parts[3]),
                "current":      1 if seq == 0 else 0,
                "autocontinue": int(parts[11]) if len(parts) > 11 else 1,
                "param1":       float(parts[4]),
                "param2":       float(parts[5]),
                "param3":       float(parts[6]),
                "param4":       float(parts[7]),
                "x":            int(float(parts[8]) * 1e7),
                "y":            int(float(parts[9]) * 1e7),
                "z":            float(parts[10]),
            })
        except (ValueError, IndexError) as e:
            log.warning("Skipping waypoint line: %s -- %s", line, e)
    for i, item in enumerate(items):
        item["seq"]     = i
        item["current"] = 1 if i == 0 else 0
    return items


def _parse_mavlink_json(content):
    data = json.loads(content)
    if isinstance(data, dict):
        data = data.get("items", [])
    items = []
    for i, raw in enumerate(data):
        items.append({
            "seq":          i,
            "frame":        int(raw.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)),
            "command":      int(raw.get("command", 16)),
            "current":      1 if i == 0 else 0,
            "autocontinue": int(raw.get("autocontinue", 1)),
            "param1":       float(raw.get("param1", 0)),
            "param2":       float(raw.get("param2", 0)),
            "param3":       float(raw.get("param3", 0)),
            "param4":       float(raw.get("param4", float('nan'))),
            "x":            int(raw.get("x", 0)),
            "y":            int(raw.get("y", 0)),
            "z":            float(raw.get("z", 0)),
        })
    return items


def _mission_ack_name(ack_type):
    names = {
        0: "ACCEPTED", 1: "ERROR", 2: "UNSUPPORTED_FRAME",
        3: "UNSUPPORTED", 4: "NO_SPACE", 5: "INVALID",
        6: "INVALID_PARAM1", 7: "INVALID_PARAM2", 8: "INVALID_PARAM3",
        9: "INVALID_PARAM4", 10: "INVALID_PARAM5_X", 11: "INVALID_PARAM6_Y",
        12: "INVALID_PARAM7", 13: "INVALID_SEQUENCE", 14: "DENIED",
        15: "OPERATION_CANCELLED",
    }
    return names.get(ack_type, f"UNKNOWN_{ack_type}")


def _cmd_upload_mission(params):
    file_content = params.get("payload")
    fmt          = params.get("format", "")
    mission_id   = params.get("mission_id", "unknown")

    # Accept inline waypoints list directly (no file encoding needed)
    if not file_content and params.get("waypoints"):
        try:
            items = []
            for i, w in enumerate(params["waypoints"]):
                items.append({
                    "seq":          i,
                    "frame":        int(w.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)),
                    "command":      int(w.get("command", 16)),
                    "current":      1 if i == 0 else 0,
                    "autocontinue": int(w.get("autocontinue", 1)),
                    "param1":       float(w.get("param1", 0)),
                    "param2":       float(w.get("param2", 0)),
                    "param3":       float(w.get("param3", 0)),
                    "param4":       float(w.get("param4", float('nan'))),
                    "x":            int(w["x"] * 1e7),
                    "y":            int(w["y"] * 1e7),
                    "z":            float(w.get("z", 0)),
                })
        except Exception as e:
            return {"success": False, "detail": f"waypoints parse error: {e}"}
    elif not file_content:
        return {"success": False, "detail": "UPLOAD_MISSION: params.payload is empty"}
    else:
        log.info("Parsing mission: id=%s format=%s len=%d",
                 mission_id, fmt, len(file_content))
        try:
            items = _parse_mission_items(file_content, fmt)
        except Exception as e:
            log.error("Mission parse failed: %s", e)
            return {"success": False, "detail": f"Mission parse error: {e}"}

        if not items:
            return {"success": False, "detail": "Mission parsed to 0 items"}

    n = len(items)
    log.info("Mission parsed: %d items", n)

    log.info("Sending MISSION_COUNT(%d)", n)
    mav_send_sync(lambda: mav.mav.mission_count_send(
        mav.target_system,
        mav.target_component,
        n,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    ))

    uploaded = 0
    while uploaded < n:
        msg = None
        for attempt in range(MISSION_ITEM_RETRIES):
            try:
                msg = _mission_queue.get(timeout=MISSION_ITEM_TIMEOUT)
            except queue.Empty:
                msg = None
            if msg:
                break
            log.warning("No MISSION_REQUEST_INT (attempt %d/%d)",
                        attempt + 1, MISSION_ITEM_RETRIES)
            if attempt < MISSION_ITEM_RETRIES - 1:
                mav_send_sync(lambda: mav.mav.mission_count_send(
                    mav.target_system,
                    mav.target_component,
                    n,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                ))

        if msg is None:
            return {"success": False,
                    "detail": f"Timeout waiting for MISSION_REQUEST_INT after item {uploaded}"}

        if msg.get_type() == "MISSION_ACK":
            ack_type = int(msg.type)
            if ack_type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                return {"success": True,
                        "detail": f"Mission uploaded: {uploaded}/{n} items",
                        "waypoints_uploaded": uploaded}
            err = _mission_ack_name(ack_type)
            return {"success": False, "detail": f"PX4 rejected mission: {err}",
                    "waypoints_uploaded": uploaded}

        seq = int(msg.seq)
        if seq >= n:
            return {"success": False, "detail": f"PX4 requested out-of-range seq={seq}"}

        item  = items[seq]
        _item = dict(item)
        log.info("Sending item seq=%d cmd=%d", seq, item["command"])
        mav_send_sync(lambda i=_item: mav.mav.mission_item_int_send(
            mav.target_system, mav.target_component,
            i["seq"], i["frame"], i["command"],
            i["current"], i["autocontinue"],
            i["param1"], i["param2"], i["param3"], i["param4"],
            i["x"], i["y"], i["z"],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        ))
        uploaded = seq + 1

    log.info("All %d items sent, waiting for MISSION_ACK ...", n)
    for attempt in range(MISSION_ITEM_RETRIES):
        try:
            msg = _mission_queue.get(timeout=MISSION_ITEM_TIMEOUT)
        except queue.Empty:
            msg = None
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "MISSION_ACK":
            ack_type = int(msg.type)
            if ack_type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                log.info("Mission upload ACCEPTED: %d items", n)
                return {"success": True,
                        "detail": f"Mission uploaded: {n} waypoints",
                        "waypoints_uploaded": n,
                        "mission_id": mission_id}
            err = _mission_ack_name(ack_type)
            return {"success": False, "detail": f"PX4 rejected mission: {err}",
                    "waypoints_uploaded": n}
        elif mt in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            # PX4 re-requesting an item — resend it and keep waiting.
            seq = int(msg.seq)
            if seq < n:
                item  = items[seq]
                _item = dict(item)
                log.info("Re-sending item seq=%d cmd=%d (PX4 re-request)", seq, item["command"])
                mav_send_sync(lambda i=_item: mav.mav.mission_item_int_send(
                    mav.target_system, mav.target_component,
                    i["seq"], i["frame"], i["command"],
                    i["current"], i["autocontinue"],
                    i["param1"], i["param2"], i["param3"], i["param4"],
                    i["x"], i["y"], i["z"],
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                ))

    return {"success": False,
            "detail": "Timeout waiting for final MISSION_ACK",
            "waypoints_uploaded": n}


# -----------------------------------------------------------------------------
# Command dispatcher
# ADDED: START_LEAD, STOP_LEAD
# -----------------------------------------------------------------------------

def send_mavlink_command(cmd_payload):
    cmd    = (cmd_payload.get("cmd") or cmd_payload.get("action") or "").upper()
    params = cmd_payload.get("params") or {}
    log.info("Executing: cmd=%s  params=%s", cmd, params)
    try:
        if cmd == "RTL":               return _cmd_rtl()
        if cmd in ("LOITER", "HOLD"):  return _cmd_loiter()
        if cmd == "TAKEOFF":           return _cmd_takeoff(params)
        if cmd == "LAND":              return _cmd_land()
        if cmd == "ARM":               return _cmd_arm()
        if cmd == "DISARM":            return _cmd_disarm()
        if cmd == "SET_MODE":          return _cmd_set_mode(params)
        if cmd == "UPLOAD_MISSION":    return _cmd_upload_mission(params)
        if cmd == "START_MISSION":     return _cmd_start_mission()
        if cmd == "START_LEAD":        return _cmd_start_lead()   # ADDED
        if cmd == "STOP_LEAD":         return _cmd_stop_lead()    # ADDED
        log.warning("Unsupported command: %s", cmd)
        return {"success": False, "detail": f"unsupported cmd: {cmd}"}
    except Exception as e:
        log.exception("Exception in send_mavlink_command")
        return {"success": False, "detail": f"exception: {e}"}

# -----------------------------------------------------------------------------
# MQTT
# -----------------------------------------------------------------------------
mqtt_client = mqtt.Client(
    client_id=f"px4-agent-{DRONE_ID}",
    clean_session=True
)

if MQTT_USER and MQTT_PASS:
    log.info("MQTT credentials: user=%s", MQTT_USER)
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
else:
    log.warning("MQTT credentials NOT set -- broker may reject connection")

cloud_link_ok  = True
last_heartbeat = 0.0

def publish_ack(cmd_id, status, exec_result=None, reason=None):
    msg = {
        "cmd_id":   cmd_id,
        "drone_id": DRONE_ID,
        "status":   status,
        "ts_utc":   datetime.utcnow().isoformat() + "Z",
    }
    if exec_result is not None:
        msg["exec_result"] = exec_result
    if reason:
        msg["reason"] = reason
    mqtt_client.publish(f"drone/{DRONE_ID}/cmd_ack", json.dumps(msg), qos=1)
    log.info("Published state: status=%s  cmd_id=%s", status, cmd_id)

def on_connect(client, userdata, flags, rc):
    log.info("MQTT on_connect rc=%s", rc)
    if rc != 0:
        log.error("MQTT connection failed rc=%s", rc)
        return
    client.subscribe(f"drone/{DRONE_ID}/cmd", qos=1)
    log.info("Subscribed to drone/%s/cmd", DRONE_ID)
    # ADDED: setpoint subscription (QoS 0 -- latency over reliability)
    client.subscribe(f"drone/{DRONE_ID}/setpoint", qos=0)
    log.info("Subscribed to drone/%s/setpoint (QoS 0)", DRONE_ID)
    if ENABLE_CLOUD_LINK_CHECK:
        client.subscribe(f"drone/{DRONE_ID}/link", qos=1)
        log.info("Subscribed to drone/%s/link", DRONE_ID)

def on_message(client, userdata, msg):
    global last_heartbeat, cloud_link_ok
    topic = msg.topic
    try:
        payload = msg.payload.decode("utf-8")
        j = json.loads(payload)
    except Exception as e:
        log.error("Invalid payload on %s: %s", topic, e)
        return

    # ADDED: setpoint -- high frequency, no logging
    if topic.endswith("/setpoint"):
        _handle_setpoint(j)
        return

    if topic.endswith("/link"):
        last_heartbeat = time.time()
        if not cloud_link_ok:
            cloud_link_ok = True
            log.info("Cloud link restored")
        return

    if topic.endswith("/cmd"):
        # Dispatch to a thread immediately so on_message never blocks the MQTT loop.
        # All DB writes and MAVLink waits happen inside the thread.
        cmd_id   = j.get("cmd_id")   or j.get("command_id")
        drone_id = j.get("drone_id") or j.get("target_id")
        if not cmd_id or not drone_id or drone_id != DRONE_ID:
            return
        threading.Thread(
            target=_handle_cmd, args=(j, payload),
            daemon=True, name=f"cmd-{cmd_id[:8]}"
        ).start()

def _handle_cmd(j, payload):
    """Full command intake + processing, run in its own thread to keep on_message non-blocking."""
    cmd_id   = j.get("cmd_id")   or j.get("command_id")
    drone_id = j.get("drone_id") or j.get("target_id")
    _cmd_check = (j.get("cmd") or j.get("action") or "").upper()
    if _cmd_check == "UPLOAD_MISSION":
        _log_j = {**j, "params": {**j.get("params", {}),
                  "payload": f"[{len(j.get('params', {}).get('payload', ''))} chars]"}}
        log.info("Message on drone/%s/cmd: %s", DRONE_ID, json.dumps(_log_j))
    else:
        log.info("Message on drone/%s/cmd: %s", DRONE_ID, payload)

    log.info("cmd_id=%s  drone_id=%s  format=%s",
             cmd_id, drone_id,
             "Lambda" if ("action" in j or "command_id" in j) else "Legacy")

    if command_exists(cmd_id):
        publish_ack(cmd_id, db_get_status(cmd_id) or "DUPLICATE")
        return
    if ENABLE_CLOUD_LINK_CHECK:
        if time.time() - last_heartbeat > HEARTBEAT_TIMEOUT:
            global cloud_link_ok
            cloud_link_ok = False
        if not cloud_link_ok:
            log.warning("Cloud link down -- rejecting %s", cmd_id)
            db_insert_command(cmd_id, DRONE_ID, j, status="REJECTED")
            publish_ack(cmd_id, "REJECTED", reason="link_down")
            db_log_event("WARN", "rejected_link_down", {"cmd_id": cmd_id})
            return

    if not db_insert_command(cmd_id, DRONE_ID, j, status="RECEIVED"):
        return

    publish_ack(cmd_id, "RECEIVED")
    db_log_event("INFO", "cmd_received", {"cmd_id": cmd_id})
    process_command(j)


def process_command(j):
    cmd_id = j.get("cmd_id") or j.get("command_id")
    cmd    = (j.get("cmd") or j.get("action") or "").upper()
    log.info("Processing: %s", cmd_id)

    known = {
        "RTL", "LOITER", "HOLD", "SET_MODE", "LAND", "ARM", "DISARM",
        "TAKEOFF", "UPLOAD_MISSION", "START_MISSION",
        "START_LEAD", "STOP_LEAD",   # ADDED
    }
    if cmd not in known:
        log.warning("Unknown command '%s' -- rejecting %s", cmd, cmd_id)
        db_update_status(cmd_id, "REJECTED", reason=f"unsupported cmd: {cmd}")
        publish_ack(cmd_id, "REJECTED", reason=f"unsupported cmd: {cmd}")
        db_log_event("WARN", "rejected_unknown_cmd", {"cmd_id": cmd_id, "cmd": cmd})
        return

    try:
        result = send_mavlink_command(j)
        if result.get("success"):
            db_update_status(cmd_id, "EXECUTED", exec_result=result)
            publish_ack(cmd_id, "EXECUTED", exec_result=result)
            db_log_event("INFO", "cmd_executed", {"cmd_id": cmd_id, "result": result})
        else:
            db_update_status(cmd_id, "FAILED", reason="send_failed", exec_result=result)
            publish_ack(cmd_id, "FAILED", exec_result=result, reason="send_failed")
            db_log_event("ERROR", "send_failed", {"cmd_id": cmd_id, "detail": result})
    except Exception as e:
        log.exception("Exception processing %s", cmd_id)
        db_update_status(cmd_id, "FAILED", reason=str(e))
        publish_ack(cmd_id, "FAILED", reason=str(e))

def _cloud_hb_watcher():
    global cloud_link_ok
    if not ENABLE_CLOUD_LINK_CHECK:
        log.info("Cloud heartbeat watcher disabled")
        return
    while True:
        if time.time() - last_heartbeat > HEARTBEAT_TIMEOUT:
            if cloud_link_ok:
                cloud_link_ok = False
                log.warning("Cloud heartbeat lost")
                db_log_event("WARN", "heartbeat_lost",
                             {"since": datetime.utcnow().isoformat() + "Z"})
        time.sleep(1)

threading.Thread(target=_cloud_hb_watcher, daemon=True, name="cloud-hb").start()

# RELAY_BT: 1Hz drone_state publisher on drone/<drone_id>/state
_state_skip_count = 0

def _publish_drone_state():
    global _state_skip_count
    while True:
        time.sleep(1.0)
        with _state_lock:
            pos     = _state_position
            bat     = _state_battery_pct
            home    = _state_home_pos
            fix     = _state_gps_fix_type
            mode    = _state_flight_mode

        if pos is None or bat == -1:
            _state_skip_count += 1
            if _state_skip_count > 5:
                log.warning(
                    "drone_state publish skipped: pos=%s battery=%s",
                    pos is not None, bat != -1
                )
            continue

        _state_skip_count = 0
        state = {
            "drone_id":     DRONE_ID,
            "timestamp":    time.time(),
            "battery_pct":  bat,
            "gps_fix_type": fix,
            "flight_mode":  mode,
            "position":     pos,
            "home_pos":     home,   # null until PX4 reports a real GPS-fix home
            "avg_speed_ms": 8.0,
            "endurance_s":  1200,
        }
        try:
            mqtt_client.publish(
                f"drone/{DRONE_ID}/state",
                json.dumps(state),
                qos=0
            )
        except Exception as exc:
            log.error("drone_state publish error: %s", exc)

threading.Thread(target=_publish_drone_state, daemon=True, name="drone-state-pub").start()
log.info("RELAY_BT drone_state publisher started (1Hz) on drone/%s/state", DRONE_ID)

# -----------------------------------------------------------------------------
# Start
# -----------------------------------------------------------------------------
def on_disconnect(client, userdata, rc):
    if rc == 0:
        log.info("MQTT clean disconnect")
    else:
        log.warning("MQTT unexpected disconnect rc=%s -- will reconnect", rc)

mqtt_client.on_connect    = on_connect
mqtt_client.on_disconnect = on_disconnect
mqtt_client.on_message    = on_message
mqtt_client.connect(MQTT_HOST, MQTT_PORT)

log.info("px4_agent started -- drone=%s  px4=%s  mqtt=%s:%s",
         DRONE_ID, PX4_URL, MQTT_HOST, MQTT_PORT)
mqtt_client.loop_forever()
