"""
geometry.py — pure relay positioning math.

BUILDSPEC: §4.12
LAYER:     1 (pure functions — no I/O, no ROS2, no side effects)
SUBSCRIBES: none
PUBLISHES:  none

Hard rules this file must satisfy (BUILDSPEC §4.12 / §7.1):
  - estimate_battery_cost() reads cruise_speed_mps and
    consumption_rate_pct_per_s from model_cfg; if either is an _Unresolved
    sentinel, arithmetic raises RuntimeError — no default, no fallback
    -> proven by test_battery_cost_raises_without_model_constants
  - return_margin_buffer_pct is a FLAT additive (cost + 10.0), not a
    multiplier — proven by test_return_margin_arithmetic
  - All other functions unchanged from pre-Wave-4 (§4.12 "unchanged functions")
"""

import math


_EARTH_R = 6_371_000.0  # metres


# ── Primitives ────────────────────────────────────────────────────────────────

def haversine(pos_a: dict, pos_b: dict) -> float:
    """Great-circle distance in metres between two {lat, lon} dicts. Altitude ignored."""
    lat1, lon1 = math.radians(pos_a["lat"]), math.radians(pos_a["lon"])
    lat2, lon2 = math.radians(pos_b["lat"]), math.radians(pos_b["lon"])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * _EARTH_R * math.asin(math.sqrt(a))


def effective_radio_range(nominal_range_m: float, jamming_severity: float,
                          jamming_severity_factor: float = 0.6) -> float:
    """
    Returns nominal_range * (1 - severity * factor).
    Severity 0.0 = no reduction; 1.0 = factor-fraction reduction.
    jamming_severity_factor defaults to 0.6 for backward compat; callers should
    read the named constant from config and pass it explicitly.
    """
    return nominal_range_m * (1.0 - jamming_severity * jamming_severity_factor)


def estimate_battery_cost(current_pos: dict, home_pos: dict, battery_pct: float,
                          model_cfg: dict, cfg: dict) -> tuple:
    """
    Returns (return_margin_ok, required_pct, cost_pct).

    cost_pct     = haversine(current, home) / cruise_speed_mps
                   * consumption_rate_pct_per_s
    required_pct = cost_pct + return_margin_buffer_pct   # FLAT additive, §4.12
    return_margin_ok = (battery_pct >= required_pct)

    HARD RULE (BUILDSPEC §7.1): model_cfg["cruise_speed_mps"] and
    model_cfg["consumption_rate_pct_per_s"] are _Unresolved sentinels until the
    airframe is decided.  Any arithmetic with them raises RuntimeError.
    Do NOT catch that error here — let it propagate to the caller.
    Do NOT supply a default value.  A confident-looking wrong number silently
    corrupts every return_margin_ok check downstream.
    """
    distance_m = haversine(current_pos, home_pos)

    # §7.1: these lines raise RuntimeError if either constant is _Unresolved.
    time_s   = distance_m / model_cfg["cruise_speed_mps"]
    cost_pct = time_s * model_cfg["consumption_rate_pct_per_s"]

    # Flat additive buffer — not a percentage-of-cost multiplier (§4.12).
    buffer       = float(cfg.get("return_margin_buffer_pct", 10.0))
    required_pct = cost_pct + buffer

    return battery_pct >= required_pct, required_pct, cost_pct


def point_in_polygon(point: dict, polygon: list) -> bool:
    """
    Ray casting algorithm. polygon is list of {lat, lon} vertices.
    Returns True if point is strictly inside polygon.
    """
    px, py = point["lon"], point["lat"]
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]["lon"], polygon[i]["lat"]
        xj, yj = polygon[j]["lon"], polygon[j]["lat"]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


# ── Dual-range relay feasibility (replaces single-range versions) ─────────────

def relay_is_feasible(gc_pos: dict, leader_pos: dict,
                      gc_radio_range_m: float, leader_radio_range_m: float,
                      margin: float = 0.85) -> tuple:
    """
    Returns (feasible: bool, reason: str).
    Feasible when the two margin-reduced reach circles overlap:
      (gc_range * margin) + (leader_range * margin) > haversine(gc, leader)
    """
    dist = haversine(gc_pos, leader_pos)
    max_span = (gc_radio_range_m * margin) + (leader_radio_range_m * margin)
    if dist < max_span:
        return True, f"feasible: D={dist:.0f}m < span={max_span:.0f}m"
    return False, f"infeasible: D={dist:.0f}m > span={max_span:.0f}m"


def compute_relay_position(gc_pos: dict, leader_pos: dict,
                           gc_radio_range_m: float, leader_radio_range_m: float,
                           margin: float = 0.85) -> dict:
    """
    Returns {lat, lon, alt} — the optimal relay point on the GC→Leader line.
    Uses band_center() of the feasible intersection band.
    If already within combined range, returns gc_pos.
    Altitude is set to leader_pos["alt"].
    """
    dist = haversine(gc_pos, leader_pos)
    max_span = (gc_radio_range_m * margin) + (leader_radio_range_m * margin)
    if dist <= 0 or dist >= max_span:
        return dict(gc_pos)

    r_G = gc_radio_range_m * margin
    r_L = leader_radio_range_m * margin
    t_lo, t_hi = band_bounds(dist, r_G, r_L)
    pos = band_center(t_lo, t_hi, gc_pos, leader_pos)
    pos["alt"] = leader_pos.get("alt", gc_pos.get("alt", 0.0))
    return pos


# ── Band pipeline (§2 positioning) ───────────────────────────────────────────

def gap_distance(gc_pos: dict, leader_pos: dict) -> float:
    """Full gap D = haversine(GC, Leader)."""
    return haversine(gc_pos, leader_pos)


def follower_reach_per_hop(cap_follower_m: float,
                           cap_gc_m: float,
                           cap_leader_m: float,
                           jamming_severity: float,
                           jamming_severity_factor: float = 0.6) -> tuple:
    """
    Returns (r_G, r_L) — follower's effective reach toward each endpoint.
    Follower radio is the spine: each hop is limited by min(follower, far_endpoint).
    """
    r_G = effective_radio_range(min(cap_follower_m, cap_gc_m),
                                jamming_severity, jamming_severity_factor)
    r_L = effective_radio_range(min(cap_follower_m, cap_leader_m),
                                jamming_severity, jamming_severity_factor)
    return r_G, r_L


def band_bounds(D: float, r_G: float, r_L: float) -> tuple:
    """
    Returns (t_lo, t_hi) as line fractions along the GC→Leader line.
    Feasible when t_lo <= t_hi; inverts exactly when r_G + r_L < D.
    The inversion is the EXIT signal — callers read it as infeasible.
    """
    if D <= 0:
        return 0.0, 1.0
    t_hi = r_G / D       # GC-side reach as fraction of D
    t_lo = 1.0 - r_L / D  # Leader-side reach as fraction from GC
    return t_lo, t_hi


def band_center(t_lo: float, t_hi: float, gc_pos: dict, leader_pos: dict) -> dict:
    """
    Max-margin relay position: geometric midpoint of the feasible band
    interpolated on the GC→Leader line. No depth-fraction constant.
    """
    t = (t_lo + t_hi) / 2.0
    lat = gc_pos["lat"] + t * (leader_pos["lat"] - gc_pos["lat"])
    lon = gc_pos["lon"] + t * (leader_pos["lon"] - gc_pos["lon"])
    alt = leader_pos.get("alt", gc_pos.get("alt", 0.0))
    return {"lat": lat, "lon": lon, "alt": alt}


def predicted_inside_band(t_now: float, t_lo: float, t_hi: float) -> bool:
    """
    True if the current follower position (expressed as fraction t_now along
    the GC→Leader line) falls inside the feasible band.
    DIAGNOSTIC ONLY — never consume in a gate or arbiter rule.
    """
    return t_lo <= t_now <= t_hi


def bucket_position(R_target: dict, bucket_m: float) -> dict:
    """
    Snap R_target to a grid of bucket_m size (Decision 5: target-stability).
    Larger than meaningful position variation so sub-bucket drift never forces re-auth.
    Pure, deterministic, no I/O.
    """
    lat = R_target["lat"]
    lon = R_target["lon"]
    alt = R_target.get("alt", 0.0)

    m_per_deg_lat = 111_000.0
    cos_lat = math.cos(math.radians(lat))
    m_per_deg_lon = 111_000.0 * cos_lat if cos_lat > 1e-9 else 111_000.0

    bucket_lat = bucket_m / m_per_deg_lat
    bucket_lon = bucket_m / m_per_deg_lon

    snapped_lat = round(lat / bucket_lat) * bucket_lat
    snapped_lon = round(lon / bucket_lon) * bucket_lon

    return {"lat": snapped_lat, "lon": snapped_lon, "alt": alt}
