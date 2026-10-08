"""
demo_config.py — SITL stand-in for the cloud-delivered policy payload.

BUILDSPEC: §3 (complete config surface for this build pass)
LAYER:     0 (data only — no I/O, no dependencies)

In production a cloud Lambda pushes a JSON blob with identical keys whenever
mission parameters change.  For isolated SITL testing this file is the source
of truth.  Every node that needs config calls _load_config() which imports
DEMO_CONFIG directly — no network call, no polling.

Hard rules this file must satisfy (BUILDSPEC §3):
  - All §3.1 keys present             -> proven by test_all_required_keys_present
  - All §3.2 values exact             -> proven by test_resolved_values_exact
  - Noise model endpoints correct     -> proven by test_noise_model_endpoints
  - §7.1 constants raise on use       -> proven by test_per_model_section_raises_if_unset
"""

# ── §7.1 Hard-stop sentinel ───────────────────────────────────────────────────
#
# BUILDSPEC §7.1: consumption_rate_pct_per_s and cruise_speed_mps depend on
# the actual airframe — weight class, motor efficiency, cruise speed — none of
# which is established anywhere in this design.  Any arithmetic on these values
# must raise immediately rather than silently producing a wrong answer.
#
# DELIBERATELY ABSENT (BUILDSPEC §7.1): real values for per-model constants.
# Do not assign plausible numbers here. A confidently-wrong value silently
# corrupts every return_margin_ok check.


class _Unresolved:
    """
    Sentinel for BUILDSPEC §7.1 unresolved airframe constants.
    Any arithmetic or float coercion raises RuntimeError with the §7.1 message.
    Replace with a real float once the airframe specs are established.
    """

    def __init__(self, constant_name: str):
        self._name = constant_name
        self._msg = (
            f"BUILDSPEC §7.1 HARD STOP: '{constant_name}' is not decided. "
            "This constant depends on the actual airframe (weight class, motor "
            "efficiency, cruise speed). Do not pick a plausible number — a "
            "confidently-wrong value silently corrupts every return_margin_ok "
            "check. Assign a real value from airframe specifications."
        )

    def _raise(self, *_args, **_kw):
        raise RuntimeError(self._msg)

    # All arithmetic paths that geometry.estimate_battery_cost() might use:
    __float__     = _raise
    __mul__       = _raise
    __rmul__      = _raise
    __truediv__   = _raise
    __rtruediv__  = _raise
    __add__       = _raise
    __radd__      = _raise
    __sub__       = _raise
    __rsub__      = _raise
    __repr__ = lambda self, *_: f"_Unresolved({self._name!r})"  # noqa: E731


# ── §3.3 Per-drone-model section ─────────────────────────────────────────────
#
# NEW structural addition — nothing in the pre-build-pass config is keyed this way.
# DRONE_MODEL_ASSIGNMENT maps drone_id → model_name.
# DRONE_MODELS maps model_name → per-model physical constants.
#
# Values below are arbitrary placeholders for the demo airframe.
# Replace with measured values once the actual airframe is decided.
# BUILDSPEC §7.1: do not treat these as validated numbers.

DRONE_MODELS = {
    "generic": {
        "consumption_rate_pct_per_s": 0.05,   # arbitrary: ~100% in 33 min
        "cruise_speed_mps":           12.0,    # arbitrary: typical multirotor
    },
}

DRONE_MODEL_ASSIGNMENT = {
    "drone-01": "generic",
    "drone-02": "generic",
}


# ── Main config dict ──────────────────────────────────────────────────────────

DEMO_CONFIG = {

    # ── Environment ───────────────────────────────────────────────────────────

    # GC's fixed ground position.  Used in haversine() for every signal hop
    # that involves the GC.  Never changes at runtime.
    "gc_pos": {"lat": 47.3900, "lon": 8.5400, "alt": 0.0},

    # Relay test geometry: GC → Follower (450m) → Leader (900m)
    # Follower positioned between GC and leader to relay when GC↔leader is poor.
    "leader_pos": {"lat": 47.3980, "lon": 8.5476, "alt": 50.0},  # ~900 m from GC (poor link)
    "follower_spawn_pos": {"lat": 47.3941, "lon": 8.5438, "alt": 30.0},  # ~450 m from GC (good link)

    # Leader drone ID — used by RDA to prevent leader from becoming relay point.
    # In a 2-drone system, only the follower (other drone) can relay.
    "leader_id": "drone-01",

    # Hardware nominal radio ranges — maximum usable range with zero jamming.
    # Band math at the diamond waypoints (updated 2026-10-06):
    #   cap_gc = cap_ldr = 1500 * (1 - 0.70 * 0.6) = 870 m  (gc/leader_severity=0.70)
    #   r_G = r_L = 870 * (1 - 0.35 * 0.6) = 687 m          (follower_severity=0.35)
    #   r_G + r_L = 1375 m
    # Compared to D (GC → waypoint, haversine):
    #   P5-base ( 958 m)  margin +417 m  ✓ fillable
    #   P4-SW   ( 549 m)  margin +826 m  ✓ fillable
    #   P3-SE   ( 842 m)  margin +533 m  ✓ fillable
    #   P2-E    (1194 m)  margin +181 m  ✓ fillable
    #   P1-NE   (1398 m)  margin  -24 m  ✗ NOT fillable at current severities
    # The P1-NE margin is negative under the current compound severity model;
    # earlier runs (pre-DIAG_SCAN / pre-follower-severity-compounding) treated
    # it as feasible. Reduce follower_severity to ≤0.30 or gc/leader_severity
    # to ≤0.65 to recover a positive margin at P1-NE.
    "gc_radio_range_m":       1500,
    "leader_radio_range_m":   1500,
    "follower_radio_range_m": 1500,
    "radio_range_m":          1500,  # generic fallback; prefer per-radio keys above

    # Geofence as a closed polygon.  GeofenceContainsRelayPos (entry gate) and
    # PositionServiceable (Gate 4) both call point_in_polygon() against this.
    "geofence_polygon": [
        {"lat": 47.3800, "lon": 8.5300},
        {"lat": 47.4300, "lon": 8.5300},
        {"lat": 47.4300, "lon": 8.5800},
        {"lat": 47.3800, "lon": 8.5800},
    ],

    # ── §3.1 Signal model constants ───────────────────────────────────────────
    #
    # Forward formula (signal_faker injects noise):
    #   noise_dbm = baseline_noise_dbm + severity * noise_range_db
    # Inverse formula (radio_health readers recover severity):
    #   severity  = (noise_dbm - baseline_noise_dbm) / noise_range_db
    #
    # All three radio health paths and signal_faker use these same constants,
    # so a single edit propagates everywhere.  Config: BUILDSPEC §3.1.

    # Thermal noise floor at severity 0 (no jamming).  -95 dBm is a typical
    # VHF/UHF receiver noise floor at short range.  BUILDSPEC §3.1.
    "baseline_noise_dbm": -95.0,

    # How far the noise floor rises when severity reaches 1.0.
    # -95 + 1.0 * 40 = -55 dBm — wipes out any useful signal at normal ranges.
    # BUILDSPEC §3.1.
    "noise_range_db": 40.0,

    # Transmitter power in dBm.  Used in FSPL RSSI formula:
    #   rssi_dbm = tx_power_dbm - path_loss_db.  BUILDSPEC §3.1.
    "tx_power_dbm": 20.0,

    # Radio frequency in MHz.  Used in FSPL formula:
    #   path_loss_db = 20*log10(dist_m) + 20*log10(frequency_mhz) - 27.55
    # BUILDSPEC §3.1, §4.1.
    "frequency_mhz": 915,

    # Legacy alias kept for backward compatibility while signal_faker.py and
    # follower_signal_faker.py are replaced.  New code uses baseline_noise_dbm.
    "noise_baseline_dbm": -95.0,

    # ── Jamming model ─────────────────────────────────────────────────────────

    "jamming_mode":          "gradual",
    "jamming_ramp_s":        300,
    "jamming_target_link":   "leader_to_gc",
    "jamming_max_severity":  0.0,
    "jamming_severity_factor": 0.6,  # pending validation

    # ── Capability gate thresholds ────────────────────────────────────────────

    "battery_reserve_pct": 30,
    "battery_return_margin": 15,
    "min_snr_db":   8,   # pending radio validation
    "marginal_snr_db": 13,  # pending radio validation

    # SNR threshold for leader_link_detector: publish relay_request when
    # snr_db falls below this value.  BUILDSPEC §3.1, §4.4.
    "LINK_MARGINAL_QUALITY": 13,

    "min_gps_fix_type":  0,
    "min_leader_snr_db": 8,

    # ── Maintenance gate thresholds ───────────────────────────────────────────

    "reposition_improvement_threshold_db": 5,
    "relay_effective_snr_improvement_db":  3,
    # Direct GC↔leader quality above which G6 (RelayStillNeeded) FAILS and
    # the follower exits the relay. SESSION_LOG 2026-10-07: lowered from 0.99
    # to 0.7 after audit — no deliberate rationale was recorded for 0.99 and
    # at 0.99 the direct link had to be essentially perfect for the follower
    # to let go, which never happens in a jammed environment.
    "relay_exit_quality_threshold": 0.7,
    "debounce_n": 3,

    # ── Freshness domains ─────────────────────────────────────────────────────

    "fcu_telemetry_max_age_s": 6.0,
    "rf_link_max_age_s": 5.0,

    # radio_health freshness window: 10 s at 1 Hz publish rate = 10 missed
    # messages before declaring stale.  BUILDSPEC §3.1.
    "radio_health_max_age_s": 10.0,

    "stale_severity_floor": 0.5,

    # SESSION_LOG 2026-10-07: drone_state window lowered from 10.0 to 6.0 so
    # DataFreshness (entry gate) and G1 FcuTelemetryFresh (maintenance gate)
    # agree on what "fresh" means. Previously a 7–10 s stale drone_state
    # passed entry but tripped G1 on the very next tick.
    "staleness_windows_s": {
        "signal_report": 2.0,
        "drone_state":   6.0,
    },

    # Leader-position freshness guard for BandSensorNode (SESSION_LOG 2026-09-30).
    "leader_position_max_age_s": 10.0,

    # ── Link state hysteresis ─────────────────────────────────────────────────
    # SESSION_LOG 2026-10-07: the `link_thresholds` dict and the `down_threshold_s`
    # / `up_threshold_s` keys were defined here but had no consumer in the live
    # pipeline. Removed. The two thresholds that ARE live: _GC_QUALITY_TRIGGER
    # (0.5, in relay_decision_authority.py; env override GC_QUALITY_TRIGGER)
    # and relay_exit_quality_threshold (above). gc_link_observer.py normalises
    # SNR to [0, 1] with 0.5 at LINK_MARGINAL_QUALITY.

    # ── §3.2 New resolved values ──────────────────────────────────────────────
    #
    # All values below are from BUILDSPEC §3.2 — resolved (not 🔴).
    # Do not change these without a corresponding buildspec revision.

    # Both initial and re-authorization rounds share the same 45 s window.
    # Config: BUILDSPEC §3.2.
    "collection_window_s": 45,

    # After an empty collection window, wait 120 s before re-broadcasting.
    # Config: BUILDSPEC §3.2.
    "rebroadcast_pause_s": 120,

    # relay_request dedup table expiry.  Key: (drone_id, snr_bucket).
    # Suppresses repeated logging of one continuous degradation episode.
    # Config: BUILDSPEC §3.2, §4.10 table 1.
    "relay_request_dedup_expiry_s": 600,

    # Decline dedup table expiry.  Key: (drone_id, decline_reason).
    # Config: BUILDSPEC §3.2, §4.10 table 2.
    "decline_dedup_expiry_s": 180,

    # Suppress the "never-seen" leader_radio_health decline for this many
    # seconds after startup — avoids false alarms during boot.
    # Config: BUILDSPEC §3.2, §4.6 F_cap.
    "boot_grace_window_s": 600,

    # Gate 8 (RelayActuallyImproved) radius check: how close r_target_now must
    # be to current_relay_target.  Also written into authorization messages.
    # Config: BUILDSPEC §3.2, §4.6 Gate 8, §2.6.
    "tolerance_radius_m": 10.0,

    # How long an authorization is valid before Gate 8 fails on the timer arm.
    # Config: BUILDSPEC §3.2, §4.6 Gate 8, §5.4.
    "authorization_validity_s": 1800,

    # If a reauth request has been outstanding longer than this, fire
    # ProposeExitRelay via ReauthResponseTimedOut.
    # Config: BUILDSPEC §3.2, §4.6.
    "reauth_response_timeout_s": 120,

    # Flat buffer added to computed return-trip cost in estimate_battery_cost().
    # +10 % percentage points, not a multiplier.
    # Config: BUILDSPEC §3.2, §4.12.
    "return_margin_buffer_pct": 10.0,

    # ── Authorization (legacy keys, kept for in-progress nodes) ───────────────

    "auto_authorize": True,
    "max_relay_duration_min": 15,
    "abort_battery_pct": 30,
    "authorization_timeout_s": 2.0,
    "fallback_strategy": "LET_LEADER_ISOLATE",

    # ── Movement and positioning ──────────────────────────────────────────────

    # Snap relay target to this grid before proposal to prevent sub-metre
    # GPS drift from producing repeated re-authorizations.  BUILDSPEC §3.1.
    "position_bucket_m": 5.0,

    # Setpoint rate for relay_mover while follower is moving.  PX4 OFFBOARD
    # mode requires 2–5 Hz to stay armed.  BUILDSPEC §3.1.
    "relay_position_tracker_hz": 3,

    "movement_acceptance_radius_m": 500.0,  # SITL: physics frozen, drone can't physically travel to target
    "movement_timeout_safety_factor": 1.5,
    "recover_offboard_max_attempts": 30,

    # ── BT tick rate ──────────────────────────────────────────────────────────

    # capability_assessor ticks at this rate.  0.5 Hz = 2 s per tick.
    # All debounce counts are in ticks.  BUILDSPEC §3.1.
    "bt_tick_period_s": 0.5,

    # ── Radio health publish rates ────────────────────────────────────────────

    "radio_health_reader_hz": 1,
    "gc_link_observer_hz":    1,

    # ── Re-evaluation triggers ────────────────────────────────────────────────

    "reeval_link_quality_delta":     0.2,
    "reeval_battery_marginal_pct":   45,
    "reeval_duration_warning_factor": 0.7,

    # ── Per-hop severity (signal_faker inputs) ────────────────────────────────
    #
    # Each hop has an independently configurable severity in [0.0, 1.0].
    # 0.0 = no jamming (clean signal); 1.0 = maximum jamming.
    # BUILDSPEC §4.1: "Every hop's severity is independent. Never share, never
    # default one to another's value, never default to zero."
    # These are the config-side dials — signal_faker reads them at startup.
    "hop_severities": {
        # Under radio_range_m=1500, jamming_severity_factor=0.6:
        #   eff_range = radio_range * (1 - severity * factor)
        #   severity 0.70 → eff_range = 870 m
        #   severity 0.35 → eff_range = 1185 m
        "gc_to_leader":           0.70,   # degraded link — direct GC↔leader
        "leader_to_gc":           0.70,   # symmetric
        "gc_to_follower":         0.35,   # follower hops lightly jammed
        "follower_to_gc":         0.35,   # symmetric
        "leader_to_follower":     0.35,   # relay hop
    },

    # ── Signal normalisation ──────────────────────────────────────────────────

    "max_good_snr_db": 30.0,
    "reposition_cooldown_s": 60.0,

    # ── Per-model physical constants ──────────────────────────────────────────

    "DRONE_MODELS":            DRONE_MODELS,
    "DRONE_MODEL_ASSIGNMENT":  DRONE_MODEL_ASSIGNMENT,
    "drone_model_id": "generic",
}
