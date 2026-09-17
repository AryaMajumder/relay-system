"""
_radio_health_core.py — shared logic for all three radio_health reader files.

BUILDSPEC: §4.2  (internal helper — not a buildspec file, supports the three readers)
LAYER:     2 (pure transform helper)

All three readers (follower, leader, gc) have identical output schema §2.2 and
identical computation.  This module holds that shared logic so a single fix
propagates to all three.  Each reader file is thin: it defines its own
subscriptions and instantiates RadioHealthReaderBase.
"""

import os
import sys
import time
import logging

log = logging.getLogger(__name__)


# ── Config loader ─────────────────────────────────────────────────────────────

def _load_config() -> dict:
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    try:
        from config.demo_config import DEMO_CONFIG
        return DEMO_CONFIG
    except Exception as e:
        log.warning("demo_config load failed (%s) — using defaults", e)
        return {
            "baseline_noise_dbm":    -95.0,
            "noise_range_db":         40.0,
            "jamming_severity_factor": 0.6,
        }


# ── Core computation ──────────────────────────────────────────────────────────

def compute_radio_health(signal_payload: dict, nominal_range_m: float,
                         hop_name: str, cfg: dict) -> dict:
    """
    Produce a §2.2 radio_health dict from a §2.1 signal payload.

    FSPL-inverse to recover severity from noise_dbm:
      severity = clamp((noise_dbm - baseline_noise_dbm) / noise_range_db, 0.0, 1.0)

    range_m = effective_radio_range(nominal_range_m, severity, factor):
      = nominal_range_m * (1 - severity * jamming_severity_factor)

    snr_db and timestamp are passed through unmodified — BUILDSPEC §2.2 Q16, §5.6.
    """
    baseline   = cfg["baseline_noise_dbm"]
    noise_rng  = cfg["noise_range_db"]
    sev_factor = cfg.get("jamming_severity_factor", 0.6)

    noise_dbm = signal_payload["noise_dbm"]

    # FSPL-inverse: invert the forward formula noise_dbm = baseline + sev * noise_rng.
    # Clamp to [0, 1] — rounding or measurement noise could produce out-of-range values.
    severity = max(0.0, min(1.0, (noise_dbm - baseline) / noise_rng))

    # effective range at this severity — same factor used by signal_faker's forward model.
    range_m = nominal_range_m * (1.0 - severity * sev_factor)

    # §2.2 schema exactly.  snr_db and timestamp pass through unmodified — BUILDSPEC §5.6.
    return {
        "severity":  severity,
        "range_m":   range_m,
        "snr_db":    signal_payload["snr_db"],      # Q16: unmodified passthrough
        "hop":       hop_name,
        "timestamp": signal_payload["timestamp"],   # origin time, NOT publish time
    }


# ── RadioHealthReaderBase ─────────────────────────────────────────────────────

class RadioHealthReaderBase:
    """
    Generic radio health reader.  All three reader files instantiate this with
    their own subscription list, nominal ranges, and publish topic.

    Injectable boundaries (TEST_PROTOCOL §3.3):
      - publish(topic, payload): output sink; defaults to no-op
      - clock: callable returning current time; unused here (timestamps come
               from signal origin) but kept for consistency with other nodes
    """

    def __init__(
        self,
        subscriptions: list,       # [(signal_topic, hop_name), ...]
        nominal_ranges: dict,      # {hop_name: float}
        publish_topic: str,
        cfg: dict,
        publish=None,              # (topic: str, payload: dict) -> None
        clock=None,
    ):
        self._subscriptions  = subscriptions
        self._nominal_ranges = nominal_ranges
        self._publish_topic  = publish_topic
        self._cfg            = cfg
        self._publish        = publish or (lambda t, p: None)
        self._clock          = clock or time.time

        # Keyed by hop_name — holds the last computed radio_health payload.
        # None means no signal has arrived yet for this hop.
        self._state: dict = {hop: None for _, hop in subscriptions}

    def on_signal(self, topic: str, payload: dict) -> None:
        """
        Callback for incoming §2.1 signal messages.

        HARD RULE (BUILDSPEC §4.2): only process topics in _subscriptions.
        The subscription list is the gate — any other topic is silently ignored.
        This is what makes test_subscribes_only_own_hops pass: feeding a wrong
        topic → on_signal is a no-op → state unchanged → publish_tick emits
        nothing for that hop.
        """
        for sub_topic, hop_name in self._subscriptions:
            if sub_topic == topic:
                nominal = self._nominal_ranges[hop_name]
                self._state[hop_name] = compute_radio_health(
                    payload, nominal, hop_name, self._cfg
                )
                return
        # Topic not in subscriptions — hard rule: ignore.  BUILDSPEC §4.2.

    def publish_tick(self) -> None:
        """
        Publish the latest radio_health for every subscribed hop.

        HARD RULE (BUILDSPEC §4.2): publish every tick even if input hasn't changed.
        Downstream freshness checks compare origin timestamp — a repeated publish
        with an aging timestamp signals "data not updated" correctly.
        Silence (not publishing) would look identical to a crashed upstream.
        """
        for _, hop_name in self._subscriptions:
            payload = self._state.get(hop_name)
            if payload is None:
                # No signal received yet for this hop — skip (not silence by choice,
                # just no data to republish yet).
                continue
            self._publish(self._publish_topic, payload)
