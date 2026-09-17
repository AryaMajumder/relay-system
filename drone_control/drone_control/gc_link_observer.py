"""
gc_link_observer.py — G_obs: GC-side link quality observer.

BUILDSPEC: §4.3
LAYER:     2 (transform — radio_health in, gc_link_quality out)
SUBSCRIBES: /gc/radio_health  (filtered to hop == "gc_to_leader" only)
PUBLISHES:  /gc/gc_link_quality at 1 Hz

Hard rules this file must satisfy (BUILDSPEC §4.3):
  - Never subscribe the leader's own radio_health — GC uses its own measurement
    -> proven by test_never_subscribes_leader_radio_health
  - SNR-derived quality only — no loss, no blending, no second input
    -> proven by test_no_loss_blending_required
  - Filters incoming radio_health to hop == "gc_to_leader" only
    -> proven by test_filters_to_gc_to_leader_hop_only
  - Does not select candidates or assign missions — quality signal only
    -> proven by test_schema_keys (no candidate/mission fields)
  - Timestamp is origin time from radio_health, NOT publish time (BUILDSPEC §5.6)
    -> proven by test_timestamp_passthrough
"""

import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from _radio_health_core import _load_config  # noqa: E402


def _derive_quality(snr_db: float, link_marginal_db: float) -> float:
    """
    Map snr_db to [0.0, 1.0].  quality = 0.5 when snr_db equals
    LINK_MARGINAL_QUALITY, 1.0 at twice that, 0.0 at 0 dB and below.
    ASSUMPTION (SESSION_LOG §4.3-quality-formula): §4.3 says "SNR-derived
    quality value" but does not specify the exact mapping.  This linear
    normalization over [0, 2×marginal] lets relay_decision_authority trigger
    on quality < 0.5  ↔  snr_db < LINK_MARGINAL_QUALITY — consistent with
    leader_link_detector's raw snr_db < LINK_MARGINAL_QUALITY test.
    """
    if link_marginal_db <= 0:
        return 0.0
    return max(0.0, min(1.0, snr_db / (2.0 * link_marginal_db)))


class GcLinkObserver:
    """
    Transforms /gc/radio_health (gc_to_leader hop) into /gc/gc_link_quality.
    Archetype C: receive → filter → derive quality → publish per tick.
    BUILDSPEC §4.3.
    """

    def __init__(self, cfg: dict, publish, clock=None):
        self._cfg     = cfg
        self._publish = publish
        self._clock   = clock or __import__("time").time
        self._last_health: dict | None = None

        # HARD RULE (BUILDSPEC §4.3): GC uses its own radio_health measurement —
        # never subscribe the leader's own radio_health topic.
        self._subscriptions = [("/gc/radio_health", "gc_to_leader")]

    def on_radio_health(self, topic: str, payload: dict) -> None:
        # HARD RULE: only accept gc_to_leader — silently ignore everything else.
        if payload.get("hop") != "gc_to_leader":
            return
        self._last_health = payload

    def publish_tick(self) -> None:
        if self._last_health is None:
            return   # no data yet — do not publish a zero-quality placeholder

        link_marginal = float(self._cfg.get("LINK_MARGINAL_QUALITY", 13))
        snr_db  = self._last_health["snr_db"]
        quality = _derive_quality(snr_db, link_marginal)

        self._publish("/gc/gc_link_quality", {
            "quality":   quality,
            "snr_db":    snr_db,
            "hop":       "gc_to_leader",
            # Carry origin timestamp from radio_health — do NOT restamp (§5.6).
            "timestamp": self._last_health["timestamp"],
        })


def make_gc_link_observer(cfg: dict, publish, clock=None) -> GcLinkObserver:
    """Factory.  Consistent with the Wave 2 reader factory pattern."""
    return GcLinkObserver(cfg=cfg, publish=publish, clock=clock)


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    cfg = _load_config()

    class GcLinkObserverNode(Node):
        def __init__(self):
            super().__init__("gc_link_observer")
            captured_pubs: dict = {}

            def _publish(topic: str, payload: dict) -> None:
                if topic not in captured_pubs:
                    captured_pubs[topic] = self.create_publisher(String, topic, 10)
                msg = String()
                msg.data = json.dumps(payload)
                captured_pubs[topic].publish(msg)

            self._obs = make_gc_link_observer(cfg, _publish)

            for sub_topic, _ in self._obs._subscriptions:
                self.create_subscription(
                    String, sub_topic,
                    lambda m, t=sub_topic: self._obs.on_radio_health(t, json.loads(m.data)),
                    10,
                )

            self.create_timer(1.0, self._obs.publish_tick)

    node = GcLinkObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
