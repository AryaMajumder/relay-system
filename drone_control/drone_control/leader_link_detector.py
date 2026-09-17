"""
leader_link_detector.py — L_det: leader-side relay request publisher.

BUILDSPEC: §4.4
LAYER:     2 (transform — radio_health in, relay_request out)
SUBSCRIBES: /{own_id}/radio_health  (filtered to hop == "gc_to_leader")
            /relay_suppression      (follower suppression signal)
PUBLISHES:  /{own_id}/relay_request (§2.3) every tick the condition holds

INTERPRETATION (SESSION_LOG §4.4-hop-name): BUILDSPEC §4.4 says "hop
leader_to_gc" but leader_radio_health_reader (§4.2) only publishes
"gc_to_leader".  The GC-leader RF link is symmetric in the FSPL model;
"gc_to_leader" is the available hop from /{own_id}/radio_health.
Filtering on "gc_to_leader" satisfies the intent.

ASSUMPTION (SESSION_LOG §4.4-suppression-topic): §4.4 names a "follower
suppression signal" but §2 defines no schema or topic for it.  Assumed
topic: /relay_suppression (shared, no per-drone prefix).  Payload field:
{"active": bool}.  Log this as ASSUMPTION.

Hard rules this file must satisfy (BUILDSPEC §4.4):
  - No sender-side dedup, throttling, or debounce — publishes every tick
    the condition holds.  All dedup is GC's job (relay_decision_authority.py).
    -> proven by test_no_sender_side_dedup
  - Suppression active → no publish; clears automatically when suppression
    stops — no separate resume logic.
    -> proven by test_suppression_blocks_publish, test_suppression_resume_automatic
  - Schema exactly matches §2.3: {drone_id, snr_db, timestamp}
    -> proven by test_schema_matches_section_23
  - Subscribes /{own_id}/radio_health, not a raw /signal/* topic
    -> proven by test_subscribes_radio_health_not_signal
"""

import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from _radio_health_core import _load_config  # noqa: E402

# DELIBERATELY ABSENT (BUILDSPEC §4.4): no sender-side dedup table.
# All dedup is relay_decision_authority's job.


class LeaderLinkDetector:
    """
    Publishes relay_request every tick that snr_db < LINK_MARGINAL_QUALITY
    AND the follower suppression signal is inactive.
    No hysteresis, no armed/disarmed state.  BUILDSPEC §4.4.
    """

    def __init__(self, drone_id: str, cfg: dict, publish,
                 suppression_topic: str = "/relay_suppression",
                 clock=None):
        self._drone_id   = drone_id
        # ROS 2 topic names disallow hyphens; keep _drone_id (hyphenated) for the
        # payload's "drone_id" field, use _topic_id in ROS topic paths only.
        self._topic_id   = drone_id.replace('-', '_')
        self._cfg        = cfg
        self._publish    = publish
        self._clock      = clock or __import__("time").time
        self._snr_db: float | None = None
        self._ts: float | None     = None
        self._suppressed: bool     = False

        radio_health_topic = f"/{self._topic_id}/radio_health"
        # Subscriptions used by the test harness and ROS2 wiring.
        self._subscriptions = [
            (radio_health_topic, "gc_to_leader"),   # see INTERPRETATION in header
            (suppression_topic,  "suppression"),
        ]

    def on_radio_health(self, topic: str, payload: dict) -> None:
        # INTERPRETATION: filter on "gc_to_leader" — the hop available from
        # leader_radio_health_reader (§4.2).  See file header.
        if payload.get("hop") != "gc_to_leader":
            return
        self._snr_db = payload["snr_db"]
        self._ts     = payload["timestamp"]   # origin time, not restamp

    def on_suppression(self, topic: str, payload: dict) -> None:
        # Follower suppression signal.  Clears automatically when active=False.
        # ASSUMPTION: topic and schema — see file header.
        self._suppressed = bool(payload.get("active", False))

    def publish_tick(self) -> None:
        if self._snr_db is None:
            return   # no radio_health received yet

        # HARD RULE (BUILDSPEC §4.4): no sender-side dedup, throttling, or debounce.
        # Publish every single tick the condition holds.  Do not add rate limiting.
        link_marginal = float(self._cfg.get("LINK_MARGINAL_QUALITY", 13))

        if self._suppressed:
            return   # follower suppression overrides; resumes automatically when cleared

        if self._snr_db < link_marginal:
            self._publish(f"/{self._topic_id}/relay_request", {
                "drone_id":  self._drone_id,
                "snr_db":    self._snr_db,
                # Carry origin timestamp from radio_health (§5.6).
                "timestamp": self._ts,
            })


def make_leader_link_detector(drone_id: str, cfg: dict, publish,
                               suppression_topic: str = "/relay_suppression",
                               clock=None) -> LeaderLinkDetector:
    """Factory.  Consistent with Wave 2 reader factory pattern."""
    return LeaderLinkDetector(
        drone_id=drone_id,
        cfg=cfg,
        publish=publish,
        suppression_topic=suppression_topic,
        clock=clock,
    )


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    cfg     = _load_config()
    drone_id = os.environ.get("DRONE_ID", "drone-01")

    class LeaderLinkDetectorNode(Node):
        def __init__(self):
            # Node name matches convention used by every other per-drone node
            # (name_{drone_id}) so per-node lookups and the topology smoke test
            # can identify the node's drone binding.
            super().__init__(f"leader_link_detector_{drone_id.replace('-', '_')}")
            captured_pubs: dict = {}

            def _publish(topic: str, payload: dict) -> None:
                if topic not in captured_pubs:
                    captured_pubs[topic] = self.create_publisher(String, topic, 10)
                msg = String()
                msg.data = json.dumps(payload)
                captured_pubs[topic].publish(msg)

            self._det = make_leader_link_detector(drone_id, cfg, _publish)

            radio_topic, _ = self._det._subscriptions[0]
            supp_topic, _  = self._det._subscriptions[1]

            self.create_subscription(
                String, radio_topic,
                lambda m, t=radio_topic: self._det.on_radio_health(t, json.loads(m.data)),
                10,
            )
            self.create_subscription(
                String, supp_topic,
                lambda m, t=supp_topic: self._det.on_suppression(t, json.loads(m.data)),
                10,
            )

            self.create_timer(1.0, self._det.publish_tick)

    node = LeaderLinkDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
