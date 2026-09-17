"""
leader_radio_health_reader.py — L_radio: leader-side radio health.

BUILDSPEC: §4.2
LAYER:     2 (transform — signal data in, radio_health out)
SUBSCRIBES: /signal/gc_to_leader
PUBLISHES:  /{own_id}/radio_health   (one message per subscribed hop, at 1 Hz)

Hard rules this file must satisfy (BUILDSPEC §4.2):
  - Publishes §2.2 schema, identical to follower/GC readers -> test_schema_identical_across_readers
  - snr_db passed through unmodified (Q16)                  -> test_snr_passthrough_unmodified
  - timestamp is origin time from §2.1, NOT publish time    -> test_timestamp_preserved
  - Publishes every tick even when input is static          -> test_publishes_when_input_static
  - Only reads gc_to_leader (not any follower hops)         -> test_subscribes_only_own_hops
  - FSPL-inverse derives severity correctly                 -> test_severity_derivation
"""

import os
import sys
import json

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from _radio_health_core import RadioHealthReaderBase, _load_config  # noqa: E402

DRONE_ID = os.environ.get("DRONE_ID", "drone-01")


def make_leader_reader(drone_id: str, cfg: dict, publish, clock=None):
    """
    Factory for a LeaderRadioHealthReader instance.
    Subscription: gc_to_leader only.  BUILDSPEC §4.2.
    The leader only cares about the GC→leader link — it does not see follower hops.
    """
    subscriptions = [
        # The leader's radio_health is derived solely from the GC-to-leader link.
        # It does not monitor any follower hops — those are the follower's responsibility.
        ("/signal/gc_to_leader", "gc_to_leader"),
    ]
    nominal_ranges = {
        "gc_to_leader": float(cfg.get("leader_radio_range_m", 800)),
    }
    publish_topic = f"/{drone_id.replace('-', '_')}/radio_health"   # ROS 2 topic names disallow hyphens
    return RadioHealthReaderBase(
        subscriptions=subscriptions,
        nominal_ranges=nominal_ranges,
        publish_topic=publish_topic,
        cfg=cfg,
        publish=publish,
        clock=clock,
    )


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    cfg = _load_config()

    class LeaderReaderNode(Node):
        def __init__(self):
            node_name = f"leader_radio_health_reader_{DRONE_ID.replace('-', '_')}"
            super().__init__(node_name)

            # Lazy-create publishers per topic — same pattern as follower reader.
            captured_pubs = {}

            def _publish(topic: str, payload: dict) -> None:
                if topic not in captured_pubs:
                    captured_pubs[topic] = self.create_publisher(String, topic, 10)
                msg = String()
                msg.data = json.dumps(payload)
                captured_pubs[topic].publish(msg)

            self._reader = make_leader_reader(DRONE_ID, cfg, _publish)

            for sub_topic, _ in self._reader._subscriptions:
                # t=sub_topic: captures loop variable by value, not by reference.
                self.create_subscription(
                    String, sub_topic,
                    lambda msg, t=sub_topic: self._on_signal(t, json.loads(msg.data)),
                    10,
                )

            # BUILDSPEC §4.2: publish every tick (1 Hz) even when input is static.
            # Downstream freshness guards rely on a continuous stream to detect silence.
            self.create_timer(1.0, self._reader.publish_tick)

        def _on_signal(self, topic: str, payload: dict):
            self._reader.on_signal(topic, payload)

    node = LeaderReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
