"""
gc_radio_health_reader.py — G_radio: GC-side radio health.

BUILDSPEC: §4.2
LAYER:     2 (transform — signal data in, radio_health out)
SUBSCRIBES: /signal/gc_to_leader
            /signal/gc_to_follower/{drone_id}  (for each follower)
PUBLISHES:  /gc/radio_health

Note: gc_to_follower output currently has no consumer — built for symmetry.
BUILDSPEC §4.2: "Build it anyway; do not wire it to anything."

Hard rules this file must satisfy (BUILDSPEC §4.2):
  - Publishes §2.2 schema, identical to follower/leader readers -> test_schema_identical_across_readers
  - snr_db passed through unmodified (Q16)                      -> test_snr_passthrough_unmodified
  - timestamp is origin time from §2.1, NOT publish time        -> test_timestamp_preserved
  - Publishes every tick even when input is static              -> test_publishes_when_input_static
  - Never reads follower_to_gc or leader_to_gc                  -> test_subscribes_only_own_hops
  - FSPL-inverse derives severity correctly                     -> test_severity_derivation
"""

import os
import sys
import json

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from _radio_health_core import RadioHealthReaderBase, _load_config  # noqa: E402

DRONE_ID = os.environ.get("DRONE_ID", "drone-01")


def make_gc_reader(follower_drone_ids: list, cfg: dict, publish, clock=None):
    """
    Factory for a GCRadioHealthReader instance.
    Subscriptions: gc_to_leader + gc_to_follower/{id} for each follower.
    BUILDSPEC §4.2.
    """
    subscriptions = [("/signal/gc_to_leader", "gc_to_leader")]
    nominal_ranges = {"gc_to_leader": float(cfg.get("gc_radio_range_m", 800))}

    for drone_id in follower_drone_ids:
        # Unique hop_name per follower prevents state dict collisions inside RadioHealthReaderBase.
        # "gc_to_follower" alone would be ambiguous when there are multiple followers.
        tid = drone_id.replace('-', '_')   # ROS 2 topic names disallow hyphens
        hop_name = f"gc_to_follower_{tid}"
        subscriptions.append((f"/signal/gc_to_follower/{tid}", hop_name))
        nominal_ranges[hop_name] = float(cfg.get("gc_radio_range_m", 800))

    return RadioHealthReaderBase(
        subscriptions=subscriptions,
        nominal_ranges=nominal_ranges,
        publish_topic="/gc/radio_health",
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
    # FOLLOWER_IDS env var is a comma-separated list; default to the single DRONE_ID
    # so a single-drone deployment works without extra configuration.
    follower_ids = os.environ.get("FOLLOWER_IDS", DRONE_ID).split(",")

    class GcReaderNode(Node):
        def __init__(self):
            super().__init__("gc_radio_health_reader")
            # Lazy-create publishers per topic — same pattern as follower/leader readers.
            captured_pubs = {}

            def _publish(topic: str, payload: dict) -> None:
                if topic not in captured_pubs:
                    captured_pubs[topic] = self.create_publisher(String, topic, 10)
                msg = String()
                msg.data = json.dumps(payload)
                captured_pubs[topic].publish(msg)

            self._reader = make_gc_reader(follower_ids, cfg, _publish)

            for sub_topic, _ in self._reader._subscriptions:
                # t=sub_topic: closure captures loop variable by value.
                self.create_subscription(
                    String, sub_topic,
                    lambda msg, t=sub_topic: self._on_signal(t, json.loads(msg.data)),
                    10,
                )

            # BUILDSPEC §4.2: publish every tick even when input is static.
            self.create_timer(1.0, self._reader.publish_tick)

        def _on_signal(self, topic: str, payload: dict):
            self._reader.on_signal(topic, payload)

    node = GcReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
