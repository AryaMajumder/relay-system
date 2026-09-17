"""
follower_radio_health_reader.py — F_radio: follower-side radio health.

BUILDSPEC: §4.2
LAYER:     2 (transform — signal data in, radio_health out)
SUBSCRIBES: /signal/leader_to_follower/{own_id}
            /signal/gc_to_follower/{own_id}
PUBLISHES:  /{own_id}/radio_health   (one message per subscribed hop, at 1 Hz)

Hard rules this file must satisfy (BUILDSPEC §4.2):
  - Publishes §2.2 schema, identical to leader/GC readers -> test_schema_identical_across_readers
  - snr_db passed through unmodified (Q16)                -> test_snr_passthrough_unmodified
  - timestamp is origin time from §2.1, NOT publish time  -> test_timestamp_preserved
  - Publishes every tick even when input is static        -> test_publishes_when_input_static
  - Never reads gc_to_leader (not in subscription list)   -> test_subscribes_only_own_hops
  - FSPL-inverse derives severity correctly               -> test_severity_derivation
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from _radio_health_core import RadioHealthReaderBase, _load_config   # noqa: E402

DRONE_ID = os.environ.get("DRONE_ID", "drone-01")


def make_follower_reader(drone_id: str, cfg: dict, publish, clock=None):
    """
    Factory for a FollowerRadioHealthReader instance.
    Subscriptions: leader_to_follower and gc_to_follower for own drone_id.
    HARD RULE (BUILDSPEC §4.2): follower never reads gc_to_leader.
    """
    tid = drone_id.replace('-', '_')   # ROS 2 topic names disallow hyphens
    subscriptions = [
        (f"/signal/leader_to_follower/{tid}", "leader_to_follower"),
        (f"/signal/gc_to_follower/{tid}",     "gc_to_follower"),
    ]
    nominal_ranges = {
        "leader_to_follower": float(cfg.get("follower_radio_range_m", 800)),
        "gc_to_follower":     float(cfg.get("follower_radio_range_m", 800)),
    }
    publish_topic = f"/{tid}/radio_health"
    return RadioHealthReaderBase(
        subscriptions=subscriptions,
        nominal_ranges=nominal_ranges,
        publish_topic=publish_topic,
        cfg=cfg,
        publish=publish,
        clock=clock,
    )


def main():
    import json
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    cfg = _load_config()

    class FollowerReaderNode(Node):
        def __init__(self):
            node_name = f"follower_radio_health_reader_{DRONE_ID.replace('-', '_')}"
            super().__init__(node_name)

            # Lazily create ROS2 publishers keyed by topic.  The core layer
            # calls publish() with arbitrary topic strings; the ROS2 layer creates
            # the publisher the first time it sees each topic.
            captured_pubs = {}

            def _publish(topic: str, payload: dict) -> None:
                if topic not in captured_pubs:
                    captured_pubs[topic] = self.create_publisher(String, topic, 10)
                msg = String()
                msg.data = json.dumps(payload)
                captured_pubs[topic].publish(msg)

            self._reader = make_follower_reader(DRONE_ID, cfg, _publish)

            for sub_topic, _ in self._reader._subscriptions:
                # Default argument t=sub_topic captures the loop variable by value.
                # Without it, all lambdas would close over the last value of sub_topic.
                self.create_subscription(
                    String, sub_topic,
                    lambda msg, t=sub_topic: self._on_signal(t, json.loads(msg.data)),
                    10,
                )

            # 1 Hz timer — BUILDSPEC §4.2: publish every tick.
            self.create_timer(1.0, self._reader.publish_tick)

        def _on_signal(self, topic: str, payload: dict):
            self._reader.on_signal(topic, payload)

    node = FollowerReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
