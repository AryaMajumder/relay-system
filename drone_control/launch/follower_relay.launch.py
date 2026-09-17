"""
follower_relay.launch.py — full node graph for one follower drone (SITL).

BUILDSPEC: §4.14
LAYER:     3 (wiring only)

Startup order
─────────────
Wave 1  (immediate)       signal_faker, state_bridge, gc_link_observer,
                          leader_link_detector
Wave 2  (on signal_faker start)   follower_radio_health_reader,
                                  leader_radio_health_reader,
                                  gc_radio_health_reader
Wave 3  (on radio_health readers)  capability_assessor
Wave 4  (on capability_assessor)  relay_strategy_evaluator,
                                  relay_decision_authority, continuous_monitor,
                                  chain_assigner, strategy_executor,
                                  relay_position_tracker, relay_mover

Usage
─────
ros2 launch drone_control follower_relay.launch.py
ros2 launch drone_control follower_relay.launch.py drone_id:=drone-02
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DOMAIN_ENV = {
    "ROS_DOMAIN_ID":      "42",
    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
    "CYCLONEDDS_URI":     "file:///etc/ros/cyclonedds.xml",
    "ROS_SECURITY_ENABLE": "false",
}


def _node(executable: str, drone_id_sub, extra_env: dict | None = None) -> Node:
    env = dict(_DOMAIN_ENV)
    env["DRONE_ID"] = drone_id_sub
    if extra_env:
        env.update(extra_env)
    return Node(
        package="drone_control",
        executable=executable,
        name=executable,
        output="screen",
        additional_env=env,
    )


def generate_launch_description():
    drone_id = LaunchConfiguration("drone_id")

    declare_drone_id = DeclareLaunchArgument(
        "drone_id",
        default_value="drone-01",
        description="Drone identifier used for topic namespacing",
    )

    # ── Wave 1 — start immediately ────────────────────────────────────────────
    signal_faker_node     = _node("signal_faker",       drone_id)
    state_bridge_node     = _node("state_bridge",       drone_id)
    gc_link_observer_node = _node("gc_link_observer",   drone_id)
    leader_link_det_node  = _node("leader_link_detector", drone_id)

    # ── Wave 2 — after signal_faker process starts ────────────────────────────
    # All three readers subscribe to signal_faker's raw output topics.
    follower_rhr_node = _node("follower_radio_health_reader", drone_id)
    leader_rhr_node   = _node("leader_radio_health_reader",   drone_id)
    gc_rhr_node       = _node("gc_radio_health_reader",       drone_id)

    # ── Wave 3 — after radio_health readers start ─────────────────────────────
    capability_assessor_node = _node("capability_assessor", drone_id)

    # ── Wave 4 — after capability_assessor process starts ────────────────────
    relay_strategy_evaluator_node = _node("relay_strategy_evaluator", drone_id)
    relay_decision_authority_node = _node("relay_decision_authority", drone_id,
                                          {"LEADER_ID": "drone-01"})
    continuous_monitor_node       = _node("continuous_monitor",       drone_id)
    chain_assigner_node           = _node("chain_assigner",           drone_id)
    strategy_executor_node        = _node("strategy_executor",        drone_id)
    relay_position_tracker_node   = _node("relay_position_tracker",   drone_id)
    relay_mover_node              = _node("relay_mover",              drone_id)

    return LaunchDescription([
        declare_drone_id,

        # Wave 1 — no dependencies
        signal_faker_node,
        state_bridge_node,
        gc_link_observer_node,
        leader_link_det_node,

        # Wave 2 — readers wait for signal_faker to publish
        RegisterEventHandler(OnProcessStart(
            target_action=signal_faker_node,
            on_start=[follower_rhr_node, leader_rhr_node, gc_rhr_node],
        )),

        # Wave 3 — BT waits for radio health data to be available
        RegisterEventHandler(OnProcessStart(
            target_action=follower_rhr_node,
            on_start=[capability_assessor_node],
        )),

        # Wave 4 — decision + execution layer waits for the BT to be ticking
        RegisterEventHandler(OnProcessStart(
            target_action=capability_assessor_node,
            on_start=[
                relay_strategy_evaluator_node,
                relay_decision_authority_node,
                continuous_monitor_node,
                chain_assigner_node,
                strategy_executor_node,
                relay_position_tracker_node,
                relay_mover_node,
            ],
        )),
    ])
