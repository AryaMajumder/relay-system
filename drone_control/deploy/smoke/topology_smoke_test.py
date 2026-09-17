#!/usr/bin/env python3
"""
topology_smoke_test.py — assert BUILDSPEC-mandated topic ownership under
running Enforce mode.

The pytest suites verify each node in isolation. They cannot catch topology
errors — e.g., two nodes publishing to a §4.9-exclusive topic, or a subscriber
listening on the wrong per-drone vs. shared path. Three real bugs of this shape
have already shipped and been fixed after the fact (CHECK 1 producer/consumer
sweep, capability_assessor per-drone /relay_tasking mismatch, capability_assessor
double-publish on /strategy_proposal). This test closes the class.

Method:
  Uses `ros2 topic info -v <topic>` under an existing enclave to introspect
  publisher/subscriber counts and node names. Asserts each SPEC row.

Prereq (Enforce mode):
  Run under an env that has ROS_SECURITY_ENCLAVE_OVERRIDE set to a signed
  enclave with broad read permissions (e.g. /gc/gc_link_observer).
  A wrapper below auto-sources /etc/default/drone-control and picks that enclave.

Exit code: 0 if every assertion passes; non-zero on any failure.

Usage:
  python3 topology_smoke_test.py
  python3 topology_smoke_test.py --topic /drone_02/strategy_proposal    (single check)
  python3 topology_smoke_test.py --list-spec                            (print rules)
"""

import argparse
import os
import sys
from dataclasses import dataclass, field
from typing import Optional

# ── SPEC rules ───────────────────────────────────────────────────────────────
# Each entry encodes ONE BUILDSPEC rule about topic ownership.
#
#   must_be_absent: bool.  If True, asserts NO publishers AND NO subscribers
#                   for this topic.  Used for topics that were removed by
#                   design (e.g. dead-code cleanup: subscription deleted, no
#                   publisher ever existed).  When True, all other fields are
#                   ignored.  PASSES when the topic is unknown to DDS; FAILS
#                   if any publisher OR subscriber is discovered.
#
#   Otherwise (presence rules):
#     pub_count:     required number of publishers.  Most §4.x-owned topics are 1.
#     pub_matches:   substring the publisher node name must contain.
#     min_subs:      minimum number of subscribers.  0 = no requirement.
#     sub_forbidden: substring that must NOT appear in any subscriber node name.
#                    (used to catch "wrong node subscribes" bugs)
#
#   spec_ref:     BUILDSPEC section this rule enforces.

@dataclass
class TopoRule:
    topic: str
    pub_count: Optional[int] = None
    pub_matches: str = ""
    min_subs: int = 0
    sub_forbidden: str = ""
    spec_ref: str = ""
    must_be_absent: bool = False


# Fleet configuration — one leader, one follower for the current SITL deployment.
DRONES = ["drone_01", "drone_02"]
LEADER = "drone_01"
FOLLOWERS = ["drone_02"]

RULES: list[TopoRule] = [
    # ── §2.4  relay_tasking — shared broadcast, single publisher (RDA) ───────
    TopoRule("/relay_tasking",
             pub_count=1, pub_matches="relay_decision_authority",
             min_subs=1, spec_ref="§2.4 shared, RDA-exclusive"),

    # ── §4.9  strategy_proposal — sole publisher per drone: evaluator ────────
    #     capability_assessor MUST NOT publish here (§4.9 CRITICAL DISCOVERY).
    #     FOLLOWERS only: only follower drones run capability_assessor + evaluator.
    *[TopoRule(f"/{d}/strategy_proposal",
               pub_count=1, pub_matches="relay_strategy_evaluator",
               sub_forbidden="capability_assessor",
               spec_ref="§4.9 relay_strategy_evaluator sole publisher")
      for d in FOLLOWERS],

    # ── §2.6  authorization — RDA per-drone, sole publisher ──────────────────
    #     FOLLOWERS only: RDA authorizes followers, not leaders.
    *[TopoRule(f"/{d}/authorization",
               pub_count=1, pub_matches="relay_decision_authority",
               spec_ref="§2.6 RDA per-drone auth publisher")
      for d in FOLLOWERS],

    # ── §2.7  relay_assignment — chain_assigner per-drone, sole publisher ────
    #     FOLLOWERS only: chain_assigner runs one-per-follower.
    *[TopoRule(f"/{d}/relay_assignment",
               pub_count=1, pub_matches=f"chain_assigner_{d}",
               spec_ref="§2.7 chain_assigner sole publisher")
      for d in FOLLOWERS],

    # ── §2.10  relay_confirmed — relay_position_tracker sole publisher ───────
    *[TopoRule(f"/{d}/relay_confirmed",
               pub_count=1, pub_matches=f"relay_position_tracker_{d}",
               spec_ref="§2.10 relay_position_tracker sole publisher")
      for d in FOLLOWERS],

    # ── §2.11  reauth_request — capability_assessor sole publisher ───────────
    *[TopoRule(f"/{d}/reauth_request",
               pub_count=1, pub_matches=f"capability_assessor_{d}",
               spec_ref="§2.11 capability_assessor sole publisher")
      for d in FOLLOWERS],

    # ── §2.12  reeval_trigger — continuous_monitor sole publisher ────────────
    *[TopoRule(f"/{d}/reeval_trigger",
               pub_count=1, pub_matches=f"continuous_monitor_{d}",
               spec_ref="§2.12 continuous_monitor sole publisher")
      for d in FOLLOWERS],

    # ── §4.11  capability_report — capability_assessor sole publisher ────────
    *[TopoRule(f"/{d}/capability_report",
               pub_count=1, pub_matches=f"capability_assessor_{d}",
               min_subs=1,  # relay_strategy_evaluator must subscribe (§4.9)
               spec_ref="§4.11 capability_assessor sole publisher")
      for d in FOLLOWERS],

    # ── §4.8  current_role — DUAL publisher by design (spec §4.8 is misleading)
    #     Two nodes publish here legitimately:
    #       strategy_executor       — MOVING_TO_RELAY / OPEN_TO_RELAY on strategy
    #       relay_position_tracker  — RELAYING on physical arrival
    #     Ownership split is by role-transition source, not by drone-id.  Smoke
    #     test asserts pub_count=2 to lock the invariant; if a THIRD ever
    #     appears that's a real §4.8 violation.
    *[TopoRule(f"/{d}/current_role",
               pub_count=2,
               spec_ref="§4.8 strategy_executor + arrival-time relay_position_tracker")
      for d in FOLLOWERS],

    # ── §4.2  radio_health — GC has one; each drone has one ──────────────────
    TopoRule("/gc/radio_health",
             pub_count=1, pub_matches="gc_radio_health_reader",
             spec_ref="§4.2 gc_radio_health_reader sole publisher"),
    TopoRule(f"/{LEADER}/radio_health",
             pub_count=1, pub_matches=f"leader_radio_health_reader_{LEADER}",
             spec_ref="§4.2 leader_radio_health_reader sole publisher"),
    *[TopoRule(f"/{d}/radio_health",
               pub_count=1, pub_matches=f"follower_radio_health_reader_{d}",
               spec_ref="§4.2 follower_radio_health_reader sole publisher")
      for d in FOLLOWERS],

    # ── §4.3  gc_link_quality — gc_link_observer sole publisher ──────────────
    TopoRule("/gc/gc_link_quality",
             pub_count=1, pub_matches="gc_link_observer",
             min_subs=1,   # RDA must subscribe
             spec_ref="§4.3 gc_link_observer sole publisher"),

    # ── §2.3  relay_request — leader_link_detector on leader drone ───────────
    TopoRule(f"/{LEADER}/relay_request",
             pub_count=1, pub_matches=f"leader_link_detector_{LEADER}",
             spec_ref="§2.3 leader_link_detector sole publisher"),

    # ── §4.11 item 11  alert_intent — capability_assessor publishes; G_task subscribes
    #     G_task = relay_decision_authority per §4.10 header.  Option (a) applied
    #     2026-08-24: RDA subscribes, observability-only, in-memory ring buffer
    #     per §4.10 hard rule "local-process, in-memory."  Closes CHECK 1
    #     [2026-08-18] BLOCKER.
    *[TopoRule(f"/{d}/alert_intent",
               pub_count=1, pub_matches=f"capability_assessor_{d}",
               min_subs=1,
               spec_ref="§4.11 item 11 — cap_assessor pub, RDA (G_task) sub")
      for d in FOLLOWERS],

    # ── follower_position — RETIRED as dead code (2026-08-24)
    #     History: CHECK 1 [2026-08-18] flagged as "subscribed, no publisher."
    #     Deeper trace revealed no code reads bb["follower_position"] either;
    #     BandSensorNode uses drone_state.position.  Cleanup applied: subscription
    #     deleted from capability_assessor.py.  Rule now asserts the topic must
    #     NOT exist.  Passes when the code is clean; regresses (fails) if anyone
    #     re-adds the subscription without adding a publisher and a reader.
    *[TopoRule(f"/{d}/follower_position",
               must_be_absent=True,
               spec_ref="RETIRED dead-code topic (CHECK 1 → cleanup 2026-08-24)")
      for d in FOLLOWERS],
]


# ── rclpy graph introspector ─────────────────────────────────────────────────
#
# `ros2 topic info -v` spawns a fresh CLI subprocess per call — each pays the
# full DDS init + discovery cost.  Under Enforce that regularly exceeds a 5s
# timeout, giving false "unknown topic" on healthy fleets.  Using rclpy in-
# process reuses a single participant that discovers once then answers all 17
# queries in milliseconds.

@dataclass
class TopicInfo:
    topic: str
    pub_count: int = 0
    sub_count: int = 0
    pub_nodes: list[str] = field(default_factory=list)
    sub_nodes: list[str] = field(default_factory=list)


class GraphInspector:
    """Single rclpy Node persisted across all queries; discovers the domain once."""

    def __init__(self, warmup_s: float = 8.0):
        import rclpy
        from rclpy.node import Node
        rclpy.init(args=[])
        self._rclpy = rclpy
        self._node = Node("topology_smoke_inspector")
        # Give DDS discovery time to hear every participant on the domain.
        # Under Enforce, first-hear latency is dominated by SPDP + authentication
        # handshake; warmup < 5s misses topics that would otherwise be visible.
        import time
        time.sleep(warmup_s)

    def close(self):
        self._node.destroy_node()
        self._rclpy.shutdown()

    def query(self, topic: str) -> TopicInfo:
        info = TopicInfo(topic=topic)
        try:
            pubs = self._node.get_publishers_info_by_topic(topic)
            subs = self._node.get_subscriptions_info_by_topic(topic)
        except Exception:
            return info   # unknown topic → counts stay 0

        info.pub_count = len(pubs)
        info.sub_count = len(subs)
        info.pub_nodes = [f"{p.node_namespace.rstrip('/')}/{p.node_name}".lstrip('/')
                          for p in pubs]
        info.sub_nodes = [f"{s.node_namespace.rstrip('/')}/{s.node_name}".lstrip('/')
                          for s in subs]
        return info


# ── Assertion runner ─────────────────────────────────────────────────────────

def check_rule(inspector: GraphInspector, rule: TopoRule) -> tuple[bool, list[str]]:
    info = inspector.query(rule.topic)
    fails: list[str] = []

    if rule.must_be_absent:
        # Absence rule: PASS when nothing publishes or subscribes to this topic.
        if info.pub_count == 0 and info.sub_count == 0:
            return True, []
        parts = []
        if info.pub_count > 0:
            parts.append(f"unexpected publisher(s): {info.pub_nodes}")
        if info.sub_count > 0:
            parts.append(f"unexpected subscriber(s): {info.sub_nodes}")
        return False, [f"topic should not exist — {'; '.join(parts)}"]

    if info.pub_count == 0 and info.sub_count == 0:
        return False, ["topic unknown to DDS — no publisher discovered "
                       "(fleet down? enclave permissions? increase warmup?)"]

    if rule.pub_count is not None and info.pub_count != rule.pub_count:
        fails.append(f"pub_count={info.pub_count}, expected={rule.pub_count} "
                     f"(publishers: {info.pub_nodes})")

    if rule.pub_matches:
        bad = [n for n in info.pub_nodes if rule.pub_matches not in n]
        if bad:
            fails.append(f"publisher(s) do not match '{rule.pub_matches}': {bad}")

    if info.sub_count < rule.min_subs:
        fails.append(f"sub_count={info.sub_count}, expected>={rule.min_subs} "
                     f"(subscribers: {info.sub_nodes})")

    if rule.sub_forbidden:
        # This catches §4.9-shape violations: any publisher whose name contains
        # the forbidden substring (e.g. "capability_assessor" on strategy_proposal).
        bad = [n for n in info.pub_nodes if rule.sub_forbidden in n]
        if bad:
            fails.append(f"forbidden publisher(s) present: {bad}")

    return (len(fails) == 0, fails)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", help="check a single topic only")
    ap.add_argument("--list-spec", action="store_true",
                    help="print all spec rules and exit")
    ap.add_argument("--warmup", type=float, default=8.0,
                    help="seconds to wait for DDS discovery after participant init "
                         "(default: 8.0; increase under heavy Enforce load)")
    args = ap.parse_args()

    if args.list_spec:
        for r in RULES:
            print(f"  {r.topic:<40s} pub={r.pub_count} match='{r.pub_matches}' "
                  f"min_subs={r.min_subs}  [{r.spec_ref}]")
        return 0

    rules = [r for r in RULES if r.topic == args.topic] if args.topic else RULES
    if not rules:
        print(f"no rule matches --topic '{args.topic}'", file=sys.stderr)
        return 2

    print(f"warming rclpy participant ({args.warmup}s DDS discovery)...", flush=True)
    inspector = GraphInspector(warmup_s=args.warmup)
    try:
        ok_count = 0
        fail_count = 0
        for rule in rules:
            passed, fails = check_rule(inspector, rule)
            if passed:
                ok_count += 1
                print(f"  ✓ {rule.topic:<42s} [{rule.spec_ref}]")
            else:
                fail_count += 1
                print(f"  ✗ {rule.topic:<42s} [{rule.spec_ref}]")
                for f in fails:
                    print(f"      {f}")
        print()
        print(f"─── {ok_count + fail_count} rules checked · {ok_count} pass · {fail_count} fail ───")
        return 0 if fail_count == 0 else 1
    finally:
        inspector.close()


if __name__ == "__main__":
    sys.exit(main())
