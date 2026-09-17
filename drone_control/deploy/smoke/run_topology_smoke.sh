#!/usr/bin/env bash
# run_topology_smoke.sh — wrapper: sources ROS + a signed enclave, then runs
# topology_smoke_test.py against the running fleet.
#
# Usage:
#   ./run_topology_smoke.sh                                  # all rules
#   ./run_topology_smoke.sh --topic /drone_02/strategy_proposal
#   ./run_topology_smoke.sh --list-spec

set -e
HERE="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /etc/default/drone-control

export ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI \
       ROS_SECURITY_ENABLE ROS_SECURITY_STRATEGY ROS_SECURITY_KEYSTORE

# The tool needs a signed enclave to be visible to the fleet under Enforce
# (governance.xml sets allow_unauthenticated_participants=false).
# /gc/relay_decision_authority currently has the broadest observed discovery
# reach; /gc/gc_link_observer works too but sees fewer topics.
#
# For a full 100%-visible baseline, generate a dedicated /gc/smoke_test enclave
# whose permissions.p7s grants broad subscribe rights across every rt/* topic.
# Not automated here — the existing sros2/generate_keystore.sh + templates
# already produce this shape; add "smoke_test" to DRONE_NODES and regenerate.
export ROS_SECURITY_ENCLAVE_OVERRIDE="${ROS_SECURITY_ENCLAVE_OVERRIDE:-/gc/relay_decision_authority}"

exec python3 "$HERE/topology_smoke_test.py" "$@"
