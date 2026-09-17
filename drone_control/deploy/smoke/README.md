# deploy/smoke — live-system smoke tests

Tests that exercise the running fleet, not per-node unit tests. Each covers a
class of bug pytest structurally cannot see (topology, DDS wiring, external
integrations).

## topology_smoke_test.py

Asserts BUILDSPEC-mandated topic ownership under running Enforce mode. Closes
the "multiple publishers on an exclusive topic" class after three real
instances shipped (CHECK 1 producer/consumer sweep, capability_assessor
per-drone `/relay_tasking` mismatch, §4.9 `/strategy_proposal` double-publish).

See ANNOTATIONS.md § "Topology smoke test" for the full rule table and
extension pattern.

**Usage:**
```bash
./run_topology_smoke.sh              # full sweep
./run_topology_smoke.sh --topic /drone_02/strategy_proposal
./run_topology_smoke.sh --list-spec
```

**Mandatory at every file gate.** All 17 rules must PASS before the gate opens.
Any violation is a §-referenced hard block.

## Prerequisites for any smoke test

- All 16 `drone-control-*.service` units active running
- `signal_faker.service` active (SITL rigs only — see `../sitl/`)
- A signed enclave override the wrapper can present (default:
  `/gc/gc_link_observer` — any enclave under `/root/sros2/keystore/enclaves/`
  with broad read perms works)
- DDS discovery healthy — a fleet that has been running >30 s without recent
  service restarts. Recent restart trauma exhausts CycloneDDS discovery FSM
  under Enforce; wait for it to settle before running.

## Adding a smoke test

Same layout: `<name>.py` + `run_<name>.sh` wrapper that sources ROS,
`/etc/default/drone-control`, and the enclave override.

Add to the mandatory file-gate list here and in SESSION_LOG when adopted.
