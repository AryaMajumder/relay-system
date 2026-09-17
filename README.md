# Relay System — Deployable Bundle

Autonomous multi-drone radio relay pipeline. A follower drone positions itself between a leader and a ground control station when their direct radio link degrades, computes the optimal relay position from live FSPL-derived signal geometry, and exits gracefully when geometry becomes infeasible.

For the design writeup, engineering decisions, test results, and known limitations: see [`drone_control/README.md`](drone_control/README.md).

This top-level bundle contains everything a fresh machine needs to run the whole system in SITL: the ROS 2 package, the companion daemon, broker configs, systemd units organised by role, PX4 launcher scripts, and operator dashboards.

## Layout

```
drone_control/          ROS 2 package — behaviour tree, geometry, signal readers, decision authority, mover, tests
deploy/
  daemons/              px4_agent.py — MAVLink↔MQTT bridge; runs one instance per drone
  mosquitto/            per-drone MQTT broker configs + systemd units
  ros/                  CycloneDDS config (domain 42, loopback)
  sitl/                 PX4 SITL tmux launcher and pxh shell helper
  dropins/              systemd .service.d overrides (SROS 2 enclaves, SITL ports, etc.)
    leader/             drop-ins for the 2 leader-role services
    follower/           drop-ins for the 7 follower-role services
    gc/                 drop-ins for the 7 GC-role services
dashboards/             Leaflet map dashboard, Foxglove Studio WebSocket bridge
setup.sh                Bootstrap for a fresh Ubuntu 22.04 (or WSL2)
upload.sh               Push this bundle to a GitHub remote
```

## Role split

The three logical roles map directly to systemd targets. On a single-host SITL rig all three run on the same machine; on a real fleet each is deployed to its own host.

| Role | What runs there | Systemd target |
|---|---|---|
| **Leader** (`drone-01`) | Own-radio health publisher, best-effort link detector | `drone-control-leader.target` |
| **Follower** (`drone-02`) | Own-radio health publisher, capability assessor (BT root), strategy evaluator, strategy executor, position tracker, mover, continuous monitor | `drone-control-follower.target` |
| **GC** | State bridges (per-drone), GC radio health reader, GC link observer (primary detector), decision authority, chain assigner, signal faker (SITL only) | `drone-control-gc.target` |

Base `.service` and `.target` files live in [`drone_control/deploy/systemd/`](drone_control/deploy/systemd/) — role-partitioned there already. The `.d/*.conf` overrides that bind SROS 2 enclaves and SITL-specific env vars live in [`deploy/dropins/`](deploy/dropins/) for keep-close-to-source review.

## Quick start (SITL, single host, Ubuntu 22.04 or WSL2)

Prerequisites the setup script assumes are already installed:
- ROS 2 Humble (`ros-humble-desktop`, `python3-colcon-common-extensions`)
- `mosquitto`, `mosquitto-clients`
- Python 3.10 + `paho-mqtt`, `pymavlink`, `py_trees`, `foxglove-websocket`
- A PX4-Autopilot checkout with `px4_sitl_default` built
- `tmux`, `git`

Then:

```bash
./setup.sh                          # copy configs, install systemd units, enable + start
cd drone_control && colcon build    # build the ROS 2 package
```

Bring up the simulator (two PX4 instances + two jMAVSim in a tmux session):

```bash
./deploy/sitl/px4-launch.sh 2
```

Watch:

```bash
python3 dashboards/dashboard.py         # http://localhost:5000  — Leaflet map, GC + both drones
python3 dashboards/foxglove_bridge.py   # ws://localhost:8765    — connect from Foxglove Studio
```

See [`setup.sh`](setup.sh) header for the full list of what it does and what it deliberately leaves manual.

## What is real vs. simulated

- **The full ROS 2 pipeline is real code** and runs unchanged whether the FCU is PX4 SITL or a real airframe.
- **Signal quality is FSPL-modelled** by `signal_faker` from configured hop severities. Swap it for a `signal_reader` node that consumes real radio telemetry on a real fleet.
- **The "GC / Cloud" lane is a local stub** (`proposal_handler.py`) that auto-authorizes every proposal. A production deployment replaces this with a fleet-level policy service.

See [`drone_control/KNOWN_LIMITATIONS.md`](drone_control/KNOWN_LIMITATIONS.md) for the full inventory.

## Tests

```bash
cd drone_control
python3 -m pytest tests/ -q
# 310 passed in ~25 s
```
