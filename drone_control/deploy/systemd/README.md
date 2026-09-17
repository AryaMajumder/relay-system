# drone_control — systemd deployment

Device-level layout: each physical machine gets exactly one role's units, plus
the two `common/` files.

## Tree

```
deploy/systemd/
├── common/                # goes on every host
│   ├── drone-control-run          → /usr/local/bin/       (0755)
│   └── drone-control.env          → /etc/default/drone-control
│
├── gc/                    # ground control station (single instance in fleet)
│   ├── drone-control-gc.target
│   ├── drone-control-relay-decision-authority.service
│   ├── drone-control-chain-assigner.service
│   ├── drone-control-gc-radio-health-reader.service
│   ├── drone-control-gc-link-observer.service
│   ├── drone-control-state-bridge-drone-01.service
│   └── drone-control-state-bridge-drone-02.service
│
├── leader/                # drone-01 companion computer
│   ├── drone-control-leader.target
│   ├── drone-control-leader-radio-health-reader.service
│   └── drone-control-leader-link-detector.service
│
├── follower/              # drone-02 companion computer
│   ├── drone-control-follower.target
│   ├── drone-control-follower-radio-health-reader.service
│   ├── drone-control-capability-assessor.service
│   ├── drone-control-relay-strategy-evaluator.service
│   ├── drone-control-strategy-executor.service
│   ├── drone-control-relay-position-tracker.service
│   ├── drone-control-relay-mover.service
│   └── drone-control-continuous-monitor.service
│
└── sitl/                  # SITL-only; deploy only on simulator rigs
    └── drone-control-signal-faker.service
```

## Prerequisites (all hosts)

- ROS 2 Humble at `/opt/ros/humble/`
- Colcon workspace built at `/root/ros2_ws/install/`
- `/etc/ros/cyclonedds.xml` deployed with the fleet's DDS config
- `px4-agent.service` running on each drone host (leader, follower)
- MQTT broker running (`mosquitto.service`) where state_bridge and px4-agent connect

## Install (per host)

```bash
# Common (all hosts)
install -m 0755 common/drone-control-run /usr/local/bin/drone-control-run
install -m 0644 common/drone-control.env  /etc/default/drone-control

# Role — pick ONE per host
ROLE=gc          # or "leader" or "follower"
install -m 0644 $ROLE/*.service $ROLE/*.target /etc/systemd/system/

systemctl daemon-reload
systemctl enable --now drone-control-$ROLE.target
```

## Verify

```bash
systemctl status drone-control-$ROLE.target
systemctl list-dependencies drone-control-$ROLE.target
journalctl -u 'drone-control-*' -f
```

## SITL overrides

On a co-hosted SITL rig running the full fleet on one box:

1. Install all three roles (`gc/`, `leader/`, `follower/`) on the same host.
2. Enable signal_faker: `install -m 0644 sitl/*.service /etc/systemd/system/ && systemctl enable --now drone-control-signal-faker`
3. Adjust mosquitto/px4-agent dependency names via drop-in overrides
   (real deployment uses `mosquitto.service` / `px4-agent.service`; SITL uses
   `mosquitto-drone01.service` + `mosquitto-drone02.service` and
   `px4-agent.service` + `px4-agent-drone-02.service`):

   ```bash
   mkdir -p /etc/systemd/system/drone-control-state-bridge-drone-02.service.d/
   cat >/etc/systemd/system/drone-control-state-bridge-drone-02.service.d/sitl.conf <<'EOF'
   [Unit]
   After=
   After=network-online.target mosquitto-drone02.service
   EOF
   systemctl daemon-reload
   ```

## Interlink safety notes

- **Same-host ordering** is expressed via `After=` (e.g. `capability_assessor`
  starts after `follower_radio_health_reader`). Cross-host ordering is NOT
  expressed — DDS discovery is async and services retry naturally.
- **Failure grouping** uses `PartOf=<role>.target`: stopping the target stops
  all its services. `Restart=always` on every unit means individual crashes
  self-heal without disturbing peers.
- **No `Requires=`** anywhere — a `Requires=` cascade would kill the whole
  follower stack if `follower_radio_health_reader` crashed. `Wants=` starts
  services together but lets each recover independently.
- **`signal_faker` has no `[Install]`** — deliberately unlinked from any
  target so `systemctl enable --now drone-control-gc.target` on a real fleet
  will never accidentally start the simulator.

## Editing after rebuild

After every `colcon build`, no unit file edits are needed — the wrapper
sources `/root/ros2_ws/install/setup.bash` at each service start, which
resolves to the newly-built code automatically.

## Uninstall

```bash
systemctl disable --now drone-control-$ROLE.target
rm /etc/systemd/system/drone-control-*.{service,target}
rm /usr/local/bin/drone-control-run /etc/default/drone-control
systemctl daemon-reload
```
