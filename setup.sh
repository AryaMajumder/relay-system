#!/usr/bin/env bash
# setup.sh — install the bundle onto a Debian/Ubuntu host for SITL demo.
#
# What this does:
#   1. Verifies expected prerequisites (ROS 2 Humble, mosquitto, python deps).
#   2. Copies mosquitto broker configs to /etc/mosquitto/ and installs their
#      systemd units.
#   3. Copies the CycloneDDS config to /etc/ros/cyclonedds.xml.
#   4. Installs the px4_agent daemon to /opt/drone-command/ and its systemd
#      units.
#   5. Installs the drone_control systemd base units from
#      drone_control/deploy/systemd/ and their .d overrides from deploy/dropins/.
#   6. Reloads systemd, enables + starts the brokers and daemon.
#
# What this does NOT do:
#   - Install ROS 2 Humble or the apt packages. Do that first per the ROS 2
#     Humble install guide.
#   - Build PX4-Autopilot. Clone and build it separately; deploy/sitl/px4-launch.sh
#     assumes it's at $PX4_ROOT.
#   - Provision the SROS 2 keystore. The .d/enclave.conf drop-ins expect one at
#     /root/sros2/keystore — generate one manually or set ROS_SECURITY_ENABLE=false
#     in /etc/default/drone-control (already the default).
#   - Set MQTT passwords. Write your own credential to /etc/drone-pub/mqtt_pass.txt
#     and add matching entries to mosquitto's password file.
#   - colcon build the ROS 2 package. Run `cd drone_control && colcon build` after.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

# --- prereq check ---
require() {
  command -v "$1" >/dev/null 2>&1 || { echo "MISSING: $1 — install it before running setup.sh" >&2; exit 1; }
}
echo "→ Checking prerequisites"
require mosquitto
require python3
require systemctl
[ -f /opt/ros/humble/setup.bash ] || { echo "MISSING: ROS 2 Humble at /opt/ros/humble" >&2; exit 1; }

# --- python deps ---
echo "→ Checking python packages"
python3 -c "import paho.mqtt.client, pymavlink, py_trees" 2>/dev/null \
  || { echo "MISSING python: pip install paho-mqtt pymavlink py_trees" >&2; exit 1; }

# --- shared env file ---
echo "→ Installing shared env file"
sudo mkdir -p /etc/default
if [ -f "$HERE/drone_control/deploy/systemd/common/drone-control.env" ]; then
  sudo install -m 0644 "$HERE/drone_control/deploy/systemd/common/drone-control.env" /etc/default/drone-control
else
  echo "  (drone-control.env template not found; skipping — services expect /etc/default/drone-control)" >&2
fi

# --- wrapper script ---
echo "→ Installing drone-control-run wrapper"
sudo install -m 0755 "$HERE/drone_control/deploy/systemd/common/drone-control-run" /usr/local/bin/drone-control-run

# --- CycloneDDS config ---
echo "→ Installing CycloneDDS config"
sudo mkdir -p /etc/ros
sudo install -m 0644 "$HERE/deploy/ros/cyclonedds.xml" /etc/ros/cyclonedds.xml

# --- mosquitto configs + units ---
echo "→ Installing mosquitto broker configs and units"
sudo install -m 0644 "$HERE/deploy/mosquitto/mosquitto-drone01.conf" /etc/mosquitto/
sudo install -m 0644 "$HERE/deploy/mosquitto/mosquitto-drone02.conf" /etc/mosquitto/
sudo install -m 0644 "$HERE/deploy/mosquitto/systemd/mosquitto-drone01.service" /etc/systemd/system/
sudo install -m 0644 "$HERE/deploy/mosquitto/systemd/mosquitto-drone02.service" /etc/systemd/system/

# --- px4_agent daemon + units ---
echo "→ Installing px4_agent daemon"
sudo mkdir -p /opt/drone-command
sudo install -m 0755 "$HERE/deploy/daemons/px4_agent.py" /opt/drone-command/px4_agent.py
[ -d /opt/drone-command/venv ] || python3 -m venv /opt/drone-command/venv
sudo /opt/drone-command/venv/bin/pip install --quiet paho-mqtt pymavlink

sudo install -m 0644 "$HERE/deploy/daemons/systemd/px4-agent.service" /etc/systemd/system/
sudo install -m 0644 "$HERE/deploy/daemons/systemd/px4-agent-drone-02.service" /etc/systemd/system/
sudo mkdir -p /etc/systemd/system/px4-agent-drone-02.service.d
sudo install -m 0644 "$HERE/deploy/daemons/systemd/px4-agent-drone-02.service.d/sitl-ports.conf" \
                     /etc/systemd/system/px4-agent-drone-02.service.d/
sudo mkdir -p /var/lib/px4_agent

# --- drone_control base systemd units (from the ROS 2 package) ---
echo "→ Installing drone_control base systemd units"
for role in common leader follower gc sitl; do
  d="$HERE/drone_control/deploy/systemd/$role"
  [ -d "$d" ] || continue
  for f in "$d"/*.service "$d"/*.target; do
    [ -f "$f" ] || continue
    sudo install -m 0644 "$f" /etc/systemd/system/
  done
done

# --- drop-ins (SROS 2 enclaves, SITL ports, drone-id) ---
echo "→ Installing systemd drop-in overrides"
for role in leader follower gc; do
  for svc_dir in "$HERE/deploy/dropins/$role"/*.service.d; do
    [ -d "$svc_dir" ] || continue
    svc=$(basename "$svc_dir")
    sudo mkdir -p "/etc/systemd/system/$svc"
    sudo install -m 0644 "$svc_dir"/*.conf "/etc/systemd/system/$svc/"
  done
done

# --- reload + enable brokers and daemon ---
echo "→ Reloading systemd"
sudo systemctl daemon-reload

echo "→ Enabling and starting brokers + px4_agent"
sudo systemctl enable --now mosquitto-drone01.service mosquitto-drone02.service
sudo systemctl enable --now px4-agent.service px4-agent-drone-02.service

echo
echo "── setup.sh complete ──"
echo
echo "Next steps:"
echo "  1. Build the ROS 2 package:    cd drone_control && colcon build"
echo "  2. Bring up the simulator:     ./deploy/sitl/px4-launch.sh 2"
echo "  3. Enable role targets:        sudo systemctl start drone-control-{gc,leader,follower}.target"
echo "  4. Watch the drones:           python3 dashboards/dashboard.py    (http://localhost:5000)"
echo
echo "See drone_control/README.md for design + test results."
