#!/usr/bin/env bash
# px4_shell.sh — open a MAVLink shell into a running PX4 SITL instance
#
# Usage:
#   ./px4_shell.sh 0    — connect to instance 0
#   ./px4_shell.sh 1    — connect to instance 1
#   ./px4_shell.sh 2    — connect to instance 2
#
# Requires: pymavlink installed (pip install pymavlink)
# The PX4 instance must already be running.
#
# Inside the shell you can type:
#   commander status
#   commander mode posctl
#   mavlink start -x -u 15100 -o 15101 -t 127.0.0.1 -r 4000000
#   listener vehicle_status
#   param show MAV_SYS_ID
#   exit

set -e

PX4_ROOT="$(cd "$(dirname "$0")" && pwd)"

die() { echo "ERROR: $*" >&2; exit 1; }

INSTANCE=$1
[ -z "${INSTANCE}" ]            && die "Usage: $0 <instance_number>  e.g. $0 0"
[[ "${INSTANCE}" =~ ^[0-9]+$ ]] || die "Instance must be a number"

# MAVLink API port — PX4 offsets by instance number automatically
PORT=$((14540 + INSTANCE))

# Find mavlink_shell.py
SHELL_SCRIPT=$(find "${PX4_ROOT}/Tools" -name "mavlink_shell.py" 2>/dev/null | head -1)
[ -z "${SHELL_SCRIPT}" ] && die "mavlink_shell.py not found under ${PX4_ROOT}/Tools"

# Check pymavlink is available
python3 -c "import pymavlink" 2>/dev/null || die "pymavlink not installed. Run: pip install pymavlink"

echo "======================================"
echo "  PX4 Shell — Instance ${INSTANCE}"
echo "  MAVLink port : ${PORT}"
echo "  Type 'exit' to disconnect"
echo "======================================"
echo ""

# Connect — this drops you into the interactive PX4 shell
python3 "${SHELL_SCRIPT}" "0.0.0.0:${PORT}"
