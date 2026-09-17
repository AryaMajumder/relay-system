#!/usr/bin/env bash
# px4_launch.sh — launch N headless PX4+JMAVSim instances in tmux
#
# Usage:
#   ./px4_launch.sh         — 2 instances (default)
#   ./px4_launch.sh 3       — 3 instances
#   ./px4_launch.sh kill    — kill everything
#
# Navigate : Ctrl+b 0 / 1 / 2 ...
# Detach   : Ctrl+b d
# Reattach : tmux attach -t px4

PX4_ROOT="$(cd "$(dirname "$0")" && pwd)"
PX4_BIN="${PX4_ROOT}/build/px4_sitl_default/bin/px4"
PX4_ETC="${PX4_ROOT}/build/px4_sitl_default/etc"
JMAVSIM="${PX4_ROOT}/Tools/simulation/jmavsim/jmavsim_run.sh"
SESSION="px4"

# How long to wait for PX4 to boot before connecting JMAVSim
JMAVSIM_DELAY=15

die() { echo "ERROR: $*" >&2; exit 1; }

cleanup() {
    pkill -9 -f "bin/px4"     2>/dev/null || true
    pkill -9 -f "jmavsim_run" 2>/dev/null || true
    pkill -9 -f "jMAVSim"     2>/dev/null || true
    sleep 1
    rm -f /tmp/px4-sock-* /tmp/px4_lock-*
    rm -f /tmp/jmavsim_*.log
    rm -rf /tmp/sitl_iris_*
    tmux kill-server 2>/dev/null || true
    sleep 1
}

[ "$1" = "kill" ] && { cleanup; echo "Done."; exit 0; }

COUNT=${1:-2}
[[ "${COUNT}" =~ ^[0-9]+$ ]] || die "Usage: $0 [count]"
[ "${COUNT}" -ge 1 ]         || die "Count must be >= 1"

command -v tmux >/dev/null 2>&1       || die "tmux not found: apt-get install -y tmux"
[ -f "${PX4_BIN}" ]                   || die "PX4 not built."
[ -f "${PX4_ETC}/init.d-posix/rcS" ] || die "Build artifacts missing."

echo "Cleaning up..."
cleanup

echo "======================================"
echo "  Launching ${COUNT} PX4 instance(s)"
echo "  Ctrl+b 0 / 1 / 2 ... to switch"
echo "  Ctrl+b d to detach"
echo "======================================"

# ── PX4 param overrides ──────────────────────────────────────────────────────
# rcS picks up any environment variable named PX4_PARAM_<PARAM_NAME> and runs
# `param set <PARAM_NAME> <value>` during boot (see rcS line ~134).  These
# persist for the SITL session (until PX4 restart) without needing to modify
# the ROMFS or hand-run `param save`.
#
# COM_OBL_RC_ACT=5 (AUTO.LOITER=HOLD) — when OFFBOARD is lost and RC is
# available (jMAVSim always reports RC available), PX4 falls back to this
# mode.  The default 0=POSCTL is unrecoverable by our relay_mover, which only
# knows how to drive PX4 back to OFFBOARD from HOLD via SET_MODE HOLD +
# START_LEAD.  See SESSION_LOG 2026-09-12 entry.
export PX4_PARAM_COM_OBL_RC_ACT=5

# ── Lockstep kept ENABLED (2026-09-14) ────────────────────────────────────────
# Investigated disabling: PX4's IMU pipeline validates monotonic sensor
# timestamps independently of the lockstep scheduler flag, so jMAVSim without
# `-lockstep` produced a flood of `vehicle_imu timestamp error` and killed
# sensor input.  Sticking with lockstep on both sides.  If sim rate is
# unacceptable, consider Gazebo (better multi-instance behaviour) instead of
# jMAVSim.

# ── Window 0: Instance 0 (uniform launch, no make) ────────────────────────────
# Using the same direct-binary + jmavsim_run.sh path as instance 1+ so the
# lockstep flag can be controlled uniformly. Skips the make step; assumes PX4
# is already built (which it is — px4-launch.sh checks for ${PX4_BIN} above).
WORK_DIR_0="/tmp/sitl_iris_0"
mkdir -p "${WORK_DIR_0}"
[ -L "${WORK_DIR_0}/etc" ] || ln -s "${PX4_ETC}" "${WORK_DIR_0}/etc"

tmux new-session -d -s "${SESSION}" -n "px4-0"
tmux send-keys -t "${SESSION}:0" "cd ${WORK_DIR_0}" Enter
tmux send-keys -t "${SESSION}:0" \
    "export PATH=\$PATH:${PX4_ETC}/init.d-posix" Enter
tmux send-keys -t "${SESSION}:0" \
    "PX4_INSTANCE=0 PX4_SIM_MODEL=jmavsim_iris \
PX4_PARAM_COM_OBL_RC_ACT=5 \
${PX4_BIN} -i 0 ${PX4_ETC} -w ${WORK_DIR_0} \
-s ${PX4_ETC}/init.d-posix/rcS" Enter

# jMAVSim for instance 0 — with -lockstep to match PX4 build.
tmux new-window -t "${SESSION}:" -n "sim-0"
tmux send-keys -t "${SESSION}:sim-0" \
    "echo 'Waiting ${JMAVSIM_DELAY}s for PX4 instance 0...' && \
sleep ${JMAVSIM_DELAY} && \
echo 'Connecting JMAVSim on port 4560...' && \
cd ${PX4_ROOT} && HEADLESS=1 ${JMAVSIM} -p 4560 -r 250 -l" Enter

# ── Windows 1..N-1 ────────────────────────────────────────────────────────────
for i in $(seq 1 $((COUNT - 1))); do
    SIM_PORT=$((4560 + i))
    WORK_DIR="/tmp/sitl_iris_${i}"

    mkdir -p "${WORK_DIR}"
    [ -L "${WORK_DIR}/etc" ] || ln -s "${PX4_ETC}" "${WORK_DIR}/etc"

    # PX4 window — foreground, becomes the live PX4 shell.
    # NOTE: use the window NAME (px4-${i}) as the send-keys target — the
    # window-INDEX numbering shifted when we added sim-0 as an extra window,
    # so index i no longer corresponds to px4-i.  Naming is unambiguous.
    tmux new-window -t "${SESSION}:" -n "px4-${i}"
    tmux send-keys -t "${SESSION}:px4-${i}" "cd ${WORK_DIR}" Enter
    tmux send-keys -t "${SESSION}:px4-${i}" \
        "export PATH=\$PATH:${PX4_ETC}/init.d-posix" Enter
    tmux send-keys -t "${SESSION}:px4-${i}" \
        "PX4_INSTANCE=${i} PX4_SIM_MODEL=jmavsim_iris \
PX4_PARAM_COM_OBL_RC_ACT=5 PX4_SIM_STANDALONE=1 \
${PX4_BIN} -i ${i} ${PX4_ETC} -w ${WORK_DIR} \
-s ${PX4_ETC}/init.d-posix/rcS" Enter

    # JMAVSim window — waits then connects.  -lockstep matches PX4 build.
    tmux new-window -t "${SESSION}:" -n "sim-${i}"
    tmux send-keys -t "${SESSION}:sim-${i}" \
        "echo 'Waiting ${JMAVSIM_DELAY}s for PX4 instance ${i}...' && \
sleep ${JMAVSIM_DELAY} && \
echo 'Connecting JMAVSim on port ${SIM_PORT}...' && \
cd ${PX4_ROOT} && HEADLESS=1 ${JMAVSIM} -p ${SIM_PORT} -r 250 -l" Enter
done

tmux select-window -t "${SESSION}:px4-0"
tmux attach -t "${SESSION}"
