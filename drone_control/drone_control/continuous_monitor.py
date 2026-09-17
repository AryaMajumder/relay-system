"""
continuous_monitor.py — ROS2 node.

Watches signal quality metrics for changes that warrant a fresh
BT assessment cycle, then publishes /drone_NN/reeval_trigger.

BUILDSPEC §4.13: SNR step-changes only. Loss branch removed.

Watches:
  /drone_NN/signal_report      — trigger if follower_to_gc SNR drops > SNR_TRIGGER_DB
  /drone_NN/relay_confirmed    — trigger on relay completion (role can now reset)
  /drone_NN/capability_report  — gates suppression logic
  /drone_NN/current_role       — suppresses if MOVING_TO_RELAY

Publishes:
  /drone_NN/reeval_trigger  — {trigger_id, drone_id, timestamp, reason, delta}

Suppression rules:
  - Cooldown: ≤1 trigger per COOLDOWN_S (default 5 s)
  - capability_report.status in (STALE_DATA, EVALUATING, WAITING_DATA) → skip
  - current_role == MOVING_TO_RELAY → skip

Environment variables:
  DRONE_ID            drone identifier                 (default: drone-01)
  SNR_TRIGGER_DB      SNR drop threshold in dB         (default: 5.0)
  REEVAL_COOLDOWN_S   minimum seconds between triggers (default: 5.0)
  LOG_LEVEL           logging level                    (default: INFO)
"""

import json
import logging
import os
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# Must match publisher QoS on /current_role (strategy_executor,
# relay_position_tracker). QoS mismatch silently drops all messages.
CURRENT_ROLE_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────
DRONE_ID       = os.environ.get("DRONE_ID",           "drone-01")
SNR_TRIGGER_DB = float(os.environ.get("SNR_TRIGGER_DB",    "5.0"))
COOLDOWN_S     = float(os.environ.get("REEVAL_COOLDOWN_S", "5.0"))
LOG_LEVEL      = os.environ.get("LOG_LEVEL", "INFO")

# Statuses that mean assessment is already blocked — no point triggering
_SUPPRESS_STATUSES = {"STALE_DATA", "EVALUATING", "WAITING_DATA"}

# ─────────────────────────────────────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=LOG_LEVEL,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)
log = logging.getLogger("continuous_monitor")


def _ros_prefix(drone_id: str) -> str:
    return f"/{drone_id.replace('-', '_')}"


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────
class ContinuousMonitor(Node):
    """
    Watches signal telemetry and relay lifecycle events.
    Publishes reeval_trigger when SNR step-changes or relay completes.
    BUILDSPEC §4.13: loss branch removed; SNR step-changes only.
    """

    def __init__(self):
        node_name = f"continuous_monitor_{DRONE_ID.replace('-', '_')}"
        super().__init__(node_name)

        prefix = _ros_prefix(DRONE_ID)

        # ── Trigger state ─────────────────────────────────────────────────────
        self._last_trigger_ts = 0.0
        self._last_cap_status = None
        self._current_role    = None
        self._baseline_snr_db = None

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(String, f"{prefix}/reeval_trigger", 10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String, f"{prefix}/signal_report",
            self._on_signal_report, 10)
        self.create_subscription(
            String, f"{prefix}/relay_confirmed",
            self._on_relay_confirmed, 10)
        self.create_subscription(
            String, f"{prefix}/capability_report",
            self._on_capability_report, 10)
        self.create_subscription(
            String, f"{prefix}/current_role",
            lambda m: setattr(self, "_current_role", m.data.strip()), CURRENT_ROLE_QOS)

        self.get_logger().info(
            f"continuous_monitor started: drone={DRONE_ID} "
            f"snr_threshold={SNR_TRIGGER_DB}dB "
            f"cooldown={COOLDOWN_S}s"
        )

    # ── Gate check ────────────────────────────────────────────────────────────

    def _should_suppress(self) -> tuple[bool, str]:
        """Return (suppress, reason). True means do NOT trigger."""
        now = time.time()

        elapsed = now - self._last_trigger_ts
        if elapsed < COOLDOWN_S:
            return True, f"cooldown ({elapsed:.1f}s < {COOLDOWN_S}s)"

        if self._last_cap_status in _SUPPRESS_STATUSES:
            return True, f"cap_status={self._last_cap_status}"

        if self._current_role == "MOVING_TO_RELAY":
            return True, "current_role=MOVING_TO_RELAY"

        return False, ""

    # ── Trigger dispatch ──────────────────────────────────────────────────────

    def _fire(self, reason: str, delta: str | None = None) -> bool:
        """
        Publish a reeval_trigger.  Returns True iff a message was actually
        published (i.e. NOT suppressed).  The return value gates the caller's
        baseline-update logic — see §4.8 ordering-hole verification 2026-08-25:
        walking the baseline on a suppressed fire permanently swallows the
        underlying signal drop, because the next comparison uses the new
        (lower) baseline and computes drop=0.
        """
        if reason == "relay_completed":
            suppress, why = self._should_suppress()
            # Race condition bypass: relay_confirmed fires on the same tick
            # relay_position_tracker detects arrival, but the RELAYING role publish
            # is a separate ROS2 message that may not have been received yet.
            # Without this carve-out, the MOVING_TO_RELAY suppression would swallow
            # the trigger that kicks off the re-authorization loop after relay completes.
            if suppress and "MOVING_TO_RELAY" not in why:
                log.debug("reeval suppressed: %s (would-be reason=%s)", why, reason)
                return False
        else:
            suppress, why = self._should_suppress()
            if suppress:
                log.debug("reeval suppressed: %s (would-be reason=%s)", why, reason)
                return False

        self._last_trigger_ts = time.time()

        payload = {
            "trigger_id": f"trig-{uuid.uuid4().hex[:8]}",
            "drone_id":   DRONE_ID,
            "timestamp":  self._last_trigger_ts,
            "reason":     reason,
            "delta":      delta,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)

        log.info(
            "reeval_trigger: reason=%s delta=%s trigger_id=%s",
            reason, delta, payload["trigger_id"],
        )
        self.get_logger().info(
            f"reeval_trigger published: reason={reason!r} delta={delta!r} "
            f"id={payload['trigger_id']}"
        )
        return True

    # ── Topic callbacks ───────────────────────────────────────────────────────

    def _on_signal_report(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        snr = (data.get("follower_to_gc") or {}).get("snr_db")
        if snr is None:
            return

        if self._baseline_snr_db is None:
            self._baseline_snr_db = snr
            return

        drop = self._baseline_snr_db - snr
        if drop > SNR_TRIGGER_DB:
            delta_str = (
                f"follower_to_gc snr_db: {self._baseline_snr_db:.1f} → {snr:.1f} "
                f"(drop={drop:.1f}dB)"
            )
            fired = self._fire("snr_degraded", delta_str)
            # Only advance baseline when the trigger actually fired.  If _fire
            # suppressed (e.g. MOVING_TO_RELAY gate), keep the original baseline
            # so the underlying degradation is caught the next time the gate
            # opens.  Verified 2026-08-25 by test_suppressed_snr_degradation_
            # is_recovered_after_role_reverts — §4.8 ordering-hole fix.
            if fired:
                self._baseline_snr_db = snr

    def _on_relay_confirmed(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        assign_id = data.get("assignment_id", "?")
        status    = data.get("status", "?")
        self._fire(
            "relay_completed",
            f"assign_id={assign_id} status={status}",
        )

    def _on_capability_report(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._last_cap_status = data.get("status")
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = ContinuousMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
