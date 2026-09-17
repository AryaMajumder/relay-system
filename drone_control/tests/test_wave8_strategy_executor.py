"""
test_wave8_strategy_executor.py — Wave 8 gate tests for strategy_executor.py.

TEST_PROTOCOL: §5.11  Archetype C (subscribe → compute → publish)
BUILDSPEC:     §4.8

PROVEN column:
  test_single_subscription          -> BUILDSPEC §4.8 "This is its only input"
  test_no_condition_logic           -> BUILDSPEC §4.8 hard rules
  test_continuous_relay_to_moving / test_chain_relay_to_moving / test_exit_relay_to_open -> BUILDSPEC §4.8 mapping (three of four rows; fourth is test_reposition_no_role_change)
  test_reposition_no_role_change    -> BUILDSPEC §4.8 REPOSITION_RELAY row
  test_exit_sources_indistinguishable -> BUILDSPEC §4.8 hard rule
"""

import sys
import os
import inspect
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
for p in (_PKG_ROOT, os.path.join(_PKG_ROOT, "drone_control")):
    if p not in sys.path:
        sys.path.insert(0, p)

from drone_control.strategy_executor import _StrategyExecutorCore


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_auth(strategy: str, proposal_id: str = "prop-abc000000001") -> dict:
    """
    Minimal §2.6 authorization payload.
    proposal_id is unique per test to avoid dedup guard suppressing subsequent calls.
    r_target / tolerance_radius_m / valid_until are §2.7 passthrough fields.
    """
    return {
        "proposal_id":        proposal_id,
        "drone_id":           "drone-01",
        "round_id":           "round-test",
        "strategy":           strategy,
        "r_target":           {"lat": 47.391, "lon": 8.541, "alt_m": 10.0},
        "tolerance_radius_m": 10.0,
        "valid_until":        12000.0,
        "timestamp":          10000.0,
    }


def _make_core():
    # publish_role_fn captures current_role publishes for assertion.
    # _StrategyExecutorCore is the pure-logic extract; StrategyExecutor is the ROS2 wrapper.
    roles = []
    core  = _StrategyExecutorCore(publish_role_fn=roles.append)
    return core, roles


# ── §4.8 thin: single subscription ───────────────────────────────────────────

class TestSingleSubscription:
    def test_single_subscription(self):
        """
        StrategyExecutor must create exactly ONE subscription: authorization.
        BUILDSPEC §4.8: "Subscribes: authorization (§2.6). This is its only input."
        Verified via source-code inspection of __init__.
        """
        import drone_control.strategy_executor as mod
        src = inspect.getsource(mod.StrategyExecutor.__init__)

        # Count create_subscription calls.
        sub_count = src.count("create_subscription(")
        assert sub_count == 1, (
            f"StrategyExecutor.__init__ must have exactly 1 create_subscription call, "
            f"found {sub_count}"
        )
        # Confirm it is for authorization.
        assert "authorization" in src, (
            "The single subscription must be for 'authorization'"
        )

    def test_no_relay_confirmed_subscription(self):
        """relay_confirmed subscription is explicitly absent (no RELAYING row in §4.8 table)."""
        import drone_control.strategy_executor as mod
        # Check code lines only (comments may mention it as a deliberate absence note).
        code_lines = [
            ln for ln in inspect.getsource(mod.StrategyExecutor.__init__).splitlines()
            if not ln.strip().startswith("#")
        ]
        code_body = "\n".join(code_lines)
        assert "relay_confirmed" not in code_body, (
            "relay_confirmed subscription must NOT exist in StrategyExecutor (§4.8 table has no such row)"
        )


# ── §4.8 hard rules: no condition logic ──────────────────────────────────────

class TestNoConditionLogic:
    def _code_lines(self, src: str) -> str:
        """Return source with comment-only lines removed."""
        return "\n".join(
            ln for ln in src.splitlines()
            if not ln.strip().startswith("#")
        )

    def test_no_condition_logic(self):
        """
        No timers, no threshold checks, no capability_report in the module.
        BUILDSPEC §4.8: "No condition checking. No timers. No independent abort logic."
        """
        import drone_control.strategy_executor as mod
        src = self._code_lines(inspect.getsource(mod))

        assert "create_timer" not in src, (
            "strategy_executor must not create any ROS2 timer (§4.8: no timers)"
        )
        assert "capability_report" not in src, (
            "strategy_executor must not reference capability_report (§4.8: no cap check)"
        )
        assert "battery" not in src.lower(), (
            "strategy_executor must not check battery (§4.8: no condition checking)"
        )

    def test_no_exit_decision(self):
        """
        This file never decides to exit — exits arrive as EXIT_RELAY through the pipeline.
        BUILDSPEC §4.8: "This file never decides to exit."
        """
        import drone_control.strategy_executor as mod
        src = self._code_lines(inspect.getsource(mod))
        # No independent snr/loss/timer checks that would cause an exit.
        assert "snr" not in src.lower(), "Must not check SNR (would be an independent exit decision)"
        assert "timeout" not in src.lower(), "Must not have timeout logic (exits arrive as EXIT_RELAY)"


# ── §4.8 mapping: all four rows ───────────────────────────────────────────────

class TestMappingComplete:
    def test_continuous_relay_to_moving(self):
        """CONTINUOUS_RELAY → MOVING_TO_RELAY. BUILDSPEC §4.8."""
        core, roles = _make_core()
        core.on_authorization(_make_auth("CONTINUOUS_RELAY"))
        assert len(roles) == 1
        assert roles[0] == "MOVING_TO_RELAY"

    def test_chain_relay_to_moving(self):
        """CHAIN_RELAY → MOVING_TO_RELAY. BUILDSPEC §4.8."""
        core, roles = _make_core()
        core.on_authorization(_make_auth("CHAIN_RELAY", proposal_id="prop-bbb000000001"))
        assert len(roles) == 1
        assert roles[0] == "MOVING_TO_RELAY"

    def test_exit_relay_to_open(self):
        """EXIT_RELAY → OPEN_TO_RELAY (§2.9 enum). BUILDSPEC §4.8."""
        core, roles = _make_core()
        core.on_authorization(_make_auth("EXIT_RELAY", proposal_id="prop-ccc000000001"))
        assert len(roles) == 1
        assert roles[0] == "OPEN_TO_RELAY", (
            f"EXIT_RELAY must map to OPEN_TO_RELAY per §2.9 enum, got {roles[0]!r}"
        )

    def test_exit_relay_not_idle(self):
        """EXIT_RELAY must produce 'OPEN_TO_RELAY', not the non-enum value 'IDLE'."""
        core, roles = _make_core()
        core.on_authorization(_make_auth("EXIT_RELAY", proposal_id="prop-ddd000000001"))
        assert roles[0] != "IDLE", "'IDLE' is not a §2.9 current_role value"


# ── §4.8 REPOSITION_RELAY: no role change ────────────────────────────────────

class TestRepositionNoRoleChange:
    def test_reposition_no_role_change(self):
        """
        REPOSITION_RELAY → no role published.
        BUILDSPEC §4.8: '*(no change)*' — chain_assigner updates target; no role emitted.
        """
        core, roles = _make_core()
        core.on_authorization(_make_auth("REPOSITION_RELAY", proposal_id="prop-eee000000001"))
        assert len(roles) == 0, (
            f"REPOSITION_RELAY must produce zero role publishes, got {roles}"
        )


# ── §4.8 hard rule: EXIT_RELAY sources indistinguishable ─────────────────────

class TestExitSourcesIndistinguishable:
    def test_exit_sources_indistinguishable(self):
        """
        EXIT_RELAY from battery vs. timeout → identical handling.
        BUILDSPEC §4.8: "It cannot tell — and must not try to tell — whether a given
        EXIT_RELAY came from a battery failure, a link failure, or the no-response
        timeout. All are identical here."
        Both must produce exactly one OPEN_TO_RELAY publish.
        """
        # Simulate "battery exit" — no extra fields, just EXIT_RELAY strategy.
        core1, roles1 = _make_core()
        auth_battery = _make_auth("EXIT_RELAY", proposal_id="prop-exit-battery")
        auth_battery["trigger_context"] = {"gate_fired": "BatterySufficientForReturn",
                                            "reason": "battery low", "source": "capability_assessor"}
        core1.on_authorization(auth_battery)

        # Simulate "timeout exit" — same shape, different trigger_context content.
        core2, roles2 = _make_core()
        auth_timeout = _make_auth("EXIT_RELAY", proposal_id="prop-exit-timeout")
        auth_timeout["trigger_context"] = {"gate_fired": "ReauthResponseTimedOut",
                                            "reason": "no response", "source": "capability_assessor"}
        core2.on_authorization(auth_timeout)

        # Both must produce exactly one OPEN_TO_RELAY.
        assert roles1 == ["OPEN_TO_RELAY"], f"Battery exit: {roles1}"
        assert roles2 == ["OPEN_TO_RELAY"], f"Timeout exit: {roles2}"

        # Outputs must be identical (indistinguishable from executor's perspective).
        assert roles1 == roles2, "EXIT_RELAY handling must be identical regardless of source"

    def test_dedup_same_proposal_not_applied_twice(self):
        """Same proposal_id received twice → role published only once.
        Defensive engineering — §4.8 is silent on proposal_id; mirrors chain_assigner's
        identical guard to prevent double role-change on ROS2 message replay."""
        core, roles = _make_core()
        auth = _make_auth("CONTINUOUS_RELAY", proposal_id="prop-dup000000001")
        core.on_authorization(auth)
        core.on_authorization(auth)  # duplicate
        assert len(roles) == 1, (
            "Same proposal_id processed twice → must produce exactly one role publish"
        )
