"""
tree_builder.py — Constructs the relay decision behavior tree.

BUILDSPEC: §4.6, §4.7
LAYER:     1 (pure construction, no I/O)
SUBSCRIBES: none
PUBLISHES:  none

Root Selector dispatches on role (RELAYING vs IDLE):

  RelayRoot [Selector]
  ├── RELAYING_BRANCH [Sequence]        gated IsAlreadyRelaying
  │     ├── IsAlreadyRelaying
  │     ├── BandSensorNode              maintains R_target every maintenance tick
  │     └── ARBITER_SCAN [Selector]     priority scan, gates 1-9 + reauth timeout
  │           ├── G1  Seq(Inv(FcuTelemetryFresh),             FollowerSafetyExit)
  │           ├── G2  Seq(Inv(BatteryStillSufficientToRelay), FollowerSafetyExit)
  │           ├── G3  Seq(Inv(OffboardModeHeld),              FollowerSafetyExit)
  │           ├── G4  Seq(Inv(PositionServiceable),           ProposeExitRelay)
  │           ├── G5  Seq(Inv(RfLinkTelemetryFresh),         ProposeExitRelay)
  │           ├── G6  Seq(Inv(RelayStillNeeded),             ProposeExitRelay)
  │           ├── G7  Seq(Inv(RelayLinkAdequate),            G7_HANDLER)
  │           │         G7_HANDLER [Selector]
  │           │           ├── Seq(DriftedFromBand, ProposeReposition)
  │           │           └── ProposeExitRelay
  │           ├── G8  Seq(RelayActuallyImproved, AlwaysFail)  writes reauth_requested_at on FAIL
  │           ├── REAUTH_TIMEOUT  Seq(Inv(ReauthResponseTimedOut), ProposeExitRelay)
  │           ├── G9  Seq(GpsHealthy, AlwaysFail)             diagnostic only
  │           └── CONTINUE  AlwaysSucceed                     all gates passed
  └── IDLE_BRANCH [Sequence]            gated NotAlreadyRelaying
        ├── NotAlreadyRelaying
        ├── RelayRequestReceived        sole entry trigger
        └── TASKING_RESPONSE [Selector]
              ├── FULL_ENTRY [Sequence]
              │     ├── TaskingIsValid
              │     ├── BandSensorNode
              │     ├── LeaderReachabilityFresh   F_cap leader-reachability (§4.6)
              │     ├── GeometryFeasible
              │     ├── DataFreshness
              │     ├── GPSFixAdequate
              │     ├── FlightModeAcceptable
              │     ├── BatteryAboveFloor
              │     ├── GeofenceContainsRelayPos
              │     ├── BatterySufficientForReturn
              │     └── STRATEGY_SELECTION [Selector]
              │           ├── Seq(SingleFollowerSufficient, ProposeContinuousRelay)
              │           ├── Seq(ChainFeasible, ProposeChainRelay)
              │           └── ProposeLetLeaderIsolate
              └── ProposeLetLeaderIsolate  (capability or tasking-validity fail)
"""

import logging

import py_trees

from .blackboard import TimestampedBlackboard
from .condition_nodes import (
    # role gates
    IsAlreadyRelaying,
    NotAlreadyRelaying,
    # band sensor
    BandSensorNode,
    # entry gates
    RelayRequestReceived,
    TaskingIsValid,
    # F_cap leader-reachability (§4.6 addition)
    LeaderReachabilityFresh,
    # entry capability checks
    GeometryFeasible,
    DataFreshness,
    GPSFixAdequate,
    FlightModeAcceptable,
    BatteryAboveFloor,
    GeofenceContainsRelayPos,
    BatterySufficientForReturn,
    SingleFollowerSufficient,
    ChainFeasible,
    # maintenance gates 1-9
    FcuTelemetryFresh,
    BatteryStillSufficientToRelay,
    OffboardModeHeld,
    PositionServiceable,
    RfLinkTelemetryFresh,
    RelayStillNeeded,
    RelayLinkAdequate,
    RelayActuallyImproved,
    GpsHealthy,
    # reauth timeout (additional, alongside the 9 gates)
    ReauthResponseTimedOut,
    # reposition condition
    DriftedFromBand,
)
from .action_nodes import (
    ProposeContinuousRelay,
    ProposeChainRelay,
    ProposeLetLeaderIsolate,
    FollowerSafetyExit,
    ProposeReposition,
    ProposeExitRelay,
)

log = logging.getLogger(__name__)


# ── Utility leaf nodes ────────────────────────────────────────────────────────

class _AlwaysFail(py_trees.behaviour.Behaviour):
    def update(self):
        return py_trees.common.Status.FAILURE


class _AlwaysSucceed(py_trees.behaviour.Behaviour):
    def update(self):
        return py_trees.common.Status.SUCCESS


def _fail(name: str) -> py_trees.behaviour.Behaviour:
    return _AlwaysFail(name=name)


def _succeed(name: str) -> py_trees.behaviour.Behaviour:
    return _AlwaysSucceed(name=name)


# ── Tree builder ──────────────────────────────────────────────────────────────

def build_relay_decision_tree(bb: TimestampedBlackboard,
                              config: dict,
                              clock=None) -> py_trees.composites.Selector:
    """
    Returns root Selector with the full relay decision tree attached.
    All nodes are new instances — no shared node objects.
    clock is injectable for tests (TEST_PROTOCOL §3.3).
    """

    def _n(cls, *args, **kwargs):
        # Pass clock so time-dependent nodes (G8, ReauthResponseTimedOut,
        # LeaderReachabilityFresh) can be tested with FakeClock.
        return cls(bb=bb, config=config, clock=clock, *args, **kwargs)

    def _inv(node) -> py_trees.decorators.Inverter:
        return py_trees.decorators.Inverter(name=f"NOT({node.name})", child=node)

    def _seq(name, *children) -> py_trees.composites.Sequence:
        # memory=False: no memory-mode hysteresis — each tick re-evaluates from the first child.
        # Memory mode would skip already-succeeded children, hiding re-check of conditions.
        s = py_trees.composites.Sequence(name=name, memory=False)
        s.add_children(list(children))
        return s

    def _sel(name, *children) -> py_trees.composites.Selector:
        # memory=False: same rationale — always re-scan from the first alternative.
        s = py_trees.composites.Selector(name=name, memory=False)
        s.add_children(list(children))
        return s

    # ── RELAYING_BRANCH ───────────────────────────────────────────────────────

    # Gate 7 handler: reposition if drifted + gain ok, else exit
    g7_handler = _sel("G7_HANDLER",
        _seq("REPOSITION",
             _n(DriftedFromBand),
             _n(ProposeReposition),
        ),
        _n(ProposeExitRelay, name="ProposeExitRelay(G7)"),
    )

    arbiter_scan = _sel("ARBITER_SCAN",
        # G1: FCU telemetry fresh (forced-safety, no debounce)
        _seq("G1_FCU",
             _inv(_n(FcuTelemetryFresh)),
             _n(FollowerSafetyExit, name="FollowerSafetyExit(G1)"),
        ),
        # G2: battery stay predicate (forced-safety, no debounce)
        _seq("G2_BATTERY",
             _inv(_n(BatteryStillSufficientToRelay)),
             _n(FollowerSafetyExit, name="FollowerSafetyExit(G2)"),
        ),
        # G3: OFFBOARD mode held (RUNNING during recovery, FAILURE → safety exit)
        _seq("G3_OFFBOARD",
             _inv(_n(OffboardModeHeld)),
             _n(FollowerSafetyExit, name="FollowerSafetyExit(G3)"),
        ),
        # G4: relay position serviceable (geofence + band feasibility)
        _seq("G4_POSITION",
             _inv(_n(PositionServiceable)),
             _n(ProposeExitRelay, name="ProposeExitRelay(G4)"),
        ),
        # G5: RF-link telemetry fresh (stale > 5s → forced exit)
        _seq("G5_RF_FRESH",
             _inv(_n(RfLinkTelemetryFresh)),
             _n(ProposeExitRelay, name="ProposeExitRelay(G5)"),
        ),
        # G6: relay still needed (direct link still poor — hysteresis at 0.85)
        _seq("G6_NEEDED",
             _inv(_n(RelayStillNeeded)),
             _n(ProposeExitRelay, name="ProposeExitRelay(G6)"),
        ),
        # G7: relay link adequate — PRIMARY trigger, N=3 debounce
        _seq("G7_LINK",
             _inv(_n(RelayLinkAdequate)),
             g7_handler,
        ),
        # G8: authorization position+timer still valid.
        # Diagnostic-style (Seq with AlwaysFail ensures scan always continues),
        # but RelayActuallyImproved writes reauth_requested_at on FAIL as a side effect.
        _seq("G8_AUTH",
             _n(RelayActuallyImproved),
             _fail("G8_AlwaysFail"),
        ),
        # REAUTH_TIMEOUT: if reauth request timed out → ProposeExitRelay
        _seq("REAUTH_TIMEOUT",
             _inv(_n(ReauthResponseTimedOut)),
             _n(ProposeExitRelay, name="ProposeExitRelay(ReauthTimeout)"),
        ),
        # G9: GPS health — diagnostic only, never blocks scan
        _seq("G9_DIAG",
             _n(GpsHealthy),
             _fail("G9_AlwaysFail"),
        ),
        # All gates passed — continue relaying
        _succeed("CONTINUE"),
    )

    relaying_branch = _seq("RELAYING_BRANCH",
        _n(IsAlreadyRelaying),
        _n(BandSensorNode, name="BandSensorNode(relay)"),
        arbiter_scan,
    )

    # ── IDLE_BRANCH ───────────────────────────────────────────────────────────

    strategy_selection = _sel("STRATEGY_SELECTION",
        _seq("TRY_CONTINUOUS",
             _n(SingleFollowerSufficient),
             _n(ProposeContinuousRelay),
        ),
        _seq("TRY_CHAIN",
             _n(ChainFeasible),
             _n(ProposeChainRelay),
        ),
        _n(ProposeLetLeaderIsolate, name="ProposeLetLeaderIsolate(NoStrategy)"),
    )

    full_entry = _seq("FULL_ENTRY",
        _n(TaskingIsValid),
        _n(BandSensorNode, name="BandSensorNode(entry)"),
        # F_cap leader-reachability check (§4.6): decline early if leader
        # radio_health is never-seen or stale, before running geometry checks.
        _n(LeaderReachabilityFresh),
        _n(GeometryFeasible),
        _n(DataFreshness),
        _n(GPSFixAdequate),
        _n(FlightModeAcceptable),
        _n(BatteryAboveFloor),
        _n(GeofenceContainsRelayPos),
        _n(BatterySufficientForReturn),
        strategy_selection,
    )

    tasking_response = _sel("TASKING_RESPONSE",
        full_entry,
        _n(ProposeLetLeaderIsolate, name="ProposeLetLeaderIsolate(CapFail)"),
    )

    idle_branch = _seq("IDLE_BRANCH",
        _n(NotAlreadyRelaying),
        _n(RelayRequestReceived),
        tasking_response,
    )

    # ── Root ──────────────────────────────────────────────────────────────────
    root = _sel("RelayRoot",
        relaying_branch,
        idle_branch,
    )

    log.info("Relay decision tree built:\n%s", render_tree_ascii(root))
    return root


# ── Helpers ───────────────────────────────────────────────────────────────────

def render_tree_ascii(tree) -> str:
    return py_trees.display.ascii_tree(tree)


def walk_tree(node):
    """Generator — yields every node in the tree via depth-first traversal."""
    yield node
    if hasattr(node, "children"):
        for child in node.children:
            yield from walk_tree(child)


def dump_state(tree) -> dict:
    """Returns {node_name: {status, feedback}} for every node in the tree."""
    return {
        node.name: {
            "status":   node.status.name,
            "feedback": getattr(node, "feedback_message", ""),
        }
        for node in walk_tree(tree)
    }
