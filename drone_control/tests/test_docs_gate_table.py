"""
test_docs_gate_table.py — guard against the README gate table drifting
away from MAINTENANCE_GATES and the DIAG_SCAN membership.

If this test fails, update drone_control/README.md gate table (and the mirror
in docs/CURRENT_DESIGN.md if applicable) rather than silencing the test.
"""

import os
import re

from drone_control.relay_bt.condition_nodes import MAINTENANCE_GATES


_README_PATH   = os.path.join(os.path.dirname(__file__), "..", "README.md")
_DIAG_IN_SCAN  = {"RelayActuallyImproved", "GpsHealthy"}      # DIAG_SCAN gates (§4.6, G8+G9)
_ARBITER_SCAN  = {"FcuTelemetryFresh", "BatteryStillSufficientToRelay",
                  "OffboardModeHeld", "PositionServiceable",
                  "RfLinkTelemetryFresh", "RelayStillNeeded", "RelayLinkAdequate"}


def _parse_gate_rows(markdown: str):
    """Extract gate rows from README's | Gx | ClassName | Selector | ... | table."""
    rows = []
    row_re = re.compile(
        r"^\|\s*\*\*G(\d)\*\*\s*`([A-Za-z_]+)`\s*\|\s*(ARBITER_SCAN|DIAG_SCAN)\b",
        re.MULTILINE,
    )
    for m in row_re.finditer(markdown):
        rows.append((int(m.group(1)), m.group(2), m.group(3)))
    return rows


def test_readme_gate_table_matches_maintenance_gates():
    """
    README's gate table must list exactly the nine gates in MAINTENANCE_GATES,
    in order G1..G9, with the correct selector attribution:
      G1..G7 → ARBITER_SCAN
      G8, G9 → DIAG_SCAN
    """
    with open(_README_PATH) as f:
        md = f.read()

    rows = _parse_gate_rows(md)
    assert rows, (
        "No `| **G<n>** `ClassName` | ARBITER_SCAN/DIAG_SCAN |` rows found "
        "in README.md. The gate table changed shape — update this test or "
        "restore the table."
    )

    # (1) Exactly nine rows, numbered 1..9
    assert [n for n, _, _ in rows] == list(range(1, 10)), (
        f"README gate table must have rows G1..G9 in order; got {[n for n,_,_ in rows]}"
    )

    # (2) Class names in that order must match MAINTENANCE_GATES
    expected_classes = [cls.__name__ for cls in MAINTENANCE_GATES]
    actual_classes   = [cls_name for _, cls_name, _ in rows]
    assert actual_classes == expected_classes, (
        "README gate class names drifted from MAINTENANCE_GATES.\n"
        f"  README:  {actual_classes}\n"
        f"  Code:    {expected_classes}"
    )

    # (3) Selector attribution — G1..G7 ARBITER_SCAN, G8..G9 DIAG_SCAN
    for n, cls_name, selector in rows:
        if n in (8, 9):
            assert cls_name in _DIAG_IN_SCAN, (
                f"Row G{n} class {cls_name!r} should be in DIAG_SCAN {_DIAG_IN_SCAN}"
            )
            assert selector == "DIAG_SCAN", (
                f"Row G{n} ({cls_name}) must be labelled DIAG_SCAN, got {selector}"
            )
        else:
            assert cls_name in _ARBITER_SCAN, (
                f"Row G{n} class {cls_name!r} should be in ARBITER_SCAN {_ARBITER_SCAN}"
            )
            assert selector == "ARBITER_SCAN", (
                f"Row G{n} ({cls_name}) must be labelled ARBITER_SCAN, got {selector}"
            )
