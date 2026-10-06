# Claude Code Standing Instructions — Relay System Build

## SESSION LOG (mandatory, no prompting required)

The session log lives at `/root/SESSION_LOG.md`. The protocol is at `/root/SESSION_LOG_PROTOCOL.md`.

**Log entries must be written automatically** — do not wait to be asked. Write an entry immediately when any of these occur:

- A choice is made between two viable approaches (DECISION)
- A gap in the BUILDSPEC is filled by judgment rather than raised (ASSUMPTION)
- Something is built differently from what the spec says (DEVIATION)
- Something is written then changed — wrong turn, corrected (CORRECTION)
- Something is discovered that changes the plan (DISCOVERY)
- A hard rule is interpreted where the reading wasn't obvious (INTERPRETATION)
- The build stops on an unanswerable question (BLOCKER)

**Timing:** write the entry before implementing the decision it describes, not after.

**Review checkpoints** (do these without prompting):
- At each file gate: re-read that file's log entries; check any ASSUMPTION before opening the gate
- At each wave boundary: read all ASSUMPTION entries for the wave together

The significance test from the protocol: *would someone reviewing this build later need to know this in order to trust or question the result?* If yes, log it.

## BUILD CONTEXT

- **BUILDSPEC:** `/root/BUILDSPEC.md` — what to build and exact specs
- **TEST_PROTOCOL:** `/root/TEST_PROTOCOL.md` — per-file test cycle, archetype harnesses
- **Build order:** Waves 0–9 in dependency order (§1 of BUILDSPEC)
- **Current wave:** 6 (`capability_assessor.py`)
- **Working directory:** `/root/ros2_ws/src/drone_control`
- **Test runner:** `python3 -m pytest tests/ -x -q` from the working directory

## HARD RULES (never violate without a DEVIATION entry)

- Layer 1 nodes (condition_nodes.py, action_nodes.py): read/write blackboard only, no I/O
- Layer 2 (capability_assessor.py): sole publisher — no other file publishes outbound messages from BT
- §7.1 hard stops: `consumption_rate_pct_per_s` and `cruise_speed_mps` are unresolved — do not assign values, do not default, raise
- loss_report is orphaned since v6.3 (BUILDSPEC §4.13) — not a freshness gate, subscription kept for diagnostics only
- Gate 8 fails on radius OR timer (§5.4) — not AND
- One file at a time, fully gated, before starting the next
