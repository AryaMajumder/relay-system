
## 2026-09-30

### Leader position freshness check (BandSensorNode)

**Problem:** BandSensorNode had no freshness guard on leader position. Severity staleness was caught via `get_with_freshness` + stale_sev fallback, but a silently stale `leader_state` would keep computing band geometry from old data indefinitely. `get_with_freshness` is the wrong tool here: `state_bridge` republishes the last-known payload at 1 Hz, refreshing the blackboard-receive timestamp every second even when the FCU is silent. The correct check is against `leader_state["timestamp"]` — the origin FCU timestamp that `state_bridge` deliberately preserves in the payload.

**Timestamp chain confirmed:** `px4_agent._publish_drone_state()` stamps `"timestamp": time.time()` (wall-clock epoch seconds). `state_bridge` preserves it unchanged (only adds its own `time.time()` when the field is absent). `self._clock()` in BandSensorNode is also `time.time()`. Same scale, same unit — but two physical clocks on separate airframes, requiring synchronized clocks (GPS time or NTP) for the age check to be meaningful.

**Changes:**
- `condition_nodes.py`: Added origin-timestamp freshness gate in `BandSensorNode.update()`. Config key `leader_position_max_age_s` (default 10.0). Fail-closed: missing timestamp treated as stale → `band_fillable=False`. Removed tasking and config leader_pos fallbacks (neither populated in production).
- `test_wave5_condition_nodes.py`: Added `TestBandSensorNodePositionFreshness` (4 tests: fresh, stale, recovery, no-timestamp). Updated 3 existing leader_state writes to include `"timestamp": clock.now()`.
- `integration_waves0_to_5.py` / `integration_waves0_to_6.py`: All 10 leader_state writes updated to include `"timestamp": now`.
- `KNOWN_LIMITATIONS.md`: New section documenting the cross-airframe clock-sync assumption.

**Result:** 351 passed (up from 347 at session start), 901 pre-existing flake8 violations unchanged.

**Commit:** `0737c8d`
