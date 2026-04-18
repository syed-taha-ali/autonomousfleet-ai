# Phase 7 — Final Tests and Cleanup: Validation Session

**Date**: 2026-04-18
**Platform**: MentorPi M1 (Pi 5), Dev PC (Ryzen 5 5600X, RTX 3060)
**Battery**: 7.30–8.30 V across sessions (2S LiPo, fully charged between runs)

---

## Changes Implemented

### 1. Removed `navigate_to` and `return_home` commands
- Removed from `nlp_command_node.py` tool schema and system prompt
- Removed from `mission_manager_node.py` dispatch table
- Removed from `safety_validator_node.py` allowed types

### 2. Reworked `scan_area`
- Three stop conditions: timed (`max_time_s`), distance (`max_distance_m`), full (frontier exhaustion)
- Auto-detects mode from parameters when `mode` not explicitly set
- Stops in place when done (no return-home behaviour)
- Mission manager configures explorer parameters via synchronous `SetParameters` call before enabling
- Mission manager creates scan timer for timed mode that disables explorer on timeout

### 3. Added `rotate_left`/`rotate_right` to `drive_for`
- Supports rotation by time (`duration_s`) or angle (`angle_deg`)
- Duration calculated from `angle_rad / angular_speed` when angle specified
- Speed capped at `self._speed_angular`

### 4. Implemented `set_speed`
- Stores `_speed_linear` and `_speed_angular` state
- Reconfigures Nav2 controller_server (`FollowPath.max_vel_x/y/theta`) at runtime via `SetParameters`
- Caps at 0.3 m/s linear, 1.0 rad/s angular

### 5. Updated `patrol` to use room IDs
- NLP node resolves `room_ids` to waypoint coordinates from rooms file
- Errors if no rooms configured or unknown room ID

### 6. Updated CLI with `/Examples` command
- Formatted Unicode table showing all 6 command types with example phrases
- Accessible via `/examples`, `/example`, or `/help`

### 7. NLP distance post-processing
- Regex-based fix for LLM confusing "meter/metre" with time
- Detects distance words in natural language, recalculates `duration_s = distance / speed`
- Workaround for Qwen2.5 7B tool-calling limitation

### 8. Detection overlay clock skew fix
- `detection_overlay_node` age check changed from Pi timestamp to local arrival time
- Fixes cross-machine clock skew causing YOLO bounding boxes to never render

---

## Bug Fixes During Testing

### scan_area parameter race condition
- **Symptom**: Explorer read stale distance parameter (e.g. 5.0m instead of 1.0m)
- **Root cause**: `call_async` returned before parameter was set; explorer's `_on_enable` read old value
- **First fix attempt**: `spin_until_future_complete` — caused deadlock in MultiThreadedExecutor (robot froze, required power cycle)
- **Final fix**: Synchronous `self._explorer_set_params.call(req)` in ReentrantCallbackGroup

### `Client.call()` timeout_sec kwarg
- **Symptom**: `call(req, timeout_sec=2.0)` raised TypeError on Humble
- **Root cause**: Humble's `Client.call()` signature is `call(self, request)` — no `timeout_sec` parameter
- **Fix**: Removed `timeout_sec` kwarg

### Battery safety rejections
- **Symptom**: All commands silently rejected
- **Root cause**: Battery sag to 7.14V after extended testing, below 7.2V safety threshold
- **Fix**: Recharge battery

### Multiple NLP nodes
- **Symptom**: Every command published 4x, robot wouldn't stop
- **Root cause**: Background NLP node launches accumulated across dev sessions
- **Fix**: Kill all stale NLP processes before launching single instance

### YOLO overlay not rendering
- **Symptom**: Camera feed visible in RViz but no bounding boxes
- **Root cause**: Detection age check compared Pi timestamp against Dev PC clock — clock skew exceeded 1s threshold
- **Fix**: Track detection arrival time locally instead of using message timestamp

---

## Hardware Test Results

| Test | Command | Result | Notes |
|------|---------|--------|-------|
| drive_for forward | "move forward 1 second" | PASS | Robot moved forward, stopped after 1s |
| drive_for backward | "move backward 1 second" | PASS | |
| drive_for left | "move left 1 metre" | PASS (with NLP fix) | NLP post-processing converts distance to duration |
| drive_for rotate | "turn left 90 degrees" | PASS | Correct angle rotation |
| drive_for rotate | "rotate right for 2 seconds" | PASS | |
| scan_area timed | "explore for 20 seconds" | PASS | Stopped at 20s (19s elapsed, 1.18m travelled) |
| scan_area distance | "explore for 1m" | PASS | Stopped at 1.08m after 8s |
| set_speed | "set speed to 0.1" | PASS | Works as per-command modifier |
| stop | "stop" | PASS | Cancels all goals, zeros cmd_vel |
| find_object | "find the suitcase" | PASS | YOLO detected suitcase (0.608 conf), temporal voting confirmed, robot stopped |
| /Examples CLI | typed "/Examples" | PASS | Table displayed correctly |
| YOLO overlay | RViz image panel | PASS | Green bounding box + label rendered (after clock skew fix) |

### Known Limitations
- NLP interprets "N meter/metre" as time without post-processing regex fix (7B model limitation)
- `set_speed` resets between separate commands (works as per-command modifier when bundled)
- `patrol` requires rooms file (none configured; out of scope for current testing)
- Single command per prompt (7B model produces one tool call per turn)

---

## RViz Screenshots
- `logs/rviz_ss/monitoring_ss_4.png` — Camera feed without YOLO overlay (before clock fix)
- `logs/rviz_ss/monitoring_ss_5.png` — Camera feed with YOLO overlay showing suitcase detection + SLAM map
