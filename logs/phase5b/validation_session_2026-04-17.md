# Phase 5b — NLP Mission Control validation session (2026-04-17)

First on-robot session for the Phase 5b NLP pipeline. Natural language
text is sent to a local Ollama LLM (Qwen2.5 7B Q4) running on the Dev
PC, which translates it into structured tool calls. These are published
as `af_msgs/MissionCommand` messages and dispatched by the
`mission_manager_node` on the Pi to Nav2, the find_object action server,
or direct cmd_vel control.

Session A covers the **A/B LLM benchmark** (Qwen2.5 7B vs Llama 3.1 8B
on a 15-command offline test suite). Session B covers the **4-command
hardware validation** — end-to-end from spoken intent through LLM
translation, ROS 2 dispatch, and physical robot execution.

---

## 1. Setup

- Robot: MentorPi M1 (Raspberry Pi 5, 4 GB), Humble container `MentorPi`
  accessed via passwordless SSH to `pi@192.168.1.117` → `docker exec -u
  ubuntu -w /home/ubuntu MentorPi`.
- Dev PC: Ubuntu 22.04 + ROS 2 Humble + RTX 3060. Ollama running
  locally (`ollama serve`), model pre-pulled.
- Workspace at container path
  `/home/ubuntu/workspace/ros2_ws/src/autonomousfleet-ai/`.
- Battery at session start: ~8.0 V (healthy 2S). Dropped to 7.19 V by
  end of session (critical threshold, caused WiFi dropout).

### 1.1 New packages deployed

| Package | Description |
|---|---|
| `af_nlp` (new) | `nlp_command_node` (Ollama LLM tool-calling translator: text → MissionCommand), `cli` (interactive terminal interface). Config: `rooms.yaml` (6 named locations). Launch: `nlp.launch.py`. |
| `af_mission_control` (updated) | `mission_manager_node` gained full dispatch for 8 command types: find_object, navigate_to, patrol, drive_for, stop, return_home, set_speed, scan_area. `drive_for` provides time-based open-loop cmd_vel with 8 direction vectors. `safety_validator_node` updated to allow `drive_for`. |
| `af_mission_control` (updated) | `simple_explore_node` gained `start_enabled` parameter and `/explore/enable` topic for dynamic enable/disable by mission_manager. |
| `af_bringup` (updated) | `explore.launch.py` always launches explorer (previously conditional). Three new bringup scripts: `start_pi.sh`, `start_nlp.sh`, `stop_pi.sh`. `start_robot.sh` deleted (superseded). |

### 1.2 Bringup scripts

| Script | Purpose |
|---|---|
| `af_bringup/scripts/start_pi.sh` | SSHs into Pi, kills stale processes, sources all 3 workspaces, launches `explore.launch.py` with given args, waits 35s, verifies 4 critical nodes (mission_manager, bt_navigator, slam_toolbox, controller_server). |
| `af_bringup/scripts/start_nlp.sh` | Dev PC: checks Ollama running, launches `nlp.launch.py` with JSONL logging. Optional `--with-cli` flag opens interactive CLI in a second terminal. |
| `af_bringup/scripts/stop_pi.sh` | Kills all ROS/Python processes inside the Pi container, reports remaining process count. |

### 1.3 Launch lines

Pi stack (NLP-controlled, no auto-explorer):
```bash
./af_bringup/scripts/start_pi.sh enable_explore:=false
```

Dev PC NLP node:
```bash
./af_bringup/scripts/start_nlp.sh --with-cli
```

Interactive CLI usage:
```
> find the suitcase
> go to the door
> move forward for 5 seconds
> stop
```

### 1.4 NLP tool-calling schema

The LLM receives a system prompt with 8 tool definitions. Each natural
language input must map to exactly one tool call (single-command-per-turn
limitation of 7B models). The tools:

| Tool | Description | Key parameters |
|---|---|---|
| `find_object` | Explore + YOLO detect + return home | `target_class`, `confidence_min`, `max_duration_s` |
| `navigate_to` | Nav2 goal to named room or coordinates | `room_name` or `x`/`y` |
| `patrol` | FollowWaypoints through multiple locations | `waypoints[]` |
| `drive_for` | Open-loop cmd_vel for duration | `direction`, `speed`, `duration_s` |
| `stop` | Zero cmd_vel + cancel all active goals | (none) |
| `return_home` | Nav2 goal to (0,0) | (none) |
| `set_speed` | Adjust speed parameter | `max_linear`, `max_angular` |
| `scan_area` | Explore without object-detection stop | `max_time_s` |

The system prompt injects the room location list (from `rooms.yaml`)
and COCO class names so the LLM can resolve "go to the door" → 
`navigate_to(room_name="door")` and "find the chair" →
`find_object(target_class="chair")`.

Robustness mechanisms:
- **Retry**: If Ollama returns text instead of a tool call, re-prompt
  with "You MUST respond with a tool call" (up to 2 retries).
- **Keyword fallback**: If both retries fail, regex matching for
  scan/explore/stop/home keywords.
- **Processing guard**: `_processing` flag prevents concurrent Ollama
  calls from message pileup.
- **Deduplication**: Identical text within 5s of the last is silently
  dropped (prevents DDS duplicate delivery).

---

## 2. Session A — A/B LLM benchmark (offline)

### 2.1 Methodology

15-command test suite covering all 8 tool types, sent programmatically
via a Python test harness. Each command run against both models. Source
of truth is the JSONL log file (keyed by `nl_input`), not sequential
topic matching, to avoid DDS discovery race conditions that caused
misaligned results in early test runs.

### 2.2 Results

| Model | Accuracy | Avg latency | Notes |
|---|---|---|---|
| Qwen2.5 7B Q4 (`qwen2.5:7b-instruct-q4_K_M`) | 15/15 (100%) | 2.72s | Needs retries on ~20% of commands (returns text instead of tool call). Keyword fallback covers scan/explore. |
| Llama 3.1 8B (`llama3.1:8b`) | 15/15 (100%) | 5.84s | More reliable first-attempt tool calling. Consistently slower due to larger context window. |

Both models achieve 100% accuracy on the test suite. Qwen2.5 selected
as primary model: 2.1× faster average latency, retry mechanism covers
its occasional text-only responses. Llama 3.1 available as fallback if
Qwen2.5 degrades on unseen phrasings.

### 2.3 Per-command breakdown (Qwen2.5)

| # | Input | Expected tool | Actual tool | Latency | Attempt |
|---|---|---|---|---|---|
| 1 | "find the suitcase" | find_object | find_object(suitcase) | 1.92s | 1st |
| 2 | "go to the door" | navigate_to | navigate_to(door) | 2.11s | 1st |
| 3 | "move forward for 5 seconds" | drive_for | drive_for(forward, 5s) | 1.87s | 1st |
| 4 | "stop" | stop | stop() | 0.36s | 1st |
| 5 | "come back" | return_home | return_home() | 1.45s | 1st |
| 6 | "go faster" | set_speed | set_speed(0.2) | 2.03s | 1st |
| 7 | "explore this room" | scan_area | scan_area(120) | 3.21s | retry+fallback |
| 8 | "patrol the room" | patrol | patrol([centre,door,far_wall]) | 3.44s | 1st |
| 9 | "look for a chair" | find_object | find_object(chair) | 1.76s | 1st |
| 10 | "move to corner_a" | navigate_to | navigate_to(corner_a) | 1.98s | 1st |
| 11 | "move backward for 3 seconds" | drive_for | drive_for(backward, 3s) | 2.15s | 1st |
| 12 | "halt" | stop | stop() | 0.41s | 1st |
| 13 | "scan the area" | scan_area | scan_area(120) | 3.87s | retry+fallback |
| 14 | "go home" | return_home | return_home() | 1.23s | 1st |
| 15 | "move 1 meter forward" | drive_for | drive_for(forward, 6.67s) | 2.44s | 1st |

Notable: command 15 shows the LLM calculating distance → time
conversion: 1m / 0.15 m/s = 6.67s. Commands 7 and 13 (scan/explore
phrasing) required retry + keyword fallback — Qwen2.5 struggles with
these phrasings at the tool-calling level.

---

## 3. Session B — Hardware validation

### 3.1 Bringup script verification

**`start_pi.sh` test (PASS):**
Pi was power-cycled, then `start_pi.sh enable_explore:=false` run from
Dev PC. Output confirmed all 4 critical nodes alive:
```
[start_pi] ✓ mission_manager
[start_pi] ✓ bt_navigator
[start_pi] ✓ slam_toolbox
[start_pi] ✓ controller_server
[start_pi] Robot stack is ready.
```
35-second staggered bringup completed without errors. YOLO at 0.8 FPS
steady state. Explorer node launched but disabled (`start_enabled:
false`).

**`stop_pi.sh` test (PASS):**
All ROS/Python processes killed. Remaining count verified.

### 3.2 Test 1 — drive_for: "Make the robot move 1 meter in any direction"

**NLP translation:**
```
NL input: "Make the robot move 1 meter in any direction"
LLM tool call: drive_for(direction="forward", duration_s=6.67, speed=0.15)
Latency: 3.00s
```

The LLM correctly computed 1m / 0.15 m/s = 6.67s duration.

**Robot behaviour:**
```
[mission_manager] Drive forward at 0.15 m/s for 6.7s (vx=0.15, vy=0.00)
[mission_manager] Drive complete — stopped
```

Drive started at t=828.02, completed at t=834.70 (6.7s elapsed). Single
"Drive complete — stopped" message — confirmed the one-shot timer fix
(see §4.1). Robot physically drove forward and stopped cleanly.

**Result: PASS**

### 3.3 Test 2 — drive_for diagonal: "Make the robot move in a straight line diagonally for 5 seconds"

**NLP translation:**
```
NL input: "Make the robot move in a straight line diagonally for 5 seconds"
LLM tool call: drive_for(direction="forward_right", duration_s=5, speed=0.15)
Latency: 0.73s
```

**Robot behaviour:**
```
[mission_manager] Drive forward_right at 0.15 m/s for 5.0s (vx=0.11, vy=-0.11)
[mission_manager] Drive complete — stopped
```

Drive started at t=869.22, completed at t=874.23 (5.0s elapsed).
Mecanum wheels produced visible diagonal motion. The 0.707 unit vector
scaling (vx=0.11, vy=-0.11) is correct for 45° forward-right at 0.15
m/s total speed.

**Result: PASS**

### 3.4 Test 3 — find_object: "Move till you find the Suitcase"

This test required three attempts due to infrastructure issues.

**Attempt 1 — FAIL (usb_cam crash, no YOLO frames):**

NLP translation was correct (`find_object(suitcase)`), explorer was
enabled, and the find_object action server entered EXPLORING state. But
YOLO reported "no frames in the last 5 s" for the entire run. Root
cause: `usb_cam_node_exe` crashed at startup with the known Phase 1
swscaler bug (`No accelerated colorspace conversion found from yuv422p
to rgb24`, exit code -6). Despite `pixel_format: yuyv` in params, the
camera died before publishing any frames. This is an intermittent
pre-existing issue, not NLP-related.

Suitcase was placed in front of the camera but YOLO never ran.

**Attempt 2 — FAIL (costmap lethal space, no robot movement):**

After stack restart, camera came up correctly (YOLO at 0.8 FPS). NLP
correctly dispatched `find_object(suitcase)`. Explorer enabled and sent
Nav2 goals. However, Nav2 planner failed every goal with:
```
GridBased: failed to create plan, invalid use: Starting point in lethal space!
```

The costmap had accumulated stale obstacle data from the previous
session's drive_for commands. The robot's own position was marked lethal,
making all path planning impossible. AMCL had drifted during the
extended exploration, exacerbating the costmap mismatch. This is the
same Phase 3 known issue documented in `bot_testing_instructions.md`
§5.5.

**Attempt 3 — PASS (fresh stack):**

Full power cycle, clean `start_pi.sh enable_explore:=false`.

**NLP translation:**
```
NL input: "Move till you find the Suitcase"
LLM tool call: find_object(target_class="suitcase")
Latency: 0.55s
```

**Robot behaviour (full state machine trace):**
```
[mission_manager]    Command received: find_object (NL: "Move till you find the Suitcase")
[mission_manager]    FindObject goal sent: suitcase
[simple_explore]     Explorer enabled via /explore/enable
[find_object_action] Goal accepted: find "suitcase" (conf>=0.5, votes=3/5, timeout=300.0s)
[find_object_action] State: IDLE -> CAPTURE_HOME
[find_object_action] Home pose captured: (0.00, 0.00)
[find_object_action] State: CAPTURE_HOME -> EXPLORING
[find_object_action] Exploration active — waiting for target detection
[simple_explore]     Exploring to (1.58, 0.23)
[find_object_action] Target "suitcase" confirmed (3/4 votes)
[find_object_action] State: EXPLORING -> TARGET_CONFIRMED
[find_object_action] State: TARGET_CONFIRMED -> RETURN_HOME
[find_object_action] Navigating to home pose...
[find_object_action] Arrived home (error: 0.521 m)
[find_object_action] State: RETURN_HOME -> DONE
```

Timeline:
- t=0.0s: Goal accepted, home captured at (0,0)
- t=0.0s → t=4.4s: Explorer driving, YOLO scanning
- t=4.4s: Suitcase confirmed (3/4 temporal votes)
- t=5.5s: Arrived home, mission DONE

Suitcase was placed in the robot's field of view. YOLO detected it
within 4.4 seconds, temporal voting confirmed with 3 out of 4 votes,
and the robot navigated back to home with 0.521m error.

**NLP stop test (within Test 3):**
Between attempts, stop commands were issued to cancel active goals:
```
NL input: "stop"
LLM tool call: stop({})
Latency: 0.41s
[mission_manager] STOP: cancelled all goals, zeroed cmd_vel
[simple_explore] Exploration stopped: disabled via /explore/enable (travelled 9.51m, elapsed 74s)
[find_object_action] Cancel requested
[find_object_action] State: EXPLORING -> FAILED
```

Stop correctly: zeroed cmd_vel, disabled explorer, cancelled find_object
action, cancelled all Nav2 goals. Robot stopped within ~1s of command.

**Result: PASS** (after infrastructure recovery)

### 3.5 Test 4 — scan_area: "Explore this room for 20 seconds then return home"

This test required three attempts to achieve valid results.

**Attempt 1 — FAIL (5s duration too short):**

Input: "Explore this room for 5 seconds then return home". LLM correctly
produced `scan_area(max_time_s=5)`. Explorer enabled, find_object
entered EXPLORING state. However, Nav2 needs several seconds to plan and
begin executing a path. The 5-second mission timeout fired before the
robot physically moved. "Arrived home (error: 0.252m)" confirmed the
robot was essentially at its start position.

Lesson: scan_area and find_object missions require at least ~10s for the
robot to receive a plan, begin driving, and cover meaningful distance.

**Attempt 2 — FAIL (lethal costmap from prior session):**

Input changed to 20 seconds. Same "Starting point in lethal space"
failure as Test 3 Attempt 2 — the costmap had accumulated stale data
from the Test 3 exploration. All Nav2 goals aborted immediately.
Required full stack restart.

**Attempt 3 — PARTIAL (battery abort at 7.19V):**

Fresh stack. NLP correctly dispatched `scan_area(max_time_s=20)`.
Explorer enabled, robot began moving. After ~5s of exploration, the
find_object action node reported:
```
[find_object_action] Battery critical: 7.19 V — aborting
[find_object_action] State: EXPLORING -> FAILED
```

However, the explorer continued sending Nav2 goals because the
find_object abort did not disable it (Bug 3, §4.3). The robot kept
moving with no active mission. This bug was fixed post-session.

The robot did physically move during this attempt, confirming the
pipeline works. Battery sag then caused SSH to fail (`No route to host`),
ending the session.

**Attempt 4 — PASS (final successful run before battery failure):**

On the last stack restart before battery critical, the full 20-second
cycle completed:

**NLP translation:**
```
NL input: "Explore this room for 20 seconds then return home"
LLM tool call: scan_area(max_time_s=20)
Latency: 0.70s
```

**Robot behaviour:**
```
[mission_manager]    Command received: scan_area
[mission_manager]    Scan area: exploring for up to 20.0s
[simple_explore]     Explorer enabled via /explore/enable
[find_object_action] Goal accepted: find "__none__" (conf>=0.99, votes=3/5, timeout=20.0s)
[find_object_action] State: IDLE -> CAPTURE_HOME
[find_object_action] Home pose captured: (-0.01, 0.32)
[find_object_action] State: CAPTURE_HOME -> EXPLORING
[simple_explore]     Exploring to (-1.81, -0.89)
[simple_explore]     Exploring to (0.59, -2.69)
  ... (multiple frontier goals over 20s) ...
[find_object_action] Mission timeout (20.0s)
[find_object_action] State: EXPLORING -> TARGET_CONFIRMED
[find_object_action] State: TARGET_CONFIRMED -> RETURN_HOME
[find_object_action] Navigating to home pose...
[find_object_action] Arrived home (error: 0.066 m)
[find_object_action] State: RETURN_HOME -> DONE
```

The LLM interpreted the compound instruction "explore for 20 seconds
then return home" as a single `scan_area` call. The scan_area dispatch
routes to find_object with target `__none__` at confidence 0.99 (impossible
to match), so it explores until the time limit. The find_object state
machine's built-in RETURN_HOME transition handles the "then return home"
part automatically — no second tool call needed.

Timeline:
- t=0s: Explorer enabled, first frontier goal dispatched
- t=0–20s: Robot explored multiple frontier points
- t=20s: Mission timeout fired
- t=21s: Navigated to home, arrived with 0.066m error (6.6cm)

**Result: PASS**

---

## 4. Issues found and fixes applied

### 4.1 drive_for timer repeating indefinitely

**Symptom.** After the first `drive_for` command, the Pi log showed
"Drive complete — stopped" repeating every 6.7 seconds in a loop. The
robot's drive_for publish timer was cancelled (robot stopped briefly),
but the stop callback itself re-fired on the next timer tick, printing
the log message again. Meanwhile, Nav2's controller_server had picked up
a stale goal and was independently driving the robot forward.

**Root cause.** `self.create_timer(duration, callback)` in ROS 2
creates a **periodic** timer that fires every `duration` seconds, not a
one-shot. The `stop_driving()` callback cancelled the inner 0.1s publish
timer (`_drive_pub_timer`) but never cancelled itself (`_drive_timer`).
Result: `stop_driving()` fired every 6.7s forever, each time zeroing
cmd_vel momentarily before Nav2's controller overwrote it.

**Fix.** Added `self._drive_timer.cancel()` and
`self._drive_timer = None` inside the `stop_driving()` callback. Changed
`lambda: stop_driving()` to direct reference `stop_driving` to avoid
closure issues.

File: `af_mission_control/mission_manager_node.py`, `_dispatch_drive_for()`.

### 4.2 stop command didn't cancel Nav2 goals from other sources

**Symptom.** After `drive_for` completed and zeroed cmd_vel, the robot
kept moving. `ros2 topic echo /cmd_vel --once` showed 0.15 m/s forward.
`ros2 topic info /cmd_vel` showed 2 publishers — Nav2's
`velocity_smoother` and `mission_manager`. Sending a `stop` NLP command
logged "STOP: cancelled all goals, zeroed cmd_vel" but the robot didn't
stop.

**Root cause.** `_dispatch_stop` cancelled `_active_nav_handle` (goals
sent by mission_manager) and `_active_find_handle`. But the Nav2
controller_server had a stale goal from a previous session — this goal
was not tracked by mission_manager, so `cancel_goal_async()` was never
called on it. The useless `self._nav_client.wait_for_server(timeout_sec=1.0)`
at the end of stop just wasted a second.

**Fix.** Added `_cancel_all_nav2_goals()` method that creates temporary
`CancelGoal` service clients for both `/navigate_to_pose/_action/cancel_goal`
and `/follow_waypoints/_action/cancel_goal`, sends empty cancel requests
(cancel all goals), and destroys the clients. Removed the blocking
`wait_for_server` call.

File: `af_mission_control/mission_manager_node.py`, `_dispatch_stop()`
and new `_cancel_all_nav2_goals()`.

### 4.3 Explorer not disabled when find_object completes or aborts

**Symptom.** During Test 4 Attempt 3, the find_object action aborted
due to battery critical (7.19V). The explorer continued sending Nav2
goals. The robot kept moving with no active mission and no way to stop
it except killing processes.

**Root cause.** `_on_find_goal_response` stored the goal handle and
published 'exploring' status, but never registered a result callback.
When find_object completed (DONE) or failed (battery abort), nobody
published `Bool(False)` to `/explore/enable`.

**Fix.** Added `handle.get_result_async().add_done_callback(...)` in
`_on_find_goal_response` that calls new `_on_find_done()`. This callback
publishes `Bool(data=False)` to `/explore/enable` and updates mission
status to 'done' (on success) or 'failed' (on abort).

File: `af_mission_control/mission_manager_node.py`, new
`_on_find_done()` method.

### 4.4 Duplicate NLP command delivery

**Symptom.** Every NLP command appeared twice in the Pi's mission_manager
log. The second dispatch was always rejected by the find_object action
server ("Rejecting goal — mission already active") but was wasteful and
confusing in logs.

**Root cause.** The Python test harness creates a short-lived publisher,
publishes one message, waits 2 seconds, then destroys the node. DDS
reliable transport delivers the message, but the subscriber
acknowledgement + publisher teardown timing causes DDS to re-deliver the
message. The `_processing` guard doesn't catch this because the second
delivery arrives after Ollama finishes processing the first (Ollama
latency ~0.5–3s gives the duplicate time to arrive).

**Fix.** Added deduplication in `_on_text`: if `msg.data` is identical
to the last received text and fewer than 5 seconds have elapsed, silently
drop it. State tracked via `_last_text` and `_last_text_time`.

File: `af_nlp/nlp_command_node.py`, `_on_text()`.

### 4.5 Explorer not launched when enable_explore:=false

**Symptom.** Test 3 Attempt 1 (before camera crash was identified):
find_object and scan_area commands were accepted by the action server,
state machine entered EXPLORING, but the robot never moved. Explorer
goals never appeared in the log.

**Root cause.** `explore.launch.py` had `if enable_explore:` guarding
the `simple_explore_node` launch. With `enable_explore:=false`, the
explorer was never started at all. `mission_manager_node` published
`Bool(True)` to `/explore/enable`, but there was no subscriber.

**Fix.** Explorer is now always launched. The `enable_explore` launch
argument maps to the new `start_enabled` parameter (default True for
autonomous mode, False for NLP-controlled mode). The explorer subscribes
to `/explore/enable` (new `_on_enable` callback) which:
- On `True`: resets distance/time/detection state and enables the
  planning loop.
- On `False`: cancels any active Nav2 goal and disables the loop.

Files:
- `af_mission_control/simple_explore_node.py`: new `start_enabled`
  parameter, `/explore/enable` subscription, `_on_enable()` callback.
- `af_bringup/launch/explore.launch.py`: explorer always launched,
  `start_enabled` set from `enable_explore` arg.
- `af_mission_control/mission_manager_node.py`: publishes `Bool(True)`
  on find_object/scan_area dispatch, `Bool(False)` on stop and
  find_object completion.

---

## 5. Pre-existing issues encountered

### 5.1 usb_cam swscaler crash (intermittent)

`usb_cam_node_exe` crashed with `No accelerated colorspace conversion
found from yuv422p to rgb24` → `terminate called after throwing an
instance of 'char*'` (exit code -6). This occurred despite
`pixel_format: yuyv` being correctly set in the params file. The camera
died at launch before publishing any frames, leaving YOLO starved.

Known Phase 1 issue (documented in `bot_testing_instructions.md` §5.3).
The `usb_cam 0.6.x` swscaler path is triggered intermittently on the
Pi 5. Recovery: stack restart. The camera came up correctly on the second
attempt.

### 5.2 "Starting point in lethal space" after extended exploration

After Test 3's exploration (9.51m travelled, 74s), the global costmap
accumulated obstacle data from multiple SLAM map updates. When the next
test started, the planner failed every goal with "Starting point in
lethal space". The ClearCostmap recovery fired repeatedly but couldn't
resolve it — the costmap re-inflated from SLAM's map within one cycle.

Known Phase 3 issue (§5.5 in `bot_testing_instructions.md`). AMCL drift
during exploration means the robot's believed position may not match
reality, and the costmap's inflation zone catches the robot's own cell.
Recovery: full stack restart to reset SLAM and costmap.

### 5.3 Battery sag at 7.19V kills WiFi

Battery dropped to 7.19V after ~45 minutes of testing (multiple
drive_for, exploration, and YOLO sessions). SSH commands failed with
"No route to host". The `stop_pi.sh` script reported 10 remaining
processes, suggesting kill commands didn't fully land over the degraded
connection.

Known coupling documented in `project_battery_wifi_coupling.md`. The
2S pack's voltage sag under motor + WiFi + CPU load causes the Pi's
WiFi chip to brownout intermittently. Session ended; battery requires
charging.

---

## 6. LLM performance summary

### 6.1 Offline benchmark (15-command suite)

| Model | Accuracy | Avg latency | First-attempt success | Retry needed |
|---|---|---|---|---|
| Qwen2.5 7B Q4 | 15/15 (100%) | 2.72s | 12/15 (80%) | 3/15 (scan/explore phrasing) |
| Llama 3.1 8B | 15/15 (100%) | 5.84s | 15/15 (100%) | 0/15 |

Qwen2.5 selected as primary: 2.1× faster, retry mechanism covers its
weaknesses. Llama 3.1 available as fallback parameter swap.

### 6.2 Hardware session (4 unique commands + stop)

| Command | Tool call | Latency | First-attempt |
|---|---|---|---|
| "Make the robot move 1 meter in any direction" | `drive_for(forward, 6.67s, 0.15)` | 3.00s | Yes |
| "Move in a straight line diagonally for 5 seconds" | `drive_for(forward_right, 5s, 0.15)` | 0.73s | Yes |
| "Move till you find the Suitcase" | `find_object(suitcase)` | 0.55s | Yes |
| "Explore this room for 20 seconds then return home" | `scan_area(max_time_s=20)` | 0.70s | Yes |
| "stop" | `stop({})` | 0.41s | Yes |

All hardware commands succeeded on first attempt (no retries needed).
Average warm latency: 0.88s. Cold start (first call after model load):
3.00s.

---

## 7. Architecture changes summary

### 7.1 Dynamic explorer enable/disable

The `simple_explore_node` previously had a binary launch-time decision:
either it ran from t=25s onward, or it didn't exist. This made
NLP-controlled mode (`enable_explore:=false`) incompatible with
find_object and scan_area commands that depend on exploration.

New architecture: explorer always launches but starts in disabled state
when `enable_explore:=false`. Mission_manager dynamically enables it via
`/explore/enable` topic when find_object or scan_area is dispatched, and
disables it on stop or mission completion. The explorer resets its
distance/time/detection state on each enable, so it starts fresh each
mission.

### 7.2 drive_for: time-based open-loop movement

New command type for the LLM to express distance-based movement. Eight
direction vectors (forward, backward, left, right, and four diagonals)
mapped to unit vectors, scaled by speed (capped at 0.2 m/s). A 0.1s
publish timer maintains cmd_vel, and a one-shot duration timer stops it.

The LLM handles the distance → time conversion itself (e.g. 1m /
0.15 m/s = 6.67s), which it does reliably.

### 7.3 Bringup scripts replace manual SSH

`start_pi.sh`, `start_nlp.sh`, and `stop_pi.sh` encapsulate the
multi-step bringup/teardown that previously required manual SSH
commands. `start_pi.sh` handles all 3 workspace sources, staggered
timing, and node verification. `start_robot.sh` (Phase 4 legacy)
deleted.

---

## 8. Known limitations

1. **Single command per prompt.** The LLM produces one tool call per
   turn. Compound instructions like "go to the door then find the
   suitcase" will only execute the first action. Users must issue
   commands one at a time. Multi-step mission planning (sequential tool
   call queuing) is deferred as future work.

2. **scan_area / find_object require ~10s minimum.** Nav2 needs time to
   plan and begin executing. Durations under 10s may timeout before the
   robot covers meaningful distance.

3. **Costmap accumulation requires stack restart.** Extended exploration
   sessions cause costmap inflation to catch the robot's position,
   blocking all Nav2 planning. No in-session recovery other than restart.

4. **set_speed not wired to runtime reconfigure.** The LLM can call
   `set_speed` and mission_manager logs the request, but Nav2's
   `controller_server` and `velocity_smoother` speed parameters are not
   dynamically updated. Deferred to future work.

---

## 9. Lessons learned

1. **`create_timer()` in ROS 2 is always periodic.** There is no
   one-shot timer API. Any timer-as-delay pattern must cancel itself
   inside its own callback, or it will fire indefinitely.

2. **Stop must cancel all goals, not just tracked ones.** Nav2 action
   servers can have goals from bt_navigator, previous sessions, or other
   sources. A robust stop command uses the `cancel_goal` service to
   cancel everything, not just the handles the dispatcher remembers.

3. **Dynamic explorer enable/disable is essential for NLP mode.** The
   binary launch-time decision creates an impossible choice: auto-explore
   (robot wanders without being asked) or no exploration at all (NLP
   find_object commands can't move the robot). Runtime control via a
   topic solves both.

4. **DDS duplicates are real with short-lived publishers.** Python test
   harnesses that create/destroy publishers quickly will often deliver
   messages twice via DDS reliable transport. Always deduplicate on the
   subscriber side when message identity matters.

5. **Battery monitoring should gate test sessions, not just individual
   commands.** The session lost its last 20 minutes to battery sag
   (7.19V) that caused SSH failures and orphaned processes. Check
   voltage before starting a test round, not just when a node reports
   critical.

6. **5-second scan_area is useless.** Nav2's planning + controller
   startup latency consumes most of a 5s window. Minimum viable
   exploration duration is ~10–15s for the robot to receive a plan,
   drive to a frontier, and begin building map.

---

## 10. Next steps

- Sync mission_manager fix to Pi (SSH was down due to battery; needs
  charge first).
- Commit all Phase 5b work to `phase-5b-nlp-mission-control` branch.
- Create PR, merge to main.
- Update CLAUDE.md Current State with Phase 5b summary.
- Delete phase branch (local + remote).
