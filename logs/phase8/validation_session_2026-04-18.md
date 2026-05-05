# Phase 8 — Chapter 5 experiments session log

**Date:** 2026-04-18
**Venue:** home lab, room1
**Robot:** MentorPi (pi@192.168.1.117), Dev PC: taha-pc (RTX 3060)
**Battery start:** 8.02 V (fully charged, user-reported)
**Battery end:** 6.78 V (below 6.8 V WARN threshold — session paused for recharge)
**Operator:** Taha

## Scope

Run the Chapter 5 hardware-dependent experiments from `report/ch5_data_inventory.md`
in the order G5 → G11 → G3 → G1 → G2 → G6. Session stopped after G3 due to
battery sag. G1/G2/G6 deferred.

## Pre-flight

- Dev PC stale ROS procs from prior sessions cleared: PIDs 4135328, 4145597,
  4186922, 4186941, 4186942, 4186943 killed; `ps -eo pid,cmd | grep _node`
  returned empty.
- Pi container `MentorPi` verified up ~2 h; inside, only `ros2-daemon` (PID 1163)
  running — no stale application nodes.
- Room1 map remapped under `af_bringup slam.launch.py mode:=mapping` because
  the existing `room1.pgm` (139×128, committed 2026-04-14) was stale. New map
  saved via `map_saver_cli`, 134×104 cells @ 0.05 m = 6.7 × 5.2 m, origin
  `[-3.3, -2.25, 0]`. Preview inspected, walls solid, user approved overwrite.
  Backup kept on Pi as `room1.backup_20260418_202442.{pgm,yaml}`.

## Experiments run

### ✓ G5 — Bandwidth + overlay CPU budget

Launched `explore.launch.py enable_explore:=false` so the full Pi stack (HAL +
SLAM + Nav2 + perception) was active but the robot stationary. Measured
bandwidth on 4 topics with `ros2 topic bw`, each ~60 s:

| topic | rate | msg size | throughput |
|---|---|---|---|
| `/camera/image_raw/compressed` | 10 Hz | 71.8 KB | **707.07 KB/s** |
| `/camera/demo/compressed`      | 3.27 Hz | 69.1 KB | 225.45 KB/s |
| `/detections`                  | 10 Hz | 177 B | **0.18 KB/s** |
| `/scan`                        | 10 Hz | 3.7 KB | 37.09 KB/s |

Pass-1 (Dev-PC offload) to Pass-2 (on-Pi YOLO) link-bandwidth reduction:
707 KB/s → 0.18 KB/s = **3900× lower** when the demo preview is off, or
707 → 225 KB/s = 3.1× with the preview on.

Dev-PC `detection_overlay_node` CPU over 60 s: steady state **~16 % of one
core** (12-thread Ryzen 5 5600x ⇒ ~1.5 % total). This pairs with the Phase 4
finding that the same node on the Pi saturated one ARM core — the "overlay
moved off-Pi" design choice is quantified here.

**Artefacts:** `report/data/bandwidth_budget.csv`,
`report/data/overlay_cpu_comparison.csv`,
`logs/ch5_experiments/g5_bandwidth_raw.txt`,
`logs/ch5_experiments/g5_pi_ps_snapshots.txt`,
`logs/ch5_experiments/g5_devpc_overlay_cpu.csv`.

### ✓ G11 — Odometry drift vs AMCL

**First attempt (invalid):** misread protocol, recorded 485 s stationary.
Drift came out effectively zero because the EKF clamps to the origin when
wheel encoders report no motion. Useful as a noise-floor sanity check but
doesn't answer the drift question. Bag kept at
`logs/ch5_experiments/g11_odom_bag/`.

**Second attempt (valid):** switched Pi from `explore.launch.py` to
`nav2.launch.py map:=<room1.yaml>` so AMCL publishes `/amcl_pose`. User set
2D Pose Estimate in RViz (mode `navigation`), then teleop-drove the robot
through the room at ~0.5 m/s for 5 min (general motion, not a strict 5×5
perimeter). Recorded `/odom`, `/odom_raw`, `/amcl_pose`, `/cmd_vel`.

Duration: 331 s, path length 41.6 m, 572 AMCL samples.

| metric | `/odom` (EKF) | `/odom_raw` (wheel-only) |
|---|---|---|
| mean \|pose − AMCL\| | **65.1 cm** | 83.7 cm |
| max                  | 182.4 cm    | 229.1 cm |
| final                | 39.3 cm     | 56.8 cm |
| final \|yaw − AMCL\| | **−34.2°** | −67.8° |

EKF (IMU-fused) reduces wheel-odom position drift by ~25 % and yaw drift by
~50 % over 5 min continuous driving, but neither matches AMCL's scan-based
correction.

**Artefacts:** `logs/ch5_experiments/g11_drive_bag/`,
`report/data/odom_drift_drive.csv`,
`report/figures/odom_drift.png`,
`report/figures/odom_vs_amcl_xy.png`,
`report/scripts/g11_drive_drift.py`.

### ✓ G3 — AMCL accuracy (perimeter-hug)

Original 10-pose tape-measured protocol judged too slow; switched to a
perimeter-hug variant. Robot teleop-driven one slow lap with the bumper
~5–10 cm from walls. Logged `/amcl_pose` + `/scan` + `/cmd_vel` for ~325 s
(315 AMCL samples).

Post-processing: `room1.pgm` loaded, Euclidean distance transform built,
each AMCL sample projected to the distance-to-nearest-occupied-cell value.
Expected distance while hugging = robot half-width ≈ 8 cm. Residual error =
`actual − 8 cm`.

Wall-close subset (distance < 0.5 m, n = 308):

| metric | value |
|---|---|
| mean distance to wall | 11.2 cm |
| std | 11.7 cm |
| residual mean | +3.2 cm |
| residual std  | 11.7 cm |
| **residual RMS** | **12.1 cm** |

The RMS is an upper bound on AMCL position error because it compounds AMCL
error with the operator's hug-distance variance. The +3 cm bias and ~12 cm
RMS is consistent with AMCL's 5 cm grid resolution plus MS200 scan noise.
~83 samples report distance = 0 (AMCL pose fell inside the wall layer) —
expected on a 5 cm occupancy grid.

**Artefacts:** `logs/ch5_experiments/g3_perimeter_bag/`,
`report/data/amcl_perimeter_errors.csv`,
`report/figures/amcl_error_histogram.png`,
`report/scripts/g3_amcl_perimeter.py`.

## Experiments deferred

### ✗ G1 — 2 additional Nav2 goals

Goal-sender script prepared at `report/scripts/g1_nav2_goals.py` targeting
`(0.0, 0.0, 0°)` and `(−1.0, −0.5, 0°)`. Did not run — battery dropped to
6.78 V before launch. Robot reported as unresponsive to command during the
pre-flight check, consistent with the RRC Lite low-voltage motor throttle.

### ✗ G2 / G6

Not attempted this session.

## Battery / thermal

- Start: 8.02 V (fully charged).
- End: 6.78 V — **below 6.8 V WARN**, above 6.4 V CRITICAL.
- Sag rate: 1.24 V over ~2.5 h of mixed idle + teleop driving + localisation.
- Per `project_battery_wifi_coupling.md` memory, sag below ~7.2 V couples
  into WiFi; one SSH hang during the map-save phase is consistent with this.
  Session paused for a full recharge before G1/G2/G6 resume.

## Bugs / oddities found

- `bash /af_bringup/scripts/start_pi.sh` via explore.launch.py does not
  publish `/camera/demo/compressed`; the throttle node is bundled in
  `robot.launch.py` only. For G5 I started the throttle manually with
  `ros2 run topic_tools throttle messages /camera/image_raw/compressed 4.0 \
   /camera/demo/compressed` on the Pi.
- `nav2.launch.py` requires a `map:=` argument (no default). Minor fix
  candidate: default to `room1.yaml` or error earlier than the included
  `slam_localization.launch.py` layer.
- `ros2 bag record --max-bag-duration N` splits files at N seconds; it does
  **not** stop the recording. Used SIGINT to stop cleanly.
- During `ros2 bag` stop on the Dev PC, `pkill -INT -f "ros2 bag record"`
  matches the wrapper shell's own process table entry, so `exit code 1` on
  the pkill is normal — the recorder itself receives SIGINT and flushes
  remaining messages to the last split.
- `ros2 topic bw` under `timeout N` often exits with code 143 but writes
  valid output up to the termination point — parsed those logs directly.

## Dev-PC-only experiments run while Pi charges

### ✓ G7 — NLP 80-utterance benchmark

Standalone script `report/scripts/g7_nlp_benchmark.py` imports `_TOOL_SCHEMA`
and `_build_system_prompt` directly from `af_nlp.nlp_command_node` and hits
the local Ollama API — no ROS spin-up. 60 structured utterances (6 command
categories × 10 paraphrases) + 20 malformed/ambiguous, run against
Qwen2.5-7B-Q4 (primary) and Llama 3.1-8B-Q4 (fallback). Grading: strict
tool-name match, plus target_class / direction-kind match for
find_object / drive_for.

| model | strict_pass | safe_on_ambig | wrong_tool | hallucinated | mean lat | p95 lat |
|---|---|---|---|---|---|---|
| Llama 3.1 8B | **58/80** | 14/20 | 1 | 6 | 0.50 s | 0.66 s |
| Qwen2.5 7B   | 51/80     | 12/20 | 9 | 8 | 0.51 s | 0.71 s |

Per-command strict pass (60 structured only):

| cmd | Llama | Qwen |
|---|---|---|
| find_object | 9/10 | 10/10 |
| drive_for (translate+rotate) | 20/20 | 17/20 |
| stop | 10/10 | 7/10 |
| set_speed | 9/10 | 8/10 |
| scan_area | 10/10 | 9/10 |

Flip vs the pre-Phase-7 A/B: Llama now leads on strict accuracy. Latencies
roughly equal (500 ms) on the Dev PC with both 7B/8B Q4 models, unlike the
earlier 2.7 s vs 5.8 s result (the `ollama` server is keeping the model warm
across consecutive requests in this run).

**Artefacts:** `report/data/nlp_6cmd_benchmark.csv`,
`report/figures/nlp_confusion_matrix.png`,
`report/figures/nlp_latency_box.png`,
`logs/phase8/g7_run.log`.

### ✓ G8 — Regex post-processor ablation

30 distance-bearing utterances ("drive forward 1 metre", "back up 0.5 m",
"strafe right 0.75 metres", wordy forms, and 2 cm-unit decoys). For each,
call Qwen2.5 once, record (a) raw LLM `duration_s` and (b) the
production regex-corrected `duration_s` from
`NlpCommandNode._fix_distance_params`. Correct if within ±15 % of
expected = distance / 0.15 m/s.

| config | correct duration | notes |
|---|---|---|
| LLM only            | **0/30 (0 %)**  | 7B reliably confuses distance with time |
| LLM + regex         | **10/30 (33 %)** | picks up "N m" / "N metre(s)" cleanly |

Failure breakdown among the remaining 20:
- 8 / 30 utterances — Qwen did not fire a `drive_for` tool call at all
  (wordy "one metre", "half a metre", "exactly 1 metre"; 7B tool-selection
  weakness). If we restrict the denominator to *tool-fired* rows, regex
  lifts correctness from 0 / 22 to 10 / 22 (45 %).
- Remaining failures: wordy numbers ("three metres", "one and a half
  metres") and cm units ("50 centimetres", "100 cm") — the production
  regex is intentionally m-only; characterised, not a bug.

Delivered as a one-line strength claim for the report: the regex fix is
strictly better than raw LLM on its target class of utterances, but it
cannot compensate for the tool-selection misses on wordy distance prompts —
those are a 7B-model limitation, not a regex gap.

**Artefacts:** `report/data/regex_ablation.csv`,
`report/figures/regex_ablation_bar.png`,
`logs/phase8/g8_run.log`.

### ✓ G4 Pass 1 — YOLOv8n Dev-PC COCO val mAP

Downloaded COCO val2017 (5 000 images, ~1 GB) + Ultralytics-hosted
segment labels into `~/datasets/coco`. Custom val-only YAML at
`~/datasets/coco_val_only.yaml` avoids the 19 GB train2017 auto-download.

Ran `report/scripts/g4_pass1_coco_map.py` — `model.val(data=..., imgsz=640,
batch=16, conf=0.001, iou=0.6, device=0)` on RTX 3070. Total wall time
~30 s (1.6 ms inference/image).

- Overall: **mAP50 = 0.525**, **mAP50-95 = 0.368** (faster-coco-eval
  cross-check: 0.532 / 0.373 — matches published YOLOv8n baseline 0.371,
  so the deployed weights and pipeline are validated).
- Mission-relevant classes: laptop 0.577, person 0.507, teddy bear 0.411,
  cup 0.347, suitcase 0.340 (primary demo target — adequate), bottle 0.296,
  cell phone 0.281, chair 0.255, backpack 0.100, book 0.094 (tiny objects).

**Artefacts:** `report/data/coco_yolov8n_map.csv`,
`logs/phase8/g4_pass1_run.log`. Dataset cache: `~/datasets/coco/`,
`~/datasets/coco_val_only.yaml`.

### ✓ G10 — MVSim Tier-1 exploration sweep

Full sweep N ∈ {1, 2, 3, 5} × 3 trials = 12 runs, 300 s timeout each,
MVSim Tier-1 (12×10 m arena, 5 objects). Sweep wrapper
`af_swarm/scripts/run_explore_benchmark.py` (patched 2026-04-18 for a
stale swarm_coordinator + TRANSIENT_LOCAL latching bug) invoked
sequentially from a for-loop; per-trial CSVs landed in
`logs/phase8/g10/trial{t}_n{n}.csv`, each ~300 samples.

All 12 runs terminated at the 300 s timeout in the `returning` state
(none reached the 90 % union-coverage stop condition within the budget).

| N | trials | final coverage (mean ± std) | objects found (mean ± std) | elapsed (s) | reached ≥90 % |
|---|---|---|---|---|---|
| 1 | 3 | 80.1 % ± 12.7 | 2.67 ± 0.47 | 302.3 | 1/3 |
| 2 | 3 | **89.3 % ± 3.7** | 2.67 ± 0.47 | 302.3 | 1/3 |
| 3 | 3 | 48.5 % ± 33.9 | 3.00 ± 1.63 | 302.3 | 0/3 |
| 5 | 3 | 31.3 % ± 2.0 | **3.33 ± 0.47** | 302.9 | 0/3 |

Mean time-to-90 %-coverage: N=1 295 s (1 trial); N=2 110 s (1 trial).
Mean time-to-all-5-objects: only N=3 trial 2 ever cleared it (188.9 s).

Union-coverage scaling is non-monotonic and peaks at N=2 in this arena.
Larger fleets claim more unique frontiers but the shared costmap and
tight 12×10 m bounds let robots block each other — N=3 shows very high
run-to-run variance (std 33.9 %) consistent with contention, and N=5
collapses to 31 % as most cycles are spent on stale-frontier
re-planning. Object discovery still rises with N (1→3 robots finds
+0.33, 1→5 finds +0.67) because multiple observers increase the chance
that the YOLO+temporal-vote pipeline catches a target within the run.
For the delivered frontier-assignment policy, N=2 is the sweet spot on
this map size; scaling beyond it needs either a larger arena or
explicit frontier-dispersion shaping.

**Artefacts:** `report/data/explore_sweep_summary.csv`,
`report/figures/explore_sweep_time.png` (final coverage + objects bars),
`report/figures/explore_sweep_curves.png` (time-series),
`report/figures/explore_sweep_scaling.png` (time-to-90%-coverage +
time-to-all-objects vs N — the §5.8 Tier-1 scaling curves), 12× per-trial
CSV at `logs/phase8/g10/trial{1..3}_n{1,2,3,5}.csv`, sweep driver log at
`logs/phase8/g10/sweep_final.log`. Legacy single-trial CSV at
`logs/phase6/benchmarks/explore_benchmark.csv` renamed to
`explore_benchmark_legacy_single_trial.csv` (superseded).

**Known gap vs plan §5.8:** frontier-claim contention rate is listed as
a Tier-1 metric but is not captured — the benchmark driver does not log
`/fleet/frontier_claims` events. Reported in Chapter 5 as an explicit
limitation; adding it requires either instrumenting the sweep driver or
post-hoc rosbag parsing, neither done in this session.

**Limitations and cheapest upgrades before Chapter 5 lock.** All 12 runs
hit the 300 s timeout, so this sweep measures *coverage in 5 min*, not
*time to full coverage*. The N≥3 collapse is consistent with frontier
contention under the delivered greedy assignment (no reservation /
dispersion) in a tight 12×10 m arena, not with a fundamental algorithmic
failure — Burgard et al. (2005) describe the standard fix. Three low-cost
upgrades if revisited:
1. Re-run at 600 s timeout (~2 h total) — N≥3 may recover once initial
   contention resolves; separates arena-scoping from pure contention.
2. Re-run in a 20×20 m MVSim world — decouples the algorithmic result
   from the lab-scale 12×10 m bound.
3. Add a 5-line frontier-dispersion rule in `swarm_coordinator` (reject
   frontiers within radius R of a peer's active target). Most
   publishable; would likely flip the N≥3 curve upward.

## Resume checklist

1. Charge MentorPi to ≥ 7.8 V (ideally 8.0+).
2. On Pi: `bash af_bringup/scripts/stop_pi.sh` then
   `bash af_bringup/scripts/start_pi.sh enable_explore:=false` for G1/G2, or
   `ros2 launch af_bringup nav2.launch.py map:=<room1.yaml>` if only Nav2
   is needed.
3. On Dev PC: RViz `navigation` mode, set 2D Pose Estimate if AMCL cold.
4. Run:
   - G1 — `python3 report/scripts/g1_nav2_goals.py` (2 goals queued).
   - G2 — DWB critic ablation (10 goals × 2 critic sets; new script TBD).
   - G6 — graceful degradation (kill `yolo_onnx_node` mid-goal, log Nav2
     recovery).
5. On-Pi / lab still pending: G4 Pass 2 on-Pi ONNX mAP, G12 latency
   box-plot (piggybacks on G1/G2/G6 lab session).
6. Simulation-only still pending: G9 sim-to-real spot-check.

**Session close 2026-04-18 — Dev-PC block complete.** 7/12 Chapter 5
experiments captured today (G3, G5, G11 on Pi; G7, G8, G4 Pass 1, G10
on Dev PC). Remaining 5 (G1, G2, G4 Pass 2, G6, G9, G12) deferred to
next session.
