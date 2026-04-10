# AutonomousFleet AI

## Context
BSc Computer Science Final Year Project (module 6500CSQR) by Syed Taha Ali (#103665) at Oryx University / Liverpool John Moores University, supervised by Prof. Moad Idrissi. Spec approved 2025-11-06.

**Hardware**: HiWonder MentorPi M1 — Raspberry Pi 5 (4GB) + STM32F407 RRC Lite, Mecanum wheels, MS200 LiDAR, depth camera, QMI8658 IMU.
**Platform**: Ubuntu 22.04 + ROS2 Humble (Docker container on Pi). Dev PC: Ubuntu 22.04 + ROS2 Humble + RTX 3060.

## Goals
1. Functional autonomous ground robot: SLAM, Nav2, sensor fusion, vision-augmented obstacle avoidance.
2. AI/NLP mission control — natural language → validated ROS2 navigation commands.
3. Swarm simulation scaling to 100+ agents in Gazebo (3-tier hybrid approach).
4. Modular Python-first ROS2 packages; academic publication if novel results emerge.

## Current State
**Phase 0 (Evidence Review)**: complete.
**Phase 1 (Foundation & HAL)**: code landed, awaiting hardware validation.
Packages created: `af_msgs`, `af_hal`, `af_description`, `af_bringup`. Ported
`ros_robot_controller_node`, `odom_publisher_node`, `mecanum.py` from the inspo
(Jazzy → Humble, `enable_reception` bug fixed, mecanum-only, single `/cmd_vel`,
hard-coded paths removed). New Python nodes: `imu_calib_node`,
`hardware_watchdog_node`. URDF ported (ackermann variant removed), EKF config
rewritten with plain REP-105 frames, `af_bringup/robot.launch.py` composes HAL
+ description + imu_complementary_filter + rf2o + EKF + usb_cam.
**Phase 2 (SLAM & Localisation)**: not started.

## Key Documents
- **Implementation plan**: `/home/taha/.claude/plans/adaptive-strolling-sky.md` — full 8-section plan (architecture, modules A–G, roadmap, risks, publication strategy).
- **Project spec**: `./project_spc.pdf` — approved FYP specification.
- **Robot documentation**: `./robot_docs/` — HiWonder MentorPi M1 tutorials and hardware refs.
- **Inspiration project**: `./inspo_project/MentorPi_ris-main/` — ROS2 Jazzy reference (we port to Humble).

## Version Control

**GitHub**: `syed-taha-ali` (nbstaha@gmail.com)
**Repository**: `autonomousfleet-ai`
**Note**: `I am the only author of this project, claude should not appear anywhere.`

**Per-phase workflow** (run after each plan phase is successfully implemented and tested):
1. `git checkout -b phase-<N>-<short-name>` from `main`
2. Commit phase work to the branch
3. `git push -u origin phase-<N>-<short-name>`
4. `gh pr create --base main --head phase-<N>-<short-name>` with phase summary
5. Merge the PR into `main`
6. Delete the phase branch (local + remote)
7. Update the **Current State** section of this CLAUDE.md and refresh **Key Documents** if new important files were added
