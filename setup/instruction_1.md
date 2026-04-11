# Connecting to the MentorPi M1 and testing Phase 1

End-to-end walkthrough. Assumes a freshly-powered factory robot and your Dev PC
(`/home/taha/ros2_ws` with ROS 2 Humble + the `autonomousfleet-ai` repo).

---

## 1. Power on and network

1. Flip the power switch on the chassis. Wait ~45 s for the Pi 5 to boot and
   the OLED to show an IP address.
2. Join the robot's network. The robot ships in **AP mode** — it broadcasts
   its own hotspot. To get apt access on the robot for dependency installs
   you need to switch it to **STA mode** (join your home Wi-Fi). The WonderPi
   app's "LAN Mode" does *not* configure Wi-Fi on the robot — it just
   discovers a robot that's already on your LAN. The actual switch is a
   config edit on the Pi host.

   **2a. First connection — always via AP mode:**
   - On your Dev PC/phone, join Wi-Fi SSID `HW-xxxxxxxx`, password `hiwonder`.
   - The robot is at `192.168.149.1`.
   - If you just want to do a quick offline test, skip to step 2 (you'll need
     to pre-download any apt packages another way).

   **2b. Switch to STA mode via VNC (recommended, gives internet for `apt`):**

   The factory image ships with a VNC server already running on port `5900`.
   SSH is not reliable on the factory image — use VNC to reach the Pi desktop
   and do the config edit from there.

   1. Install **RealVNC Viewer** on your Dev PC (once):
      `sudo apt install realvnc-vnc-viewer`  (or download from
      https://www.realvnc.com/en/connect/download/viewer/).
   2. While still on the `HW-xxxxxxxx` hotspot, open VNC Viewer and connect to:
      - Address: `192.168.149.1` (port `5900` is the default, no suffix needed)
      - On the auth prompt: **Username `pi`**, **Password `raspberrypi`**
      - "Connection is not secure" warning → Continue.
   3. The Pi desktop appears. Click the **LXTerminal icon** (top-left of the
      desktop, looks like `>_`). A shell opens on the Pi *host* (not inside
      the Docker container — good, that's what you want).
   4. In that terminal, edit the Wi-Fi config:

      ```bash
      sudo nano ~/hiwonder-toolbox/wifi_conf.py
      ```

      If that path doesn't exist, locate it:

      ```bash
      sudo find / -name "wifi_conf.py" 2>/dev/null
      ```

      Set these three values:

      ```python
      HW_WIFI_MODE = 2                      # 1 = AP, 2 = STA
      HW_WIFI_STA_SSID = "YourHomeWifi"
      HW_WIFI_STA_PASSWORD = "your-password"
      ```

      Save (`Ctrl-O`, Enter, `Ctrl-X`), then apply:

      ```bash
      sudo systemctl restart wifi.service
      ```

   5. The VNC session will freeze/disconnect — that's expected, the hotspot
      you were connected through just dropped. After ~10 s the robot's OLED
      shows the new LAN IP. Rejoin your home Wi-Fi on your Dev PC, then
      reconnect VNC to the new IP (same credentials) for the rest of this
      walkthrough.

   To revert to AP mode later: VNC in, edit `wifi_conf.py`, set
   `HW_WIFI_MODE = 1`, `sudo systemctl restart wifi.service`. The robot comes
   back as `HW-xxxxxxxx` / `192.168.149.1`.

## 2. Open a shell on the Pi

You already have VNC working from step 1. Connect again to `<robot-ip>` (the
LAN IP after the STA switch) in **RealVNC Viewer**, auth `pi` /
`raspberrypi`. Click the **LXTerminal** icon on the desktop — all subsequent
commands run in that terminal (or any additional LXTerminal window you open
on the desktop).

## 3. Enter the factory Docker container

The factory image runs everything inside a container named `MentorPi` (image
`ros:humble`) that's permanently alive via `tail -f /dev/null`:

```bash
docker ps                                                      # verify "MentorPi" is "Up"
docker exec -it -u ubuntu -w /home/ubuntu MentorPi /bin/bash
```

You are now `ubuntu@raspberrypi:~$` inside the container. Everything from
step 4 onwards happens **inside the container**.

## 4. Stop the factory auto-start stack

The factory image auto-launches its own ROS nodes on boot. They bind the
STM32 serial port and the camera, which will conflict with our HAL. This
step has two parts: host-level systemd stops, then a container-level node
list check.

**4a. Host-level (runs on the Pi, not inside Docker):**

The factory auto-launch is a systemd unit called `start_node.service`
(description "My Docker App") that runs `docker exec` into the `MentorPi`
container and launches ~20 factory ROS nodes (ros_robot_controller,
odom_publisher, usb_cam, MS200 lidar, ekf_filter_node, joystick_control,
etc.). These will fight for the STM32 serial port, camera, and LiDAR. Stop
and disable it:

```bash
# In LXTerminal on the Pi desktop (or over SSH) — you are NOT in the container yet.
sudo systemctl stop start_node.service
sudo systemctl disable start_node.service
```

`ros2` is **not** installed on the host — it only lives inside the Docker
container — so `ros2: command not found` on the host is expected.

To revert to factory mode later: `sudo systemctl enable --now start_node.service`.

**4b. Container-level (enter the Docker container and verify):**

The factory container's shell does not auto-source ROS 2 in new sessions.
Fix that once so every future `docker exec` just works:

```bash
docker exec -it -u ubuntu -w /home/ubuntu MentorPi /bin/bash

# One-time setup — persist ROS 2 on the PATH for every new shell:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

# Verify no factory nodes are running:
ros2 daemon stop && ros2 daemon start
ros2 node list        # expect empty output, or only /rosout
```

If factory nodes are still listed (leftover processes that didn't die when
you stopped `start_node.service`), kill them inside the container:

```bash
pkill -9 -f 'ros2|_node|ros_robot_controller|usb_cam|slam' 2>/dev/null
sleep 2
ros2 daemon stop && ros2 daemon start
ros2 node list        # must be empty or only /rosout before proceeding
```

Do not touch `wifi.service` — that's what keeps you connected to the network.

## 5. Clone the repo into the container workspace

Phase 1 code currently lives on the `phase-1-foundation` branch — it will only
be on `main` after PR #1 is merged (which happens *after* this hardware
validation). Clone and check out that branch:

```bash
mkdir -p ~/workspace/ros2_ws/src
cd ~/workspace/ros2_ws/src
git clone -b phase-1-foundation https://github.com/syed-taha-ali/autonomousfleet-ai.git
cd ~/workspace/ros2_ws
```

Once Phase 1 is merged to `main`, drop the `-b phase-1-foundation` flag.

## 6. Install dependencies

The factory image's ROS 2 apt signing key (`F42ED6FBAB17C654`) is expired. If
`apt update` prints `EXPKEYSIG … Open Robotics`, rotate it first:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then install. Note: `ros-humble-rf2o-laser-odometry` is **not** in the apt index
for this image — we build it from source below. `ros-humble-topic-tools` is
required for the LiDAR topic relay in step 9.

```bash
sudo apt update
sudo apt install -y ros-humble-robot-localization ros-humble-imu-complementary-filter ros-humble-usb-cam ros-humble-teleop-twist-keyboard ros-humble-topic-tools python3-serial python3-yaml xterm
rosdep update
rosdep install --from-paths src/autonomousfleet-ai --ignore-src -r -y
```

(Single line because bash `\` line continuations break when pasted through VNC
— trailing whitespace turns each line into a standalone command.)

Build `rf2o_laser_odometry` from source:

```bash
cd ~/workspace/ros2_ws/src
git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/workspace/ros2_ws
```

Harmless noise to expect during `apt` operations: `ldconfig: File
/lib/aarch64-linux-gnu/libcudnn*.so … is empty` and similar for `libnvinfer`,
`libvisionworks`, `libv4l*`. These are pre-existing broken symlinks in the
factory image pointing at NVIDIA Jetson libraries that don't exist on a Pi 5 —
ignore them.

## 7. Build (Pi 5 with 4 GB — keep jobs low)

```bash
cd ~/workspace/ros2_ws
MAKEFLAGS="-j2" colcon build --packages-select af_msgs af_hal af_description af_bringup rf2o_laser_odometry --symlink-install
source install/setup.bash
echo "source ~/workspace/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Expect `setup.py install is deprecated` / `easy_install is deprecated` stderr
warnings from `af_bringup`, `af_description`, `af_hal` — these are the standard
ament_python deprecation warnings on Humble and do **not** indicate failure.
Look for `Summary: 5 packages finished` to confirm success.

## 8. Verify hardware devices

```bash
ls /dev/ttyUSB* /dev/ttyACM*     # expect /dev/ttyUSB0 (STM32) and /dev/ttyACM0 (MS200)
ls /dev/video*                   # /dev/video0 is the USB camera; the many video2x
                                 #   nodes are v4l2 metadata devices, ignore them
```

If the STM32 port is not `/dev/ttyUSB0`, note which it is —
`ros_robot_controller_node` parameter `port` defaults to `/dev/ttyUSB0` and
can be overridden with `--ros-args -p port:=/dev/ttyUSB1`. The MS200 is bound
by the factory driver via a udev symlink at `/dev/ldlidar`, not by
`/dev/ttyACM0` directly.

## 9. Start the factory MS200 LiDAR driver

We don't write a LiDAR driver; we consume `/scan` from HiWonder's factory
`peripherals` package. That package lives in a **separate** factory workspace
at `/home/ubuntu/ros2_ws/` (not our `~/workspace/ros2_ws`), so our
`.bashrc` overlay doesn't see it — source the factory overlay on top:

```bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 pkg list | grep -iE 'peripherals|lidar'
# expect: ldlidar_stl_ros2, oradar_lidar, peripherals, sllidar_ros2, ydlidar_ros2_driver
```

**Do not use `peripherals lidar.launch.py`** — it reads `need_compile` and
`LIDAR_TYPE` env vars set by the factory's startup scripts and will crash with
`KeyError: 'need_compile'` otherwise. Use the MS200 include launch directly:

```bash
ros2 launch peripherals ms200_scan.launch.py &
```

**Topic name gotcha**: the driver logs `ROS topic:MS200/scan` but actually
publishes to **`/scan_raw`** (the log string is misleading). Verify with
`ros2 topic list`, then relay to `/scan` so downstream consumers (Nav2,
slam_toolbox, rf2o) see it on the standard name:

```bash
ros2 topic list                          # should include /scan_raw
ros2 run topic_tools relay /scan_raw /scan &
sleep 1
ros2 topic hz /scan                      # expect ~10 Hz. Ctrl-C to stop the check.
```

Keep both background jobs alive (`jobs` should show the launch and the relay)
for step 10.

**Next-phase TODO**: bake the `peripherals ms200_scan.launch.py` include and
the `/scan_raw → /scan` relay into `af_bringup` so the LiDAR comes up
automatically without sourcing the factory workspace or running a manual
relay.

## 10. Launch Phase 1 bringup

Open a **second** `docker exec` shell (same command as step 3) so the LiDAR
keeps running:

```bash
source ~/workspace/ros2_ws/install/setup.bash
ros2 launch af_bringup robot.launch.py
```

You should see startup logs from:

- `robot_state_publisher`
- `ros_robot_controller_node`, `odom_publisher_node`, `imu_calib_node`, `hardware_watchdog_node`
- `complementary_filter_gain_node`
- `ekf_filter_node` (fuses `/odom_raw` + `/imu` only — rf2o was dropped during Phase 1 validation because the factory MS200 driver emits partial 316° scan chunks and rf2o's scan-matching produced sign-flipped deltas that corrupted `/odom`)
- `lidar_frame_bridge` (static TF `lidar_frame → laser_frame`; the MS200 driver tags scans with the vendor's `laser_frame` while the URDF link is `lidar_frame`, so rviz's TF filter needs the bridge)
- `usb_cam` (raw `yuyv` pixel format; **do not** use `yuyv2rgb` — the usb_cam 0.6.x swscaler path aborts with `Invalid v4l2 format` on this camera after ~7 s)

Errors to watch for: `SerialException` (STM32 port wrong), `Could not open
/dev/video0` (camera taken by factory service — re-run step 4).

## 11. Set up the Dev PC side

On your **Dev PC** (`/home/taha/ros2_ws`), in separate terminals:

```bash
# Match the robot's ROS_DOMAIN_ID (factory default is 0; check with
# `echo $ROS_DOMAIN_ID` inside the container).
export ROS_DOMAIN_ID=0

# Verify you see the robot's topics
ros2 topic list                          # expect /scan /odom /imu /cmd_vel ...
ros2 node list                           # expect the af_hal nodes
```

If nothing appears: the Pi and Dev PC must be on the same subnet, and the DDS
multicast must traverse the AP. If you're in AP mode on a `HW-xxxxxxxx`
hotspot this usually just works; on a corporate LAN, set
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on both ends and restart.

Then launch RViz2 and teleop:

```bash
# Terminal 1 — RViz2
rviz2

# Terminal 2 — keyboard teleop (publishes /cmd_vel)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

A pre-built RViz config lives at `af_bringup/config/phase1_validation.rviz`
with all the displays already wired up (Fixed Frame `odom`, RobotModel, TF,
LaserScan `/scan` BEST_EFFORT, Odometry `/odom`, Image `/image_raw` raw
transport, Imu `/imu` BEST_EFFORT):

```bash
rviz2 -d ~/ros2_ws/src/project/af_bringup/config/phase1_validation.rviz
```

**Image display transport**: use raw `/image_raw`, **not**
`/image_raw/compressed`. usb_cam publishes compressed frames with
`format: yuv422_yuy2; jpeg compressed mono8`, a non-standard variant rviz's
`compressed` transport plugin cannot decode (you'll see "No Image" with no
error). The Phase 4 Dev PC perception pipeline will handle YUY2 decoding
explicitly; for bringup validation, raw over LAN is fine.

**LaserScan QoS**: the MS200 driver publishes with BEST_EFFORT reliability.
The provided rviz config already sets this — if you add the display manually,
set Reliability Policy to Best Effort or all scans will be dropped.

**If you launch rviz2 from a VS Code integrated terminal** and it crashes on
`libpthread.so.0: undefined symbol: __libc_pthread_init`, that's the VS Code
snap env (`GTK_PATH`, `LOCPATH`) leaking into the child process and pulling in
snap libs. Workaround — scrub the env:

```bash
env -i HOME=$HOME USER=$USER DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY \
    PATH=/opt/ros/humble/bin:/usr/bin:/bin \
    bash -c 'source /opt/ros/humble/setup.bash && rviz2 -d ~/ros2_ws/src/project/af_bringup/config/phase1_validation.rviz'
```

## 12. Walk through the Phase 1 test criteria

Run each check and tick it off. **Keep the robot on blocks for the first motor
test** so a runaway wheel doesn't launch it off the desk.

| # | Test | Command | Pass criterion |
|---|------|---------|----------------|
| 1 | **All 4 motors spin** | In teleop: press `i` (fwd), `,` (rev), `j`/`l` (yaw), `u`/`o` (strafe FL/FR), `m`/`.` (strafe BL/BR) | Each wheel actually spins in the commanded direction. Mecanum strafe (`j`/`l` vs `u`/`o`) should translate the chassis sideways, not rotate. |
| 2 | **IMU publishes ~50 Hz with orientation** | `ros2 topic hz /imu` then `ros2 topic echo /imu --once` | Rate 40–60 Hz; `orientation` quaternion is not `(0,0,0,0)`; rotate the robot by hand and watch yaw change in RViz. |
| 3 | **Odometry integrates ~correctly** | Place tape at 1 m. `ros2 topic echo /odom_raw --field pose.pose.position`. Drive forward 1 m. **Do not use `teleop_twist_keyboard` for this test** — key-repeat timing jitter produces 5–10% overshoot on a 10 s run. Use a tight in-process `rclpy` publisher (`/tmp/straight_test.py` pattern, 20 Hz × 10 s × 0.1 m/s). | Reported x delta within ±5% of 1.0 m. Repeat for lateral (`y` strafe) — should register `y` movement. Note `odom_publisher_node` integrates commanded velocity, not encoder feedback, so slippage/stalls are invisible to `/odom_raw`. |
| 4 | **EKF fuses without divergence** | `ros2 topic echo /odom --field pose.pose.position` while driving a 1 m square (four forward legs + three 90° turns at 1.0 rad/s — lower rates under-rotate on smooth tile due to stiction). | Position returns near origin; no NaNs; no warnings from `ekf_filter_node` about "failed to transform". EKF fuses `/odom_raw` + `/imu` only — rf2o is deliberately excluded. |
| 5 | **LiDAR at ~10 Hz in RViz2** | `ros2 topic hz /scan`; view `LaserScan` in the provided rviz config. | 7–10 Hz (MS200 spec 7–15 Hz), points match the room layout. Note: the factory driver publishes **partial 316° scan chunks per message**, not full 360° accumulated scans — RViz accumulates fine, but Phase 2 SLAM consumers may need a scan aggregator. |
| 6 | **Camera stream decodable on Dev PC** | `ros2 topic echo /image_raw --no-arr --once` to confirm frames arrive; view `/image_raw` (raw) in the rviz config. | Stable visible image at ~11 Hz raw / ~16 Hz compressed. **rviz Image display must use `raw` transport on `/image_raw`** — the `compressed` transport cannot decode usb_cam's `yuv422_yuy2; jpeg` variant. |
| 7 | **Battery voltage correct** | `ros2 topic echo /ros_robot_controller/battery` | UInt16 in **millivolts**, around `6800`–`8400` for a 2S 7.4 V Li-ion pack (the shipped pack is **2S**, not 3S — the firmware reports mV, not centivolts). `hardware_watchdog_node` converts via `msg.data / 1000.0` and defaults to `warn_voltage=6.8`, `error_voltage=6.4`. |
| 8 | **Watchdog diagnostics** | `ros2 topic echo /diagnostics` | Publishing at ~2 Hz with level `OK` and text `battery <V>`. To confirm the `WARN` transition fires, relaunch the watchdog alone with an elevated threshold: `ros2 run af_hal hardware_watchdog_node --ros-args -p warn_voltage:=8.5` — you should see a `[WARN] battery low` log within one cycle. |

## 13. Reporting back

Capture a rosbag of the full session for the PR record:

```bash
ros2 bag record -o phase1_validation /scan /odom /odom_raw /imu /cmd_vel \
    /diagnostics /tf /tf_static /image_raw/compressed /ros_robot_controller/battery
```

Drive the square, trigger all eight checks, `Ctrl-C` the bag. Report which
rows passed/failed and paste any red error lines from the `robot.launch.py`
terminal — triage fixes before merging **PR #1**.

**Rosbag rates sanity check**: a clean single-instance stack should produce
`/imu ~48 Hz`, `/odom ~49 Hz`, `/scan ~7 Hz`, `/diagnostics ~2 Hz`,
`/image_raw/compressed ~16 Hz`. If any rate is a clean integer multiple of
these, you have duplicate processes from a previous bringup (see pkill note
below).

**`pkill` 15-char gotcha**: Linux truncates `/proc/<pid>/comm` to 15 chars, so
`pkill odom_publisher_node` (19 chars) silently matches nothing. Each
restart then layers new processes on top of old ones and topic rates come
out 2× or 3× inflated. Kill by 15-char comm prefixes instead:

```bash
pkill -KILL ros_robot_contr    # ros_robot_controller_node
pkill -KILL odom_publisher_    # odom_publisher_node
pkill -KILL complementary_f    # complementary_filter_gain_node
pkill -KILL imu_calib_node     # imu_calib_node
pkill -KILL hardware_watchd    # hardware_watchdog_node
pkill -KILL usb_cam_node_ex    # usb_cam_node_exe
pkill -KILL ekf_node           # ekf_node (already ≤15 chars)
```

**Do NOT** put these in a `bash -c "..."` string together with `pkill -f
robot.launch.py` — `pkill -f` matches the running bash's own argv and will
SIGKILL the parent shell (exit 137). If you need a broader kill, write the
commands to a script file first (`/tmp/af_kill.sh`) and run
`bash /tmp/af_kill.sh` so the patterns only appear in the script's argv,
never in a live command line.

## Teardown

```bash
# In each ros2 launch terminal: Ctrl-C
exit             # leaves the container
# On Dev PC: Ctrl-C rviz/teleop
# Power off the robot via the switch (not a yank — let filesystems flush)
```

---

**Phase 2 bake-in**: the MS200 driver is `peripherals` package, launch file
`ms200_scan.launch.py`, publishes on `/scan_raw`. Include it from
`af_bringup/robot.launch.py` with an embedded `topic_tools/relay` node for
`/scan_raw → /scan` so the manual source-and-relay dance in step 9 goes away.

**SSH/docker-exec fast path** (documented in memory): once the Pi is on the
LAN, passwordless SSH from the Dev PC works via
`ssh pi@192.168.1.117 'docker exec -u ubuntu -w /home/ubuntu MentorPi bash -lc
"source /opt/ros/humble/setup.bash && source
/home/ubuntu/workspace/ros2_ws/install/setup.bash && <ros2 cmd>"'`. This
removes the need to open VNC for every command. Use it for `ros2 topic hz`,
one-off test drivers, rosbag starts, etc. For long-running launches, `docker
cp` a script into the container and run it with
`nohup ... > /tmp/af_logs/*.log 2>&1 & disown`.
