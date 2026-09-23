# Running the mission — aircraft and SITL

Three flight nodes, same code underneath:

| node | flight |
|---|---|
| `mission_fsm_part1` | pad → window marker, land on it |
| `mission_fsm_part2` | window marker → dark room → back to the marker |
| `mission_fsm_full` | **both in one flight**, with no landing at the marker |

Every real value lives in `config/hw_part1.yaml`, `hw_part2.yaml`, `hw_full.yaml`.
Change the mission there, not on the command line.

---

## 1. PX4 parameters (once, in QGC)

| parameter | value | why |
|---|---|---|
| `UXRCE_DDS_NS` | `uav_2` | every node talks to `/uav_2/fmu/...` |
| `EKF2_RNG_K_GATE` | `5.0` | the window sill must not kick the TFmini out of EKF2 |
| `MPC_LAND_SPEED` | `0.1` | PX4 only confirms touchdown at ≥ 0.9 × this; the missions land at 0.10 m/s |
| `MPC_TKO_RAMP_T` | `4.0` | gentler lift-off: less to correct in the first metre |

The FC must also export these two topics (`/etc/uxrce_dds_client/dds_topics.yaml`,
then reboot). `preflight_check` prints the exact YAML if they are missing:

```yaml
  - topic: /fmu/out/failsafe_flags
    type: px4_msgs::msg::FailsafeFlags
  - topic: /fmu/out/estimator_status_flags
    type: px4_msgs::msg::EstimatorStatusFlags
```

## 2. THE PILOT ARMS. ALWAYS.

`request_offboard_from_ros` and `arm_from_ros` both default to **false**: the
flight node streams setpoints and waits. **You** switch Offboard and **you**
arm, from the transmitter or the imav_bringup dashboard. There is no timeout
on either wait, and moving the switch back takes the aircraft off the node.

The node's own sequence, printed as it goes:

```
Waiting for you to flip the Offboard switch on the TX...
Offboard active. WAITING FOR YOU TO ARM (transmitter, or the imav_bringup dashboard).
ARMED. Holding on the ground for 5 s.
Climb speed 0.30 m/s (the first metre, gently)
```

## 3. Terminal by terminal

**T1 — the stack** (agent, RealSense + VIO, C920, zenoh). Wait for
`OK: topics under /uav_2/fmu`:

```bash
ros2 launch imav_bringup bringup.launch.py
```

**T2 — preflight, before every session.** Read-only: no publishers, no
command can arm. Safe with props on. It names the blocking arming check:

```bash
ros2 run drone_testing preflight_check --stream
```

**T3 — the lidar (PART 2 AND FULL ONLY).** Put the aircraft ON the marker,
pointing exactly at the window, BEFORE this: it seeds the room frame from
that heading, and refuses to start if it cannot read one.

```bash
ros2 launch drone_testing lidar_real.launch.py
#   -> "lidar_real: seed_yaw_offset +0.0000 rad (pad heading ...)"
#   starts the LDS-01 + scan_leveler + wall_localizer + pose_kf,
#   remapped onto /uav_2/fmu/... (lidar_loc subscribes un-namespaced)
```

**T4 — the support stack** (aruco_pose on the C920, floor_line, and for
part 2/full also window_detect and the doll model):

```bash
ros2 launch drone_testing mission_fsm_part1.launch.py     # or _part2 / _full
```

**T5 — the flight.** `q` aborts into a descent, `k` force-disarms:

```bash
P=$(ros2 pkg prefix drone_testing)/share/drone_testing/config

ros2 run drone_testing mission_fsm_part1 --ros-args --params-file $P/hw_part1.yaml \
  2>&1 | tee ~/part1_$(date +%H%M).log
ros2 run drone_testing mission_fsm_part2 --ros-args --params-file $P/hw_part2.yaml \
  2>&1 | tee ~/part2_$(date +%H%M).log
ros2 run drone_testing mission_fsm_full  --ros-args --params-file $P/hw_full.yaml \
  2>&1 | tee ~/full_$(date +%H%M).log
```

Browser views while it flies: down camera `http://<jetson>:8080`, window
camera `:8081`, carpet track `:8082`.

## 4. The arguments worth knowing

Anything in the YAML can be overridden with `-p name:=value` after it.

**Geometry (measured, change if the arena changes)**

| parameter | value | what |
|---|---|---|
| `window_marker_id` / `pad_marker_id` | `0` | the 80 × 80 cm ArUco |
| `window_offset` | `0.15` | m left, marker → window axis |
| `standoff_distance` / `inside_distance` / `outside_distance` | `1.67` / `0.80` / `1.67` | m |
| `traverse_centre_offset` | `0.07` | m above the window centre (see below) |
| `window_min_size` / `window_max_size` | `0.45` / `0.80` | the blue 60 cm window; rejects the red 40 cm one |
| `room_x` / `room_y` | `2.5` | the dark room |
| `room_scan_sequence` | `forward 0.5, right 0.5, backward 0.5, left 0.5, yaw 180` | |
| `relock_standoff` | `1.25` | m off the wall for the exit search |
| `track_width_m` / `track_width_tol` | `0.60` / `0.25` | our lane; the centre lane is 1.20 m and carries the obstacles |
| `track_lane_guard` | `0.60` | m off the lane line before the track is not ours (lanes are 1.50 m apart) |

`traverse_centre_offset` is a trade: the lidar sits 13.5 cm above the aim, and
its plane must stay inside the 60 cm opening for the window-gap fix. `+0.07`
leaves 9.5 cm to the top edge (~3° of pitch at 1.67 m); `-0.05` leaves 21.5 cm
(~7°) and is the robust alternative.

**Speeds**

| parameter | value |
|---|---|
| `track_speed` | `0.6` m/s along the carpet track, and the cap on \|(vx, vy)\| |
| `leg_speed` / `traverse_speed` | `0.6` m/s |
| `approach_speed` | `0.30` m/s onto the standoff point |
| `strafe_speed` | `0.25` m/s for the 0.15 m strafes |
| `speed` | `0.35` m/s between room points |
| `room_yaw_rate` | `0.12` rad/s (~7°/s) for the half turn |
| `slow_land_speed` | `0.10` m/s |
| `gentle_climb_speed` / `gentle_climb_alt` | `0.30` m/s below `1.0` m |

**Safety inside the room**

| parameter | value | what |
|---|---|---|
| `room_lidar_recover` | `15.0` | s holding still for the lidar before giving up |
| `room_yaw_reset_max_deg` | `20.0` | summed EKF2 heading resets before the yaw is not trusted |
| `room_lost_action` | `exit` | `exit` = fly out on a good heading; `land` = always put down |

Lidar lost → hold still, wait, then fly out. Yaw untrusted → land in place,
because "out" is flown on the heading. Both say so loudly in the log.

**Switching things off**

```
follow_line:=false     # do not follow the carpet track on the leg
exit_mode:=retrace     # do not re-find the window from inside; fly back out blind
room_mode:=box         # dead-reckoned room pattern instead of the lidar
dolls:=false           # (launch arg) no doll model
line_stream_port:=0    # (launch arg) no carpet browser view
```

## 5. The doll count

`doll_detect` reads the **C920 straight down**, ranges by the TFmini, and tags
each doll in the **lidar room frame** so EKF2 drift cannot double-count.
It prints in the flight terminal:

```
DOLL COUNT: 2 (raw /doll_count)          <- the flight node, on every change
Tile 3/4 done. Dolls so far: 2
ROOM DONE. DOLLS COUNTED: 3 (raw)
FINAL DOLL COUNT: 3. Positions (ARENA, m): ...
```

Restart it per flight: the count is cumulative within one run.

## 6. Calibrate the down camera (worth doing once)

Everything that turns pixels into metres — marker offsets, doll positions, the
carpet lane width check — assumes a 70.4° field of view and no distortion:

```bash
ros2 run drone_testing calibrate_camera --ros-args \
  -p image_topic:=/image_raw -p squares_x:=9 -p squares_y:=6 -p square_size:=0.025
```
Show a chessboard around the frame; it writes a `camera_info` YAML and prints
the true FOV. Point `usb_cam`'s `camera_info_url` at it and every node picks it
up.

---

## 7. SITL (the laptop)

The simulator has **no transmitter**, so there the node must do it itself, and
the world is 2.2× scale, so the lanes and heights differ from the aircraft.

```bash
# 1. the sim (exits when PX4 is up, ~60 s)
bash ~/simlogs/sim_up.sh world:=imav2026_scaled x:=-4.4 y:=-14.3 \
  marker_size:=0.88 lidar_config:=sim lidar_range:=7.7 \
  discovery_range:=SUBNET operator:=false

# 2. the flight   (log: ~/simlogs/pfull.txt)
bash ~/simlogs/pfull.sh          # full mission
bash ~/simlogs/part1.sh          # part 1
bash ~/simlogs/p2detect.sh       # part 2
```

What those scripts override, and why:

```
-p request_offboard_from_ros:=true -p arm_from_ros:=true   # no TX in the sim
-p track_width_m:=1.32 -p track_lane_guard:=1.5            # 2.2x lanes
-p cruise_altitude:=5.50 -p window_altitude_m:=3.85        # 2.2x heights
-p traverse_centre_offset:=0.154                           # 2.2x
-p room_x:=5.412 -p room_y:=5.412 -p relock_standoff:=1.76 # 2.2x room
lidar_range:=7.7                                           # 3.5 m x 2.2
```

SITL-only PX4 parameters, set by `sitl.launch.py` and **not** wanted on the FC:
`COM_OF_LOSS_T 3.0` (the laptop runs the world at 0.2–0.7× real time, so a
wall-clock setpoint stream thins out in sim time) and `EKF2_RNG_A_HMAX 8.0`
(the scaled 5.5 m leg is above the 5 m default, and with no baro the height
then goes invalid).

## 8. When something stops the flight

| symptom | cause | fix |
|---|---|---|
| node waits, never arms | you have not flipped Offboard / armed | that is the design — do it from the TX |
| node looks hung, then arms when you kill something | a second flight node was streaming setpoints | it now says so 3 s after startup; `pgrep -af mission_fsm` |
| refuses to take off, "no second opinion" | `estimator_status_flags` not exported | §1, then reboot the FC |
| `offboard_control_signal_lost` | the setpoint stream stalled | in SITL it is sim-time; on the aircraft check CPU load |
| lands right after the sill | EKF2 dropped the rangefinder | `EKF2_RNG_K_GATE 5.0` |
| room scan gives up at once | lidar has no attitude | use `lidar_real.launch.py` (it remaps `/uav_2`) |
