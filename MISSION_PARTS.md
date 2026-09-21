# Mission split in two — real drone and SITL

The full mission (`mission_fsm`) split into two flights, each its own node.
Both subclass `MissionFSM`, so the legs, marker centring, window traversal,
dark-room scan and doll counting are the same code the full mission flies.

| | **Part 1** — `mission_fsm_part1` | **Part 2** — `mission_fsm_part2` |
|---|---|---|
| Start | takeoff pad, facing the course | **on the window marker, facing the window** |
| Altitude | 2.10 m | 1.90 m |
| Flight | forward up to 9.3 m, watching for the window marker (id 0) | strafe 0.15 m left → window in (0.80 m) → room `forward 0.8, right 0.8, backward 0.8, left 0.8, yaw 180` (dolls geotagged + counted) → window out (1.67 m) → strafe back |
| Landing | marker seen: centre (15 cm) → hold → **slow descent (0.10 m/s) holding the point**. No marker: same slow descent at 9.3 m | hover at the strafe end P, look 10 s: marker seen → centre → normal descent. Centring fails → back to P, look again (2 tries) → slow descent at P |

Directions are in the **takeoff frame** (the heading at arming), not the
drone's current heading. After the 180° turn in the room, "right" is still
right of the armed heading, so the strafe out mirrors the strafe in.

---

## A. Real drone

### A.1 PX4 parameters (QGC)

**Both configurations**

| parameter | value | why |
|---|---|---|
| `MPC_LAND_SPEED` | **`0.10`** | **required.** Both parts land at 0.10 m/s. PX4's land detector only accepts touchdown while the descent is ≥ 0.9 × `MPC_LAND_SPEED`; at the 0.7 default it never agrees the drone has landed and refuses the disarm (seen in SITL). |
| `SENS_EN_TFMINI` | `1` | TFmini Plus |
| `SENS_TFMINI_CFG` | your port | |
| `EKF2_RNG_CTRL` | `1` | rangefinder fusion — a hard arming gate in the node |
| `EKF2_HGT_REF` | `2` | height reference = rangefinder (not the baro, which drifts indoors) |
| `EKF2_MIN_RNG` | `0.10` | the standing height |
| `EKF2_RNG_POS_Z` | measured | lidar offset below the IMU |
| `EKF2_GPS_CTRL` | `0` | no GPS indoors |
| `UXRCE_DDS_CFG` | your TELEM port | |
| `SER_TEL2_BAUD` | `921600` | must match the agent |

**VIO (the default — `lateral_source:=vio`)**

| parameter | value | why |
|---|---|---|
| `EKF2_EV_CTRL` | `1` | vision horizontal position only (not 2 — that hands height to the camera) |
| `EKF2_OF_CTRL` | `0` | don't fuse flow and vision at once |
| `EKF2_EV_DELAY` | measure (start 40 ms) | the number that matters most |
| `EKF2_EVP_NOISE` | `0.1` | |
| `EKF2_EV_NOISE_MD` | `0` | use the message covariance |

**Optical flow instead (`lateral_source:=flow`)**

| parameter | value | why |
|---|---|---|
| `SENS_EN_PMW3901` | `1` | flow driver (SPI) |
| `EKF2_OF_CTRL` | `1` | fuse flow |
| `EKF2_EV_CTRL` | **`0`** | no vision — a leftover value leaves EKF2 waiting for vision that never comes |
| `EKF2_MAG_TYPE` | `0` | compass on: nothing else supplies heading |
| `EKF2_OF_QMIN` | tune | the PMW3901's quality number is coarse |
| `SENS_FLOW_ROT`, `EKF2_OF_POS_X/Y/Z` | as mounted | |

> **Check this on the bench before flying on flow.** In SITL (PX4 `a64536`,
> the version our `px4_msgs` match), EKF2 **never started fusing flow** with
> `EKF2_HGT_REF=2`: flow start needs a terrain estimate and there was none
> (`cs_rng_terrain: False`). Flow only started with the rangefinder as a
> height *aid* rather than the reference. With the drone armed-off on the
> floor, run `listener estimator_status_flags` on the FC console: if
> `cs_opt_flow` stays `False`, that is this. `local_position_invalid` ~1 s
> after arming is the symptom.

### A.2 Part 1 — on the Jetson

**VIO**

```bash
# T1 — RealSense + RTAB-Map VIO -> PX4. agent:=false: the mission launch runs the agent.
ros2 launch rtabmap_realsense_vio realsense_stereo_imu_rtabmap.launch.py \
    agent:=false cam_x:=0.105 cam_z:=-0.04

# T2 — support stack: DDS agent + aruco_pose (down camera).
#      camera:=false: T1 already owns the RealSense.
ros2 launch drone_testing mission_fsm_part1.launch.py camera:=false

# T3 — the flight, by hand so q/k work (q = abort to descent, k = kill)
ros2 run drone_testing mission_fsm_part1
```

**Optical flow** — no T1, and:

```bash
ros2 launch drone_testing mission_fsm_part1.launch.py camera:=false
ros2 run drone_testing mission_fsm_part1 --ros-args -p lateral_source:=flow
```

Part 1's defaults are the course: 2.10 m, 9.3 m leg, marker id 0.
Override with `-p`, e.g. `-p outbound_distance:=9.5 -p cruise_altitude:=2.2`.

### A.3 Part 2 — on the Jetson

Place the drone **on the window marker, pointing at the window.**

```bash
# T1 — VIO, with colour + aligned depth (the window detector and dolls need them)
ros2 launch rtabmap_realsense_vio realsense_stereo_imu_rtabmap.launch.py \
    agent:=false enable_color:=true align_depth:=true cam_x:=0.105 cam_z:=-0.04

# T2 — the 2D lidar wall localizer for the room
ros2 launch lidar_loc localize.launch.py

# T3 — support stack + the flight. agent_only:=false starts the flight node
#      from the launch, so it gets every window/room parameter the launch
#      sets. The q/k keys are not available this way: your RC kill switch is.
ros2 launch drone_testing mission_fsm_part2.launch.py camera:=false agent_only:=false
```

Check before arming that the window detector sees T1's camera:
`ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw`.

The doll count prints in T3 (`DOLLS:` from `doll_report_text`, count minus
`doll_offset`; `DOLL COUNT:` from the flight node, raw). For the big number,
in another pane: `ros2 run drone_testing doll_count_gui`.

---

## B. SITL (laptop: Ubuntu 24.04, ROS 2 Jazzy, gz sim 8)

Part 1 flies end to end in SITL: takeoff, 20 m leg on optical flow, marker
acquired, centred to 4 cm, slow descent, PX4-confirmed touchdown, disarm.

### B.1 One-time setup

```bash
# PX4 at the version our px4_msgs match (has the gz optical-flow + rangefinder support)
git clone --recursive https://github.com/PX4/PX4-Autopilot ~/PX4-Autopilot
cd ~/PX4-Autopilot && git checkout -f a64536802b5a5b6ba8fe6ef1b7dcb6a54a0a99ea
git submodule update --init --recursive --force
bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
pip3 install --user --break-system-packages -r Tools/setup/requirements.txt
sudo apt install -y libopencv-dev          # the optical-flow plugin needs it
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=$(ls -d /opt/ros/jazzy/opt/*_vendor | tr '\n' ':')$CMAKE_PREFIX_PATH
export GZ_DISTRO=harmonic                  # find ROS's vendored gz-sim 8
rm -rf build/px4_sitl_default && make px4_sitl
find build/px4_sitl_default -name libOpticalFlowSystem.so   # must print a path

# DDS agent
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent ~/xrce
cd ~/xrce && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig

# workspace
cd ~/ros2_ws/src     # or your workspace
git clone <this repo> drone_testing
git clone https://github.com/Legendparth/imav_indoor_2026_sitl.git
git clone https://github.com/PX4/px4_msgs.git && (cd px4_msgs && git checkout 86d8239)
cd .. && colcon build --packages-select px4_msgs imav_indoor_2026 drone_testing
```

`px4_msgs` **must** match the PX4 build. A mismatch shows up as
`RTPS_READER_HISTORY ... payload size` or `Fast CDR exception deserializing`
and the node never sees position.

### B.2 Run part 1

```bash
# T1 — simulator. First line kills any PX4 left from an earlier run.
pkill -x px4; pkill -f "[M]icroXRCEAgent"; pkill -f "[g]z sim"; sleep 2
cd ~/ros2_ws && source install/setup.bash
ros2 launch drone_testing sitl.launch.py sitl_src:=$HOME/ros2_ws/src/imav_indoor_2026_sitl
# wait for:  ===== END HEALTH REPORT =====   (~45 s)

# T2 — the mission
cd ~/ros2_ws && source install/setup.bash
ros2 run drone_testing mission_fsm_part1 --ros-args \
  -p lateral_source:=flow -p window_marker_id:=2 -p outbound_distance:=22.5 \
  -p rangefinder_checks:=false -p takeoff_timeout:=60.0 \
  -p leg_timeout_margin:=200.0 -p flight_seconds:=600.0
```

Expect ~3 min: climb 2.1 m → `Flow healthy: latching x/y hold` → leg →
`marker id 2 ACQUIRED` → `CENTRED` → slow descent → `Disarmed. Flight complete.`
The down camera is at http://localhost:8080.

To fly again: Ctrl-C both, start T1 again.

### B.3 What the sim does differently, and why

**World.** `imav2026_scaled` is the arena × 2.2 — the only SITL world with
every sensor plugin PX4 needs (baro, mag, navsat, flow). Hence the run
parameters:

| sim parameter | real value | why it differs |
|---|---|---|
| `window_marker_id:=2` | 0 | the sim world's window marker is id 2 (its takeoff pad is id 0) |
| `outbound_distance:=22.5` | 9.3 | marker is 21.45 m away (9.75 m × 2.2) |
| `lateral_source:=flow` | vio | no RTAB-Map in the sim |
| `rangefinder_checks:=false` | true | noiseless sim range on the pad trips EKF2's "stuck" flag |
| `takeoff_timeout`, `leg_timeout_margin`, `flight_seconds` | defaults | the sim runs slower than real time; the node's clocks are wall time |

Altitude is **not** scaled: 2.1 m over a 2.2× world is like 0.95 m over the
real one, so the camera sees proportionally more floor than it will on the day.

**PX4 settings the launch applies** (at boot via `PX4_PARAM_*`, then again with
`px4-param`; all SITL-only, none reach the aircraft):

| | |
|---|---|
| `EKF2_GPS_CTRL 0`, `EKF2_EV_CTRL 0`, `EKF2_BARO_CTRL 0` | x/y from flow only, height from the rangefinder. EV off because the SITL model's Gazebo odometry is forwarded as vision in the wrong frame (EKF2 reset position/heading to it at arming). |
| `EKF2_OF_CTRL 1`, `EKF2_RNG_CTRL 1`, `EKF2_HGT_REF 1`, `EKF2_MIN_RNG 0.1` | flow on; see the warning in A.1 about `HGT_REF 2` |
| `EKF2_MULTI_IMU 0`, `SENS_IMU_MODE 1`, `EKF2_MULTI_MAG 0`, `SENS_MAG_MODE 1` | one EKF: the sim has 3 IMUs + 2 mags and the selector switched to a bad instance at arming |
| `COM_RC_IN_MODE 4`, `NAV_RCL_ACT 0`, `NAV_DLL_ACT 0`, `COM_RCL_EXCEPT 4`, `COM_ARM_WO_GPS 1` | no RC, no GCS, no GPS in the sim |
| `CBRK_SUPPLY_CHK 894281` | no power module in the sim |
| `MPC_LAND_SPEED 0.1` | as on the aircraft (A.1) |

The sim drone (`sim/sim_drone.urdf.xacro`) is the imav_indoor_2026 x500 plus
the down camera (78°, 800×600). Its flow camera is 42° (the PMW3901's) and the
launch clips its lidar to the TFmini Plus's 0.1–12 m with 2 cm noise.
`aruco_pose` reads the sim camera through `image_topic:=/down_cam/image`
(empty on the aircraft = the USB camera).

### B.4 If it doesn't fly

| symptom | cause |
|---|---|
| `local_position_invalid` flapping every ~2 s, heading/position jumps, arming denied | **two PX4s running.** `pgrep -a px4` must show one. The launch kills leftovers at start. |
| `Fast CDR exception` / `RTPS_READER_HISTORY` | `px4_msgs` ≠ the PX4 build (B.1) |
| `WARNING: libOpticalFlowSystem.so NOT BUILT` | PX4 < 1.16, or built without `libopencv-dev` / the vendor `CMAKE_PREFIX_PATH` (B.1) |
| `Leg ... TIMED OUT` | sim too slow for the leg clock: raise `leg_timeout_margin` |
| `Disarm refused ... AUTO.LAND` | `MPC_LAND_SPEED` above 0.11 |

T1 prints a **SITL HEALTH REPORT** ~40 s after start: every applied parameter,
`commander check`, `ekf2 status`, the estimator flags and five samples of the
pre-flight innovation checks. Read it first.
