# Bring-up: RealSense D435i window detection → traversal

Step-by-step from a cold Jetson to a window traversal, on the **D435i +
PMW3901 + TFmini Plus** airframe. Every command here was run on this Jetson
except the ones marked **NOT YET TESTED** (those need the flight controller
and the airframe, which were not connected when this was written).

**Work through the stages in order. Each one can fail safely on its own; the
next one cannot.**

| stage | what | props | risk |
|---|---|---|---|
| [A](#a-one-time-setup) | build, permissions | off | none |
| [B](#b-camera-alone-on-the-table) | camera alone, on the table | off | none |
| [C](#c-detection-tuning-on-the-table) | tune the detection on the table | off | none |
| [D](#d-geometry-on-the-table) | `/window_geometry` sanity | off | none |
| [E](#e-pixhawk-link-still-on-the-table) | Pixhawk link + estimator health | off | none |
| [F](#f-the-pose-estimate-walked-by-hand) | carry the airframe, watch `/window_pose` | **off** | none |
| [G](#g-first-flight-detection-only) | fly, detect, land. No traversal | on | flies |
| [H](#h-the-traversal) | the real thing | on | flies through a hole |

---

## A. One-time setup

### A.1 Environment

This Jetson runs **ROS 2 Jazzy** (the main README says Humble — substitute
`jazzy` throughout). Workspace is `~/imav26-ws-2/ws_ros2`.

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/imav26-ws-2/ws_ros2/install/setup.bash' >> ~/.bashrc
exec bash
```

### A.2 Packages

**Nothing to install.** Both dependencies are already on this Jetson, and
neither comes from apt on Jazzy:

| need | where it is | check |
|---|---|---|
| RealSense driver | built in the workspace, not `/opt/ros` | `ros2 pkg list \| grep realsense` |
| uXRCE-DDS agent | built from source at `/usr/local/bin/MicroXRCEAgent` | `which MicroXRCEAgent` |

> **Do not run `sudo apt install ros-jazzy-micro-ros-agent`.** There is no
> such package — `micro_ros_agent` was a Humble-era binary and was never
> published for Jazzy. `E: Unable to locate package` is apt telling you the
> truth, and because apt aborts the whole command, the `realsense2_camera`
> half does not install either (you do not need it anyway).
>
> The launch files therefore start the agent with `ExecuteProcess` on the
> standalone binary, not as a ROS node. If yours is somewhere else:
>
> ```bash
> ros2 launch drone_testing window_traverse.launch.py \
>   agent_cmd:=/path/to/MicroXRCEAgent agent_dev:=/dev/ttyTHS1 agent_baud:=921600
> ```

### A.3 Build

The package lives in `~/offboard_imav26_test` and is symlinked into the
workspace as `src/drone_testing_port`.

> **There is a second, older copy** of this package at
> `src/IMAV-26/offboard_imav26_test`. colcon refuses to build with two
> packages of the same name, so that one carries a `COLCON_IGNORE` file.
> Do not delete it without deleting the symlink too.

```bash
cd ~/imav26-ws-2/ws_ros2
colcon build --packages-select drone_testing
source install/setup.bash
```

Do **not** use `--symlink-install` (see main README §2).

### A.4 Serial port for the Pixhawk

```bash
sudo usermod -aG dialout $USER          # then log out and back in
systemctl status serial-getty@ttyTHS1
sudo systemctl disable --now serial-getty@ttyTHS1
```

### A.5 PX4 parameters — set these in QGC before anything flies

New for this airframe. The flow sensor and the rangefinder are now **two
separate devices**; on the ARK Flow they were one.

| parameter | value | why |
|---|---|---|
| `SENS_EN_PMW3901` | `1` | flow driver (SPI) |
| `EKF2_OF_CTRL` | `1` | fuse optical flow |
| `EKF2_OF_QMIN` | tune | PMW3901 reports a **coarser quality** number than the ARK Flow's PAW3902. If flow never latches, look here first |
| `SENS_EN_TFMINI` | `1` | TFmini Plus |
| `SENS_TFMINI_CFG` | *your port* | which serial port |
| `EKF2_RNG_CTRL` | `1` | fuse rangefinder — hard arming gate |
| `EKF2_HGT_REF` | `2` | height reference is the rangefinder |
| **`EKF2_EV_CTRL`** | **`0`** | **no external vision.** A leftover value from a VIO experiment leaves EKF2 waiting for vision that never comes |
| `EKF2_MAG_TYPE` | `0` | magnetometer ON. This flight has no vision to supply heading |
| `UXRCE_DDS_CFG` | your TELEM port | |
| `SER_TEL2_BAUD` | `921600` | must match the agent |

---

## B. Camera alone, on the table

Nothing but the D435i. No Pixhawk, no ROS package.

```bash
rs-enumerate-devices -s
```

Expect a `RealSense D435I` with a serial and firmware. If not, it is a USB
problem — **use a USB 3 port and the cable that came with it**; a USB 2 link
silently drops you to lower profiles or fails to open colour and depth
together.

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  rgb_camera.color_profile:=1280x720x30 \
  depth_module.depth_profile:=848x480x30
```

In a second terminal:

```bash
ros2 topic list | grep camera
```

**The three topics that matter:**

```
/camera/camera/color/image_raw                    rgb8, 1280x720
/camera/camera/color/camera_info                  intrinsics
/camera/camera/aligned_depth_to_color/image_raw   16UC1, MILLIMETRES
```

> **If `aligned_depth_to_color` is missing you forgot
> `align_depth.enable:=true`.** The plain `depth/image_rect_raw` is *not* a
> substitute — it is in the depth imager's own frame, ~15 mm from the colour
> lens and a wider FOV, so a window corner sampled out of it reads the wall
> behind the frame. Everything downstream will look like it works and be
> quietly wrong.

Verify encodings:

```bash
ros2 topic echo /camera/camera/color/image_raw --once --field encoding
ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw --once --field encoding
```

Expect `rgb8` and `16UC1`.

> **`ros2 topic hz` will show nothing on these topics.** It subscribes with
> reliable QoS and the camera publishes best-effort. This is not a fault.
> Use the detector's own log lines instead.

Kill it before the next stage (`Ctrl-C`) — librealsense will not share the
device, and a second copy fails in a way that looks like a dead camera.

---

## C. Detection tuning, on the table

This is the whole camera side with **no agent and no flight node**.

```bash
ros2 launch drone_testing window_scan.launch.py flight:=false
```

Watch the log. One line a second either way:

```
[window_detect]: CameraInfo from /camera/camera/color/camera_info: fx=906.8 fy=907.4 ... /window_geometry angles are now metric.
[window_detect]: First frame from /camera/camera/color/image_raw: 1280x720.
[window_detect]: window: no   (streak 12 misses, 143 frames processed, ...)
[window_detect]: WINDOW DETECTED  (centre=(671,342) offset=+0.05 area=18422px dist=2.31m)
```

**You must see the `CameraInfo` line.** Without it the node guesses the FOV
and every angle on `/window_geometry` is wrong by the ratio of the guess to
the truth. If it warns that it is guessing, fix `camera_info_topic` — do not
tune `fallback_hfov_deg` around it.

### Watch the picture

Browser, nothing needed on the viewing machine:

```
http://<jetson-ip>:8080/
```

Over ssh, tunnel it:

```bash
ssh -L 8080:localhost:8080 ark-jetson-orin-2@<jetson-ip>
# then open http://localhost:8080/
```

### Tune the threshold

Put the actual window frame (or a piece of the arena's card) on the table at
1.5–3 m and adjust until it locks on that and nothing else:

```bash
ros2 launch drone_testing window_scan.launch.py flight:=false \
  color:=green min_area:=1500 publish_mask:=true
```

| parameter | what to do with it |
|---|---|
| `color` | `green`, `blue`, `red` |
| `min_area` | raise it until room clutter stops registering. The default 1500 px² is small — on a 1280×720 frame that is a 39 px square |
| `publish_mask:=true` | publishes the HSV mask so you can see *what* it is thresholding |

> **Expect false positives indoors at the default `min_area`.** On this
> Jetson, plain office clutter tripped `green` repeatedly at ~1800 px². Tune
> against your actual arena lighting, not a desk.

---

## D. Geometry, on the table

`/window_geometry` is what the traversal actually consumes. Detection working
does **not** mean geometry is working.

```bash
ros2 topic echo /window_geometry
```

15 floats = 5 points × (depth_m, azimuth_deg, elevation_deg), in the order
top-left, top-right, bottom-right, bottom-left, centre.

**What good looks like** (measured on this Jetson, camera ~3 m from target):

```
2.824  -3.75  -2.06      <- corner 1: depth in METRES, angles in degrees
2.902  -1.85  -1.62
2.992  -0.61  -4.54
2.841  -3.81  -4.25
3.779  -2.51  -3.12      <- centre
```

Check:
- depths are **metres** and match a tape measure
- the four corner depths agree with each other to a few cm
- angles are small when the target is centred

If corners read `0.0` or wildly disagree, the depth map has holes where the
frame is — that is a stereo problem (more light, or turn the IR emitter on,
which the launch files already do).

---

## E. Pixhawk link, still on the table

Props **off**. Airframe on the bench.

Terminal 1 — the agent:

```bash
ros2 launch drone_testing window_scan.launch.py flight:=false camera:=false
# ^ this brings up only the detector; for the agent alone use:
ros2 launch drone_testing takeoff_test.launch.py
```

Terminal 2:

```bash
ros2 topic list | grep /fmu/
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

**Must be true, on the ground, props off:**

| field | expected |
|---|---|
| `z_valid` | `true` — no height estimate, no flight |
| `dist_bottom_valid` | `true` — the TFmini is being fused |
| `dist_bottom` | roughly your actual height above the floor |
| `xy_valid` | may be `false` on the ground; normal for flow |

Then the estimator flags:

```bash
ros2 topic echo /fmu/out/estimator_status_flags --once \
  | grep -E "cs_rng_hgt|cs_rng_kin_consistent|cs_baro_hgt|cs_ev_pos"
```

- `cs_rng_hgt: true` and `cs_rng_kin_consistent: true` — both required
- `cs_baro_hgt` must **not** be carrying height alone (baro drifts metres indoors)
- `cs_ev_pos` should be `false` — there is no vision in this flight

> **`cs_rng_kin_consistent` is sticky.** It can only go false in flight and
> can only recover in flight, so one bad run poisons the whole power cycle.
> **Reboot the flight controller at the start of every session** and after any
> flight where it tripped. The launch files take `reboot_fc:=true`.
> Full explanation in main README §10.1.

---

## F. The pose estimate, walked by hand

**Props off. This is the single most important test before flying, and it is
the one that catches a wrong camera mounting before it costs you an airframe.**

Measure the camera pose in the **body frame, ROS convention** — x forward,
y **left**, z **up**, metres and radians — **to the D435i's left imager**,
not the middle of the case. That is where librealsense puts the optical
origin.

Terminal 1:

```bash
ros2 launch drone_testing window_traverse.launch.py \
  cam_x:=0.10 cam_y:=0.0 cam_z:=0.05 cam_pitch:=0.0
```

Terminal 2:

```bash
ros2 topic echo /window_pose
```

Format: `x|y|z|yaw_deg|width|height|samples|age`, in **NED**.

Now **carry the airframe around** in front of the window.

| check | what it means |
|---|---|
| the centre `x,y,z` sits still in NED to within a few cm while you move | the mounting numbers are right. **This is the whole test** |
| the centre wanders as you move | wrong `cam_roll/cam_pitch/cam_yaw` or lever arm, or `/fmu/out/vehicle_attitude` is not in the PX4 DDS topic list |
| `width`/`height` match a tape measure | intrinsics and depth scale are right. Expect ~2% under — the detector pads corners 5 px inwards |
| `/window_pose` stays empty | no usable estimate. Read the rejection tally the node logs on exit |

The node prints a one-line tally of accepted vs rejected samples and **which
test did the rejecting**. That line is the first thing to read when an
approach does not converge:

- *"failed the planarity test"* → sample boxes landing on the wall behind the frame
- *"corner depth missing"* → depth map has holes where the frame is
- *"corner depths disagree"* → same, one corner reading the background

### Check your standoff against your window

New on the D435i: its colour sensor is **70° × 43°**, where the ZED was
~90° × 60°. Vertical binds. A window of height `H` needs **`1.26 × H`** of
distance just to fit in frame.

| your window height | minimum distance | `standoff_distance` 2.0 m gives |
|---|---|---|
| 1.0 m | 1.26 m | 1.59× margin ✅ |
| 1.2 m | 1.51 m | 1.32× margin ✅ |
| 1.5 m | 1.89 m | 1.06× — the node will push it out for you |
| 2.0 m | 2.52 m | pushed out to ~3.1 m |

`window_traverse` recomputes this at runtime from the aperture it has
measured and **logs when it moves the approach point**. If it warns it hit
`max_standoff_distance` (4.0 m), your window is too big for this lens.

---

## G. First flight — detection only, NO traversal

**NOT YET TESTED — no flight controller was connected when this was written.**

Fly the §6c scan: climb, hold, look at the window, land. **No traversal.**
This proves the flight stack and the detection work together in the air
before anything flies at a hole.

Terminal 1 — support stack:

```bash
ros2 launch drone_testing window_scan.launch.py reboot_fc:=true
```

Terminal 2 — the flight node, **by hand**:

```bash
ros2 run drone_testing window_scan --ros-args \
  -p takeoff_altitude:=0.8 \
  -p flight_seconds:=40.0 \
  -p scan_span_deg:=0.0
```

> **Run the flight node with `ros2 run`, in its own pane, always.** A node
> started by `ros2 launch` has no tty, and the `q` / `k` keyboard aborts are
> dead. Your RC kill switch is the real safety net either way.

| key | effect |
|---|---|
| `q` | abort into a controlled descent |
| `k` | force-disarm — **motors cut, the vehicle drops** |

Start at **0.8 m**, `scan_span_deg:=0.0` (no yaw sweep — point it at the
window before arming). Work up only after a clean flight.

Watch:

```bash
ros2 topic echo /takeoff_status
```

**Before you go on to H**, you want: it climbed, it held, the flow latched
(`POS` in the status line, not `FLO` or `---`), it saw the window, it landed.

---

## H. The traversal

**NOT YET TESTED.**

Terminal 1:

```bash
ros2 launch drone_testing window_traverse.launch.py \
  reboot_fc:=true \
  cam_x:=0.10 cam_y:=0.0 cam_z:=0.05 cam_pitch:=0.0
```

Terminal 2:

```bash
ros2 run drone_testing window_traverse --ros-args \
  -p takeoff_altitude:=1.2 \
  -p standoff_distance:=2.0 \
  -p approach_speed:=0.30 \
  -p traverse_speed:=0.45 \
  -p cam_x:=0.10 -p cam_z:=0.05 -p cam_pitch:=0.0
```

> **Pass the same `cam_*` numbers to both.** The launch file feeds one set to
> the detector and the flight node so they cannot disagree; if you run the
> flight node by hand you must repeat them.

### Stages, and what the LCD/status shows

```
climb → hold → SCAN → LOCK → AIM → ALIGN → TRAVERSE → CLEAR → land
```

| stage | detail field | means |
|---|---|---|
| `AIM` | `aim24` | 24° left to turn onto the window normal |
| `ALIGN` | `algn0.42` | 0.42 m to the approach point |
| `TRAVERSE` | `thru1.8/3.1` | 1.8 m flown of 3.1 m |
| `CLEAR` | `3s` | seconds of far-side hold left |

### Clear space you need

```
standoff_distance + exit_distance  =  2.0 + 1.5  =  3.5 m
```

...along the approach line, **plus** the `align_tolerance` basket either side,
**plus** room for the descent wherever it ends up. It lands on the far side.

### What it does when things go wrong

| situation | behaviour |
|---|---|
| vision lost **before** `TRAVERSE` | holds, then abandons into a landing |
| vision lost **during** `TRAVERSE` | **pushes on open-loop** along the committed heading for `blind_traverse_seconds` (3.0), then lands. Deliberate — stopping inside an aperture is worse |
| approach will not settle | `align_timeout` (60 s), then lands |
| anything else | `flight_seconds` (150 s) from the start of the climb forces a descent |

**An abandoned attempt lands. It does not retry.** Read the rejection tally,
fix the cause, fly it again.

---

## Quick reference

```bash
# bench, camera + detection only
ros2 launch drone_testing window_scan.launch.py flight:=false

# bench, with the mask, tuning the threshold
ros2 launch drone_testing window_scan.launch.py flight:=false publish_mask:=true

# bench, full traversal camera side (for /window_pose)
ros2 launch drone_testing window_traverse.launch.py flight:=false

# camera already running elsewhere? do not start a second one
ros2 launch drone_testing window_traverse.launch.py camera:=false

# the topics
ros2 topic echo /window_detected        # Bool, debounced
ros2 topic echo /window_info            # u|v|offset|area|d1..d4|dc
ros2 topic echo /window_geometry        # 15 floats, 5 x (depth,az,el)
ros2 topic echo /window_pose            # x|y|z|yaw|w|h|samples|age, NED
ros2 topic echo /takeoff_status         # STAGE|ARM|alt|xy-mode|detail

# the picture
http://<jetson-ip>:8080/

# estimator health
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /fmu/out/estimator_status_flags --once | grep -E "cs_rng|cs_ev|cs_yaw"
```

## Troubleshooting

| symptom | cause / fix |
|---|---|
| `aligned_depth_to_color` topic missing | driver started without `align_depth.enable:=true` |
| `ros2 topic hz` shows nothing on camera topics | reliable-vs-best-effort QoS. Not a fault. Use the node's logs |
| camera opens, then a second launch kills it | librealsense will not share the device. Use `camera:=false` on the second one |
| node warns it is guessing the FOV | `camera_info_topic` is wrong. Fix the topic, do not tune `fallback_hfov_deg` |
| `Not arming: need z_valid and dist_bottom_valid` | TFmini not being fused. Check `EKF2_RNG_CTRL`, `EKF2_HGT_REF`, and the wiring |
| `cs_rng_kin_consistent false` | sticky flag. **Reboot the flight controller.** Main README §10.1 |
| `dist_bottom` frozen at exactly `EKF2_MIN_RNG` | lidar unhealthy; EKF2 is synthesising that value. It is not a measurement |
| flow never latches (`FLO`/`---`, never `POS`) | PMW3901 reports coarser quality than the ARK Flow. Look at `EKF2_OF_QMIN` first |
| `local_position_invalid` ~1 s after arming | usually a leftover `EKF2_EV_CTRL`. It must be **0** — there is no vision in this flight |
| never leaves `LOCK` | no usable window pose. **Read the rejection tally the node logs** |
| `/window_pose` wanders as you move | wrong `cam_*` mounting numbers, or `vehicle_attitude` missing from the DDS topic list |
| `q` / `k` do nothing | node was started by `ros2 launch` — no tty. Use `ros2 run` in its own pane |
| `option --uninstall not recognized` on build | stale `--symlink-install` state. Main README §2 |
| `E: Unable to locate package ros-jazzy-micro-ros-agent` | expected — no such package on Jazzy. Nothing to install; see §A.2 |
| launch dies with `package 'micro_ros_agent' not found` | an old launch file still uses `Node(package='micro_ros_agent')`. `window_scan` and `window_traverse` are fixed; the others are not |
