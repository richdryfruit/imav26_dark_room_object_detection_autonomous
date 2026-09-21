# drone_testing

> **Mission split into two flights** (straight leg → marker; window + dark room → back to the marker), for the real drone and in SITL: see **[MISSION_PARTS.md](MISSION_PARTS.md)**.

ROS 2 package for autonomous offboard flight on a PX4 vehicle, running on a
Jetson companion computer.

> **Humble or Jazzy?** This was written against Humble. **The Jetson it now
> runs on is Jazzy** (`/opt/ros/jazzy`), and the package builds and runs there
> unchanged — section 1 and section 2 say `jazzy`; anywhere further down that
> still says `ros-humble-*`, substitute. Nothing in the package pins a
> distro.

Airframe this is written for: **Pixhawk 6C** (internal IMUs) + **PMW3901**
optical flow + **Benewake TFmini Plus** rangefinder, with an **Intel RealSense
D435i** as the forward-looking camera. There is **no GPS and no external
position source** in the takeoff test — height comes from the rangefinder and
lateral position from optical flow, and the code is written around the
limitations that implies.

> **This airframe was previously a ZED stereo camera + ARK Flow** (which had
> optical flow *and* an integrated rangefinder in one unit). If you are reading
> a log, a commit or a comment that talks about the ZED or the ARK Flow, see
> **[section 0](#0-the-zed--ark-flow--realsense-d435i--pmw3901-port)** for
> exactly what changed and what did not.

The headline node is `offboard_takeoff`: arm → sit on the ground → climb to a
set altitude → hold → descend → disarm, entirely on its own. `offboard_translate`
adds a horizontal leg to that — climb, then move a set distance forward,
backward, left or right, then land (see section 6). `offboard_sequence` goes one
further: climb, then a list of motions — translations, altitude changes and yaws
— flown one at a time, then land (see section 6b). `precision_land` lands the
vehicle on an ArUco marker seen by a downward camera, to within 15 cm (see
section 6d).

The competition mission is sections **6f** and **6g**: `window_traverse` finds
a window with the D435i, lines up square in front of it and flies through;
`window_room_traverse` carries on from there — around the dark room on the
moves you give it, back to the window from the inside, out through it, and
optionally running the TensorRT doll model the whole time and putting a live
count in QGroundControl.

---

## 0. The ZED + ARK Flow → RealSense D435i + PMW3901 port

The airframe changed two things at once. They are very different amounts of
work and it is worth keeping them apart.

### The flow sensor: ARK Flow → PMW3901 + TFmini Plus — **no code changed**

Not one line. Every horizontal health gate in this package
(`flow_is_healthy`, `rangefinder_is_healthy`, `estimate_is_healthy`) is
written against **EKF2's own status flags and `VehicleLocalPosition`**, never
against a sensor-specific topic — so which flow sensor is bolted underneath is
a PX4 parameter question and nothing more. Set on the flight controller:

| parameter | value | why |
|---|---|---|
| `SENS_EN_PMW3901` | `1` | the flow driver (SPI) |
| `EKF2_OF_CTRL` | `1` | fuse optical flow |
| `EKF2_OF_QMIN` | tune | the PMW3901 reports a **coarser quality number** than the ARK Flow's PAW3902 did. If the flow never latches, this is the first parameter to look at |
| `SENS_EN_TFMINI` | `1` | the TFmini Plus |
| `SENS_TFMINI_CFG` | *your port* | which serial port it is on |
| `EKF2_RNG_CTRL` | `1` | fuse the rangefinder — still a hard arming gate |
| `EKF2_HGT_REF` | `2` | height reference is the rangefinder |

**The one real behavioural difference: the rangefinder is now a separate
device.** The ARK Flow had it built in, so "the flow works" and "the
rangefinder works" were one question. They are now two, on two buses, and
`rangefinder_is_healthy()` is a hard arming gate that does not care which one
is at fault. Section 11's pre-flight check is not optional any more.

Everything in **section 10.1** about the sticky `cs_rng_kin_consistent` flag
still applies unchanged — that is EKF2 behaviour, not ARK Flow behaviour.

### The camera: ZED → RealSense D435i — topics, units, and field of view

| | ZED (gen-1, HD720) | RealSense D435i |
|---|---|---|
| colour topic | `/zed/zed_node/rgb/image_rect_color` | `/camera/camera/color/image_raw` |
| depth topic | `/zed/zed_node/depth/depth_registered` | `/camera/camera/aligned_depth_to_color/image_raw` |
| camera info | `/zed/zed_node/rgb/camera_info` | `/camera/camera/color/camera_info` |
| colour encoding | `bgra8` | `rgb8` |
| depth encoding | `32FC1`, **metres** | `16UC1`, **millimetres** |
| colour FOV | ~90° H × ~60° V | **70° H × 43° V** (measured 70.4 × 43.3 on this unit) |
| odometry | yes (stereo VO) | **none** — that was the T265 |

Three of those cost nothing. Both encodings were already handled by
`imgmsg_to_bgr()` / `imgmsg_to_depth()` in `window_detect.py`, and the
intrinsics come from `CameraInfo`, so resolution is not baked in anywhere.

**Two of them are load-bearing:**

1. **It must be the `aligned_depth_to_color` topic.** The D435i's depth imager
   is a different, wider lens sitting ~15 mm from the colour one, so
   `/camera/camera/depth/image_rect_raw` is *not* pixel-registered to the
   colour frame the window contour was found in. Sampling a window corner out
   of the unaligned map reads the wrong part of the scene — and at a window
   frame, "the wrong part of the scene" is the wall metres behind it, which is
   exactly the outlier `window_traverse`'s corner filters exist to catch.
   This is why every launch file passes `align_depth.enable:=true`; without
   it the topic does not exist at all.

2. **The field of view got much narrower, and vertically it is what binds.**
   To keep a window of height `H` fully in frame the camera must be at least
   `d = (H/2) / tan(VFOV/2)` away — `1.26 × H` on the D435i where it was
   `0.87 × H` on the ZED. The old `standoff_distance` of 1.6 m therefore went
   from comfortable to none at all:

   | window | ZED needs | D435i needs | margin at old 1.6 m | margin at new 2.0 m |
   |---|---|---|---|---|
   | 1.0 × 1.0 m | 0.87 m | 1.26 m | 1.27× | 1.59× |
   | 1.2 × 1.2 m | 1.04 m | 1.51 m | **1.06× — none** | 1.32× |
   | 1.5 × 1.5 m | 1.30 m | 1.89 m | **0.85× — will not fit** | 1.06× |

   So `standoff_distance` defaults to **2.0 m** now, and `window_traverse`
   gained `effective_standoff()`, which pushes the approach point further back
   at run time if the aperture it has actually measured needs it. See
   section 6f.

### What was NOT ported, and why

| node | status |
|---|---|
| `aruco_pose` / `precision_land` | **unaffected.** `aruco_pose` opens the down-facing USB camera directly with `cv2.VideoCapture` and never touched the ZED. Only the flow notes above apply |
| `zed_localization` / `offboard_sequence_vio` | **not ported.** The D435i has no odometry of its own, so this would mean standing up RTAB-Map or OpenVINS first. `window_traverse` does not use VIO — it localizes on flow + rangefinder, because the ZED's VO was resetting EKF2 6–7×/s (see section 6f) |
| `bar_detect` / `bar_cross` | **not ported.** Still on ZED topic defaults |
| `window_detect_darkroom` | **not ported.** Still on ZED topic defaults |

---

## 1. Prerequisites

### Workspace layout

On **this** Jetson the workspace is `~/imav26-ws-2/ws_ros2`, not the
`~/px4_ros_ws` the older commands below assume — substitute it throughout, or
just `cd` to wherever your `src/` actually is:

```
~/imav26-ws-2/ws_ros2/
└── src/
    ├── px4_msgs/                 # must match your PX4 firmware version
    ├── px4_ros_com/
    ├── realsense-ros/            # built from source here, not from apt
    ├── librealsense/
    └── drone_testing/            # this package
```

### System packages

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions
sudo apt install ros-jazzy-micro-ros-agent       # or build micro-XRCE-DDS-Agent from source
pip3 install pyserial pymavlink
```

`realsense2_camera` on this Jetson is **built from source** in the same
workspace (`src/realsense-ros` over `src/librealsense`), not installed from
apt — there is no `ros-jazzy-realsense2-camera` package in the index here.
Check what you have with `ros2 pkg prefix realsense2_camera`.

### The doll model's runtime (section 6g, rung 4 only)

**This is installed and working on this Jetson.** It lives in a venv, not in
the system or user site, and this section is the record of why and how — read
it before you "fix" anything by pip-installing into the system Python.

```bash
/home/ark-jetson-orin-2/venvs/dolls/bin/python -c \
    "import torch, ultralytics, tensorrt, cv2, numpy; print(torch.cuda.is_available())"
# -> True
```

| piece | version |
|---|---|
| torch | 2.9.1+cu130 (CUDA 13.0 build, aarch64) |
| torchvision | 0.24.1 |
| ultralytics | **8.4.118** — pinned to the version `db.engine` was exported with |
| lap | 0.5.13 — ByteTrack's linear assignment |
| tensorrt / cv2 / numpy | 10.16.2.10 / 5.0.0 / 2.5.3, **inherited from the system** |

#### Why a venv and not `pip install --user`

Ubuntu 24.04 marks its Python externally managed (PEP 668), so a system or
user install needs `--break-system-packages`. That is not a formality here:
ultralytics declares `numpy>=1.23.0` and `opencv-python>=4.7.0`, and this
machine's **numpy 2.5.3 and opencv-python 5.0.0.93 in `~/.local` are what the
entire vision stack rides on**. Letting a resolver move either one changes
`window_detect`, `doll_detect` and the RealSense path underneath you, and you
find out in the air.

That failure has already happened on this machine, twice over:

- it is why `window_detect` hand-rolls `imgmsg_to_bgr()` in pure NumPy instead
  of using `cv_bridge` — a pip-installed NumPy 2 made cv_bridge segfault on the
  first frame;
- and **the system Python on this Jetson cannot import matplotlib right now**
  (`ImportError: numpy.core.multiarray failed to import`), because apt's
  matplotlib 3.6.3 is built against NumPy 1.x and `~/.local` carries NumPy 2.
  That breakage predates this venv and is untouched by it. The venv carries its
  own matplotlib 3.11.2, which is why the doll node works despite it.

The venv is created with **`--system-site-packages`**, so it *inherits* numpy,
cv2 and tensorrt rather than duplicating them — there is exactly one of each on
the path and nothing shadows the working install. pip inside it refuses to
touch them: `Not uninstalling matplotlib at /usr/lib/python3/dist-packages,
outside environment`. And it is reversible: `rm -rf ~/venvs/dolls` returns the
machine to exactly where it was.

#### How it was built, if you ever need to rebuild it

```bash
python3 -m venv --system-site-packages ~/venvs/dolls
V=~/venvs/dolls/bin
printf 'numpy==2.5.3\n' > /tmp/c.txt          # numpy may not move, ever

$V/pip install --no-deps -c /tmp/c.txt ultralytics==8.4.118
$V/pip install --no-deps \
  https://download.pytorch.org/whl/cu130/torch-2.9.1%2Bcu130-cp312-cp312-manylinux_2_28_aarch64.whl
$V/pip install --no-deps \
  https://download.pytorch.org/whl/cu130/torchvision-0.24.1-cp312-cp312-manylinux_2_28_aarch64.whl
$V/pip install -c /tmp/c.txt \
  nvidia-cusparselt-cu13 nvidia-nccl-cu13 \
  filelock typing-extensions "sympy>=1.13.3" networkx jinja2 fsspec \
  polars nvidia-ml-py ultralytics-thop "lap>=0.5.12" "matplotlib>=3.9"
```

`--no-deps` on the first three is load-bearing: it is what stops pip from
pulling its own numpy and opencv over the working ones.

#### The one non-obvious thing: sm_87

This is a **Jetson Orin Nano (`sm_87`)** on JetPack 7 / CUDA 13.2, and the
official `cu130` aarch64 torch wheel is built for **sm_80, sm_90, sm_100,
sm_110, sm_120 — there is no sm_87 in it**, and the only PTX it ships is
`compute_120`, which cannot JIT down to 8.7:

```
-gencode;arch=compute_80,code=sm_80; ... ;-gencode;arch=compute_120,code=compute_120
```

It works anyway, because CUDA guarantees binary compatibility *upwards across
minor versions within a major*: an `sm_80` cubin runs on an `sm_87` device.
That was verified on this board before anything was installed, by compiling a
kernel for `sm_80` only and running it:

```bash
nvcc -gencode arch=compute_80,code=sm_80 -o t t.cu && ./t   # -> OK
```

and confirmed afterwards in torch itself:

```
compiled arch list: ['sm_80', 'sm_90', 'sm_100', 'sm_110', 'sm_120', 'compute_120']
device: Orin (8, 7)        cuda available: True        matmul on GPU OK
```

So **do not** go hunting for a "Jetson-specific" torch wheel because
`get_arch_list()` has no `sm_87` in it. jetson-ai-lab has no JP7 stage, and
NVIDIA's `redist/jp/v70` does not exist. This wheel is correct.

The CUDA libraries themselves come from **JetPack**, not from pip, wherever
possible: `ldconfig` already resolves `libcudart.so.13`, `libcublas.so.13`,
`libcudnn.so.9`, `libcufft`, `libcurand`, `libcusparse`. Only `libcusparseLt`
and `libnccl` were genuinely missing, which is why those two are installed
explicitly above.

### Serial port permissions

The Pixhawk is on `/dev/ttyTHS1` on the Jetson. You need to be in `dialout`:

```bash
sudo usermod -aG dialout $USER   # log out and back in (or reboot)
```

Nothing else may hold that port. Jetsons often bind a serial console to it:

```bash
systemctl status serial-getty@ttyTHS1
sudo systemctl disable --now serial-getty@ttyTHS1
```

### PX4 side

On the flight controller, the port wired to the Jetson must be running the
uXRCE-DDS client at a matching baud rate (921600 here):

- `UXRCE_DDS_CFG` → the TELEM port you are using
- `SER_TEL2_BAUD` (or whichever port) → 921600

---

## 2. Build

```bash
cd ~/imav26-ws-2/ws_ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drone_testing
source install/setup.bash
```

> **Do not use `--symlink-install` on this machine.** setuptools ≥ 80 removed
> `develop --uninstall`, which colcon calls when cleaning a previous symlink
> install, and the build fails with `option --uninstall not recognized`. If you
> already hit it:
>
> ```bash
> rm -rf ~/imav26-ws-2/ws_ros2/build/drone_testing \
>        ~/imav26-ws-2/ws_ros2/install/drone_testing
> colcon build --packages-select drone_testing
> ```

Add the sourcing to your shell so every new terminal has it:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/imav26-ws-2/ws_ros2/install/setup.bash' >> ~/.bashrc
```

---

## 3. Verify the link before you fly

Start the DDS agent on its own and confirm PX4 topics appear.

**Terminal 1 — agent only (this is the default):**

```bash
ros2 launch drone_testing takeoff_test.launch.py
```

**Terminal 2 — check:**

```bash
ros2 topic list | grep /fmu/
ros2 topic echo /fmu/out/vehicle_status_v1 --once   # unversioned on some builds
ros2 topic echo /fmu/out/vehicle_local_position --once
```

In that last message you want to see, **props off, on the ground**:

| field              | expected                                            |
|--------------------|-----------------------------------------------------|
| `z_valid`          | `true` — no height estimate, no flight               |
| `dist_bottom_valid`| `true` — the rangefinder is being fused              |
| `dist_bottom`      | roughly your actual height above the floor           |
| `xy_valid`         | may be `false` on the ground; that is normal for flow |

`offboard_takeoff` refuses to arm without `z_valid` **and** `dist_bottom_valid`.
If they are false, fix the sensor before going further — the log line
`Not arming: need z_valid and dist_bottom_valid` is telling you the truth.

---

## 4. Run the autonomous takeoff

### The recommended way (keyboard aborts stay live)

Run the agent from the launch file and the flight node **by hand in a second
pane**. Launching the node through `ros2 launch` means its stdin is not a tty,
which kills the `q` / `k` keyboard aborts.

**Pane 1:**

```bash
ros2 launch drone_testing takeoff_test.launch.py
```

**Pane 2:**

```bash
ros2 run drone_testing offboard_takeoff --ros-args \
  -p takeoff_altitude:=0.30 \
  -p hold_seconds:=5.0 \
  -p ground_wait_seconds:=5.0 \
  -p climb_speed:=0.35 \
  -p land_speed:=0.15 \
  -p request_offboard_from_ros:=true
```

**Start low.** 0.30 m for the first flight, then work up.

### Keyboard aborts (only when run as above)

| key | effect                                                             |
|-----|--------------------------------------------------------------------|
| `q` | abort into a controlled descent from wherever it is                 |
| `k` | force-disarm **immediately** — motors cut, the vehicle drops        |

Your **RC kill switch is the real safety net**. The keyboard is a convenience.
Flipping the TX out of Offboard also makes the node stand down and let go.

### Everything from one launch file

If you accept losing the keyboard aborts:

```bash
ros2 launch drone_testing takeoff_test.launch.py \
  agent_only:=false \
  takeoff_altitude:=0.30 \
  hold_seconds:=5.0
```

### Launch arguments

| argument                    | default  | meaning                                                        |
|-----------------------------|----------|----------------------------------------------------------------|
| `agent_only`                | `true`   | `true` = start only the DDS agent (+ LCD); run the node by hand |
| `takeoff_altitude`          | `0.80`   | metres above the arming point                                   |
| `hold_seconds`              | `15.0`   | station-keeping time once the altitude is reached               |
| `ground_wait_seconds`       | `5.0`    | armed on the ground before the climb starts                     |
| `climb_speed`               | `0.35`   | m/s the climb setpoint ramps at                                 |
| `land_speed`                | `0.15`   | m/s the descent setpoint ramps at                               |
| `request_offboard_from_ros` | `true`   | `false` = you flip the Offboard switch on the TX yourself        |
| `lcd`                       | `true`   | start the Arduino LCD status node                               |
| `lcd_port`                  | `''`     | Arduino serial port; empty = auto-detect `ttyACM*` / `ttyUSB*`   |

With `request_offboard_from_ros:=false` the node waits **indefinitely** for you
to flip the Offboard switch, so it is safe to start it long before you are
ready to fly.

---

## 5. What the flight actually does

```
PREPARATION      stream setpoints, wait for a healthy z + rangefinder estimate
OFFBOARD_REQUEST enter Offboard (from ROS, or wait for your TX switch)
ARMING           arm, and latch the height datum
GROUND_WAIT      sit armed, setpoint pressed 0.15 m *below* ground so it stays planted
TAKEOFF          ramp the z setpoint up to the target
HOLD             station-keep; latch x/y position hold once optical flow is healthy
LANDING          ramp back down, overshooting 0.5 m below ground
DISARMING        disarm once the land detector confirms touchdown
DONE             stop streaming setpoints and let go of the aircraft
```

Two things worth knowing about why it is written this way:

- **Horizontal is flown as a zero-velocity setpoint, not a position setpoint,**
  for takeoff and landing. On a flow-only airframe the x/y estimate near the
  ground is dead-reckoned garbage; a latched position would be flown out the
  moment flow started correcting it. Position hold is latched only once
  airborne with healthy flow, onto a *fresh* estimate.
- **Arrival at altitude requires three independent agreements** — the land
  detector says airborne, the EKF-relative altitude is in band, and the
  rangefinder roughly agrees. An EKF height reset alone can otherwise make the
  node "arrive" while still sitting on the ground.

### Reading the status output

`offboard_takeoff` publishes `/takeoff_status` as one pipe-separated line:

```
STAGE|ARM|altitude|xy-mode|detail
```

where `xy-mode` is `POS` (position hold latched), `FLO` (flow healthy, still on
velocity hold) or `---` (no usable flow).

```bash
ros2 topic echo /takeoff_status
```

---

## 6. The translate test (`offboard_translate`)

Same flight as above with a horizontal leg in the middle: arm → ground wait →
climb to **1.0 m** → hold → **move 1.0 m** in a body-frame direction → hold →
land. Everything about the estimator gating, aborts and landing is identical —
it is the takeoff node plus two stages.

### Running it

**Pane 1:**

```bash
ros2 launch drone_testing translate_test.launch.py
```

**Pane 2:**

```bash
ros2 run drone_testing offboard_translate --ros-args \
  -p takeoff_altitude:=1.0 \
  -p move_distance:=1.0 \
  -p move_direction:=forward \
  -p move_speed:=0.30 \
  -p hold_seconds:=5.0 \
  -p post_hold_seconds:=5.0
```

**Start small.** `takeoff_altitude:=0.5`, `move_distance:=0.5` for the first
flight. A flow-only lateral move needs far more clear floor than a hover does —
give it several metres in the direction of travel and be ready on `q`.

### Directions

`move_direction` is **body frame**, relative to the yaw the vehicle held when it
armed (yaw is pinned for the whole flight — it never rotates):

| value      | where it goes                |
|------------|------------------------------|
| `forward`  | out the nose (default)       |
| `backward` | out the tail                 |
| `left`     | out the left side            |
| `right`    | out the right side           |

### Parameters

| parameter            | default   | meaning                                              |
|----------------------|-----------|------------------------------------------------------|
| `move_distance`      | `1.0`     | metres to travel                                     |
| `move_direction`     | `forward` | `forward` / `backward` / `left` / `right`            |
| `move_speed`         | `0.30`    | m/s the horizontal setpoint is walked at             |
| `takeoff_altitude`   | `1.0`     | metres above the arming point                        |
| `hold_seconds`       | `5.0`     | hold **before** the move — the flow latch happens here |
| `post_hold_seconds`  | `5.0`     | hold **after** the move, before the descent          |

The rest (`ground_wait_seconds`, `climb_speed`, `land_speed`,
`request_offboard_from_ros`, `lcd`, `lcd_port`) are the same as the takeoff
test. `translate_test.launch.py` takes all of them as launch arguments too, with
`agent_only:=true` by default.

### Stages

```
... TAKEOFF, then:
HOLD        station-keep and wait for optical flow to latch x/y position hold
TRANSLATE   walk the held point to the target at move_speed
POST_HOLD   station-keep at the new point
LANDING     as before
```

### Why the move is a position setpoint, not a velocity one

A velocity setpoint is open loop *with respect to distance* — "1 m forward"
becomes "0.3 m/s for 3.3 s and hope", and flow bias, the accel/decel ramps and
any wind integrate straight into the distance actually flown. A position
setpoint closes that loop: the vehicle flies to a point and brakes itself
there, so bias shows up as a bounded offset instead of unbounded drift.

The cost is that a position setpoint is only as good as the x/y estimate it is
written in, so the move is **gated on the flow actually working**:

- the ground and the climb stay on zero-velocity hold, exactly as before;
- x/y position hold is latched only once airborne on a *fresh* estimate;
- only then does the move start, and it moves the **latched point**, walking it
  to the target at `move_speed` — a carrot. That is what sets the flight speed
  (rather than `MPC_XY_VEL_MAX`) and keeps the position error PX4 is correcting
  small the whole way. The carrot is leashed to 0.40 m ahead of the measured
  position so it cannot run away, or drag a snagged vehicle.

If the flow never latches within 15 s, or drops out mid-move, the node
**abandons the move and lands**. It will not dead-reckon the move on velocity:
a move you cannot measure is not a move worth flying.

On completion it logs the distance actually travelled against the distance
commanded — that number is your flow accuracy, and it is worth writing down
after each flight.

### Status output

Same `/takeoff_status` topic and format as the takeoff node, so the LCD works
unchanged. During the move the detail field reads e.g. `for0.62` — direction
plus metres still to go.

---

## 6b. The sequence test (`offboard_sequence`)

Same machinery again, but instead of one horizontal leg it flies **a list of
motions, one at a time**: arm → ground wait → climb → hold → step 1 → settle →
step 2 → settle → step 3 → settle → step 4 → hold → land.

A step is one of:

| step                                 | units   | what it does                          |
|--------------------------------------|---------|---------------------------------------|
| `forward` / `backward` / `left` / `right` | metres  | horizontal translation           |
| `up` / `down`                        | metres  | altitude change from where it is now  |
| `yaw`                                | degrees | rotate in place, `+` = clockwise seen from above |

### Running it

**Pane 1:**

```bash
ros2 launch drone_testing sequence_test.launch.py
```

**Pane 2:**

```bash
ros2 run drone_testing offboard_sequence --ros-args \
  -p takeoff_altitude:=1.0 \
  -p sequence:="forward 1.0, yaw 30, up 0.5, right 1.0"
```

The whole mission is that one `sequence` string: comma-separated items, each a
name and a number separated by a space, a colon or an `=`. Four steps is what
this test was written for; any number up to 12 is accepted. A malformed string
is **fatal at startup** — the node refuses to run rather than fly a mission
other than the one you typed.

**Start small.** `takeoff_altitude:=0.5` and half-metre steps for the first
flight, and check you have clear floor along the *whole* path, not just the
first leg. Be ready on `q`.

### Which frame the directions are in

`forward` means **the direction the vehicle was facing when it armed**, and it
keeps meaning that for the entire flight. A `yaw 30` step rotates the airframe
but does **not** rotate what `forward` means — so in the example above, `right
1.0` after the yaw flies the same ground track it would have flown without the
yaw, with the airframe crabbing 30°.

This is deliberate, and it is the same convention as a velocity setpoint in
SITL: every setpoint that leaves this node is in the NED local frame, not the
body frame, so a fixed reference yaw is the only reading that does not silently
depend on how well the yaw step tracked.

Pass `direction_frame:=current` if you want the other convention, where each
move is resolved against the yaw commanded at that point and the example flies
a 30° dog-leg.

### Parameters

| parameter           | default                                | meaning                                              |
|---------------------|----------------------------------------|------------------------------------------------------|
| `sequence`          | `forward 1.0, yaw 30, up 0.5, right 1.0` | the mission                                        |
| `direction_frame`   | `home`                                 | `home` / `current` — see above                       |
| `step_hold_seconds` | `3.0`                                  | settle time **between** steps                        |
| `yaw_rate`          | `0.35`                                 | rad/s (~20°/s) the yaw setpoint is walked at         |
| `min_altitude`      | `0.4`                                  | m a `down` step may not go below                     |
| `max_altitude`      | `3.0`                                  | m an `up` step may not exceed                        |

`takeoff_altitude`, `hold_seconds`, `post_hold_seconds`, `move_speed`,
`ground_wait_seconds`, `climb_speed`, `land_speed`,
`request_offboard_from_ros`, `lcd`, `lcd_port` are all the same as the translate
test, and `sequence_test.launch.py` takes every one of them as a launch
argument with `agent_only:=true` by default.

### How a step can end without ending the flight

Steps are individually recoverable — a bad one is reported and the sequence
carries on, because the next step may not depend on whatever failed:

| outcome        | when                                                          |
|----------------|---------------------------------------------------------------|
| `done`         | reached and settled inside tolerance                          |
| `SKIPPED`      | a horizontal step, but optical flow never latched x/y         |
| `ABANDONED`    | a horizontal step, flow lost part-way through it              |
| `TIMED OUT`    | did not get there in time; holds wherever it actually is      |

Yaw and altitude steps do **not** need the lateral estimate — they are measured
by the gyro/compass and the lidar — so they still run on a flight where the
flow never comes good and the horizontal steps are skipped. Anything more
serious than a failed step (lost height estimate, rangefinder fusion stopping,
Offboard taken away) lands or stands down exactly as in the other tests.

The per-step results are printed as one summary line at the end of the flight,
and again after disarm.

### Settle time between steps

`step_hold_seconds` exists so each step starts from a **stationary** vehicle.
Without it, step *n+1* samples its start point while the vehicle is still
overshooting step *n*, and the errors compound down the sequence instead of
each step correcting from where the previous one really finished.

### Status output

Same `/takeoff_status` topic and format as the other nodes, so the LCD works
unchanged. The detail field carries the step counter: `2/4 yaw18`, `1/4
for0.62`, `3/4 up1.50`.

---

## 6c. The window scan (`window_detect` + `window_scan`)

Takeoff, sweep the nose through a 90 degree arc until the D435i sees the
window, lock onto it, land 40 s after the climb started.

Two nodes:

| node            | what it does |
|-----------------|--------------|
| `window_detect` | subscribes to the colour and aligned-depth topics from `realsense2_camera`, runs the HSV / quadrilateral window detection, publishes `/window_detected` |
| `window_scan`   | the flight. Everything about arming, the climb, the health gates and the landing is inherited from `offboard_sequence`; only the middle of the flight is different |

### The camera side

`window_detect` reads two topics published by `realsense2_camera`:

```
/camera/camera/color/image_raw                  colour, rgb8, 1280x720
/camera/camera/aligned_depth_to_color/image_raw depth, 16UC1 in MILLIMETRES,
                                                registered to the colour frame
```

**The depth topic must be the `aligned_depth_to_color` one, not
`depth/image_rect_raw`.** The D435i's depth imager is a different lens in a
different place, so the raw depth map is not pixel-registered to the colour
frame the contour was found in — see [section 0](#0-the-zed--ark-flow--realsense-d435i--pmw3901-port).
It only exists if the driver was started with `align_depth.enable:=true`,
which the launch files do for you.

**Check these names on the Jetson first** — they change with `camera_name`
and `camera_namespace`:

```bash
ros2 topic list | grep camera
```

and if yours differ, pass `image_topic:=...` / `depth_topic:=...`. Depth is
optional (`use_depth:=false`): without it the window is still detected, only
the corner distances go missing — and with them `/window_geometry`, so the
traversal in section 6f will never leave `LOCK`.

It publishes:

| topic                     | type               | what |
|---------------------------|--------------------|------|
| `/window_detected`        | `std_msgs/Bool`    | debounced: true after 3 consecutive hits, false after 5 misses |
| `/window_info`            | `std_msgs/String`  | `u\|v\|offset\|area\|d1\|d2\|d3\|d4\|dc` — centre pixel, horizontal offset as a fraction of half the frame (-1 left, 0 centred, +1 right), contour area, the four corner depths and the centre depth |
| `/window_detection/image` | `sensor_msgs/Image`| the annotated frame, with a `WINDOW LOCKED` / `searching...` banner |

`window_detect` does **not** use `cv_bridge`. Its conversion lives in a
compiled extension built against the distro's NumPy, and a pip-installed
NumPy 2 in `~/.local` makes it segfault on the first frame (`process has
died ... exit code -11`). The node converts `sensor_msgs/Image` in pure
NumPy instead, so it runs whichever NumPy is on the path.

### Seeing whether the window is detected

In the terminal — the node logs one line a second either way, plus a WARN
the moment the detection latches or is lost:

```
[INFO] [window_detect]: window: no   (streak 12 misses, 143 frames seen)
[WARN] [window_detect]: WINDOW DETECTED  (centre=(671,342) offset=+0.05 area=18422px dist=2.31m)
[INFO] [window_detect]: window: YES  centre=(671,342) offset=+0.05 area=18422px dist=2.31m
```

or straight off the topics:

```bash
ros2 topic echo /window_detected
ros2 topic echo /window_info
ros2 topic hz /window_detection/image      # is the camera actually feeding us?
```

### Watching the feed live while it flies

Raw `bgr8` at 1280x720x15 fps is about 40 MB/s. WiFi will not carry that, so
neither route below ever puts a raw frame on the network — both are fed by
one JPEG encode, downscaled by `stream_scale` (0.5 = quarter the pixels) at
`jpeg_quality` (60). That works out around 10 KB a frame, ~150 KB/s.

**A browser — nothing needed on the viewing machine, not even ROS:**

```
http://<jetson-ip>:8080/
```

The node serves the annotated frame as MJPEG on that port (`/snapshot.jpg`
for a single still). If the Jetson is only reachable through ssh, tunnel it
and open `http://localhost:8080/` on your laptop:

```bash
ssh -L 8080:localhost:8080 ark-jetson-orin@<jetson-ip>
```

`stream_port:=0` turns the server off.

**rqt_image_view over the ROS network** (laptop on the same subnet, same
`ROS_DOMAIN_ID`): open `/window_detection/image` and switch the transport
dropdown to **compressed** — that selects
`/window_detection/image/compressed`, which is the JPEG topic. Do not view
the raw topic over WiFi.

```bash
ros2 run rqt_image_view rqt_image_view
```

On the Jetson itself with a monitor, the raw topic is fine:

```bash
ros2 run rqt_image_view rqt_image_view /window_detection/image
```

With a monitor on the Jetson you can also have the original OpenCV windows
back — the frame and the HSV mask, same as the standalone script:

```bash
ros2 run drone_testing window_detect --ros-args -p show_windows:=true
```

On the **LCD**: `lcd_status` subscribes to `/window_detected` itself and
row 4 becomes `flow ok  win YES` / `win no` / `win --` (the last one means
the detector is not publishing at all). The banner reads `SCANNING` during
the sweep and `WIN LOCK` once it has locked on.

### Running it

Bench test, no props — camera and detection only, no DDS agent and no
flight node. This is how you tune the HSV thresholds:

```bash
ros2 launch drone_testing window_scan.launch.py flight:=false
ros2 launch drone_testing window_scan.launch.py flight:=false publish_mask:=true
```

Flight. The default starts the agent, the camera and the detector but not the
flight node, so you run that by hand and keep the `q` / `k` aborts:

```bash
ros2 launch drone_testing window_scan.launch.py
ros2 run drone_testing window_scan --ros-args \
    -p takeoff_altitude:=1.0 -p flight_seconds:=40.0
```

Everything from the launch file (no keyboard abort — RC kill switch only):

```bash
ros2 launch drone_testing window_scan.launch.py agent_only:=false
```

Add `camera:=false` if `realsense2_camera` is already running from somewhere else,
or you will start a second copy of it and the SDK will refuse the camera.

### What the flight does

| stage | what happens |
|---|---|
| `PREPARATION` … `TAKEOFF` | identical to the other nodes: health gates, Offboard, arm, ground wait, ramped climb |
| `HOLD` | `hold_seconds` at altitude, waiting for the flow to latch x/y. If the window is already in sight when the hold ends, it skips straight to `LOCK` |
| `SCAN` | the nose sweeps +45 deg, then -90, then +90, … about the takeoff heading at `yaw_rate`, until the window is confirmed |
| `LOCK` | yaw frozen at the heading the airframe actually has, x/y hold re-latched here, and it sits there |
| `LANDING` | starts `flight_seconds` after the **start of the climb**, whether or not a window was ever found |

The 40 s clock is checked before every stage handler, so a stage that gets
stuck cannot postpone the landing. The descent itself takes as long as it
takes on top of that.

A detection only stops the sweep if it is **live and sustained**:
`/window_detected` is already debounced in the detector, and `window_scan`
additionally requires it to have been true for `detect_seconds` and to be
no more than a second old. A camera that dies goes quiet, and quiet reads
as "keep looking" — never as a lock.

### Parameters

| parameter | default | what |
|---|---|---|
| `takeoff_altitude` | 1.0 | m above the arming point |
| `flight_seconds` | 40.0 | s from the start of the climb to the descent |
| `scan_span_deg` | 90.0 | total sweep width, centred on the takeoff heading |
| `yaw_rate` | 0.35 | rad/s (~20 deg/s) the yaw setpoint is walked at |
| `detect_seconds` | 0.4 | how long the detection must hold before the sweep stops |
| `relock_on_loss` | false | true = resume sweeping if the window is lost after the lock |
| `hold_seconds` | 5.0 | station keeping at altitude before the sweep |
| `image_topic` / `depth_topic` | see above | the RealSense topics (`window_detect`) |
| `color` | green | which HSV range to look for: green, blue, red |
| `min_area` | 1500 | px^2 the contour must exceed |
| `show_windows` | false | cv2.imshow windows; needs a display |

Plus everything `offboard_sequence` takes for the climb and the descent
(`ground_wait_seconds`, `climb_speed`, `land_speed`, `min_altitude`,
`max_altitude`, `request_offboard_from_ros`).

### Status output

Same `/takeoff_status` topic and format. The detail field carries the sweep
and the clock: `scan37 22s` (37 deg of setpoint left in this leg, 22 s to
the landing), `lock-30 14s` (locked on a heading of -30 deg).

---

## 6d. Precision landing on an ArUco marker (`aruco_pose` + `precision_land`)

Takeoff, look straight down for an ArUco marker, fly over it until the vehicle
is centred to within 15 cm, hold there, and land on it.

Two nodes:

| node             | what it does |
|------------------|--------------|
| `aruco_pose`     | opens the USB down-facing camera directly (not the D435i, no `cv_bridge`), detects one known-size ArUco marker, solves its pose with `solvePnP` / `IPPE_SQUARE`, publishes `/aruco/detected` and `/aruco/point`. Knows nothing about PX4 |
| `precision_land` | the flight. Arming, the climb, the health gates, the descent and the touchdown detection are inherited unchanged from `offboard_sequence`; only the middle of the flight is new |

This flies on the same **ARK Flow + lidar + IMU** stack as everything else in
this README. The camera provides the lateral *target*; it is not a position
source, and PX4's estimator is untouched by it.

---

### Run it in this order. Do not skip a rung.

Each step is one strictly larger commitment than the last, and each one can
fail safely on its own.

| # | what | risk |
|---|---|---|
| 1 | `mode:=bench` — axis sign check on the ground | none: nothing is armed, nothing is published |
| 2 | `mode:=inspect` — fly and report the target | flies, but never commands a lateral move |
| 3 | `mode:=align land_after_align:=false` — close the loop at altitude | commands lateral moves, stays up, `q` available |
| 4 | `mode:=align` — the real thing | lands itself |

---

### 1. The bench sign check (`mode:=bench`) — DO THIS FIRST

**Propellers off. No flight controller needed.** This is the single most
important test in this section, and `bench` is the default mode precisely so
that a launch you forgot to configure does nothing at all.

```bash
ros2 launch drone_testing precision_land.launch.py flight:=false
ros2 run drone_testing precision_land --ros-args -p mode:=bench
```

In bench mode `precision_land` never arms, never requests Offboard, and never
publishes a single setpoint or vehicle command. It only reads the detector and
prints where the marker is, in vehicle terms:

```
BENCH: marker is FORWARD 0.20 m, RIGHT 0.30 m, 1.85 m below
       ->  the drone would move FORWARD 0.20 m, RIGHT 0.30 m
```

Put the marker on the floor, hold the airframe over it, and check both axes
against reality:

- move the marker to the drone's **RIGHT** → it must say **RIGHT**
- move the marker towards the **NOSE** → it must say **FORWARD**

**If either axis is inverted or the two are swapped, STOP.** Fix `image_rotate`
(or the physical mounting) and repeat until it reads true. A sign error here
does not produce a wobble that you can catch — the vehicle flies *away* from
the pad and keeps accelerating, because every new frame reports the marker as
further away in the same direction.

While this runs, the annotated camera view is on `http://<jetson-ip>:8080/`.

**Also measure your blind altitude while you are here.** Walk the airframe down
over the marker and note the height at which `/aruco/detected` goes false — the
0.80 m marker stops fitting in the frame somewhere around 0.66 m. Set
`blind_commit_altitude` to whatever you actually measure.

### 2. Inspect, in the air (`mode:=inspect`)

Flies the climb, the hold and the search, then prints the point it *would* fly
to — and holds position instead. Nothing lateral is ever published.

```bash
ros2 launch drone_testing precision_land.launch.py
ros2 run drone_testing precision_land --ros-args \
    -p mode:=inspect -p takeoff_altitude:=2.0
```

```
INSPECT: marker FORWARD 0.31 m, RIGHT 0.12 m | err 0.33 m |
         would move from (+1.42, -0.88) to (+1.61, -0.81) NED. Nothing published.
```

This is where you confirm the numbers are sane in flight — that the reported
error shrinks when you nudge the vehicle towards the marker by hand on the TX,
and that it does not jump around while the vehicle is holding still.

### 3. Align without landing (`land_after_align:=false`)

Closes the loop with the vehicle still at altitude, so you can watch it
converge with the abort key under your finger.

```bash
ros2 run drone_testing precision_land --ros-args \
    -p mode:=align -p land_after_align:=false -p takeoff_altitude:=2.0
```

It aligns, announces `ALIGNED to 9 cm`, and then holds indefinitely. Press `q`
to bring it down.

### 4. The real thing (`mode:=align`)

```bash
ros2 run drone_testing precision_land --ros-args \
    -p mode:=align -p takeoff_altitude:=2.0
```

Align → hold `aligned_hold_seconds` (10 s) → descend onto the marker.

`agent_only` defaults to **true**, the same as every other launch file here, so
the support stack comes up from launch and you run the flight node by hand in a
second pane. That is what keeps stdin a tty, and the `q` / `k` aborts only work
when it is. Everything in one shot, with no keyboard abort:

```bash
ros2 launch drone_testing precision_land.launch.py agent_only:=false mode:=align
```

Your RC kill switch is the real safety net either way.

---

### The camera side

`aruco_pose` opens the camera itself with `cv2.VideoCapture` — it does not go
through `realsense2_camera` — it is **not** the D435i — and like `window_detect` it avoids `cv_bridge`.

| topic | type | what |
|---|---|---|
| `/aruco/detected` | `std_msgs/Bool` | debounced: true after `detect_frames` consecutive hits, false after `lost_frames` misses |
| `/aruco/point` | `geometry_msgs/PointStamped` | marker centre in the **camera body frame**, metres, published only on a frame where the pose solved |
| `/aruco/info` | `std_msgs/String` | one human-readable line, the same one the node logs |
| `http://<jetson-ip>:8080/` | MJPEG | the annotated frame in a browser, no ROS needed on the viewing machine |

Check it standalone at any time:

```bash
ros2 topic echo /aruco/info
ros2 topic hz /aruco/point
```

**Why 800x600 and not something bigger.** The camera offers both 4:3 and 16:9,
and 4:3 is the right choice: the taller vertical field of view is what decides
how low the vehicle can go before the marker stops fitting in the frame, and
4:3 keeps it about 20 cm longer. 800x600 also runs at 30 fps where 1280x960
drops to 15, and for a control loop the frame rate is worth more than the
pixels — the marker is still ~198 px across at 2 m, which is plenty of corner
precision.

The camera is read on its own thread and only the newest frame is ever
processed. `cap.read()` blocks for a frame interval, and doing that inside a
ROS timer would stall the node for 33 ms at a time.

### The frame, and the one error that matters

`aruco_pose` publishes the marker centre in the **camera body frame**:

```
+x  RIGHT in the image
+y  UP in the image (towards the top)
+z  UP, i.e. opposite to where the camera looks   -> a marker below has NEGATIVE z
```

With the camera mounted **image-up towards the nose** and **image-right to the
vehicle's right**, that becomes body FRD:

```
forward = y        right = x        down = -z
```

`precision_land` then rotates that whole vector into NED using the vehicle's
**full attitude quaternion** from `VehicleAttitude` — not just its heading.
That is what makes the measurement tilt-compensated, and it is not optional:
the camera rolls and pitches with the airframe, so at 2 m a 10 degree pitch is
a 0.35 m phantom lateral offset. Worse, that error is *correlated with the
correction* — the vehicle pitches in order to move — so uncorrected it becomes
an oscillation rather than a bias. Rotating the full 3D vector by the attitude
removes it exactly.

If `VehicleAttitude` is not being published the node says so and refuses to use
the marker at all. It deliberately does **not** fall back to a heading-only
rotation, which would silently reintroduce the very error the quaternion is
there to remove.

### Why the correction is a position setpoint and not a velocity

Because the marker offset is re-measured every tick, a scale error in the
vision is a **loop gain, not a bias**: commanding a correction of `s·e` leaves
`(1-s)·e`, which converges for any `0 < s < 2` and never accumulates. So the
scale is not what picks between position and velocity here. Two other things
are:

- **Losing the marker.** A latched position setpoint means the vehicle *parks*.
  A velocity setpoint means it *coasts* until something explicitly zeroes it,
  which near the ground is exactly the wrong default.
- **Reuse.** `offboard_sequence` already owns a leashed, EKF2-reset-aware x/y
  carrot — `hold_x`/`hold_y` walked towards `move_target_x/y` by
  `_step_xy_ramp`, capped at `MOVE_LEASH` ahead of the measured position.
  Driving that is reusing a control path that has already flown, rather than
  inventing a new one.

`align_gain` (0.6) commands only a fraction of the measured offset each cycle.
That is *not* for the scale — it is phase margin against camera and link
latency, and it guarantees the approach is monotone even if the vision scale is
off by a third in the wrong direction.

### Height comes from the lidar, never from the marker

`fx` is derived from `hfov_deg`, not from a calibration, so it carries whatever
error the quoted field of view has. That error does **not** reach x and y:

```
apparent marker width in px   p = fx_true · S / Z_true          (measured)
solver, using fx = k·fx_true  Z = fx·S/p   = k·Z_true           (height wrong by k)
                              X = u·Z/fx   = u·Z_true/fx_true   (lateral EXACT)
```

The inflated range and the deflated bearing cancel. So the lateral offsets are
right even when the FOV is wrong, and only the *height* is scaled — which is
why this node takes x and y from the vision and leaves every altitude decision
on the rangefinder that `offboard_sequence` already gates arming on.

What does not cancel is **lens distortion**, which is currently uncorrected
(`distortion_coeffs` is zero). A wide lens bends the corners worst at the edge
of the frame, which is where the marker sits when the vehicle is most
off-centre. Running `cv2.calibrateCamera` on a chessboard and passing the real
`fx`/`fy`/`cx`/`cy` and `distortion_coeffs` removes it. Until then treat the
numbers as good near the centre and slightly optimistic at the edge.

### The capture basket, and the blind last metre

The marker has to be **fully** in frame for `solvePnP` to have four corners, so
the usable basket is the footprint minus the marker. With a 78 degree
horizontal FOV in 4:3 and an 0.80 m marker:

| altitude | footprint (across x fore-aft) | marker centre must be within | marker size |
|---|---|---|---|
| 2.0 m | 3.24 x 2.43 m | ±1.22 m left/right, ±0.81 m fore/aft | ~198 px |
| 1.5 m | 2.43 x 1.82 m | ±0.81 m left/right, ±0.51 m fore/aft | ~264 px |
| 1.0 m | 1.62 x 1.21 m | ±0.41 m left/right, ±0.21 m fore/aft | ~395 px |
| 0.66 m | — | marker no longer fits vertically | — |

Two consequences, and they drive the whole design:

1. **Align high, not on the way down.** The basket shrinks as you descend, so
   the alignment runs at `takeoff_altitude` — 2.0 m is a good choice — and must
   be *finished* by roughly 1 m.
2. **The last stretch is open loop.** There is no smaller nested marker to hand
   over to, so below `blind_commit_altitude` the descent is flown on the last
   held point. The node logs the moment it crosses that line so the ulog says
   exactly where the closed loop ended.

Because of that, the x/y hold is **kept through the descent**, unlike
`offboard_sequence`'s `_begin_landing`, which drops to a zero-velocity hold.
Ten seconds of unheld descent from 2 m would drift further than the 15 cm the
alignment just worked to achieve. It still falls back to zero-velocity hold the
instant `flow_is_healthy()` goes false, which is the inherited gate — and every
*other* path into the landing (operator abort, a dead height estimate, a lost
rangefinder, an overshoot, the flight clock) keeps the inherited behaviour
untouched. Those are emergencies, and in an emergency "do not translate" is the
correct horizontal command.

### What the flight does

| stage | what happens |
|---|---|
| `PREPARATION` … `TAKEOFF` | identical to the other nodes: health gates, Offboard, arm, ground wait, ramped climb |
| `HOLD` | `hold_seconds` at altitude, waiting for the flow to latch x/y. If the marker is already in sight when the hold ends, it skips straight to `ALIGN` |
| `SEARCH` | holds station and watches for up to `search_seconds` |
| `ALIGN` | re-measures every tick and walks the x/y hold point towards the marker at `align_gain` × the measured offset, until the error is inside `align_tolerance` for `align_settle_seconds` |
| `ALIGNED_HOLD` | stops correcting, holds `aligned_hold_seconds`. Drifting back outside 1.5 × the tolerance sends it to `ALIGN` again |
| `LANDING` | descends on the aligned point, keeping the x/y hold until the flow gives out |

**The search does not sweep the yaw.** Unlike `window_scan`, which has a
forward-facing camera where a yaw sweeps new ground, this camera looks straight
down: yawing rotates the footprint but barely changes which patch of floor is
inside it. It would cost tracking quality and buy almost no coverage, so the
search is stationary and the basket is simply the footprint above.

A marker only counts if it is **live**: `/aruco/detected` is already debounced
in the detector, and `precision_land` additionally requires the pose to be no
older than `marker_max_age`. A camera that dies goes quiet, and quiet reads as
"no marker", never as a lock.

Losing the marker mid-align **freezes the carrot** — the vehicle holds where it
is rather than walking on towards a target derived from a measurement it no
longer has. If it stays lost for `marker_lost_seconds` the flight goes back to
`SEARCH`.

`flight_seconds` is checked before every stage handler and is measured from the
**start of the climb**, so a stage that gets stuck cannot postpone the descent.

### Parameters

| parameter | default | what |
|---|---|---|
| `mode` | `bench` | `bench` \| `inspect` \| `align` — see the ladder above |
| `takeoff_altitude` | 2.0 | m above the arming point. Sets the capture basket |
| `align_tolerance` | 0.15 | m radius that counts as centred over the marker |
| `align_settle_seconds` | 1.5 | s inside that radius before the alignment is believed |
| `align_gain` | 0.6 | fraction of the measured offset commanded per cycle. Below 1 guarantees monotone convergence. Raise slowly if at all |
| `aligned_hold_seconds` | 10.0 | s held over the marker before the descent |
| `land_after_align` | true | false = align and stay up (rung 3) |
| `search_seconds` | 20.0 | s hovering and looking before giving up |
| `align_timeout` | 45.0 | s trying to centre before giving up |
| `flight_seconds` | 120.0 | s from the start of the climb to a forced descent, whatever else is happening |
| `on_fail` | `land` | what a search / align timeout does: `land` \| `hold` |
| `marker_max_age` | 0.5 | s after which the last pose is not evidence of anything |
| `marker_lost_seconds` | 5.0 | s without a marker during `ALIGN` before returning to `SEARCH` |
| `precision_descent` | true | keep the aligned x/y hold through the descent |
| `blind_commit_altitude` | 1.0 | m below which the marker is expected to be out of frame. Logged, not enforced — **measure it** |
| `marker_id` | 0 | ArUco id to track (`aruco_pose`) |
| `marker_size` | 0.80 | marker edge length, m. Must be right: it sets the metric scale of the whole pose |
| `aruco_dict` | `DICT_5X5_50` | `cv2.aruco` predefined dictionary name |
| `hfov_deg` | 78.0 | horizontal FOV. Scales the reported height, which nothing uses |
| `image_rotate` | 0 | `0\|90\|180\|270`, applied before detection. Use this if the camera is bolted on rotated |
| `width` / `height` | 800 / 600 | 4:3 on purpose — see above |
| `fourcc` | `MJPG` | MJPG gets 30 fps at 800x600 on this camera; YUYV does not |
| `stream_port` | 8080 | browser MJPEG view. 0 disables it |
| `show_gui` | false | `cv2.imshow`; needs a display, leave false on a headless Jetson |

Plus everything `offboard_sequence` takes for the climb and the descent
(`hold_seconds`, `ground_wait_seconds`, `climb_speed`, `land_speed`,
`move_speed`, `min_altitude`, `max_altitude`, `request_offboard_from_ros`).

The `sequence` parameter the parent declares is parsed and then ignored — this
node flies its own plan and never enters the `STEP` stage.

### Status output

Same `/takeoff_status` topic and format, so the LCD needs no changes. The
detail field carries the stage and the flight clock: `srch 94s` (searching,
94 s until the forced descent), `err0.33 88s` (aligning, 33 cm out),
`algn 7s` (aligned, 7 s of hold left).

---

## 6e. The sequence test on ZED vision (`offboard_sequence_vio`)

> **NOT PORTED to the RealSense.** This whole section still describes the ZED.
> A D435i has no odometry of its own (that was the T265), so running this on
> the current airframe means standing up RTAB-Map or OpenVINS first and
> pointing the bridge at *its* odometry topic. Nothing else in this README
> depends on this section — `window_traverse` (6f) runs on optical flow. Left
> here as-is rather than half-ported, because a vision section that has been
> edited but never flown is worse than one that is honestly stale.

The **same mission as 6b**, flown with lateral position coming from ZED visual
odometry instead of the ARK Flow's optical flow. Height still comes from the
lidar. `offboard_sequence_vio` subclasses `OffboardSequence` and changes
exactly one thing — what counts as a healthy horizontal estimate — so the
ramps, leashes, aborts and status output are all identical to 6b.

The one behavioural difference in the air: **x/y hold latches on the ground.**
Flow has no usable estimate until the vehicle is airborne, so 6b flies the
ground wait and the climb as "zero velocity and hope". Vision is valid sitting
still on the floor, so the climb is a real position hold against the take-off
point and the vehicle goes straight up instead of sliding off. Set
`hold_xy_from_ground:=false` for the old behaviour.

### Set the PX4 parameters first

Read the header block of `launch/sequence_vio_test.launch.py` — it is the
authority and it explains the reasoning. The minimum, in QGC:

| parameter        | value | why                                              |
|------------------|-------|--------------------------------------------------|
| `EKF2_EV_CTRL`   | `1`   | horizontal position only — **not** bit 1 (2), that is vertical position and hands height back to the camera |
| `EKF2_HGT_REF`   | `2`   | height reference stays the rangefinder            |
| `EKF2_RNG_CTRL`  | `1`   | lidar fusion on — still a hard arming gate        |
| `EKF2_OF_CTRL`   | `0`   | flow off; do not fuse flow and vision at once     |
| `EKF2_EV_DELAY`  | `40`  | ms, starting point. The number that matters most  |
| `EKF2_EVP_NOISE` | `0.1` | m                                                 |
| `EKF2_EV_NOISE_MD` | `0` | use the covariance from the message               |

The node will refuse to arm if EKF2 is not actually fusing what it expects, so
a mistake here shows up as a refusal to arm, not as a crash.

### Measure the camera mounting

`cam_x/y/z/roll/pitch/yaw` all default to `0.0`, which means "camera at the CoG
pointing dead ahead". That is a no-op and almost certainly not where yours is.
Getting the rotation wrong **tilts every commanded translation**; getting the
lever arm wrong turns every yaw into a phantom sideways step.

Measure the pose of the camera **in the body frame, ROS convention** — x
forward, y **left**, z **up**, metres and radians. Leave `EKF2_EV_POS_X/_Y/_Z`
at zero: the bridge applies the full rigid transform before publishing, which
EKF2 cannot do because it has no parameter for the camera's *rotation*.

### Running it

`agent_only` defaults to `true`, so the launch file brings up the support stack
only — agent, ZED wrapper, bridge, LCD — and you run the flight node yourself
in a second pane, which is what keeps stdin a tty and the `q`/`k` aborts alive.

**Pane 1 — support stack:**

```bash
cd ~/px4_ros_ws
source install/setup.bash
ros2 launch drone_testing sequence_vio_test.launch.py \
  cam_x:=0.10 cam_y:=0.0 cam_z:=0.05 cam_pitch:=0.0
```

Wait for the bridge to print

```
VIO healthy: 15 Hz from /zed/zed_node/odom
```

**Do not go on until you have seen that line.**

**Pane 3 — sanity check:**

```bash
source ~/px4_ros_ws/install/setup.bash
ros2 topic echo /vio_healthy --once             # must be data: true
ros2 topic hz /fmu/in/vehicle_visual_odometry   # should sit near 15 Hz
```

**Pane 2 — the flight node:**

```bash
cd ~/px4_ros_ws
source install/setup.bash
ros2 run drone_testing offboard_sequence_vio --ros-args \
  -p takeoff_altitude:=0.5 \
  -p sequence:="forward 0.5"
```

**Start much smaller than the launch defaults.** The defaults are 1.0 m and
`forward 1.0, yaw 30, up 0.5, right 1.0`; for a first vision flight use 0.5 m
and a single 0.5 m step, get one clean log, then add steps back one at a time.
`q` aborts into a controlled descent, `k` force-disarms, and the RC kill switch
is still the real safety net.

### Parameters

Every parameter from 6b applies unchanged. These are the additions:

| parameter                | default                 | meaning                                                        |
|--------------------------|-------------------------|----------------------------------------------------------------|
| `hold_xy_from_ground`    | `true`                  | latch x/y hold before the climb instead of after                |
| `vio_settle_seconds`     | `2.0`                   | s the vision estimate must be continuously healthy before latching |
| `allow_missing_bridge_status` | `true`             | fall back to EKF2's flags alone if `/vio_healthy` is absent      |
| `zed`                    | `true`                  | start the ZED wrapper here; `false` if you run it elsewhere      |
| `camera_model`           | `zed`                   | gen-1 ZED — **no IMU**, so this is visual odometry only          |
| `camera_name`            | `zed`                   | sets the topic prefix and the odom child frame                   |
| `odom_topic`             | `/zed/zed_node/odom`    | ZED odometry the bridge converts                                 |
| `pose_frame`             | `frd`                   | vision heading has an unknown offset from North; EKF2 estimates it |
| `publish_rate`           | `15.0`                  | Hz sent to PX4. Do not raise on a UART link — see below          |
| `publish_velocity`       | `false`                 | only `true` if `EKF2_EV_CTRL` bit 2 (4) is also set              |
| `move_speed`             | `0.30`                  | m/s. Raise one flight at a time; fast translation breaks stereo VO |
| `yaw_rate`               | `0.35`                  | rad/s. Keep slow — fast yaw is the surest way to lose tracking   |
| `cam_x/y/z`              | `0.0`                   | m, camera position in body frame (x fwd, y **left**, z **up**)   |
| `cam_roll/pitch/yaw`     | `0.0`                   | rad; `cam_pitch` **positive = nose down**                        |

`publish_rate` is a throttle on what goes *down the link*, not on the ZED. The
uXRCE-DDS UART cannot carry 30 Hz of odometry alongside the setpoint streams —
it starves the offboard heartbeat and PX4 takes the aircraft. Only raise it on
Ethernet.

### When vision drops out

Losing vision mid-flight is survivable, not fatal. If EKF2 stops fusing the
external vision, the inherited logic falls back to zero-velocity hold and
horizontal steps are **skipped** rather than dead-reckoned. Height is still the
lidar's, so the descent is unaffected. Expect dropouts on motion blur, on
featureless walls and in low light — an IMU-less stereo camera has nothing to
coast on.

### Flow or vision?

You cannot pick from first principles; it depends on your arena's floor and
walls. Fly the identical sequence three times and compare the logs:

1. **Flow only** — `EKF2_OF_CTRL=1`, `EKF2_EV_CTRL=0`, via `sequence_test.launch.py` (6b).
2. **Vision only** — `EKF2_OF_CTRL=0`, `EKF2_EV_CTRL=1`, via this launch file.
3. **Both** — only if step 2 showed a dropout you actually need covered.

Running both as the *normal* configuration is discouraged: they are two
independent, differently-scaled, differently-delayed measurements of the same
lateral state, and where they disagree the filter splits the difference and the
vehicle drifts toward whichever one is lying.

---

## 6f. Flying through the window (`window_traverse`)

Section 6c ends with the vehicle stopped, facing the window, doing nothing
about it. This is the rest: **estimate where the window actually is, line up
square in front of it, and fly through.** Localisation is **PMW3901 optical
flow + the TFmini Plus**, fused in PX4 — *not* visual odometry.

> **This corrects an earlier version of this section**, which said the
> traversal ran on ZED visual odometry (section 6e). It does not, and the code
> has not for some time: `window_traverse` subclasses `WindowScan` →
> `OffboardSequence`, deliberately **not** `OffboardSequenceVio`, because the
> ZED's VO was resetting EKF2's horizontal estimate 6–7 times a second on this
> airframe (`tools/ekf_reset_rate.py` measures it). The camera is for *seeing
> the window* and nothing else. The D435i swap does not revisit that: a D435i
> has no odometry of its own at all — that was the T265 — so putting the
> position back on the camera would mean standing up RTAB-Map or OpenVINS
> first. **There is no `EKF2_EV_*` in this flight; clear `EKF2_EV_CTRL` to 0.**

```bash
# bench, no props, camera only
ros2 launch drone_testing window_traverse.launch.py flight:=false

# flight: support stack from launch, flight node by hand so q/k stay alive
ros2 launch drone_testing window_traverse.launch.py
ros2 run drone_testing window_traverse --ros-args \
    -p takeoff_altitude:=1.2 -p cam_x:=0.10 -p cam_pitch:=0.0
```

Three nodes, and one of them is new:

| node              | what it does |
|-------------------|--------------|
| `realsense2_camera` | the D435i driver, started by the launch file with `enable_color:=true align_depth.enable:=true` |
| `window_detect`   | the 6c detection, **plus** `/window_geometry`: the four corners and the centre as (depth, azimuth, elevation) in the camera frame |
| `window_traverse` | the flight. Subclasses `WindowScan` (the sweep and the lock) over `OffboardSequence` (arming, the climb, the ramps, the landing, and `flow_is_healthy`), so the only new flight code is the stages after the lock |

### The stages

```
 ... climb -> hold -> SCAN -> LOCK -> AIM -> ALIGN -> TRAVERSE -> CLEAR -> land
```

| stage | what happens | how it ends |
|---|---|---|
| `SCAN` | the 6c yaw sweep | `/window_detected` holds true for `detect_seconds` |
| `LOCK` | stop, freeze the yaw, hold position facing the window — and **build the pose estimate**, which is the best geometry of the whole flight: stationary, square on, whole window in frame | `pose_min_samples` accepted samples exist |
| `AIM` | yaw onto the window normal **standing still** | heading within 12 deg of the normal |
| `ALIGN` | fly to a point `standoff_distance` in front of the window, on its axis, at its height, re-derived from the live estimate every tick | position, altitude **and** heading all in tolerance together for `align_settle_seconds` |
| `TRAVERSE` | **commit** — freeze the target, ignore the camera, fly through to `exit_distance` beyond | the distance along the committed line is flown |
| `CLEAR` | hold on the far side | `clear_seconds` |

`AIM` turns before it translates on purpose. An IMU-less stereo camera loses
tracking on a fast yaw and loses it much more readily when the scene is also
translating; after `AIM` the remaining yaw corrections are a few degrees and
ride along with the approach unnoticed.

### Which frame the setpoints are in

Every setpoint is an **absolute point in the PX4 local NED frame** — the same
frame `/fmu/out/vehicle_local_position` reports `x`, `y`, `z` in, z positive
down. Not body-relative. Section 6b hides that behind direction words
("forward 1.0"), but underneath it walks an NED hold point towards an NED
target and publishes that point; this node computes the NED targets directly
because a window's position is naturally an absolute point.

Altitude is the one relative number, and only in the bookkeeping:
`commanded_altitude` is metres above the **arming point**, turned into NED as
`home_z - commanded_altitude` before it reaches PX4.

### Where the window's position comes from

`window_detect` publishes `/window_geometry` every frame it sees a
quadrilateral: five rows of `(depth_m, azimuth_deg, elevation_deg)` — four
corners in the order top-left, top-right, bottom-right, bottom-left, then the
centre. Angles rather than pixels, because the angles are the part that needs
the intrinsics, and the intrinsics arrive on `camera_info_topic` where the
detector already is. Change the camera resolution and nothing downstream cares.

`window_traverse` turns each frame into a point in NED in three steps:

1. **rays to camera-frame points** — `x = d`, `y = d·tan(az)`, `z = −d·tan(el)`.
   Exact, not approximate, because the D435i's depth is the distance along the
   optical axis rather than the slant range.
2. **camera → body FRD** — the mounting rotation and lever arm, from
   `cam_x/cam_y/cam_z` and `cam_roll/cam_pitch/cam_yaw`. **The same six
   numbers, in the same ROS convention (x fwd, y LEFT, z UP), that
   any vision bridge on this airframe takes.** Measure them to the D435i's
   **left imager**, which is where librealsense puts the optical frame origin
   — not to the middle of the case. The launch file feeds one set to both nodes so
   they cannot disagree; if you run either by hand, pass the same numbers.
3. **body FRD → NED** — rotate by the `VehicleAttitude` quaternion, add the
   vehicle position.

Step 3 uses the **full attitude, not just the heading**. A vehicle translating
at 0.4 m/s sits at 5–10 degrees of pitch, and at 3 m range a 10 degree pitch
error puts the window half a metre off vertically — the window would appear to
bob up and down every time the vehicle accelerated.

### The depth outliers, which are the actual problem

Stereo depth on a thin frame fails in one specific way: a sample box a few
pixels off the frame reads the **wall behind** (metres too far) or returns
nothing at all, and one such corner drags a naive four-corner average metres
out of position. So no single frame is ever trusted. Five filters stand
between a depth pixel and a setpoint:

| filter | what it rejects |
|---|---|
| per corner | depth ≤ 0, or outside `[depth_min, depth_max]` |
| per sample | corner depths disagreeing with their own median by more than `max(corner_spread, corner_spread_frac × range)`; corners more than `plane_tolerance` off their own best-fit plane; a reconstructed aperture outside `[window_min_size, window_max_size]`; opposite sides disagreeing by more than 40%; a normal more than `max_tilt_deg` off horizontal (a window is vertical — a horizontal normal is the floor) |
| innovation | once an estimate exists, a sample whose centre is more than `gate_metres` from it or whose normal is more than `gate_yaw_deg` off it |
| temporal | the estimate is the component-wise **median** over the last `buffer_seconds`, not a mean and not an EMA |
| quorum | nothing is flown to until `pose_min_samples` accepted samples exist and the newest is younger than `pose_max_age` |

The median is the point. An EMA with α = 0.25 — what
`drone_imav_obs_course.py` uses — still moves 25 cm towards a sample that is a
metre wrong, on the first frame. A median moves not at all until half the
buffer agrees.

Note the factor of four on `plane_tolerance`: a best-fit plane through four
points splits one bad corner's error across all of them, so a corner X out of
plane only shows a residual of X/4. The depth-spread test is the primary
defence; planarity is the backstop for a corner displaced *across* the frame
rather than along the ray. Tuned together, a corner 0.5 m out at 3 m range is
rejected, and a genuine window seen at 35 degrees of obliquity is not.

If the innovation gate rejects `gate_reset_count` samples in a row, the buffer
is discarded and rebuilt: at that point the *estimate* is the minority opinion,
and refusing every sample forever is worse than starting again.

### Why it commits

The estimate keeps updating through `SCAN`, `LOCK`, `AIM` and `ALIGN` — the
approach target is recomputed every tick, and the inherited carrot does the
smoothing, so a 10 cm shift in the estimate is a slightly different direction
of travel rather than a 10 cm step in what PX4 is asked for.

At the start of `TRAVERSE` the target freezes and the camera stops steering.
Passing through a window means the window leaves the field of view, fills it,
and ends up behind the camera; depth on a frame edge at half a metre is the
least trustworthy data the camera produces, and it arrives when the aircraft
is least able to act on it. The estimate that lined the aircraft up from
1.6 m away, square on, with the whole window in frame, is better than anything
measurable from inside the aperture.

**If vision dies mid-traverse the aircraft does not stop in the window.** It
pushes on open-loop along the committed heading at `traverse_speed` for up to
`blind_traverse_seconds`, then lands. Open loop is exactly what the rest of
this codebase refuses to do, and rightly; the alternative here is stopping
inside an aperture with no way to tell which side of it you are on.

### Watching the estimate

```bash
ros2 topic echo /window_geometry   # 15 floats: 5 points x (depth, az, el)
ros2 topic echo /window_pose       # x|y|z|yaw_deg|width|height|samples|age, NED
```

`/window_pose` is empty while there is no usable estimate. When the run
finishes, the node logs a one-line tally of how many samples were accepted out
of how many arrived and **which test did the rejecting** — that line is the
first thing to read when an approach did not converge. "37 of the last 40
failed the planarity test" tells you the sample boxes are landing on the wall
behind the frame; "corner depth missing" tells you the depth map has holes
where the frame is.

### Before the first flight

1. **Walk the estimate with the props off.** Carry the airframe around in
   front of the window and watch `/window_pose`. The centre should sit still
   in NED to within a few centimetres while the airframe moves. That is the
   whole point of the estimate being in NED rather than in the camera frame,
   and it is the one test that catches a wrong `cam_pitch` or a wrong
   `pose_frame` before it costs you an airframe.
2. **Measure the window.** `/window_pose` reports the reconstructed width and
   height. If they do not match a tape measure, the intrinsics or the depth
   scale are wrong, and every distance in the approach is wrong by the same
   factor. (Expect ~2% under: the detector pads its corners 5 px inwards.)
3. **Confirm `EKF2_EV_CTRL` is 0.** This step used to say to check
   `cs_yaw_align` and set `EKF2_EV_CTRL=9` per 10.3 — that was for the
   visual-odometry version of this flight, which is not what runs. On flow +
   rangefinder there is no external vision at all, and a leftover
   `EKF2_EV_CTRL` from a VIO experiment leaves EKF2 waiting for vision that
   never arrives. The 10.3 yaw-alignment trap only bites when the
   magnetometer is off *and* vision is supposed to supply the heading; keep
   the magnetometer on for this flight.
4. **Check the rangefinder separately from the flow.** New since the ARK
   Flow: they are two devices on two buses now. Section 11's check is not
   optional.
5. **Check the standoff against your window.** The D435i needs `1.26 × window
   height` just to fit the aperture in frame. `window_traverse` pushes the
   standoff out on its own if the measured aperture needs it, and logs when
   it does — but if it warns that it hit `max_standoff_distance`, your window
   is too big for this lens at any sane distance.
6. Give yourself `standoff_distance + exit_distance` of clear space on the
   approach side and beyond, plus the `align_tolerance` basket.

### Parameters

The traversal's own, on top of everything 6c and 6e take:

| parameter | default | what |
|---|---|---|
| `standoff_distance` | **2.0** | m in front of the window plane the approach lines up on, along the normal. **Raised from 1.6 for the D435i** — at 43° of vertical FOV a 1.2 m window needs 1.51 m just to fit in frame, so 1.6 m had no margin. See [section 0](#0-the-zed--ark-flow--realsense-d435i--pmw3901-port) |
| `camera_hfov_deg` / `camera_vfov_deg` | 70.4 / 43.3 | **new.** The FOV the standoff clamp is computed against — the D435i's *colour* sensor, measured on this unit. Not read from `CameraInfo`: `window_detect` already bakes the true intrinsics into the angles it publishes, so this is only used for the one geometric clamp |
| `min_standoff_margin` | 1.25 | **new.** How much further than "just fits" the approach must stand off. 1.0 would put the window corners on the frame edge, where `window_detect` flags them `TRUNCATED` and the corner depths are least trustworthy |
| `max_standoff_distance` | 4.0 | **new.** Ceiling on that clamp, so a wildly over-estimated aperture cannot walk the approach point out of the arena |
| `exit_distance` | 1.5 | m beyond the window plane the run ends |
| `altitude_offset` | 0.0 | m added to the estimated window centre height |
| `approach_speed` | 0.30 | m/s during `ALIGN` |
| `traverse_speed` | 0.45 | m/s through the window |
| `align_tolerance` | 0.18 | m radius around the approach point |
| `align_yaw_tolerance_deg` | 8.0 | deg off the window normal |
| `align_settle_seconds` | 1.5 | how long all three must hold together |
| `align_timeout` | 60.0 | s before the approach is abandoned into a landing |
| `traverse_timeout` | 25.0 | s for the run through |
| `clear_seconds` | 4.0 | station keeping on the far side |
| `blind_traverse_seconds` | 3.0 | s of open-loop push if vision dies mid-run |
| `flight_seconds` | 150.0 | hard limit from the start of the climb — fires from every stage **except** `TRAVERSE` |
| `cam_x/y/z`, `cam_roll/pitch/yaw` | 0.0 | camera pose in the body frame, ROS convention, measured to the D435i's **left imager**. |
| `depth_min` / `depth_max` | 0.35 / 8.0 | m, believable corner depths |
| `corner_spread` / `corner_spread_frac` | 0.25 / 0.15 | m and fraction of range, the primary outlier filter |
| `plane_tolerance` | 0.15 | m off the best-fit plane |
| `window_min_size` / `window_max_size` | 0.35 / 3.0 | m, believable aperture |
| `max_tilt_deg` | 35.0 | deg the normal may be off horizontal |
| `buffer_seconds` | 2.5 | s the median is taken over |
| `pose_min_samples` | 6 | accepted samples before anything is flown to |
| `pose_max_age` | 1.5 | s before the newest sample stops being evidence |
| `pose_lost_timeout` | 6.0 | s without a pose during `AIM`/`ALIGN` before abandoning |
| `gate_metres` / `gate_yaw_deg` | 1.0 / 40.0 | innovation gate |

And on `window_detect`: `camera_info_topic` (where the intrinsics come from),
`publish_geometry` (true), `fallback_hfov_deg` (90, used only until the first
`CameraInfo` arrives — the node warns loudly while it is guessing).

### Status output

Same `/takeoff_status` topic and format. The detail field: `aim24` (24 deg
left to turn), `algn0.42` (0.42 m to the approach point), `thru1.8/3.1`
(1.8 m flown of 3.1 m), `3s` (seconds of `CLEAR` left).

### When it gives up

An abandoned attempt **lands**, it does not retry. Going back to `SCAN` after
a failed approach means a vehicle that is now somewhere other than where it
swept from, with an unknown amount of battery, starting the same attempt under
the same conditions that just failed. Land, read the rejection tally, fly it
again.

---

## 6g. Into the dark room, around it, and back out (`window_room_traverse`)

Section 6f ends on the far side of the window. This is the rest of the
competition run: **fly in through the window, move around inside the room on
the moves you give, find the window again from the inside, and fly back out** —
optionally running the doll model the whole time it is in there and putting a
live count in QGroundControl.

It is all **one node and one launch file**. The four missions below differ only
in arguments, and they are a ladder: each rung is one strictly larger
commitment than the last, and each one lands safely on its own. Fly them in
order.

| # | mission | what is new | command |
|---|---|---|---|
| 1 | **traverse** | in through the window, hold, land on the far side | section 6f, `window_traverse` |
| 2 | **in and around** | the room moves you type, then land **inside** | `return_through_window:=false dolls:=false` |
| 3 | **in, around, out** | find the window again from inside, fly back out, land | `return_through_window:=true dolls:=false` |
| 4 | **the whole run** | the TensorRT doll model, geotagged count, live to QGC | `dolls:=true qgc:=true` |

Everything about the traversal itself is section 6f's, unchanged.
`window_room_traverse` is a subclass of `window_traverse` and nothing else: the
sweep, the lock, the pose estimator, `RECENTRE`/`AIM`/`ALIGN`/`TRAVERSE`/`CLEAR`,
the airframe clearance arithmetic, the yaw cone, the blind-traverse fallback
and every PX4 gate come straight from the node that has actually flown. **Read
6f first.** Nothing it says is repeated here.

---

### The three new nodes

| node | what it does |
|---|---|
| `window_room_traverse` | the flight. `WindowTraverse` plus the room pattern (`ROOM_MOVE` / `ROOM_TURN` / `ROOM_HOLD`) and `RELOCK`, which throws away everything the estimator believes about the first window so the same approach stages can run a second time from the far side of it |
| `doll_detect` | the TensorRT engine from `Downloads/DroneImpl_v7`, run through Ultralytics with ByteTrack, counting dolls by **where they are in the room** rather than by track id. Gated on `/doll_detect_enable` |
| `qgc_doll_status` | speaks MAVLink straight to QGC over WiFi and puts the count in the message panel and in MAVLink Inspector. Does not touch PX4 or the flight |

`room_display` (the Arduino TFT) is started too, if you have the board. It
needs `arduino/room_status/room_status.ino` flashed — **not** the older
`tft_status.ino` from section 7; the two sketches take different protocols and
`lcd:=true` is pinned off in this launch file so they cannot fight over the
same serial port.

---

### The flight, in order

```
climb -> hold -> find the window -> LOCK -> AIM -> ALIGN -> TRAVERSE
      -> CLEAR              (the hold inside the room, clear_seconds)
      -> ROOM_MOVE / ROOM_TURN / ROOM_HOLD, one step at a time, your list
      -> RELOCK -> AIM -> ALIGN -> TRAVERSE (back out) -> CLEAR -> land
```

`CLEAR` is the inherited stage from 6f and it does double duty here: on the way
in it is the settle inside the room, and when its `clear_seconds` are up it
starts the room moves instead of landing. On the way out it is 6f's ending,
unchanged.

| stage | what happens | how it ends |
|---|---|---|
| … 6f's stages … | identical, to `inside_distance` (1.20 m) past the window plane | as 6f |
| `ROOM_MOVE` | one translation leg, in the **current heading frame**, on the inherited carrot | inside `MOVE_TOLERANCE` and settled, or `room_move_timeout` |
| `ROOM_TURN` | one yaw on the spot, ramped at `yaw_rate` | inside tolerance, or `room_turn_timeout` |
| `ROOM_HOLD` | `room_hold_seconds` of settling between legs | the clock |
| `RELOCK` | stationary, facing the wall, **rebuilding the window pose from nothing** | `pose_min_samples` accepted samples, or `relock_timeout` → land inside |
| `AIM` … `CLEAR` | 6f's approach and traversal again, outbound to `outside_distance` | as 6f |

Each room leg is flown and then **settled** before the next starts, for the
same reason section 6b gives: otherwise step *n+1* samples its start point
while the vehicle is still overshooting step *n*, and the errors compound down
the pattern instead of each step correcting from where the last one really
finished.

---

### The moves you give (`room_sequence`)

The in-room moves are a string, in **exactly the grammar `offboard_sequence`
takes** (section 6b) — comma-separated items, each a name and a number
separated by a space, a colon or an `=`:

```bash
ros2 run drone_testing window_room_traverse --ros-args \
    -p room_sequence:="forward 0.5, yaw 90, left 0.3, backward 0.4, yaw 90"
```

or as a launch argument, which is the same string:

```bash
ros2 launch drone_testing window_room_traverse.launch.py \
    room_sequence:="forward 0.5, yaw 90, left 0.3, backward 0.4, yaw 90"
```

| step | units | what it does |
|---|---|---|
| `forward` / `backward` / `left` / `right` | metres | translation |
| `yaw` | degrees | rotate in place, **`+` = to the right**, clockwise seen from above |

`up` and `down` are **rejected**, and the node refuses to start rather than
ignoring them: the room stages dispatch only to a move handler and a turn
handler, there is no altitude handler among them, and an altitude change inside
the room is really a change to the height the return approach lines up at —
which is what `altitude_offset` is for. A malformed string is likewise fatal at
startup, exactly as in 6b: a typo here is a typo in a flight plan and must not
be quietly guessed at.

```
room_sequence: 'up' is not a room move; only forward/backward/left/right/yaw
are flown inside the room (use altitude_offset to change the height the
traversal is flown at)
```

**The frame is the current heading, not the takeoff heading.** This is the
opposite of section 6b's default and it is deliberate: one reference yaw is
sampled when each step *starts*, so "forward" after a `yaw 90` means along the
**new** heading. That is what makes a list of moves a box rather than a
dog-leg.

**Leave `room_sequence` empty and you get the six-leg box** the node was
written around, built from its own parameters — 0.30 m left, 0.30 m forward,
90° right, 0.30 m on, 90° right, 0.30 m on. Nothing that already flies changes.
The whole plan is printed once at startup, whichever form you used, and that
line is the thing to read back before you arm:

```
ROOM MISSION: through the window and 1.20 m in, then 0.50 m forward,
yaw +90 deg, 0.30 m left, 0.40 m backward, then find the window again and fly
back out 1.20 m, then land. Doll detection runs from the inbound commit to the
outbound clear. Hard clock 300 s.
```

#### The one constraint your list must satisfy

**`RELOCK` does not sweep.** It stands still facing wherever your list left the
nose and rebuilds the pose from there. So on rung 3 and rung 4 the moves must
end with **the window in frame** — with the defaults, two 90° right turns leave
the nose 180° from the entry heading, i.e. pointing back at the wall it came
through, and that is the geometry `RELOCK` needs. Change the turns and you
change that.

Check it without flying: stand where your list ends, facing where it ends up
facing, and look at `/window_detected`. If the answer is no, the aircraft will
sit there for `relock_timeout` (45 s) and then land in the room. That is a
survivable outcome, not a crash, but it is not the mission.

---

### Why the estimator is cleared and the cone moved on the way out

Three things are genuinely different about the second approach, and all three
are about the same fact: the aircraft is now on the other side of the window
and pointing roughly backwards.

1. **The pose estimate is thrown away at `RELOCK`.** Not an optimisation — the
   estimator's innovation gate refuses any sample whose normal has swung more
   than `gate_yaw_deg` from what it already believes, and the normal is always
   resolved to point back at the aircraft. From inside the room the same
   physical window produces a normal 180° from the one on file, so **every**
   return sample would be gated out as "normal swung". Clearing the buffer is
   what makes the second approach possible at all.

2. **The yaw cone is re-centred.** `window_traverse` refuses to believe in, or
   turn towards, anything more than `yaw_cone_deg` off the heading it *armed*
   on. That is the right rule going in and exactly backwards coming out. The
   cone is kept, at the same width; only its centre moves, to the heading the
   aircraft holds at the end of your room moves. The return leg is still
   protected against locking onto a doorway behind it — it is just protected
   about the right axis.

3. **`exit_distance` is two numbers.** The inbound run ends
   `inside_distance` (1.20 m) past the plane, the outbound run
   `outside_distance` (1.20 m). The inherited code reads one parameter; the
   subclass points it at the inbound number and swaps it at `RELOCK`, so
   nothing else has to know there are two.

---

### The room has to fit, and it is deeper than you think

The return approach flies to the **standoff point**, which is
`standoff_distance` (2.0 m, and further if the window is large — see
`effective_standoff` in 6f) *in front of* the window on its axis, i.e. **inside
the room**. So the room must be at least `standoff_distance` plus the airframe
deep, measured from the window wall — not merely deep enough for your moves.
Pace it out with the props off.

A room that cannot give that depth needs `standoff_distance` lowered, and
lowering it costs the return approach its view of the whole window. That is the
trade `return_through_window:=false` exists for: fly in, do the moves, land
inside. **That is rung 2, and it is the right first flight in any new room** —
whether the pattern fits is a separate question from whether it can be
reversed, and asking them one at a time is how you get an airframe back.

---

### The doll count (rung 4)

#### The model, and where its runtime lives

`db.engine` is a **YOLO26n** TensorRT engine, one class (`dolls`), exported
with **Ultralytics 8.4.118** on 2026-09-03, FP16, `imgsz` **1280x1280**, batch
1, and `end2end: True` — NMS is inside the engine, so its output is a fixed
`(1, 300, 6)` and nothing downstream runs NMS.

All of that is readable off the file itself, because an Ultralytics-exported
`.engine` is **not** a bare TensorRT engine: it is a 4-byte little-endian
length, then that many bytes of JSON metadata, then the serialized engine.
Worth knowing, because feeding the whole file to
`IRuntime::deserializeCudaEngine` fails with

```
Serialization assertion header.magicTag == kEXPECTED_MAGIC_TAG failed.
Trying to load an engine created with incompatible serialization version
(556 != 1953657958)
```

which reads like a version mismatch and **is not one** — 556 is just the JSON
length. Ultralytics skips the header for you. To check the engine by hand:

```python
import json, tensorrt as trt
data = open('db.engine','rb').read()
n = int.from_bytes(data[:4], 'little')
print(json.loads(data[4:4+n]))                      # the metadata
eng = trt.Runtime(trt.Logger()).deserialize_cuda_engine(data[4+n:])
```

The engine loads and runs on this Jetson: ~20–40 ms a frame, comfortably
inside `doll_max_fps` (6.0). TensorRT does log one warning every time:

```
Using an engine plan file across different models of devices is not supported
and is likely to affect performance or even cause errors or deadlock.
```

That is because the engine was built on a **different Jetson model** from this
one. It deserializes and runs because both are `sm_87`, and it has been
exercised here end to end. If you want the warning gone — or hit anything odd
that smells like it — re-export on this board with the same Ultralytics
version. That needs the original `.pt`, which is **not on this machine**: the
only model artefact here is `db.engine` itself.

torch and ultralytics live in a **venv** (`~/venvs/dolls`), not in the system
Python, and the launch file puts it on `PYTHONPATH` for the doll node only.
See [section 1](#the-doll-models-runtime-section-6g-rung-4-only) for what is
installed, why it is not a system install, and the `sm_87` wrinkle. To run the
node **by hand**, you have to supply that path yourself:

```bash
PYTHONPATH=/home/ark-jetson-orin-2/venvs/dolls/lib/python3.12/site-packages:$PYTHONPATH \
  ros2 run drone_testing doll_detect --ros-args -p require_enable:=false
```

Forget it and the node starts, subscribes, and then logs
`Cannot load …/db.engine: No module named 'ultralytics'` on the first frame —
counting zero while the flight carries on unaffected, because nothing in the
flight is gated on the detector.

#### What a doll's identity is, and why it is not a track id

`jw_px4_arduinocount.py` identified a doll by its position **in the image**: a
ByteTrack id, plus a pixel-distance re-identification fallback for when the
tracker dropped it. That is a perfectly good answer for a fixed camera. On a
drone that yaws 90° twice it is not one — the same doll leaves the frame on one
side and comes back at a completely different pixel, and every time it does,
the old logic mints a new doll.

`doll_detect` identifies a doll by **where it is in the room**. Every detection
is projected out of the camera, through the airframe, into the PX4 local NED
frame, and lands on an existing doll if it is within `doll_merge_radius` of it.
The track id is still there — it is what makes consecutive frames cheap and
what the confirmation counter counts — but it is not the identity. Turn the
aircraft round twice and fly back past the same doll and it is still doll #3,
because it is still in the same corner of the room.

That is what the id *is*, and it is published as one: `/doll_report` carries
`id x y z` for every doll counted, in metres NED relative to the arming point,
so the count can be checked against the room afterwards instead of taken on
trust.

#### How a pixel becomes a room position

The same three transforms `window_traverse` uses for the window, with the
**same camera mounting parameter names**, so one launch file sets both:

```
pixel + depth  -> camera frame   (pinhole intrinsics from CameraInfo)
camera frame   -> body FRD       (cam_x/y/z, cam_roll/pitch/yaw)
body FRD       -> local NED      (VehicleAttitude quaternion + position)
```

Depth is the aligned depth image, median-sampled over the middle
`doll_depth_patch` (30%) of the box, exactly the way `window_detect` samples a
window corner, so one dead pixel does not place a doll on the far wall. **A box
with no usable depth is dropped, not guessed.** A doll at an assumed range
lands somewhere arbitrary in NED, and that is the one input that could actually
corrupt the count. It is still published as *visible*; it just cannot be
*counted*.

#### When the model runs

Gated on `/doll_detect_enable`, which the flight node publishes: **true from
the moment it commits to the inbound traverse, false once it is clear of the
window on the way back out.** That is the window of the flight the dolls can
possibly be in, and running the model outside it is Jetson time spent on the
wall — on a machine sharing a USB3 bus and a GPU with the RealSense and the
window detector, that matters. The topic is republished every tick rather than
on the edge, so a detector that starts late still gets told to run.

`require_enable:=false` runs it free, on the ground included. That is how you
bench the model and the geotagging without flying.

#### Topics

| topic | type | what |
|---|---|---|
| `/doll_count` | `Int32` | cumulative, never decreases |
| `/dolls_visible` | `Int32` | this frame |
| `/doll_report` | `String` | `n\|id:x,y,z\|id:x,y,z\|…` in NED |
| `/doll_image` | `Image` | annotated (`doll_publish_image:=true` only) |
| `/doll_detect_enable` | `Bool` | from the flight node |
| `/mission_phase` | `String` | `PHASE\|STAGE\|detail`, what the TFT shows |

Phases, in order: `OUTSIDE` → `SEARCH` → `WINDOW_IN` → `ENTERING` →
`INSIDE` → `ROOM` → `SEARCH_OUT` → `WINDOW_OUT` → `EXITING` → `OUT` →
`LANDING`. They are derived from the stage every tick rather than stored, so
the label cannot drift out of step with what the aircraft is actually doing.

#### Getting the count into QGroundControl

`/dev/ttyTHS1` on this Jetson is the uXRCE-DDS link to PX4, so there is no
spare MAVLink serial port to the flight controller, and PX4 has no uORB topic
in the default `dds_topics.yaml` that comes out of the other end as
`STATUSTEXT`. So `qgc_doll_status` **does not go through PX4 at all**. It
speaks MAVLink straight to QGC over the WiFi the Jetson is already on, as an
extra *component* of the same vehicle — `source_system` = the vehicle's
`MAV_SYS_ID` (1), `source_component` = 191 (`MAV_COMP_ID_ONBOARD_COMPUTER`) —
so QGC files everything under the aircraft it is already showing instead of
popping up a second vehicle.

What then appears in QGC:

| message | where you see it |
|---|---|
| `STATUSTEXT` `"DOLLS 3 (now 1)"` | the message panel at the bottom, and spoken aloud if audio is on. Sent when the total changes, and repeated periodically so a GCS that connected late still learns the count |
| `NAMED_VALUE_INT` `dolls`, `dolls_now` | MAVLink Inspector, live — and plottable in Analyze, which is how you **watch the count climb during the run** rather than reading a log afterwards |
| `STATUSTEXT`, once at the end | the per-doll NED positions from `/doll_report`, one line per doll, sent when the phase reaches `DONE`/`LANDING`, so the result is in the QGC log |

Addressing: `qgc_host` defaults to `255.255.255.255`, i.e. broadcast on the
subnet, which finds QGC without anyone typing an IP — both are on the same
WiFi. Point it at the laptop's address if broadcast is filtered. If QGC is
reached over a telemetry radio instead of WiFi, this node cannot help; that
case needs `MAV_*_FORWARD` on the FC and a serial port this Jetson does not
have free.

Nothing here touches the flight. If the socket cannot be opened, or QGC is not
there, it logs once and keeps counting quietly.

---

### Running it

`agent_only` defaults to **true**, as in every other launch file here: the
launch brings up the support stack — agent, RealSense, `window_detect`,
`doll_detect`, `qgc_doll_status`, the TFT — and you run the flight node by hand
in a second pane, which is what keeps stdin a tty and the `q` / `k` aborts
alive.

**Bench, no props, no agent, no flight node** — camera and window detector
only, so you can check the detection and walk the window estimate by hand:

```bash
ros2 launch drone_testing window_room_traverse.launch.py flight:=false
ros2 topic echo /window_detected
ros2 topic echo /window_pose
```

`flight:=false` suppresses the **doll node too** — it is grouped with the
flight node, on the same startup delay. To bench the model and the geotagging
against that camera, start it yourself in a third pane, ungated:

```bash
PYTHONPATH=/home/ark-jetson-orin-2/venvs/dolls/lib/python3.12/site-packages:$PYTHONPATH \
  ros2 run drone_testing doll_detect --ros-args \
    -p require_enable:=false -p publish_image:=true \
    -p cam_x:=0.105 -p cam_z:=-0.04
ros2 topic echo /doll_count
ros2 topic echo /doll_report
```

The `PYTHONPATH` prefix is not optional for a hand-started node — the launch
file sets it via `doll_venv`, `ros2 run` does not.

The geotag needs `/fmu/out/vehicle_local_position` and
`/fmu/out/vehicle_attitude`, so with no agent running it will detect and report
dolls as *visible* but count none. To bench the count, use the default
`agent_only:=true` launch (which does start the agent) with
`dolls:=true require_enable:=false`, and carry the airframe around the room by
hand with the props off.

**Rung 2 — in, your moves, land inside:**

```bash
# pane 1
ros2 launch drone_testing window_room_traverse.launch.py \
    dolls:=false return_through_window:=false \
    cam_x:=0.105 cam_z:=-0.04

# pane 2
ros2 run drone_testing window_room_traverse --ros-args \
    -p takeoff_altitude:=1.2 \
    -p return_through_window:=false \
    -p room_sequence:="forward 0.5, yaw 90, left 0.3" \
    -p cam_x:=0.105 -p cam_z:=-0.04
```

**Rung 3 — and back out through the window:**

```bash
ros2 run drone_testing window_room_traverse --ros-args \
    -p takeoff_altitude:=1.2 \
    -p room_sequence:="forward 0.3, yaw 90, forward 0.3, yaw 90, forward 0.3" \
    -p cam_x:=0.105 -p cam_z:=-0.04
```

**Rung 4 — the whole run, with the count in QGC:**

```bash
ros2 launch drone_testing window_room_traverse.launch.py \
    dolls:=true qgc:=true qgc_host:=192.168.1.42 \
    room_sequence:="forward 0.3, yaw 90, forward 0.3, yaw 90, forward 0.3"

ros2 run drone_testing window_room_traverse --ros-args \
    -p takeoff_altitude:=1.2 \
    -p room_sequence:="forward 0.3, yaw 90, forward 0.3, yaw 90, forward 0.3"
```

> **The room moves must be given to the node you actually run.** Started by
> hand with `ros2 run`, the flight node takes `room_sequence` from *its own*
> `--ros-args`, not from the launch file's argument — the launch file's copy
> only reaches the flight node it starts itself (`agent_only:=false`). Pass the
> same string to both, or run with `agent_only:=false` and accept losing the
> keyboard aborts.

Everything in one shot, no keyboard abort (the RC kill switch still works, and
it is the one that matters):

```bash
ros2 launch drone_testing window_room_traverse.launch.py agent_only:=false \
    room_sequence:="forward 0.5, yaw 90, left 0.3"
```

Useful variations:

| argument | effect |
|---|---|
| `return_through_window:=false` | fly in, do the moves, land inside. **Rung 2** |
| `dolls:=false` | no model at all. Rungs 2 and 3 |
| `require_enable:=false` | run the model the whole time, ground included — how you bench it |
| `qgc:=false` | no MAVLink to QGC |
| `display:=false` | no Arduino |
| `camera:=false` | the RealSense is already running from another stack; do not start a second copy (librealsense refuses the device rather than sharing it, and the failure looks like a dead camera) |
| `flight:=false` | camera and window detector only: the bench test. **Suppresses the doll node as well** — it is grouped with the flight node |

---

### A dark room, specifically

Two different sensors, and only one of them cares that the lights are off.

- **Depth does not need light.** The D435i projects its own IR pattern, and
  `emitter:=1` (the default) keeps it on. So the window's corner depths, the
  pose estimate, and every doll's range keep working in the dark.
- **Everything else is the colour image, and that does need light.**
  `window_detect` is an HSV threshold — it finds the window by its *colour*
  (`color:=blue` is the default here) — and `doll_detect` runs the model on the
  same colour frame. A room dark enough to starve the RGB sensor gives you a
  detector that never latches and a model that sees nothing, with depth working
  perfectly the whole time.

So the thing to check in the actual arena, before flying it, is
`/window_detection/image` (in a browser at `http://<jetson-ip>:8080/`, no ROS
needed on the viewing machine) with the room lit the way it will be on the day.
The window frame is usually the lit part — it is a hole into a brighter space —
which is what makes this work at all; the dolls are not.

`window_detect_darkroom.py` exists in the package and is **not** wired into
`setup.py`, so there is no `ros2 run` entry point for it. It is an older
ZED-topic copy of the detector, kept for reference. Do not reach for it
expecting a low-light mode.

---

### Parameters

On top of everything 6c and 6f take:

| parameter | default | what |
|---|---|---|
| `room_sequence` | *(empty)* | the in-room moves, 6b's grammar. Empty = the six-leg box below |
| `room_strafe` | 0.30 | m left — first leg of the default box |
| `room_forward_1` | 0.30 | m forward |
| `room_turn_1_deg` | 90.0 | deg right |
| `room_forward_2` | 0.30 | m on the new heading |
| `room_turn_2_deg` | 90.0 | deg right |
| `room_forward_3` | 0.30 | m on again |
| `room_speed` | 0.20 | m/s for the room legs. Slow: they are short, and this is also the speed the doll detector sees the room at |
| `room_hold_seconds` | 2.0 | s of settling between legs |
| `room_move_timeout` | 20.0 | s for one leg |
| `room_turn_timeout` | 25.0 | s for one turn |
| `return_through_window` | true | false = land inside (rung 2) |
| `inside_distance` | 1.20 | m past the plane the inbound run ends |
| `outside_distance` | 1.20 | m past the plane the outbound run ends |
| `relock_timeout` | 45.0 | s standing still looking at the wall before giving up and landing inside. Generous — it is the one stage with nothing to fall back on |
| `relock_settle_seconds` | 2.0 | s of holding still before the new estimate is believed at all. The moves end with a turn, and a turn smears both the flow and the depth |
| `flight_seconds` | 300.0 | hard clock from the start of the climb. Two traversals, the moves and two lots of settling do not fit in 6f's 150 s |

`doll_detect`. The **launch arguments** are the `doll_`-prefixed names below;
the node's own parameters, if you run it by hand with `ros2 run`, drop the
prefix (`model_path`, `tracker_path`, `confidence`, `min_frames_to_confirm`, `max_fps`,
`merge_radius`, `depth_min`, `depth_max`, `depth_patch`, `min_depth_pixels`,
`publish_image`).

| parameter | default | what |
|---|---|---|
| `doll_model` | `~/Downloads/DroneImpl_v7/db.engine` | the TensorRT engine. **Built for this Jetson and this TensorRT version** — an engine copied from another machine will not load |
| `doll_tracker` | `~/Downloads/DroneImpl_v7/custom_bytetrack.yaml` | ByteTrack config: `track_buffer: 500` is the important line — a doll that leaves the frame during a 90° turn keeps its track |
| `doll_confidence` | 0.55 | the same gate as the standalone script |
| `doll_min_frames` | 5 | frames a track must survive before it may create or join a doll. Kills single-frame false positives before they reach the geotagger |
| `doll_max_fps` | 6.0 | cap. The model is not the expensive thing here — the flight node's setpoint timer is what must not be starved, and a doll does not move |
| `doll_merge_radius` | 0.60 | m. Two detections closer than this **are** the same doll. Must be comfortably smaller than the smallest gap between two real dolls and comfortably bigger than the position error. **Measure the gap in your arena** |
| `doll_depth_min` / `doll_depth_max` | 0.30 / 8.0 | m. Closer is the airframe; further is the far wall showing through a box |
| `doll_depth_patch` | 0.30 | fraction of the box side the depth median is taken over. The middle 30% of a doll is doll; the edges are the floor behind it |
| `doll_min_depth_pixels` | 12 | valid depth samples needed in that patch |
| `doll_publish_image` | false | publish the annotated frame |
| `require_enable` | true | false = run the model regardless of the flight phase |
| `doll_venv` | `~/venvs/dolls/lib/python3.12/site-packages` | prepended to `PYTHONPATH` **for the doll node only**, so the plain `doll_detect` entry point can import torch and ultralytics. A path that does not exist is ignored by Python |

`qgc_doll_status`: `qgc_host` (`255.255.255.255`), `qgc_port` (14550),
`qgc_sysid` (1, must match the vehicle's `MAV_SYS_ID`).

---

### Before the first flight

Everything section 6f's checklist says still applies — the camera mounting
measurement, walking the window estimate with the props off, the flow health
check. In addition:

1. **The camera mounting is shared.** `cam_x/cam_y/cam_z/cam_roll/cam_pitch/cam_yaw`
   go to **both** the flight node and the doll node, in the ROS convention
   (x forward, y **left**, z **up**). Get them wrong and the window is
   misplaced by the offset *and* every doll is — which merges dolls that are
   not the same one, and the count comes out low with no sign that anything
   went wrong.
2. **Pace out the room against `standoff_distance`,** not against your moves.
   See above.
3. **Stand where your moves end, facing where they end, and check
   `/window_detected`.** `RELOCK` does not sweep.
4. **Set `doll_merge_radius` from the real doll spacing.**
5. **Fly rung 2 first in any new room.** Then 3. Then 4.

### Status output

Same `/takeoff_status` topic and format as every other node here, so the
display works unchanged. The detail field carries the leg counter:
`2/5 for0.32` (leg 2 of 5, 0.32 m to go), `3/5 yaw42`, `4/5 2s` (settling),
`relock 12s`.

`/mission_phase` carries `PHASE|STAGE|detail` for the TFT and anything else
that wants to know which side of the wall the aircraft is on.

### When it gives up

Same rule as 6f, at both ends: an abandoned approach **lands**, it does not
retry. A failed `RELOCK` lands *inside the room* — which is the correct
outcome, because the alternative is an aircraft with an unknown amount of
battery flying at a wall it cannot find a hole in. A room leg that times out or
loses the flow is reported and the pattern **carries on**, exactly as in 6b,
because the next leg may not depend on whatever failed. The per-leg results are
printed as one line at the end of the flight:

```
Room mission: inbound done; room forward 0.50 -> done, 0.48 m of 0.50 m;
yaw 1.57 -> done; left 0.30 -> TIMED OUT 0.11 m short; outbound done.
```

---

## 7. Optional: LCD status display

An Arduino running `arduino/tft_status/tft_status.ino` shows the stage, arm
state and altitude. It is started by default with the launch file:

```bash
ros2 launch drone_testing takeoff_test.launch.py lcd:=true lcd_port:=/dev/ttyACM0
```

Disable it with `lcd:=false`.

---

## 8. Optional: start at boot via systemd

`drone_testing/px4-agent.service` brings the Jetson up flight-ready: DDS agent
plus the takeoff node waiting for your Offboard switch.

```bash
sudo cp ~/imav26-ws-2/ws_ros2/src/drone_testing/drone_testing/px4-agent.service \
        /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now px4-agent.service
```

```bash
systemctl status px4-agent.service
journalctl -u px4-agent.service -f          # the flight log
sudo systemctl stop px4-agent.service       # abort from a shell
sudo systemctl disable --now px4-agent.service
```

**Read this before enabling it:**

- systemd gives the node no tty, so `q` / `k` are **dead**. Your RC kill switch
  and the TX mode switch are the only aborts.
- It runs on **every** boot — including a battery swap in the field or a
  brownout reboot. The drone is armed-and-waiting whenever it is powered.
- `Restart=no` is deliberate. `on-failure` would relaunch a node that had
  aborted and let it re-arm on its own.

Altitude and hold time live in the `ExecStart=` line; after editing:

```bash
sudo systemctl daemon-reload && sudo systemctl restart px4-agent.service
```

---

## 9. Other nodes in the package

| node               | what it does                                                     |
|--------------------|------------------------------------------------------------------|
| `offboard_takeoff` | the autonomous takeoff / hold / land test (this README's subject) |
| `offboard_translate` | takeoff, then a 1 m horizontal move, then land (section 6)      |
| `offboard_sequence` | takeoff, then a list of moves / climbs / yaws, then land (section 6b) |
| `offboard_mission` | multi-waypoint offboard mission                                   |
| `zed_localization` | feeds ZED visual odometry into PX4 as `vehicle_visual_odometry`. **Not used on this airframe** — there is no ZED and no external vision; see sections 0 and 6e |
| `lcd_status`       | drives the Arduino status display                                 |
| `pixhawk_node`     | MAVLink telemetry reader                                          |
| `cam`              | camera capture helper                                             |
| `window_detect`    | RealSense D435i window detection, publishes `/window_detected` and `/window_geometry` (sections 6c, 6f) |
| `window_scan`      | takeoff, yaw sweep, lock onto the window, land after 40 s (section 6c) |
| `window_traverse`  | takeoff, sweep, estimate the window's pose, line up and fly through it (section 6f) |
| `window_room_traverse` | 6f, then the room moves you give, then find the window again from inside and fly back out (section 6g) |
| `doll_detect`      | the TensorRT doll model + ByteTrack, counting dolls by their position in the room; publishes `/doll_count`, `/dolls_visible`, `/doll_report` (section 6g) |
| `qgc_doll_status`  | puts the live doll count in QGroundControl over MAVLink/UDP, bypassing PX4 entirely (section 6g) |
| `room_display`     | drives the `room_status.ino` Arduino TFT from `/mission_phase` and the doll topics (section 6g) |
| `aruco_pose`       | down-camera ArUco pose, publishes `/aruco/detected` and `/aruco/point` (section 6d) |
| `precision_land`   | takeoff, find the marker, centre on it, land on it (section 6d)   |

Other launch files:

- `translate_test.launch.py` — agent + `offboard_translate` (section 6)
- `sequence_test.launch.py` — agent + `offboard_sequence` (section 6b)
- `window_scan.launch.py` — agent + RealSense D435i + `window_detect` + `window_scan` (section 6c)
- `window_traverse.launch.py` — agent + RealSense D435i + `window_detect` + `window_traverse` (section 6f)
- `window_room_traverse.launch.py` — includes the above for the support stack, and adds `window_room_traverse` + `doll_detect` + `qgc_doll_status` + `room_display` (section 6g)
- `precision_land.launch.py` — agent + `aruco_pose` + `precision_land` (section 6d)
- `arm_test.launch.py` — agent + `offboard_mission`, for arm/disarm bench tests
- `offboard_launch.launch.py` — agent + ZED localization + `offboard_mission`

---

## 10. Troubleshooting

| symptom | cause / fix |
|---|---|
| `Waiting for VehicleStatus from PX4...` forever | DDS link down. Check the agent is running, the baud is 921600 both ends, `UXRCE_DDS_CFG` is set, and nothing else holds `/dev/ttyTHS1`. |
| `Not arming: rangefinder is NOT being fused (cs_rng_kin_consistent false)` | **Reboot the flight controller.** This flag is sticky: EKF2 only updates it while `in_air` is true (`range_height_control.cpp` runs the consistency check inside `if (_control_status.flags.in_air)`), so once it latches false in flight nothing on the ground can clear it. It comes back true at boot. See section 11.1. |
| `dist_bottom` stuck at exactly `EKF2_MIN_RNG` | The lidar is **not** healthy and EKF2 is synthesising the on-ground value: `_range_sensor.setRange(_params.ekf2_min_rng); setValidity(true)`. That number is not a measurement. `rng_ok=False` in the same log line confirms it. |
| `Not arming: need z_valid and dist_bottom_valid` | Rangefinder not being fused. Check the ARK Flow wiring and `EKF2_HGT_REF` / `EKF2_RNG_CTRL`. |
| `Offboard mode not entered in time` | PX4 rejected the mode. Usually pre-arm checks failing — look at the PX4 console or QGC for the reason. |
| `Arming rejected / timed out` | Pre-arm check failure, or the safety switch is not pressed. |
| `Altitude disagreement: ekf=... lidar=...` | The EKF datum and the rangefinder disagree by more than 32 cm. Normally an estimator reset mid-climb; the node correctly refuses to accept arrival. |
| `Offboard lost; PX4 has control now (nav_state AUTO_LAND(18))` | A PX4 failsafe fired. The node now logs `PX4 failsafe: ...` on the same line — read that. Offboard's *only* special mode requirement is `mode_req_offboard_signal`, so the usual culprit is `offboard_control_signal_lost` (a gap > `COM_OF_LOSS_T`, default 1.0 s, in the setpoint stream). See section 11.2. |
| `Offboard lost; PX4 has control now` | The TX switch moved, or PX4 failsafed. The node lets go on purpose. |
| `error: option --uninstall not recognized` on build | Stale `--symlink-install` state; see the build section above. |
| `q` / `k` do nothing | The node was started via `ros2 launch` or systemd, so stdin is not a tty. Run it with `ros2 run` in its own pane. |
| `BENCH: no /aruco/detected messages` | `aruco_pose` is not running, or it is on a different ROS domain. Start it: `ros2 launch drone_testing precision_land.launch.py flight:=false`. |
| `precision_land` never leaves `SEARCH` | The marker is outside the footprint, or `marker_id` / `marker_size` / `aruco_dict` do not match the marker you actually printed. Check `ros2 topic echo /aruco/info` and the browser view on `:8080`. |
| `No VehicleAttitude is being published` | `vehicle_attitude` is not in the PX4 DDS topic list, so the marker vector cannot be tilt-compensated and is refused. Add it to `dds_topics.yaml` and reboot the FC. |
| The vehicle moves the **wrong way** towards the marker | An axis sign is inverted. Land, and go back to `mode:=bench` (section 6d) — this is exactly what that mode exists to catch. Fix `image_rotate` or the mounting. |
| Aligns, then oscillates around the marker | `align_gain` too high for the camera latency, or the marker is near the frame edge where the uncorrected lens distortion is worst. Lower `align_gain`, and calibrate the camera. |
| `window_traverse` never leaves `LOCK` | No usable window pose. The node logs which test is rejecting the samples — read that tally. Usual causes: `camera_info_topic` wrong (the log says it is guessing the FOV), the depth map has holes where the frame is (`corner depth missing`), or the sample boxes are landing on the wall behind it (`corner depths disagree` / `corners not coplanar`). |
| `/window_pose` centre wanders as you move the airframe | The estimate is not being placed correctly in NED. Check `cam_roll/cam_pitch/cam_yaw` and the lever arm — they must be measured to the D435i's left imager — and that `/fmu/out/vehicle_attitude` is actually in the PX4 DDS topic list. |
| `Traverse abandoned: could not settle on the approach point` | VO noise is larger than `align_tolerance`, or the estimate is still moving. Loosen `align_tolerance`, or raise `pose_min_samples` / `buffer_seconds` so the target stops shifting under the aircraft. |
| `window_room_traverse` never leaves `RELOCK`, then lands inside | The window is not in frame from where your moves left the nose. `RELOCK` does not sweep. Stand where the moves end, face where they end, and check `/window_detected` — then fix the last `yaw` in `room_sequence`, not the timeout. |
| The return approach backs into a wall, or `ALIGN` times out on the way out | The room is shallower than `standoff_distance` (2.0 m, more for a large window) measured from the window wall. That point is *inside* the room. Lower `standoff_distance`, or fly `return_through_window:=false` and land inside. |
| `room_sequence: unknown motion '…'` and the node exits | A typo in the room moves. Only `forward`/`backward`/`left`/`right`/`yaw` are accepted; `up`/`down` are rejected on purpose (use `altitude_offset`). The node refuses to fly a plan other than the one you typed. |
| A room leg reads `SKIPPED, flow never latched x/y` | The PMW3901 never gave a healthy horizontal estimate inside the room — dark floor, featureless floor, or too low. The turns still fly (gyro), the translations do not. This is the inherited rule from 6b: a move that cannot be measured is not flown. |
| `Cannot load …/db.engine: No module named 'ultralytics'` | The node was started without the venv on `PYTHONPATH`. The launch file sets it (`doll_venv`); a bare `ros2 run` does not — prefix it yourself, see 6g. |
| `Cannot load …/db.engine: …` anything else | The engine did not deserialize. Check it by hand with the metadata-header snippet in 6g; a raw `deserialize_cuda_engine` on the whole file **always** fails and is not evidence of a bad engine. |
| `Using an engine plan file across different models of devices` | Expected, and not fatal: `db.engine` was built on a different Jetson model. Both are `sm_87`, and it runs. Re-export on this board to silence it — which needs the `.pt`, not present here. |
| Doll count stays 0 while the model is loaded | No depth on the boxes (`N boxes dropped for no depth`) or no vehicle pose (`dropped for no pose`) — the per-second log line says which. Without the DDS agent there is no pose, so a bench run counts nothing by design. |
| `torch` warns `sm_87 is not compatible` / `no kernel image` | See section 1: the wheel has no `sm_87`, runs on `sm_80` binary compatibility, and this is verified working. Do not swap the wheel on the strength of the warning alone — test `torch.cuda.is_available()` and a GPU matmul first. |
| Doll count comes out low | `doll_merge_radius` (0.60 m) is larger than the real gap between dolls, so two dolls merge into one. Measure the spacing in your arena. Or the camera mounting is wrong, which displaces every doll by the same offset — it must match what the flight node has. |
| Doll count comes out high | `doll_merge_radius` is smaller than the position error, so one doll splits in two. Raise it, or check the flow is not drifting: the geotag is only as good as the NED position under it. |
| Nothing appears in QGC's message panel | `qgc_doll_status` is broadcasting to `255.255.255.255:14550` and the subnet is filtering it, or QGC is on a telemetry radio rather than the WiFi. Set `qgc_host` to the laptop's actual IP. Check `NAMED_VALUE_INT` `dolls` in MAVLink Inspector first — it is the same socket. |
| Lands 30–40 cm off after a good alignment | Drift during the open-loop descent. Check `precision_descent` is true, and that the flow stays healthy (`flow_ok=True`) down to `FLOW_MIN_AGL`. |

---

### 10.1 The sticky rangefinder flag (`cs_rng_kin_consistent`)

This is the single most common reason the node refuses to arm, and it is
**not** a wiring fault — the sensor is usually fine.

EKF2 runs its rangefinder kinematic-consistency check only while airborne:

```c
// range_height_control.cpp
if (_control_status.flags.in_air) {
    _rng_consistency_check.update(...);
}
```

and `updateConsistency()` can only set the flag back to true when
`|vz| > 0.5 m/s`. So the flag starts `true` at boot, can only go false in
flight, and can only recover in flight. **On the ground it is frozen.** A run
that trips it poisons every subsequent run in that power cycle.

- **Fix:** reboot the flight controller (`reboot` in the nsh console, QGC's
  reboot button, or a power cycle). Then confirm before you touch anything:

  ```bash
  ros2 topic echo /fmu/out/estimator_status_flags --once | grep -E "cs_rng_hgt|cs_rng_kin_consistent"
  ```

  You want `cs_rng_hgt: true` **and** `cs_rng_kin_consistent: true`. If
  `cs_rng_hgt` is false and `cs_baro_hgt` is true, EKF2 has given up on the
  lidar and fallen back to the barometer — do not fly, the height datum is
  the baro and it drifts metres indoors.

- **Avoid:** reboot the FC at the start of every test session, and again after
  any flight where the flag tripped. Fly over flat, uniform floor — a mat, a
  cable, or a door threshold under the vehicle looks like vertical motion to
  the check and is a good way to trip it.

- **If it keeps tripping in flight:** the likely cause on a DroneCAN sensor
  like the ARK Flow is sensor lag. `EKF2_RNG_DELAY` (default 5 ms) is compared
  against the EKF's own `vz`; DroneCAN adds more latency than that, which
  produces a systematic innovation exactly when the vehicle is climbing or
  descending. Raise it (try 20–40 ms) and/or loosen `EKF2_RNG_K_GATE`.

Also worth knowing: `EKF2_MIN_RNG` is **not** a validity threshold. The
validity window comes from the sensor's own reported `min_distance` /
`max_distance` (0.02 m / 30 m on the ARK Flow). `EKF2_MIN_RNG` is the value
EKF2 *substitutes* when the lidar is unhealthy and the vehicle is at rest on
the ground — which is why a `dist_bottom` frozen at exactly that value means
"no measurement", not "9 cm".

### 10.2 Losing Offboard shortly after arming

Offboard's *static* mode requirements in PX4 (`mode_requirements.cpp`) are
angular velocity, attitude, and **offboard signal**. But the requirements are
also built **dynamically from the contents of `offboard_control_mode`**: when
that message has `position = true` — which these nodes set whenever they are
not in a blind descent — PX4 adds `local_position` to Offboard's requirements.
You can watch this happen live:

```bash
ros2 topic echo /fmu/out/failsafe_flags --once | grep mode_req_local_position
```

Bit 14 (value `16384`, `NAVIGATION_STATE_OFFBOARD`) appears in that bitmask
only while a node is publishing a position-flavoured `offboard_control_mode`.
So **an invalid local position absolutely can kick you out of Offboard**, and
`local_position_invalid` in the failsafe line is a cause to take seriously
rather than noise alongside `offboard_control_signal_lost`.

| flag in the new `PX4 failsafe:` log line | meaning | fix |
|---|---|---|
| `local_position_invalid` + `local_velocity_invalid`, **flickering on and off every 1–2 s while the vehicle sits still** | EKF2 has no yaw alignment (`cs_yaw_align` false), so the horizontal estimate is never anchored to a heading. Vision position can be fusing happily (`cs_ev_pos` true, `xy_valid` true) and this still bites — but only once **armed**, because the commander only enforces mode requirements then. | See 10.3. |
| `offboard_control_signal_lost` | No `OffboardControlMode` reached PX4 for `COM_OF_LOSS_T` (default **1.0 s**). Sometimes a stall in the uXRCE-DDS uplink — but it is also set as a *side effect* when PX4 drops Offboard for another reason, so do not stop reading at this flag. | Rule out 10.3 first. Then: run the node with `ros2 run`, not inside a busy launch; cut the number of `/fmu/out` topics being bridged; check the agent with `-v6` for dropped uplink; consider `COM_OF_LOSS_T` 1.5–2.0. |
| `manual_control_signal_lost` | RC link lost while armed. | Keep the TX on. If you deliberately fly without RC, set `COM_RCL_EXCEPT` bit 2 (value `4`) to exempt Offboard. |
| `gcs_connection_lost` | QGC/datalink dropped, `COM_DL_LOSS_T` expired. | `COM_DLL_EXCEPT`, or keep QGC connected. |

Why it lands rather than holds: `COM_OBL_RC_ACT` defaults to **0 = Position
mode**, and this airframe has no usable horizontal position estimate on the
ground, so Position mode is unavailable and PX4 escalates down to Land —
`nav_state -> AUTO_LAND(18)`. Set `COM_OBL_RC_ACT = 4` (Land) so the
behaviour is at least explicit and predictable rather than the result of a
fallback chain.

### 10.3 No yaw alignment: `cs_yaw_align` is false

**Symptom.** The vehicle arms, sits on the ground, and about a second later:

```
PX4 failsafe SET: local_position_invalid, local_velocity_invalid, offboard_control_signal_lost
nav_state -> POSCTL(2)
```

and after the node stands down the two position flags keep toggling on and off
every second or two while the vehicle has not moved at all.

**Cause.** EKF2 needs an **absolute heading** before a horizontal position
estimate means anything. The bridge declares its odometry as `POSE_FRAME_FRD`
— "z is down, my heading is offset from North by a constant I do not know" —
so EKF2 must learn that offset from another source. There are only three:

| source | flag | parameter |
|---|---|---|
| magnetometer | `cs_mag_hdg` / `cs_mag_3d` | `EKF2_MAG_TYPE` |
| vision yaw | `cs_ev_yaw` | `EKF2_EV_CTRL` **bit 3** (value 8) |
| GNSS yaw | `cs_gnss_yaw` | `EKF2_GPS_CTRL` |

Turn the magnetometer off for indoor flight (`EKF2_MAG_TYPE = 5`) **without**
also enabling vision yaw and all three are off, `cs_yaw_align` never latches,
and you get the symptom above. This is easy to walk into because everything
else looks healthy: vision really is being fused, `xy_valid` really is true,
and the node reports `ekf_fusing=True`. Nothing complains until you arm.

**Check it:**

```bash
ros2 topic echo /fmu/out/estimator_status_flags --once \
  | grep -E "cs_yaw_align|cs_ev_pos|cs_ev_yaw|cs_mag_hdg|cs_gnss_yaw"
```

`cs_yaw_align: false` is the answer. **The fix has two halves, and doing only
the first is the trap** — it leaves you with `cs_ev_yaw: true` and
`cs_yaw_align: false`, which looks like progress and is not.

**Half 1 — give EKF2 a yaw source.** For indoor vision flight:

```
EKF2_EV_CTRL  = 9    # 1 (horizontal position) + 8 (yaw)
EKF2_MAG_TYPE = 5    # None
```

**Half 2 — declare the vision frame as NED**, or half 1 does nothing:

```bash
ros2 launch drone_testing sequence_vio_test.launch.py pose_frame:=ned ...
```

EKF2 refuses to align yaw from a `POSE_FRAME_FRD` estimate *by construction* —
FRD means "my heading is offset from North by a constant I don't know", and a
frame like that cannot align anything to North:

```c
// ev_yaw_control.cpp, LOCAL_FRAME_FRD branch
resetQuatStateYaw(...);
_control_status.flags.yaw_align = false;   // explicitly false
_control_status.flags.ev_yaw    = true;
```

Only the `LOCAL_FRAME_NED` branch sets `yaw_align = true`. So FRD is a promise
that *something else* owns the heading — the magnetometer — and with
`EKF2_MAG_TYPE = 5` there is nothing else. Declaring NED is correct here
precisely *because* the magnetometer is off: nothing on the airframe knows
where North is, so the ZED's start-up heading may as well define it. The nodes
capture their own reference yaw at arming and fly relative to it, so the only
thing you lose is a meaningful compass rose in QGC.

| magnetometer | `EKF2_MAG_TYPE` | `pose_frame` | `EKF2_EV_CTRL` |
|---|---|---|---|
| on | `0` | `frd` | `1` (position only) |
| off | `5` | `ned` | `9` (position + yaw) |

Do not mix the rows. Re-check the flags and confirm `cs_yaw_align` **and**
`cs_ev_yaw` are both true before arming.

`offboard_sequence_vio` now refuses to arm while `cs_yaw_align` is false and
prints which sources are off, so this fails on the ground instead of a second
after arming.

---

## 11. Pre-flight checklist

1. Props **off** for the first run of any changed code.
2. **Reboot the flight controller.** `cs_rng_kin_consistent` is sticky across a
   whole power cycle and is the usual reason the node will not arm (10.1).
3. `ros2 topic echo /fmu/out/estimator_status_flags --once` → `cs_rng_hgt` and
   `cs_rng_kin_consistent` both true, `cs_baro_hgt` is *not* carrying the
   height on its own.
4. `ros2 topic echo /fmu/out/vehicle_local_position --once` → `z_valid` and
   `dist_bottom_valid` both true.
5. RC kill switch tested on the bench, this session.
6. `takeoff_altitude` set low (0.30 m).
7. Clear space around and above the vehicle — flow-only hold drifts.
8. You know which pane has the `q` key.
9. **Precision landing only:** the `mode:=bench` sign check has been run *this
   session*, on this airframe, and both axes read true (section 6d). A camera
   that has been unplugged and replugged can come back on a different index.
10. **Precision landing only:** `marker_id`, `marker_size` and `aruco_dict`
    match the marker actually laid out, and the marker is inside the capture
    basket for your `takeoff_altitude`.
11. **Room mission only (6g):** the whole plan has been read back from the
    `ROOM MISSION:` startup line and is the one you meant, the room is at
    least `standoff_distance` deep from the window wall, and you have stood
    where the moves end and confirmed `/window_detected` from there.
12. **Room mission only:** `cam_x/cam_y/cam_z/cam_roll/cam_pitch/cam_yaw` are
    the measured mounting and are the **same numbers** on the flight node and
    the doll node.
13. **Doll counting only:** the venv answers
    `~/venvs/dolls/bin/python -c "import torch, ultralytics; print(torch.cuda.is_available())"`
    with `True`, and the run log says `Doll model ready.` rather than
    `Cannot load …` — or you have accepted that the count will be zero.
