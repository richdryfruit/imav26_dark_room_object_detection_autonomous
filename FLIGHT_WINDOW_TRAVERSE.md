# Flight day: window traversal on the D435i + PMW3901 airframe

Everything that has to be true before `window_traverse` flies at a hole, and
the order to do it in. Bench stages A–F are in **[BRINGUP.md](BRINGUP.md)** —
this picks up where those end and does not repeat them.

**Read section 1 first.** Several defaults in this package were measured on
the *old* airframe (ZED + ARK Flow) and are wrong for yours. Two of them will
fly the aircraft into a wall if left alone.

---

## 1. Values you MUST change — these are not safe defaults

### 1.1 The camera mounting — WRONG on your airframe right now

| parameter | current default | why it's wrong |
|---|---|---|
| `cam_x` | `0.105` | measured to the **ZED**, on the old mount |
| `cam_y` | `0.0` | ditto |
| `cam_z` | `-0.04` | ditto |
| `cam_roll` / `cam_pitch` / `cam_yaw` | `0.0` | assumes the camera points dead ahead, perfectly level |

These are the **single most dangerous** placeholders in the package. Every
window position is built by rotating the camera-frame measurement into NED
through these numbers. A wrong `cam_pitch` at 3 m range puts the window half a
metre off vertically; a wrong lever arm turns every yaw into a phantom
sideways step.

**Measure them in the body frame, ROS convention — x forward, y LEFT, z UP,
metres and radians — to the D435i's LEFT IMAGER**, not the middle of the case.
That is where librealsense puts the optical frame origin. On a D435i the left
imager is the one nearest the USB-C connector end; the datasheet gives its
offset from the case datum.

`cam_pitch` positive = **nose down**. If the camera is tilted down 10°, that's
`cam_pitch:=0.1745`.

> **Verify by walking the estimate (BRINGUP §F) before you fly.** Carry the
> airframe around in front of the window with props off and watch
> `/window_pose`. The centre must sit still in NED to within a few cm while
> the airframe moves. That test exists precisely to catch these six numbers,
> and it is the only one that will.

### 1.2 The airframe dimensions — also from the old aircraft

| parameter | current | what it means |
|---|---|---|
| `drone_width` | `0.260` | widest point, **prop tip to prop tip** |
| `drone_height` | `0.260` | total, bottom of gear to highest point |
| `gear_below_camera` | `0.120` | camera down to the bottom of the landing gear |

These decide whether an aperture is declared flyable and how high above the
sill the aircraft aims. The comment in `window_traverse.py` records why they
exist: flying the vehicle origin at the window centre *put the landing gear on
the sill and tipped the aircraft over*. Get a tape measure on your actual
airframe.

### 1.3 The window colour — the traverse launch default is `red`

| launch file | `color` default |
|---|---|
| `window_scan.launch.py` | `green` |
| `window_traverse.launch.py` | **`red`** |

This inconsistency pre-dates the RealSense port; both are just defaults nobody
reconciled. **Pass `color:=` explicitly every time** so you are never relying
on which file you happened to launch.

### 1.4 `min_area` — 1500 px² is far too permissive

At 1280×720 that is a 39 px square. It gave junk detections on a bench
(`area=312px dist=18.64m`). Tune it in BRINGUP §C against your real window at
the range you will fly, then pass the value you found. Start around
`min_area:=6000`.

### 1.5 `scan_span_deg` — the launch file and the node disagree

`window_traverse.py` sets `SCAN_SPAN = 0.0` and its comment says the launch
file passes 0 — **it passes `20.0`**. So launching via the launch file gives
you a 20° yaw sweep, and running the node bare gives you none. Pre-existing,
not introduced by the port.

Decide deliberately and pass it: `scan_span_deg:=0` if you point the aircraft
at the window before arming (recommended for the first flights — a sweep is
one more thing to go wrong), or a real value if you need to search.

### 1.6 Sizes and envelope — check against your actual arena

| parameter | default | check |
|---|---|---|
| `window_min_size` / `window_max_size` | `0.35` / `3.0` m | believable aperture. Narrow these to bracket your real window — it is a free outlier filter |
| `min_altitude` / `max_altitude` | `0.4` / `3.0` m | the flight envelope. `max_altitude` must clear your window top |
| `takeoff_altitude` | `1.2` m | must put the camera near the window centre height |
| `altitude_offset` | `0.0` | add here if the detected quad sits off-centre on the frame |

### 1.7 `standoff_distance` — check it against YOUR window

Default is `2.0` m. The D435i needs **`1.26 × window height`** just to fit the
aperture in frame:

| window height | minimum | 2.0 m gives |
|---|---|---|
| 1.0 m | 1.26 m | 1.59× ✅ |
| 1.2 m | 1.51 m | 1.32× ✅ |
| 1.5 m | 1.89 m | 1.06× — the node pushes it out itself |

`effective_standoff()` recomputes this at runtime from the aperture it
measures and logs when it moves the approach point. If it warns it hit
`max_standoff_distance` (4.0 m), your window is too big for this lens.

### 1.8 Speeds — start slower than the defaults

`approach_speed` 0.30 and `traverse_speed` 0.45 were flown on the **ARK Flow**.
The PMW3901 is a coarser sensor. For first flights halve them
(`approach_speed:=0.15 traverse_speed:=0.25`) and work up over successive
flights, one change at a time.

Similarly `align_tolerance` (0.18 m) is only as tight as the PMW3901's noise
floor allows. If ALIGN keeps timing out just outside tolerance, **loosen the
tolerance — do not raise the gain.**

---

## 2. PX4 parameters

Set in QGC. The flow sensor and the rangefinder are **two separate devices**
now; on the ARK Flow they were one, so this is two ways to fail, not one.

| parameter | value | why |
|---|---|---|
| `SENS_EN_PMW3901` | `1` | flow driver (SPI) |
| `EKF2_OF_CTRL` | `1` | fuse optical flow |
| `EKF2_OF_QMIN` | tune | PMW3901 reports a **coarser quality** number than the ARK Flow's PAW3902. **If the flow never latches, this is the first parameter to look at.** |
| `SENS_EN_TFMINI` | `1` | TFmini Plus |
| `SENS_TFMINI_CFG` | *your port* | which serial port it is on |
| `EKF2_RNG_CTRL` | `1` | fuse rangefinder — **hard arming gate** |
| `EKF2_HGT_REF` | `2` | height reference is the rangefinder |
| **`EKF2_EV_CTRL`** | **`0`** | **no external vision.** A leftover value from a VIO experiment shows up as `local_position_invalid` ~1 s after arming and looks like an Offboard bug |
| `EKF2_MAG_TYPE` | `0` | magnetometer **ON**. This flight has no vision to supply heading, so the 10.3 yaw-alignment trap only bites if you turn the mag off |
| `COM_OBL_RC_ACT` | `4` (Land) | makes the Offboard-loss behaviour explicit instead of a fallback chain |
| `COM_OF_LOSS_T` | `1.0`–`2.0` | raise only if you see `offboard_control_signal_lost` |
| `UXRCE_DDS_CFG` | your TELEM port | |
| `SER_TEL2_BAUD` | `921600` | must match the agent |

**Reboot the flight controller at the start of every session.**
`cs_rng_kin_consistent` is sticky — it can only go false in flight and only
recover in flight, so one bad run poisons the whole power cycle. The launch
files take `reboot_fc:=true`.

---

## 3. Gates that must pass before the props go on

Do not skip these. Each is in BRINGUP.md with the expected output.

- [ ] **§B** camera alone: `color/image_raw` is `rgb8`, `aligned_depth_to_color/image_raw` is `16UC1`
- [ ] **§C** detection tuned: locks on the real window, `area` in the thousands, **steady** `dist`
- [ ] **§D** `/window_geometry` depths in metres, four corners agreeing to a few cm
- [ ] **§E** `z_valid` and `dist_bottom_valid` both true; `cs_rng_hgt` and `cs_rng_kin_consistent` both true; `cs_ev_pos` **false**
- [ ] **§F** `/window_pose` centre stays put in NED while you carry the airframe — **this is the cam_* check**
- [ ] **§F** `/window_pose` width/height match a tape measure (expect ~2 % under; the detector pads corners 5 px inwards)
- [ ] **§G** a detection-only flight (`window_scan`) has climbed, held, latched flow (`POS` in `/takeoff_status`, not `FLO` or `---`), seen the window, and landed

> **If §G has not been flown, do not fly §H.** The traversal shares all its
> arming, climb and landing code with the scan; flying the scan first proves
> that half with nothing pointed at a hole.

Also confirm exactly one camera process before every launch — a second one
loops on `RS2_USB_STATUS_BUSY` *while the detector keeps working off the
first*, which reads like a camera fault and is not:

```bash
pkill -f realsense2_camera_node; pkill -f window_detect
ps -eo pid,etimes,cmd | grep [r]ealsense2_camera_node    # want no output
```

---

## 4. Space you need

```
standoff_distance + exit_distance  =  2.0 + 1.5  =  3.5 m
```

along the approach line, **plus** the `align_tolerance` basket either side,
**plus** room to land on the far side — it lands where it ends up, not where
it started. Clear floor matters more than usual: a flow-only lateral move
needs texture under it the whole way.

---

## 5. The flight

### Terminal 1 — support stack

```bash
cd ~/imav26-ws-2/ws_ros2 && source install/setup.bash

ros2 launch drone_testing window_traverse.launch.py \
  reboot_fc:=true \
  lcd:=false \
  color:=green \
  min_area:=6000 \
  cam_x:=<measured> cam_y:=<measured> cam_z:=<measured> \
  cam_roll:=<measured> cam_pitch:=<measured> cam_yaw:=<measured>
```

Wait for `RealSense Node Is Up!`, `CameraInfo from ...`, `Set parameter
successful`, and the agent's `running...`. **There should be zero
"Parameter ... is not supported" warnings** — if you see them, you are running
a stale build.

### Terminal 2 — the flight node, by hand

```bash
cd ~/imav26-ws-2/ws_ros2 && source install/setup.bash

ros2 run drone_testing window_traverse --ros-args \
  -p takeoff_altitude:=1.2 \
  -p standoff_distance:=2.0 \
  -p scan_span_deg:=0.0 \
  -p approach_speed:=0.15 \
  -p traverse_speed:=0.25 \
  -p drone_width:=<measured> -p drone_height:=<measured> \
  -p gear_below_camera:=<measured> \
  -p cam_x:=<measured> -p cam_y:=<measured> -p cam_z:=<measured> \
  -p cam_roll:=<measured> -p cam_pitch:=<measured> -p cam_yaw:=<measured>
```

> **Pass the same `cam_*` numbers to both.** The launch file feeds one set to
> the detector and the flight node so they cannot disagree; run the flight
> node by hand and you must repeat them yourself.

> **Always `ros2 run` in its own pane.** A node started by `ros2 launch` has no
> tty and the `q`/`k` aborts are dead.

| key | effect |
|---|---|
| `q` | abort into a controlled descent |
| `k` | force-disarm — **motors cut, the aircraft drops** |

**Your RC kill switch is the real safety net.** Flipping out of Offboard also
makes the node stand down.

### Terminal 3 — watch

```bash
ros2 topic echo /takeoff_status     # STAGE|ARM|alt|xy-mode|detail
ros2 topic echo /window_pose        # x|y|z|yaw|w|h|samples|age, NED
```

Browser: `http://<jetson-ip>:8080/`

---

## 6. What you should see

```
climb → hold → SCAN → LOCK → AIM → ALIGN → TRAVERSE → CLEAR → land
```

| stage | `detail` | means |
|---|---|---|
| `AIM` | `aim24` | 24° left to turn onto the window normal |
| `ALIGN` | `algn0.42` | 0.42 m to the approach point |
| `TRAVERSE` | `thru1.8/3.1` | 1.8 m flown of 3.1 m |
| `CLEAR` | `3s` | far-side hold remaining |

**Watch the `xy-mode` field.** `POS` means flow has latched a position hold.
`FLO` or `---` means it has not, and horizontal motion will be refused.

**AIM turns before it translates on purpose** — a yaw while translating is the
surest way to lose an estimate.

---

## 7. When it goes wrong

| situation | behaviour |
|---|---|
| vision lost **before** `TRAVERSE` | holds, then abandons into a landing |
| vision lost **during** `TRAVERSE` | **pushes on open-loop** along the committed heading for `blind_traverse_seconds` (3.0), then lands. Deliberate — stopping inside an aperture is worse than finishing the run |
| never leaves `LOCK` | no usable pose. **Read the rejection tally the node logs** — it names the test that rejected the samples |
| approach won't settle | `align_timeout` (60 s) → lands |
| anything else | `flight_seconds` (150 s) from start of climb forces a descent |

**An abandoned attempt lands. It does not retry** — deliberately: it would be
somewhere else, with unknown battery, under the same conditions that just
failed. Land, read the tally, fly it again.

### Reading the rejection tally

| tally says | means |
|---|---|
| "failed the planarity test" | sample boxes landing on the wall behind the frame |
| "corner depth missing" | depth map has holes where the frame is — try `spatial_filter:=true`, and re-check the reconstructed size afterwards |
| "corner depths disagree" | one corner reading the background |
| "guessing the FOV" | `camera_info_topic` wrong — fix the topic, don't tune `fallback_hfov_deg` |

---

## 8. Build the confidence up, not in one go

1. `scan_span_deg:=0`, aircraft pointed at the window, half speeds.
2. Get **one clean flight** and read `/window_pose` from the log against a tape measure.
3. Then raise the speeds — one change per flight.
4. Then add the sweep back, if you need it.

Do not change two things between flights. When something goes wrong you want
to know which one did it.

---

## 9. Still not verified by me

Honest limits on everything above: **stages A–F were run on this Jetson with
the real D435i. Sections 5–7 of this document have never been flown** — no
flight controller was connected when the port was written, so everything past
`Waiting for VehicleStatus from PX4...` is derived from the code, not
observed. The PX4 parameter table is from the PX4 docs and the existing
README, not from a successful arm on this airframe.

Treat the first flight accordingly: props off for the first run of the changed
code, low altitude, hand on the kill switch.
