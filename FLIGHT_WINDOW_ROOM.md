# Flight day: window → room → window, with geotagged doll counting

Everything that has to be true before `window_room_traverse` flies through a
hole, runs a box pattern inside, and comes back out through the same hole.

This document **does not repeat
[FLIGHT_WINDOW_TRAVERSE.md](FLIGHT_WINDOW_TRAVERSE.md)**. The mission is that
one, flown twice, with a pattern in between — `WindowRoomTraverse` is a
subclass of `WindowTraverse` and the sweep, the lock, the estimator, the
approach, the airframe-clearance arithmetic and every PX4 gate are that node's
code, unchanged. Read it first and do section 1 of it (the camera mounting,
the airframe dimensions, the colour, `min_area`, the speeds). Bench stages A–F
are in **[BRINGUP.md](BRINGUP.md)**.

**Then read section 2 of this file.** There are three new ways for this
mission to fail that the single traversal does not have, and two of them are
about the size of the room rather than about the code.

---

## 1. What it flies

```
climb → hold → SCAN → LOCK → (RECENTRE) → AIM → ALIGN → TRAVERSE
     → CLEAR            ... 1.20 m inside the room, holding
     → ROOM_MOVE  0.30 m LEFT
     → ROOM_MOVE  0.30 m FORWARD
     → ROOM_TURN  90° RIGHT
     → ROOM_MOVE  0.30 m straight ahead (on the NEW heading)
     → ROOM_TURN  90° RIGHT          ... now facing the window wall
     → ROOM_MOVE  0.30 m straight ahead
     → RELOCK     stand still, rebuild the window pose from inside
     → (RECENTRE) → AIM → ALIGN → TRAVERSE → CLEAR → land
```

Every leg settles (`ROOM_HOLD`, 2 s) before the next starts, so errors do not
compound down the pattern.

**Doll detection runs from the inbound `TRAVERSE` commit to the outbound
`CLEAR`** — the whole time the aircraft is on the room side of the wall, and
not a second of the approach or the landing.

### The three things that are genuinely new code

Everything else is inherited. These three had to be written, and each one is a
thing that would have silently broken a copy-paste of the single traversal:

1. **The estimator is cleared at `RELOCK`.** The window normal is always
   resolved to point *back at the aircraft*, so from inside the room the same
   physical window has a normal 180° from the one on file, and the innovation
   gate would refuse every return sample as `normal swung`. Clearing the
   buffer is what makes the second approach possible at all.
2. **The yaw cone is re-centred** on the heading the pattern ends at. The
   inherited rule — "refuse to believe in, or turn towards, anything more than
   `yaw_cone_deg` off the heading you armed on" — is correct going in and
   exactly backwards coming out. Same width, right axis. The return leg is
   still protected against locking onto a doorway behind it.
3. **Room moves are in the CURRENT heading frame**, not the takeoff frame
   `OffboardSequence`'s own steps use. "Go straight" after "yaw 90 right" has
   to mean the new straight, or the pattern is a dog-leg.

---

## 2. Values you MUST check — the room-specific ones

Section 1 of FLIGHT_WINDOW_TRAVERSE.md still applies in full. These are on
top of it.

### 2.1 Room depth — `standoff_distance` is the number that bites

The pattern itself is small. **The return approach is not.**

```
pattern ends ≈ 0.3 m left of, 0.3 m behind the entry point,
               ≈ inside_distance − room_forward_3 ≈ 0.90 m from the wall

return approach flies to standoff_distance IN FRONT of the window,
               i.e. 2.00 m INTO the room  ← the deepest point of the flight
```

So the room must be at least **`standoff_distance` + the airframe** deep,
measured from the window wall, or `ALIGN` backs the aircraft into a wall it
cannot see. There is no obstacle avoidance on this airframe.

`effective_standoff()` can push that number *further out* at run time if the
aperture is big — a 1.3 m window on the D435i's 43° vertical FOV needs more
than 2.0 m to stay fully in frame. Read the standoff the node logs, don't
assume the parameter.

**Pace it out with the props off before you fly it.** If the room cannot give
that depth, lower `standoff_distance`, and understand what it costs: the
return approach loses its view of the whole window, which is when
`TRUNCATED` samples and `RECENTRE` start happening a metre from a wall. A room
that is genuinely too small is what `return_through_window:=false` is for.

### 2.2 The window must be visible from where the pattern ends

`RELOCK` **does not sweep**. It stands still on whatever heading the pattern
left it and rebuilds the pose. Yawing to search from a metre off a wall is how
it finds a doorway instead of the window.

> **The one pre-flight check with no fallback.** Stand where the pattern ends
> — about 0.3 m left of and 0.3 m behind the entry point, facing back at the
> window wall — hold the airframe at flight altitude, and confirm
> `ros2 topic echo /window_detected` goes `true` and `/window_pose` fills in.

If it doesn't, the aircraft lands in the room after `relock_timeout` (45 s).
That is the designed failure, and it is a safe one, but it is not a mission.
Shorten `room_forward_3` (the leg flown *towards* the wall) or lengthen
`inside_distance` until the whole aperture is in frame from there.

### 2.3 `doll_merge_radius` — what makes two sightings one doll

`0.60` m by default. A detection lands on an existing doll if it is within
this distance of it **in the room**, not in the image.

It must be:

* **smaller** than the smallest gap between two real dolls in your arena — or
  two dolls merge into one and the count reads low;
* **bigger** than the position error — or one doll seen from two sides counts
  twice.

Measure the gap in the actual arena. If the dolls are closer together than
about a metre, lower it, and expect the count to be more sensitive to depth
noise. At 3 m range the error is dominated by depth noise and the flow's own
drift, not by the pixel.

### 2.4 `inside_distance` — how far in "through the window" means

`1.20` m past the window plane. It decides three things at once: how far into
the room the aircraft commits, how much room the pattern needs, and — with
`room_forward_3` — how far from the wall `RELOCK` looks at the window from.
Changing it changes all three.

### 2.5 `flight_seconds` — 300, not 150

Two approaches, two traversals and a six-leg pattern do not fit in the
single-traversal clock. The launch default is already 300; if you run the node
by hand, pass it, or the hard clock lands the aircraft mid-pattern.

---

## 3. The Arduino

**Flash `arduino/room_status/room_status.ino`.** Arduino IDE, libraries
`MCUFRIEND_kbv` (David Prentice) and `Adafruit GFX`. Wiring is the shield on
D2–D9 + A0–A4 and the USB cable to the Jetson — nothing else.

> The older `arduino/tft_status/tft_status.ino` will **not** work with this
> mission. Different row count and an extra `D:` key for the counts; the two
> sketches are not protocol-compatible. `lcd_status` (the node that drives the
> old sketch) is pinned OFF in this launch file so it cannot fight
> `room_display` for the same serial port.

What the screen shows:

| area | content |
|---|---|
| banner | the state: `SEARCHING` `WINDOW` `ENTERING` `IN ROOM` `ROOM RUN` `FIND WIN2` `WINDOW 2` `EXITING` `OUTSIDE` `LANDING` |
| banner colour | 0 grey idle, 1 **green** — a milestone reached, 2 amber — in transit, 3 red — alarm |
| big number | **cumulative dolls counted** this flight |
| small number | dolls visible in the current frame |
| 3 rows | stage + detail, altitude + armed, xy mode + detector state |

Glance at the colour: green means it has *got somewhere* (window found,
inside, window found again, outside). Amber means it is still moving between
those. `NO LINK` in red means the Jetson stopped talking — the last known
total stays on screen underneath it.

---

## 4. Build

```bash
cd ~/imav26-ws-2/ws_ros2 && source /opt/ros/jazzy/setup.bash
colcon build --packages-select drone_testing --symlink-install
```

---

## 5. The flight

The same two-pane shape as the single traversal, for the same reason: a node
started by `ros2 launch` has no tty and the `q`/`k` aborts are dead.

`agent_only` defaults to `true`, which starts the support stack **plus the
doll node and the display** and leaves the flight node to you.

### Terminal 1 — support stack, doll model, screen

```bash
cd ~/imav26-ws-2/ws_ros2 && source install/setup.bash

ros2 launch drone_testing window_room_traverse.launch.py \
  color:=blue \
  min_area:=6000 \
  cam_x:=<measured> cam_y:=<measured> cam_z:=<measured> \
  cam_roll:=<measured> cam_pitch:=<measured> cam_yaw:=<measured> \
  doll_merge_radius:=0.60 \
  reboot_fc:=false
```

This includes `window_traverse.launch.py` with `agent_only:=true`, so the
agent, the RealSense (colour + **aligned** depth), the emitter parameter and
`window_detect` all come from the file that already works — there is one
definition of each and the two cannot drift apart.

Wait for all of:

```
RealSense Node Is Up!
CameraInfo from /camera/camera/color/camera_info ...
Set parameter successful          (the emitter)
... running...                    (the agent)
Room TFT connected on /dev/ttyACM0 @ 115200
doll_detect: ... Waiting for doll_detect_enable.
```

**Zero "Parameter ... is not supported" warnings.** If you see them you are
running a stale build.

### Terminal 2 — the flight node, by hand

```bash
cd ~/imav26-ws-2/ws_ros2 && source install/setup.bash

ros2 run drone_testing window_room_traverse --ros-args \
  -p takeoff_altitude:=1.2 \
  -p standoff_distance:=2.0 \
  -p inside_distance:=1.20 \
  -p outside_distance:=1.20 \
  -p scan_span_deg:=0.0 \
  -p approach_speed:=0.15 \
  -p traverse_speed:=0.25 \
  -p room_strafe:=0.30 \
  -p room_forward_1:=0.30 -p room_turn_1_deg:=90.0 \
  -p room_forward_2:=0.30 -p room_turn_2_deg:=90.0 \
  -p room_forward_3:=0.30 \
  -p room_speed:=0.20 \
  -p flight_seconds:=300.0 \
  -p drone_width:=<measured> -p drone_height:=<measured> \
  -p gear_below_camera:=<measured> \
  -p cam_x:=<measured> -p cam_y:=<measured> -p cam_z:=<measured> \
  -p cam_roll:=<measured> -p cam_pitch:=<measured> -p cam_yaw:=<measured>
```

> **Pass the same `cam_*` numbers to both panes.** The launch file feeds one
> set to the detector *and* the doll node so they cannot disagree; run the
> flight node by hand and you repeat them yourself. Wrong here misplaces the
> window **and** every doll by the same offset, which merges dolls that are
> not the same one.

> **The node still calls itself `window_traverse` when you run it bare.** The
> name is hard-coded in the base class's constructor and this mission does not
> touch that file. It is the right node — check the `ROOM MISSION:` banner it
> prints at startup. The launch file gives it its own name.

| key | effect |
|---|---|
| `q` | abort into a controlled descent |
| `k` | force-disarm — **motors cut, the aircraft drops** |

**Your RC kill switch is the real safety net.** Flipping out of Offboard also
makes the node stand down.

### Terminal 3 — watch

```bash
ros2 topic echo /mission_phase      # PHASE|STAGE|detail
ros2 topic echo /takeoff_status     # STAGE|ARM|alt|xy-mode|detail
ros2 topic echo /window_pose        # x|y|z|yaw|w|h|samples|age, NED
ros2 topic echo /doll_count         # cumulative, never decreases
ros2 topic echo /dolls_visible      # this frame
ros2 topic echo /doll_report        # n|id:x,y,z|id:x,y,z|...  ← the evidence
```

Browser: `http://<jetson-ip>:8080/`

---

## 6. The parameters worth your attention

### Flight

| parameter | default | why you'd change it |
|---|---|---|
| `cam_x/y/z`, `cam_roll/pitch/yaw` | `0.105`/`0`/`-0.04`/`0,0,0` | **Measure them.** See FLIGHT_WINDOW_TRAVERSE §1.1 — placeholders from the old ZED mount |
| `inside_distance` | `1.20` | how far into the room the inbound run goes |
| `outside_distance` | `1.20` | how far past the plane the outbound run ends |
| `standoff_distance` | `2.0` | needs that much clear depth **inside** the room — §2.1 |
| `room_strafe` / `room_forward_1..3` | `0.30` | the box pattern |
| `room_turn_1_deg` / `room_turn_2_deg` | `90.0` | **positive = RIGHT.** Both turns right leaves the nose 180° from entry, which is what `RELOCK` needs |
| `room_speed` | `0.20` | m/s inside. The legs are 30 cm; faster only buys overshoot |
| `room_hold_seconds` | `2.0` | settle between legs |
| `relock_timeout` | `45.0` | s looking for the window from inside before landing in the room |
| `relock_settle_seconds` | `2.0` | s of holding still before the estimate is believed at all — the pattern ends with a turn, and a turn smears both flow and depth |
| `yaw_cone_deg` | `50.0` | the cone's **width**, applied both times; its centre moves at `RELOCK` |
| `flight_seconds` | `300.0` | hard clock from the start of the climb |
| `return_through_window` | `true` | `false` = fly in, do the pattern, land inside |
| `min_area` | `1500` | raise to ~6000 if the detector picks up blue junk |
| `color` | `blue` | the window |

### Dolls

| parameter | default | why you'd change it |
|---|---|---|
| `doll_model` | `~/Downloads/DroneImpl_v7/db.engine` | the TensorRT engine. Built for **this** Jetson and this TensorRT version — an engine copied from another machine will not load |
| `doll_tracker` | `~/Downloads/DroneImpl_v7/custom_bytetrack.yaml` | ByteTrack config |
| `doll_merge_radius` | `0.60` | §2.3 — the number that decides the count |
| `doll_confidence` | `0.55` | detections below this never reach the counter |
| `doll_min_frames` | `5` | frames a track must survive before it can be counted. Kills flickery false positives |
| `doll_max_fps` | `6.0` | inference cap. A doll does not move; the flight node's 20 Hz setpoint timer must not be starved |
| `doll_depth_patch` | `0.30` | fraction of the box the depth median is taken over. The middle of a doll is doll; the edges are the floor behind it |
| `doll_publish_image` | `false` | annotated frames on `/doll_image`. Bench only — it costs the CPU margin |
| `require_enable` | `true` | `false` = run the model always, which is how you bench it |

---

## 7. How the counting actually works, and why

`jw_px4_arduinocount.py` identified a doll by **where it was in the image** — a
ByteTrack id, plus a pixel-distance fallback when the tracker dropped it.
That is the right answer for a fixed camera. On this mission the aircraft
**yaws 90° twice**, so the same doll leaves the frame on one side and comes
back at a completely different pixel, and every time it does, that logic mints
a new doll.

This node identifies a doll by **where it is in the room**:

```
pixel + depth  →  camera frame   (pinhole, intrinsics from CameraInfo)
camera frame   →  body FRD       (cam_x/y/z, cam_roll/pitch/yaw)
body FRD       →  local NED      (VehicleAttitude quaternion + position)
```

— the same three transforms `window_traverse` uses for the window, with the
same parameter names, so one launch file configures both. A detection lands on
an existing doll if it is within `doll_merge_radius` of it. Turn round twice,
fly back past the same doll, and it is still doll #3, because it is still in
the same corner of the room. **That is what the id is: a room position**, and
`/doll_report` publishes it as one so the count can be checked against the
room afterwards instead of taken on trust.

Three independent things have to be true before the total goes up:

1. confidence ≥ `doll_confidence`;
2. the track has survived `doll_min_frames` frames;
3. it has a **position** — and it is further than `doll_merge_radius` from
   every doll already counted.

**A box with no usable depth is dropped, not guessed.** A doll placed at an
assumed range lands somewhere arbitrary in NED, and that is the one input that
could genuinely corrupt the count. It still shows as "visible"; it just cannot
be counted until depth comes back. Same for a frame with no valid vehicle
pose. The track keeps its hits, so it is counted on the first frame that *can*
place it.

The total is monotonic by construction: nothing removes a doll or decrements
anything.

---

## 8. What you should see

| stage | `detail` | means |
|---|---|---|
| `AIM` | `aim24` | 24° left to turn onto the window normal |
| `ALIGN` | `algnX+0.04` | 4 cm of cross-track error |
| `TRAVERSE` | `thru1.8/3.2` | 1.8 m flown of 3.2 m |
| `CLEAR` | `3s` | far-side hold remaining — then the pattern, not a landing |
| `ROOM_MOVE` | `2/6 for0.12` | leg 2 of 6, forward, 12 cm to go |
| `ROOM_TURN` | `3/6 yaw47` | leg 3 of 6, 47° of setpoint left to walk |
| `ROOM_HOLD` | `3/6 1s` | settling |
| `RELOCK` | `relock 12s` | 12 s spent looking for the window from inside |

`/mission_phase` carries the coarse version the screen shows:
`OUTSIDE → SEARCH → WINDOW_IN → ENTERING → INSIDE → ROOM → SEARCH_OUT →
WINDOW_OUT → EXITING → OUT → LANDING`.

**Watch the `xy-mode` field.** `POS` means flow has latched a position hold.
`FLO` or `---` means it has not, and horizontal motion will be refused — the
room legs will wait, then skip.

In the log, the lines that matter:

```
TRAVERSE: committed. ...                        ← inbound commit
DOLL DETECTION ON: committed to the inbound traverse.
INSIDE: holding, 2.1 s to the room pattern.
ROOM: inside and 1.20 m past the window. Flying the 6-leg pattern ...
Room leg 3/6 (right): done, heading -87 deg
DOLL 1 counted at (+2.31, -0.88, -1.05) NED. Total 1.
ROOM pattern complete: left 0.30 -> done, 0.29 m of 0.30 m; ...
RELOCK: facing -178 deg, which is now the centre of the 50 deg yaw cone. ...
RELOCK: window found from inside. window at ... 
DOLL DETECTION OFF: cleared the window on the way out.
FINAL DOLL COUNT: 3. Positions (NED, m): 1:2.31,-0.88,-1.05|...
```

---

## 9. When it goes wrong

Everything in FLIGHT_WINDOW_TRAVERSE §7 still applies to both traversals.
These are the new ones:

| situation | behaviour |
|---|---|
| flow never latches during a room leg | waits `move_latch_timeout`, then **skips that leg** and carries on. The pattern completes short; `RELOCK` may then not see the window |
| flow lost mid-leg | stops where it is, marks the leg `ABANDONED`, carries on to the next |
| a room leg times out | `room_move_timeout` (20 s) / `room_turn_timeout` (25 s) — accepts wherever it got to and moves on |
| `RELOCK` never finds the window | `relock_timeout` (45 s) → **lands inside the room**. This is the designed failure and it is safe; it is not a mission |
| the return approach is abandoned | lands from wherever it is — inside the room |
| the engine will not load | logged once, doll detection off for the rest of the flight, **flight unaffected**, count reads 0 |
| no `CameraInfo` | dolls cannot be placed, so none are counted. Logged every 10 s |
| the Arduino is absent or unplugged | logged every 5 s, everything else runs |
| anything else | `flight_seconds` (300 s) from the start of the climb forces a descent |

**An abandoned attempt lands. It does not retry.** Same reasoning as the
single traversal: it would be somewhere else, with unknown battery, under the
conditions that just failed.

Note what is *not* on this list: nothing in the doll pipeline can abort,
delay, or steer the flight. The flight node publishes a Bool and forgets about
it.

---

## 10. Build the confidence up, not in one go

1. **Bench the model, no flight.**
   `ros2 launch drone_testing window_room_traverse.launch.py flight:=false
   require_enable:=false doll_publish_image:=true` — walk a doll around in
   front of the camera and watch `/doll_count` and `/doll_image`. Carry it out
   of frame and back in from the other side: the count must **not** go up.
2. **Pace the room out** with the props off. §2.1 and §2.2 — both of them, at
   flight altitude, with the airframe in your hands.
3. **Fly in only.** `-p return_through_window:=false`. Half speeds. This tells
   you whether the traversal and the pattern fit the room.
4. **Read `/doll_report` from that flight against the actual room** with a
   tape measure. The positions are the count; if they are wrong, the count is
   wrong even when the number looks right.
5. **Then the return leg**, still at half speeds.
6. Then raise the speeds — one change per flight.

Do not change two things between flights. When something goes wrong you want
to know which one did it.

---

## 11. Still not verified

Honest limits. **None of this has been flown.** What has actually been run on
this Jetson: the package builds; all three new nodes start, declare their
parameters, publish their topics and shut down cleanly; the launch file
resolves and its arguments are correct; the geotagging de-duplication was
exercised directly (two detections 0.3 m apart → one doll, a third 2 m away →
two dolls). Everything past `Waiting for VehicleStatus from PX4...` is derived
from the code, not observed — no flight controller was connected when this was
written, and the doll model was never loaded, because loading a TensorRT
engine on a Jetson with no camera streaming tells you nothing.

Specifically **unverified and worth watching on the first flight**:

* whether the pattern's two 90° turns actually leave the window in frame —
  §2.2 is the check that substitutes for having flown it;
* whether clearing the estimator at `RELOCK` is *sufficient*, i.e. whether the
  second pose builds as fast from a metre away as the first did from across
  the room. It has less of the window in frame and worse depth geometry;
* the CPU margin with the model running. The doll node is capped at 6 Hz and
  the setpoint timer is on its own thread, but `offboard_control_signal_lost`
  about a second after arming is this stack's characteristic failure, and this
  mission adds a TensorRT engine to the same Jetson. Watch for it, and if you
  see it, drop `doll_max_fps` first and `publish_image:=false` second.

Treat the first flight accordingly: props off for the first run of the changed
code, low altitude, hand on the kill switch.
