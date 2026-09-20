#!/usr/bin/env python3
"""
A headless flight of mission_fsm, with PX4 and the cameras faked.

    python3 tools/fsm_sim.py            # run the scenario, print the timeline
    python3 tools/fsm_sim.py --verbose  # ...and every stage's setpoint

Exits non-zero if the mission does not reach DISARMING having visited the
stages it is supposed to, so it is usable as a regression check.

WHAT THIS IS, AND WHAT IT IS NOT
--------------------------------
PX4 SITL is not installed on this machine, so there is no vehicle, no EKF2
and no Gazebo. This substitutes the thinnest possible stand-in for all three
and runs THE REAL mission_fsm node against it:

    a kinematic vehicle    integrates the node's own TrajectorySetpoint and
                           feeds the result back as VehicleLocalPosition, so
                           the control loop is genuinely closed
    a command responder    answers VEHICLE_CMD_DO_SET_MODE and ARM_DISARM,
                           so the arming handshake is the real one
    a window               the scaled arena's actual dark room, projected
                           into the camera each tick and published as
                           /window_geometry in window_detect's exact layout

It therefore tests the part that is mine -- the stage machine, the leg
dispatch, the frames, the strafe arithmetic, the doll gating, the handover
into and out of the inherited window mission. It tests NOTHING about the
PX4 controller, EKF2, the optical flow, the depth camera or the physics, and
a pass here is not evidence that any of those work.

It is a bench check. It stands in the same relation to a flight as
`precision_land mode:=bench` does.

THE WORLD
---------
Taken from src/drone_testing/imav2026_scaled.sdf, whose arena is the real one
scaled by 2.2. The dark room:

    floor centre    (-4.95, 12.65)      walls x in [-7.7, -2.2],
                                              y in [ 9.9, 15.4]
    south wall       y = 9.9            the wall the window is in
    blue window      centre (-5.72, 9.9, 3.85), 1.386 wide, 1.166 tall

The aircraft takes off 1 m south of that wall, on the ROOM's centre-line:

    takeoff         (-4.95, 8.9)

which is the scenario asked for -- and note what falls out of it. The room
centre is x = -4.95 and the window centre is x = -5.72, so the window is
0.77 m to one side of where the aircraft starts. Facing the room, that is
0.77 m to its LEFT. This is exactly the offset the mission's strafe exists
to remove, measured off the real arena rather than guessed.

Local NED has its origin at the takeoff point, with

    N = +y_gazebo      E = +x_gazebo      D = down

which is right-handed (N x E = down, because y x x = -z with z up) and puts
the aircraft's armed heading along +N, facing the room.

WHAT IS DELIBERATELY NOT SIMULATED
----------------------------------
THE ARUCO MARKERS. The reference world has none, so there is nothing to
project. Every leg therefore ends on its DISTANCE rather than on a marker,
which is a supported ending for all of them but the pad leg -- so the pad leg
lands off-pad at the end, and that is the correct behaviour, not a failure.
The marker logic is covered by test_mission_fsm.py instead, which can
synthesise detections directly.

THE DOLL MODEL. doll_detect is a separate node with a TensorRT engine and is
not run. What IS checked is the flight node's half of the contract: that
/doll_detect_enable goes true on the inbound commit and false on the
outbound clear, which is the only part of the doll mission this FSM owns.
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from px4_msgs.msg import (EstimatorStatusFlags, FailsafeFlags,
                          VehicleAttitude, VehicleLandDetected,
                          VehicleLocalPosition, VehicleStatus)
from std_msgs.msg import Bool, Float32MultiArray, String

from drone_testing.mission_fsm import MissionFSM
from drone_testing.window_traverse import quat_rotate


TICK = 0.05                     # s, the node's own timer period
MAX_SECONDS = 900.0


class SimClock:
    """A monotonic clock the simulation owns.

    Every stage in the inherited machine times itself with time.monotonic():
    _in_stage_for(), the flow-health dwell, the pose ages, the flight clock.
    Left alone, those read WALL time while this loop advances a simulated
    clock as fast as the CPU allows, and the two diverge immediately -- a
    1 s ground wait measured against a loop running 300x real time appears to
    last 300 simulated seconds.

    So the simulation's clock replaces the module-level one for the duration
    of the run. Everything then agrees, the run is deterministic rather than
    dependent on how fast this machine happens to be, and it finishes in
    about a second.
    """

    def __init__(self):
        self.t = 0.0
        self._real = time.monotonic

    def __enter__(self):
        time.monotonic = lambda: self.t
        return self

    def __exit__(self, *exc):
        time.monotonic = self._real

    def advance(self, dt):
        self.t += dt


# ----------------------------------------------------------------- the world

class World:
    """The scaled arena's dark room, in local NED about the takeoff point."""

    # Gazebo, straight out of imav2026_scaled.sdf.
    TAKEOFF_GZ = (-4.95, 8.9)       # set from --takeoff-distance at runtime
    WALL_Y = 9.9
    WINDOW_GZ = (-5.72, 9.9, 3.85)
    WINDOW_W = 1.386                # blue_R.x - blue_L.x
    WINDOW_H = 1.166                # blue_top.z - blue_bot.z

    ROOM_X = (-7.7, -2.2)
    ROOM_Y = (9.9, 15.4)

    def __init__(self, takeoff_distance=3.0):
        self.takeoff_distance = takeoff_distance
        self.TAKEOFF_GZ = (-4.95, self.WALL_Y - takeoff_distance)
        wx, wy, wz = self.WINDOW_GZ
        tx, ty = self.TAKEOFF_GZ
        # N = +y_gz, E = +x_gz, D = -z_gz, origin at the takeoff point.
        self.centre = np.array([wy - ty, wx - tx, -wz])
        self.half_w = self.WINDOW_W / 2.0
        self.half_h = self.WINDOW_H / 2.0
        # The wall faces south; the normal points back at the aircraft, which
        # is what everything downstream assumes (+normal = in front of it).
        self.normal = np.array([-1.0, 0.0, 0.0])

    @property
    def offset_east(self):
        """How far, and which way, the window is off the takeoff centre-line."""
        return float(self.centre[1])

    def corners(self):
        """TL, TR, BR, BL in NED, walking round the quad.

        The detector's order, because that is what the estimator's side and
        diagonal arithmetic assumes. Facing +N the camera's right is +E and
        its up is -D, so 'left' is the smaller E and 'top' the more negative D.
        """
        n, e, d = self.centre
        return np.array([
            [n, e - self.half_w, d - self.half_h],   # top-left
            [n, e + self.half_w, d - self.half_h],   # top-right
            [n, e + self.half_w, d + self.half_h],   # bottom-right
            [n, e - self.half_w, d + self.half_h],   # bottom-left
        ])

    def inside_room(self, p):
        n, e = float(p[0]), float(p[1])
        tx, ty = self.TAKEOFF_GZ
        return (self.ROOM_Y[0] - ty < n < self.ROOM_Y[1] - ty
                and self.ROOM_X[0] - tx < e < self.ROOM_X[1] - tx)


# --------------------------------------------------------------- the vehicle

class Vehicle:
    """A kinematic stand-in for PX4 plus the airframe.

    Deliberately crude: it moves the aircraft towards whatever
    TrajectorySetpoint the node published, at a bounded speed, and reports
    back where it got to. There is no attitude dynamics, no wind and no
    controller -- so this can demonstrate that the SEQUENCE is right and can
    never demonstrate that the flight is.
    """

    MAX_SPEED_XY = 1.0          # m/s the position setpoint is chased at
    MAX_SPEED_Z = 0.8
    YAW_RATE = 0.8              # rad/s

    def __init__(self):
        self.p = np.array([0.0, 0.0, 0.0])      # NED, origin at takeoff
        self.v = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        self.armed = False
        self.offboard = False
        self.landed = True

    def step(self, sp, dt):
        """One tick against a TrajectorySetpoint. NaN means 'not controlled'."""
        if sp is None or not self.armed or not self.offboard:
            self.v[:] = 0.0
            return

        target = np.array(sp.position, dtype=float)
        vel = np.array(sp.velocity, dtype=float)
        new = self.p.copy()

        for i, limit in enumerate((self.MAX_SPEED_XY, self.MAX_SPEED_XY,
                                   self.MAX_SPEED_Z)):
            if math.isfinite(target[i]):
                step = np.clip(target[i] - self.p[i], -limit * dt, limit * dt)
                new[i] = self.p[i] + step
            elif math.isfinite(vel[i]):
                new[i] = self.p[i] + np.clip(vel[i], -limit, limit) * dt

        # The floor. Without it a descent runs to the setpoint's overshoot
        # below ground and the land detector never fires.
        new[2] = min(new[2], 0.0)

        self.v = (new - self.p) / dt
        self.p = new

        if math.isfinite(sp.yaw):
            err = math.atan2(math.sin(sp.yaw - self.yaw),
                             math.cos(sp.yaw - self.yaw))
            self.yaw += np.clip(err, -self.YAW_RATE * dt, self.YAW_RATE * dt)

        # Landed once we are on the deck and descending or still.
        self.landed = self.p[2] > -0.06 and abs(self.v[2]) < 0.35

    @property
    def agl(self):
        return max(0.0, -float(self.p[2]))


# -------------------------------------------------------------- the harness

class Sim:

    def __init__(self, node, world, clock, verbose=False):
        self.node = node
        self.world = world
        self.clock = clock
        self.veh = Vehicle()
        self.verbose = verbose
        self.setpoint = None
        self.timeline = []          # (t, stage)
        self.doll_windows = []      # (t, enabled)
        self.last_stage = None
        self.last_doll = None
        self.geometry_sent = 0

        # Intercept everything the node publishes. The setpoint is the only
        # one fed back; the rest are recorded so the scenario can assert on
        # them.
        node.trajectory_setpoint_pub.publish = self._on_setpoint
        node.vehicle_command_pub.publish = self._on_command
        node.offboard_control_mode_pub.publish = lambda msg: None
        node.status_pub.publish = lambda msg: None
        node.phase_pub.publish = lambda msg: None
        node.window_pose_pub.publish = lambda msg: None
        node.doll_enable_pub.publish = self._on_doll_enable

        # The keyboard thread wants a tty and there is none; it has already
        # logged that and disabled itself, but make certain nothing reads it.
        node.abort_requested = False
        node.kill_requested = False

    # ---- what the node publishes ----

    def _on_setpoint(self, msg):
        self.setpoint = msg

    def _on_command(self, msg):
        from px4_msgs.msg import VehicleCommand
        if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            # param1=1, param2=6 is PX4's "custom mode, offboard".
            if abs(msg.param2 - 6.0) < 0.5:
                self.veh.offboard = True
        elif msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.veh.armed = msg.param1 > 0.5
        elif msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            pass

    def _on_doll_enable(self, msg):
        if msg.data != self.last_doll:
            self.last_doll = msg.data
            self.doll_windows.append((self.t, bool(msg.data)))

    # ---- what the node is fed ----

    def _feed_px4(self):
        n = self.node
        v = self.veh

        st = VehicleStatus()
        st.arming_state = (VehicleStatus.ARMING_STATE_ARMED if v.armed
                           else VehicleStatus.ARMING_STATE_DISARMED)
        st.nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if v.offboard
                        else VehicleStatus.NAVIGATION_STATE_MANUAL)
        n.vehicle_status_callback(st)

        lp = VehicleLocalPosition()
        lp.xy_valid = True
        lp.z_valid = True
        lp.v_xy_valid = True
        lp.x, lp.y, lp.z = (float(c) for c in v.p)
        lp.vx, lp.vy, lp.vz = (float(c) for c in v.v)
        lp.heading = float(v.yaw)
        lp.dist_bottom = float(v.agl)
        lp.dist_bottom_valid = True
        n.local_position_callback(lp)

        f = EstimatorStatusFlags()
        # The rangefinder IS being fused. cs_rng_kin_consistent especially:
        # with it false the node refuses to arm, which is the behaviour
        # README section 10.1 documents and is not what this scenario is for.
        f.cs_rng_hgt = True
        f.cs_rng_kin_consistent = True
        f.cs_rng_fault = False
        f.cs_rng_stuck = False
        n.estimator_flags_callback(f)

        n.failsafe_flags_callback(FailsafeFlags())

        ld = VehicleLandDetected()
        ld.landed = v.landed
        ld.maybe_landed = v.landed
        n.land_detected_callback(ld)

        att = VehicleAttitude()
        half = v.yaw / 2.0
        att.q = [math.cos(half), 0.0, 0.0, math.sin(half)]
        n.attitude_callback(att)

    # ---- the camera ----

    def _feed_window(self):
        """Project the window into the camera and publish it, if it is visible.

        The inverse of what the estimator does with the message:

            cam  = (body - t_cam) @ r_cam
            body = R(q)^-1 (ned - p)

        with depth along the optical axis, azimuth positive right and
        elevation positive up -- window_detect's convention exactly.
        """
        n = self.node
        v = self.veh
        q = [math.cos(v.yaw / 2.0), 0.0, 0.0, math.sin(v.yaw / 2.0)]
        q_inv = [q[0], -q[1], -q[2], -q[3]]

        rows = []
        for corner in list(self.world.corners()) + [self.world.centre]:
            body = quat_rotate(q_inv, np.asarray(corner) - v.p)
            cam = (body - n.t_cam) @ n.r_cam
            depth = float(cam[0])
            if depth <= 0.05:
                return                      # behind the camera
            az = math.atan2(float(cam[1]), depth)
            el = math.atan2(-float(cam[2]), depth)
            # The D435i colour sensor, 70 x 43 degrees. Outside it there is
            # no detection at all, which is what makes the yaw sweep and the
            # standoff matter.
            if abs(az) > math.radians(35.0) or abs(el) > math.radians(21.5):
                return
            if not (0.35 <= depth <= 8.0):
                return
            rows.append([depth, math.degrees(az), math.degrees(el)])

        data = [c for row in rows for c in row] + [0.0, 999.0, 0.0]
        msg = Float32MultiArray()
        msg.data = [float(x) for x in data]
        n.geometry_callback(msg)
        n.window_callback(Bool(data=True))
        n.window_info_callback(String(data='sim'))
        self.geometry_sent += 1

    # ---- the loop ----

    @property
    def t(self):
        return self.clock.t

    def run(self, max_seconds=MAX_SECONDS):
        node = self.node
        while self.t < max_seconds:
            self._feed_px4()
            self._feed_window()
            node.timer_callback()

            stage = node.current_stage
            if stage != self.last_stage:
                self.last_stage = stage
                self.timeline.append((self.t, stage))
                extra = ''
                if self.verbose and self.setpoint is not None:
                    p = self.setpoint.position
                    extra = (f"   sp=({p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f})")
                print(f"  {self.t:7.2f}s  {stage:<14} "
                      f"N{self.veh.p[0]:+.2f} E{self.veh.p[1]:+.2f} "
                      f"D{self.veh.p[2]:+.2f}  yaw{math.degrees(self.veh.yaw):+.0f}"
                      f"{extra}")
                if stage == node.DONE:
                    break

            self.veh.step(self.setpoint, TICK)
            self.clock.advance(TICK)
        return self.timeline


# ------------------------------------------------------------------ scenario

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--takeoff-distance', type=float, default=3.0,
                    help='m south of the dark room wall to start from. '
                         'The scenario asked for 1.0, which CANNOT acquire '
                         'the window -- see the note printed below.')
    args = ap.parse_args()

    world = World(args.takeoff_distance)
    print(__doc__.split('THE WORLD')[0].strip().splitlines()[0])
    print()
    print(f"World: dark room south wall 1.00 m ahead, window centre "
          f"N{world.centre[0]:+.2f} E{world.centre[1]:+.2f} D{world.centre[2]:+.2f}, "
          f"{world.WINDOW_W:.3f} x {world.WINDOW_H:.3f} m")
    side = 'left' if world.offset_east < 0 else 'right'
    print(f"       the window is {abs(world.offset_east):.2f} m to the "
          f"aircraft's {side.upper()} at the takeoff point")

    # The D435i colour sensor is 70 x 43 deg. The aperture has to FIT IN THE
    # FRAME before it can be locked onto, and for this window the vertical
    # binds: 1.166 m of height needs 1.48 m of depth, which is also what
    # effective_standoff()'s 1.26 x height rule says. standoff_distance is
    # 2.00 m on top of that. So a takeoff point 1 m from the wall cannot
    # acquire the window at all -- not a code fault, a lens.
    fit_v = world.WINDOW_H / 2.0 / math.tan(math.radians(21.5))
    fit_h = world.WINDOW_W / 2.0 / math.tan(math.radians(35.0))
    print(f"       aperture fits in frame beyond {max(fit_v, fit_h):.2f} m "
          f"(vertical binds: {fit_v:.2f} m, horizontal {fit_h:.2f} m)")
    if world.takeoff_distance < max(fit_v, fit_h):
        print(f"       !! takeoff is {world.takeoff_distance:.2f} m from the "
              "wall: the window can never be fully in frame, so no lock is")
        print("          possible. This is the lens, not the code. "
              "Use --takeoff-distance 3.0")
    print()

    rclpy.init()

    # The scenario asked for: take off 1 m from the room, strafe onto the
    # window axis, in, room, out, mirror the strafe, land. There are no
    # markers in the reference world, so the legs are configured to their
    # real lengths for THIS geometry and each ends on its distance.
    overrides = {
        'window_offset': abs(world.offset_east),
        'window_offset_direction': side,
        'outbound_distance': 0.30,   # already at the spot; just settle
        'return_distance': 0.60,
        'turn_distance': 0.30,
        'pad_distance': 0.30,
        'max_altitude': 5.0,         # the window centre is at 3.85 m; the
                                     # 3.0 default clamps the traverse BELOW
                                     # the sill and flies into the wall
        # RELOCK has to see the WHOLE aperture again from inside, and in this
        # 2.2x world that needs 1.48 m of depth. The 1.20 m default parks the
        # aircraft closer than it can focus on its own way out, and the room
        # pattern's last leg is flown TOWARDS the window wall, closer still.
        # Unscaled the window is 0.53 m tall and needs 0.67 m, so 1.20 m is
        # correct on the real arena -- this is the scaling, not the mission.
        'inside_distance': 2.50,
        'outside_distance': 1.50,
        # The altitude schedule, exercised: cruise high for the markers,
        # drop for the window. The window centre in this world is at 3.85 m,
        # so these bracket it.
        'cruise_altitude': 4.20,
        'window_altitude_m': 3.50,
        # The room is flown at a height WE pick, and in this 2.2x world
        # that has to be near the window's 3.85 m -- RELOCK has to see
        # the aperture again from inside, and it cannot do that from the
        # real course's 1.75 m when the window is 2 m above it.
        # LEVEL WITH THE WINDOW CENTRE, not just near it. Flying the room
        # 0.35 m low puts the aperture's TOP corner at 21.9 deg on the
        # relock, against a 21.5 deg half-FOV -- one corner out of frame
        # is a truncated quad, which the estimator refuses outright.
        'room_altitude': 3.85,
        'alt_change_timeout': 30.0,
        # The search failsafe. Left at its real values so the ladder is the
        # one that would fly; the window is visible here so it should never
        # be climbed.
        'window_search_seconds': 8.0,
        'marker_max_retries': 0,
        'altitude_offset': 0.0,
        'flight_seconds': 880.0,
        'ground_wait_seconds': 1.0,
        'hold_seconds': 2.0,
        'clear_seconds': 2.0,
        'room_hold_seconds': 1.0,
        'marker_hold_seconds': 1.0,
        'leg_settle_seconds': 1.0,
        'window_marker_id': 2,
        'turn_marker_id': 3,
        'pad_marker_id': 1,
    }
    # Parameter name -> class attribute, where the two differ. Guessing with
    # k.upper() silently misses these: window_altitude_m is WINDOW_ALTITUDE,
    # and an override that lands on a non-existent attribute does nothing at
    # all while looking like it worked.
    ATTR = {
        'window_altitude_m': 'WINDOW_ALTITUDE',
        'window_backoff': 'WINDOW_BACKOFF_M',
        'marker_tolerance': 'MARKER_TOLERANCE',
    }
    for k, val in overrides.items():
        attr = ATTR.get(k, k.upper())
        if not hasattr(MissionFSM, attr):
            raise SystemExit(
                f"override '{k}' -> MissionFSM.{attr}, which does not exist. "
                "Fix the mapping rather than letting it silently do nothing.")
        setattr(MissionFSM, attr, val)

    node = MissionFSM()
    # Apply the overrides that are read into differently-named attributes, or
    # that the constructor has already consumed from the class defaults.
    node.WINDOW_OFFSET = overrides['window_offset']
    node.WINDOW_OFFSET_DIRECTION = side
    node.MAX_ALTITUDE = overrides['max_altitude']
    node.CRUISE_ALTITUDE = overrides['cruise_altitude']
    node.WINDOW_ALTITUDE = overrides['window_altitude_m']
    node.TAKEOFF_ALTITUDE = overrides['cruise_altitude']
    node.FLIGHT_SECONDS = overrides['flight_seconds']

    print("Timeline:")
    with SimClock() as clock:
        sim = Sim(node, world, clock, verbose=args.verbose)
        timeline = sim.run()

    stages = [s for _, s in timeline]
    print()
    print(f"Stages visited ({len(stages)}): {' -> '.join(stages)}")
    print(f"/window_geometry messages published: {sim.geometry_sent}")
    print("Doll detection windows: "
          + (', '.join(f"{'ON' if on else 'OFF'} at {t:.1f}s"
                       for t, on in sim.doll_windows) or 'never enabled'))

    ok = True

    def want(cond, what):
        nonlocal ok
        print(('  PASS  ' if cond else '  FAIL  ') + what)
        ok = ok and cond

    print()
    print("Checks:")
    want('CRUISE' in stages, "flew at least one leg")
    want(node.legs_done and 'outbound' in node.legs_done,
         "the outbound leg ran")
    want('offset' in node.legs_done, "the strafe onto the window axis ran")
    want(sim.geometry_sent > 0, "the window was visible to the camera")
    want('LOCK' in stages, "locked onto the window")
    want('TRAVERSE' in stages, "committed to a traverse")
    on = [t for t, en in sim.doll_windows if en]
    off = [t for t, en in sim.doll_windows if not en]
    traverse_t = next((t for t, st in timeline if st == 'TRAVERSE'), None)
    want(bool(on), "doll detection was enabled inside the room")
    want(bool(on) and traverse_t is not None
         and abs(on[0] - traverse_t) < 2.0,
         "...and it was enabled AT the inbound commit, not before or after")
    want(bool(on) and len(off) > 1 and off[-1] > on[0],
         "...and disabled again on the way out")
    want('ALT_CHANGE' in stages, "the altitude schedule ran")
    want('RELOCK' in stages, "tried to re-acquire the window from inside")
    want(stages.count('TRAVERSE') >= 2, "traversed the window a SECOND time")
    want('offset_back' in node.legs_done, "the mirror strafe ran on the way out")
    want('return' in node.legs_done, "the return leg ran")
    want(node.DISARMING in stages or node.DONE in stages,
         "the mission reached a landing")

    print()
    print("RESULT:", "PASS" if ok else "FAIL")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
