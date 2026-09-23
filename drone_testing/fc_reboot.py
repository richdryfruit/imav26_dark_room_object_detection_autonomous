"""
Reboot the flight controller from the Jetson, over the DDS link.

The ARK Flow's DroneCAN node enumerates well after PX4 boots, so EKF2 comes
up without a rangefinder and never anchors its height estimate on it:
cs_rng_hgt and cs_rng_kin_consistent both stay false and the flight nodes
correctly refuse to arm. Rebooting the FC *after* the sensor is alive fixes
it -- but doing that meant a laptop and QGC, which is no use when the drone
is powered from a battery and flown over ssh.

PX4 accepts VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246, param1 = 1) on the
same /fmu/in/vehicle_command topic the setpoints already use, so the reboot
is one message on the link that is already there.

    ros2 run drone_testing fc_reboot                 # reboot if unfused
    ros2 run drone_testing fc_reboot -p force:=true  # reboot regardless

By default it is CONDITIONAL: it waits for the rangefinder to be publishing
and for EKF2's flags to say it is NOT being fused, and only then reboots. If
EKF2 already has the rangefinder anchored there is nothing to fix and it
exits without touching anything, so it is safe to run before every flight --
which is the point, since the launch file runs it for you.

The wait is in two stages, because "the sensor has not arrived yet" and "the
sensor is here and EKF2 is ignoring it" want opposite responses:

    stage 1  up to sensor_wait (90 s) for range data to reach EKF2 at all.
             Rebooting during this window is pointless -- the ARK Flow takes
             45-50 s to enumerate on DroneCAN either way.
    stage 2  once data is arriving, fuse_grace (8 s) for EKF2 to fuse it.
             Fused means there was never a problem. Still not fused means
             EKF2 anchored on the baro at boot and only a reboot will move
             it, so we reboot immediately rather than waiting out stage 1.

SAFETY
    It refuses to send anything while the vehicle is armed, whatever the
    parameters say. PX4 refuses the command in that state too; this is the
    belt to that braces.

AFTER THE REBOOT
    The DDS session dies with the FC and micro_ros_agent re-establishes it a
    few seconds later. This node waits for VehicleStatus to come back, checks
    the flags again, and reports whether the reboot actually helped -- so
    over ssh you get a straight answer instead of having to guess.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    EstimatorStatusFlags,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from drone_testing.px4_topics import subscribe_versioned


class FcReboot(Node):

    WAIT_FOR_PX4 = 15.0      # s waiting for the first VehicleStatus before we
                             # decide the link is not there at all
    SETTLE_SECONDS = 3.0     # s of flags observed before judging them, so a
                             # half-initialised EKF2 is not misread
    SENSOR_WAIT = 90.0       # s to wait for the rangefinder to turn up and be
                             # fused BEFORE deciding a reboot is needed. The
                             # ARK Flow's DroneCAN node can take the better
                             # part of a minute to enumerate; rebooting at t=5s
                             # would just restart that wait with the node no
                             # earlier than before. Waiting first also means we
                             # do not reboot at all when EKF2 picks the sensor
                             # up on its own.
    FUSE_GRACE = 8.0         # s allowed between range data first reaching EKF2
                             # and EKF2 actually fusing it. Once data is
                             # arriving, EKF2 has everything it needs; if it
                             # still is not fusing after this, it never will on
                             # this boot, so there is nothing to gain by
                             # sitting out the rest of SENSOR_WAIT.
    REBOOT_WAIT = 45.0       # s to wait for PX4 to come back afterwards
    RECHECK_SECONDS = 12.0   # s after it is back before the flags are judged
                             # again -- EKF2 needs a moment to start fusing

    def __init__(self):
        super().__init__('fc_reboot')

        self.force = bool(self.declare_parameter('force', False).value)
        self.wait_only = bool(self.declare_parameter('check_only', False).value)
        self.timeout = float(self.declare_parameter('timeout', self.WAIT_FOR_PX4).value)
        self.sensor_wait = float(self.declare_parameter(
            'sensor_wait', self.SENSOR_WAIT).value)
        self.fuse_grace = float(self.declare_parameter(
            'fuse_grace', self.FUSE_GRACE).value)
        self.started_at = time.monotonic()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand, '/uav_2/fmu/in/vehicle_command', 10)

        self.status_subs = subscribe_versioned(
            self, VehicleStatus, 'vehicle_status', self.status_callback,
            sensor_qos)
        self.create_subscription(EstimatorStatusFlags,
                                 '/uav_2/fmu/out/estimator_status_flags',
                                 self.flags_callback, qos_profile=sensor_qos)
        self.position_subs = subscribe_versioned(
            self, VehicleLocalPosition, 'vehicle_local_position',
            self.position_callback, sensor_qos)

        self.arming_state = None
        self.flags = None
        self.local_position = None
        self.status_seen_at = None

    # ------------------------------------------------------------------ subs

    def status_callback(self, msg):
        self.arming_state = msg.arming_state
        if self.status_seen_at is None:
            self.status_seen_at = time.monotonic()

    def flags_callback(self, msg):
        self.flags = msg

    def position_callback(self, msg):
        self.local_position = msg

    # ------------------------------------------------------------- decisions

    def armed(self):
        return self.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def rangefinder_is_fused(self):
        """The same question the flight nodes gate arming on."""
        f = self.flags
        if f is None:
            return None
        return bool((f.cs_rng_hgt or f.cs_rng_terrain)
                    and not f.cs_rng_fault and not f.cs_rng_stuck
                    and f.cs_rng_kin_consistent)

    def rangefinder_is_present(self):
        """Is range data reaching EKF2 at all, fused or not?

        Stock PX4 does not bridge distance_sensor outbound -- dds_topics.yaml
        lists it under subscriptions: only, so /fmu/in/distance_sensor is the
        direction a companion computer feeds range data INTO PX4 and never a
        readback of the FC's own sensor. Adding it to publications: and
        reflashing is the only way to watch the sensor directly from here.
        Until then, these two are the earliest evidence available:

          * the terrain estimate being valid AND its sensor bitfield naming a
            range sensor. dist_bottom_valid on its own is not enough: optical
            flow alone can raise it, so without the bitfield check this would
            report the rangefinder present whenever only flow had arrived.
            NOTE: this airframe carries a Holybro PMW3901 (flow only, no
            onboard lidar) and a Benewake TFmini, so the TFmini is the sole
            distance_sensor publisher and will be uORB instance 0. NOTE ALSO
            that with EKF2_HGT_REF = 2 (Range) dist_bottom_valid is pinned
            false on v1.17 regardless of sensor health -- see
            offboard_takeoff.rangefinder_is_healthy() -- so on this vehicle
            the bitfield branch below never fires and only the cs_rng_* branch
            is load-bearing;
          * any cs_rng_* flag being set, including fault and stuck, which EKF2
            can only judge once samples are arriving.
        """
        p = self.local_position
        if (p is not None and getattr(p, 'dist_bottom_valid', False)
                and (getattr(p, 'dist_bottom_sensor_bitfield', 0)
                     & VehicleLocalPosition.DIST_BOTTOM_SENSOR_RANGE)):
            return True

        f = self.flags
        if f is not None and (f.cs_rng_hgt or f.cs_rng_terrain
                              or f.cs_rng_fault or f.cs_rng_stuck):
            return True

        return False

    def flag_summary(self):
        f = self.flags
        if f is None:
            return "no estimator_status_flags being published"
        return (f"rng_hgt={f.cs_rng_hgt} rng_terrain={f.cs_rng_terrain} "
                f"rng_kin_consistent={f.cs_rng_kin_consistent} "
                f"rng_fault={f.cs_rng_fault} rng_stuck={f.cs_rng_stuck} "
                f"baro_hgt={f.cs_baro_hgt}")

    # ----------------------------------------------------------------- steps

    def spin_for(self, seconds, until=None):
        """Spin the node for a while, stopping early if `until` returns True."""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if until is not None and until():
                return True
        return False

    def send_reboot(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        msg.param1 = 1.0        # 1 = reboot the autopilot
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        # Sent a few times: the command goes out best-effort over a link that
        # is about to be torn down by the very command we are sending.
        for _ in range(3):
            self.command_pub.publish(msg)
            self.spin_for(0.2)

    def run(self):
        self.get_logger().info("Waiting for PX4 over the DDS link...")
        if not self.spin_for(self.timeout, lambda: self.status_seen_at is not None):
            self.get_logger().error(
                "No VehicleStatus from PX4. The DDS agent is not connected, so "
                "there is nothing to reboot. Not sending anything.")
            return 1

        # Let the flags settle before judging them.
        self.spin_for(self.SETTLE_SECONDS)

        if self.armed():
            self.get_logger().error("Vehicle is ARMED. Refusing to reboot.")
            return 1

        self.get_logger().info(f"EKF2 flags: {self.flag_summary()}")
        if self.local_position is not None:
            self.get_logger().info(
                f"dist_bottom={self.local_position.dist_bottom:.3f} m "
                f"z_valid={self.local_position.z_valid}")

        # Give the sensor time to turn up before concluding anything. This is
        # the whole point: the node enumerates tens of seconds after PX4 boots,
        # and a reboot issued before it is on the bus achieves nothing.
        if not self.rangefinder_is_fused():
            self.get_logger().info(
                f"Rangefinder not fused yet. Waiting up to "
                f"{self.sensor_wait:.0f} s for it to appear -- the DroneCAN "
                "node enumerates well after PX4 boots.")
            deadline = time.monotonic() + self.sensor_wait
            last_log = 0.0
            appeared_at = None
            while rclpy.ok() and time.monotonic() < deadline:
                self.spin_for(0.5)
                if self.rangefinder_is_fused():
                    break

                now = time.monotonic()

                # The moment range data reaches EKF2 at all. From here we owe
                # it fuse_grace and no more: the sensor is on the bus, so if
                # EKF2 is not fusing it shortly after this it has already
                # anchored elsewhere and only a reboot will change that.
                if appeared_at is None and self.rangefinder_is_present():
                    appeared_at = now
                    self.get_logger().info(
                        f"Range data has reached EKF2, "
                        f"{now - self.started_at:.0f} s after this node started. "
                        f"Giving EKF2 {self.fuse_grace:.0f} s to fuse it.")

                if (appeared_at is not None
                        and now - appeared_at >= self.fuse_grace):
                    self.get_logger().warning(
                        f"Range data is arriving but EKF2 is not fusing it "
                        f"after {self.fuse_grace:.0f} s. Not sitting out the "
                        f"remaining {deadline - now:.0f} s.")
                    break

                if now - last_log >= 5.0:
                    last_log = now
                    self.get_logger().info(
                        f"  still waiting ({deadline - now:.0f} s left): "
                        f"{self.flag_summary()}")

        if self.rangefinder_is_fused():
            self.get_logger().info(
                "Rangefinder is fused -- nothing to fix, not rebooting.")
            return 0

        if self.wait_only:
            self.get_logger().warning(
                "Rangefinder is NOT fused, but check_only is set. Not rebooting.")
            return 1

        if not self.force and self.flags is None:
            self.get_logger().error(
                "No estimator flags to judge by. Not rebooting on a guess; "
                "pass -p force:=true if you want it rebooted anyway.")
            return 1

        self.get_logger().warning(
            f"Rangefinder still not fused after {self.sensor_wait:.0f} s. "
            "Rebooting the flight controller "
            "so EKF2 starts with the ARK Flow already on the bus.")
        self.send_reboot()

        self.get_logger().info(
            f"Reboot sent. Waiting up to {self.REBOOT_WAIT:.0f} s for PX4 to "
            "come back (the DDS session has to be re-established).")
        self.status_seen_at = None
        self.flags = None
        if not self.spin_for(self.REBOOT_WAIT,
                             lambda: self.status_seen_at is not None):
            self.get_logger().error(
                "PX4 did not come back. Check micro_ros_agent is still running "
                "and reconnected before flying anything.")
            return 1

        self.get_logger().info(
            f"PX4 is back. Waiting up to {self.sensor_wait:.0f} s for EKF2 to "
            "anchor on the rangefinder.")
        self.spin_for(self.RECHECK_SECONDS)
        self.spin_for(self.sensor_wait, self.rangefinder_is_fused)

        self.get_logger().info(f"EKF2 flags now: {self.flag_summary()}")
        if self.rangefinder_is_fused():
            self.get_logger().warning("Rangefinder is fused. Good to fly.")
            return 0

        self.get_logger().error(
            "Rangefinder is STILL not fused after the reboot. Do not fly: the "
            "height estimate is on the barometer and drifts metres indoors. "
            "Check EKF2_HGT_REF=2, EKF2_RNG_CTRL=2, and that the ARK Flow is "
            "on the bus before PX4 finishes booting.")
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = FcReboot()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    main()
