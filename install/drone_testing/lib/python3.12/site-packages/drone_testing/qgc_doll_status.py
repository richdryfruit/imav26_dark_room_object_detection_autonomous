#!/usr/bin/env python3
"""
Put the doll count in QGroundControl, without the Arduino and without a screen.

The problem this solves: /dev/ttyTHS1 on this Jetson is the uXRCE-DDS link to
PX4, so there is no spare MAVLink serial port to talk to the flight controller
on, and PX4 has no uORB topic in the default dds_topics.yaml that comes out of
the other end as STATUSTEXT. So this node does not go through PX4 at all. It
speaks MAVLink straight to QGC over the network the Jetson is already on, as
an extra COMPONENT of the same vehicle:

    source_system     = the vehicle's MAV_SYS_ID (1 by default), so QGC files
                        everything under the aircraft it is already showing
                        instead of popping up a second vehicle
    source_component   = 191, MAV_COMP_ID_ONBOARD_COMPUTER

What QGC then shows:

    STATUSTEXT         "DOLLS 3 (now 1)" in the message panel at the bottom,
                       and spoken aloud if the user has audio on. Sent when
                       the total changes, and repeated every REPEAT_SECONDS
                       so a GCS that connected late still learns the count.
    NAMED_VALUE_INT    "dolls" and "dolls_now" at NAMED_HZ. Visible live in
                       MAVLink Inspector, and plottable in Analyze -> MAVLink
                       Console/Chart, which is how you watch it climb during
                       the run rather than reading a log afterwards.
    STATUSTEXT final   the per-doll NED positions from /doll_report, one line
                       per doll (STATUSTEXT is 50 characters), sent once when
                       the mission phase reaches DONE/LANDING, so the result
                       of the flight is in the QGC log.

Addressing. QGC listens on UDP 14550 and, once it has heard from an endpoint,
talks back to it. Point `qgc_host` at the laptop running QGC. The default is
255.255.255.255, i.e. broadcast on the subnet, which finds QGC without anyone
typing an IP -- that works because both are on the same WiFi. If QGC is
reached over the telemetry radio instead of WiFi, set `qgc_host` to nothing
and this node cannot help; that case needs MAV_*_FORWARD on the FC and a
serial port this Jetson does not have free.

Nothing here touches the flight. If the socket cannot be opened, or QGC is
not there, the node logs once and keeps counting quietly.

Parameters
    qgc_host        default '255.255.255.255'
    qgc_port        default 14550
    system_id       default 1, must match the vehicle's MAV_SYS_ID
    component_id    default 191 (onboard computer)
"""

import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


class _UDPOut:
    """The file-like object pymavlink writes its packed bytes into."""

    def __init__(self, host, port):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def write(self, data):
        self.sock.sendto(data, self.addr)

    def close(self):
        self.sock.close()


class QGCDollStatus(Node):

    NAMED_HZ = 2.0
    HEARTBEAT_HZ = 1.0
    # QGC suppresses a STATUSTEXT identical to one it saw in the last second,
    # so a slow repeat is what keeps the count on screen for a late joiner
    # without spamming the panel.
    REPEAT_SECONDS = 10.0

    def __init__(self):
        super().__init__('qgc_doll_status')

        host = str(self.declare_parameter('qgc_host', '255.255.255.255').value)
        port = int(self.declare_parameter('qgc_port', 14550).value)
        self.sysid = int(self.declare_parameter('system_id', 1).value)
        self.compid = int(self.declare_parameter('component_id', 191).value)

        self.total = 0
        self.visible = 0
        self.report = ''
        self.reported_final = False
        self._last_sent_total = None
        self._last_text_time = 0.0
        self._t0 = time.monotonic()

        self.out = None
        self.mav = None
        try:
            self.out = _UDPOut(host, port)
            self.mav = mavlink2.MAVLink(self.out, srcSystem=self.sysid,
                                        srcComponent=self.compid)
            self.get_logger().info(
                f"Doll status -> QGC at {host}:{port} as {self.sysid}/{self.compid}.")
        except Exception as exc:
            self.get_logger().error(f"Could not open UDP to QGC: {exc}")

        self.create_subscription(Int32, 'doll_count', self.count_callback, 10)
        self.create_subscription(Int32, 'dolls_visible',
                                 self.visible_callback, 10)
        self.create_subscription(String, 'doll_report',
                                 self.report_callback, 10)
        self.create_subscription(String, 'mission_phase',
                                 self.phase_callback, 10)

        self.create_timer(1.0 / self.NAMED_HZ, self.named_timer)
        self.create_timer(1.0 / self.HEARTBEAT_HZ, self.heartbeat_timer)

    # --------------------------------------------------------------- send

    def _boot_ms(self):
        return int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF

    def _statustext(self, text, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
        if self.mav is None:
            return
        try:
            # 50 bytes is the field width; longer text is silently truncated
            # by some GCSs, so cut it here where the cut is visible.
            self.mav.statustext_send(severity, text[:50].encode('ascii', 'replace'))
        except Exception as exc:
            self.get_logger().warning(f"STATUSTEXT failed: {exc}",
                                      throttle_duration_sec=5.0)

    def _named_int(self, name, value):
        if self.mav is None:
            return
        try:
            self.mav.named_value_int_send(self._boot_ms(),
                                          name[:10].encode('ascii'), int(value))
        except Exception as exc:
            self.get_logger().warning(f"NAMED_VALUE_INT failed: {exc}",
                                      throttle_duration_sec=5.0)

    # --------------------------------------------------------------- subs

    def count_callback(self, msg):
        self.total = int(msg.data)
        if self.total != self._last_sent_total:
            self._statustext(f"DOLLS {self.total} (now {self.visible})",
                             mavutil.mavlink.MAV_SEVERITY_NOTICE)
            self._last_sent_total = self.total
            self._last_text_time = time.monotonic()

    def visible_callback(self, msg):
        self.visible = int(msg.data)

    def report_callback(self, msg):
        self.report = msg.data

    def phase_callback(self, msg):
        phase = msg.data.split('|')[0]
        if phase in ('LANDING', 'DONE') and not self.reported_final:
            self.reported_final = True
            self._send_final()

    def _send_final(self):
        self._statustext(f"FINAL DOLL COUNT: {self.total}",
                         mavutil.mavlink.MAV_SEVERITY_WARNING)
        # "n|id:x,y,z|id:x,y,z|..." -- one STATUSTEXT per doll so each fits.
        parts = self.report.split('|')[1:]
        for part in parts:
            if part:
                self._statustext(f"doll {part}"[:50])

    # ------------------------------------------------------------- timers

    def named_timer(self):
        self._named_int('dolls', self.total)
        self._named_int('dolls_now', self.visible)

        now = time.monotonic()
        if now - self._last_text_time > self.REPEAT_SECONDS:
            self._statustext(f"DOLLS {self.total} (now {self.visible})")
            self._last_text_time = now

    def heartbeat_timer(self):
        """QGC ignores a component it has never heard a HEARTBEAT from."""
        if self.mav is None:
            return
        try:
            self.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE)
        except Exception:
            pass

    def destroy_node(self):
        if self.mav is not None and not self.reported_final:
            self._statustext(f"FINAL DOLL COUNT: {self.total}",
                             mavutil.mavlink.MAV_SEVERITY_WARNING)
        if self.out is not None:
            self.out.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QGCDollStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
