#!/usr/bin/env python3
"""QGC doll-status reporter — uXRCE-DDS edition (no pymavlink).

Sends doll counts and status updates to QGC via two paths:

  1. STATUSTEXT  → published to /fmu/in/log_message (px4_msgs/LogMessage).
                   PX4's uXRCE-DDS agent forwards these as MAVLink STATUSTEXT
                   to QGC over whichever MAVLink link (radio / UDP) the FC
                   is already using.  No separate UDP socket needed.

  2. NAMED_VALUE_INT → packed with Python struct and sent as a raw MAVLink v2
                   frame over a UDP socket to QGC (host:port parameter).
                   This avoids pymavlink while still letting the MAVLink
                   Inspector in QGC show live "dolls" / "dolls_now" values.
                   If the socket cannot be opened the node falls back to
                   STATUSTEXT-only mode (silent degradation).

Why no HEARTBEAT?  This node is a companion-computer ROS node that never
needs to appear as a separate MAVLink system.  The FC (via the agent) already
sends its own HEARTBEAT, which is all QGC requires to connect.

Severity constants mirror MAVLink MAV_SEVERITY values (RFC 5424 / linux kern):
    EMERGENCY=0  ALERT=1  CRITICAL=2  ERROR=3
    WARNING=4    NOTICE=5  INFO=6     DEBUG=7

Addressing
    qgc_host    default '255.255.255.255'  (broadcast finds QGC without IP)
    qgc_port    default 14550
    system_id   default 1   (must match FC's MAV_SYS_ID)
    component_id default 191 (MAV_COMP_ID_ONBOARD_COMPUTER)
"""

import socket
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from px4_msgs.msg import LogMessage

# ─── MAVLink severity constants (MAV_SEVERITY_* without pymavlink) ────────────
MAV_SEVERITY_WARNING = 4
MAV_SEVERITY_NOTICE  = 5
MAV_SEVERITY_INFO    = 6

# ─── Minimal MAVLink v2 framer (struct only, no pymavlink) ───────────────────
# NAMED_VALUE_INT (message id 252):
#   u32   time_boot_ms
#   s32   value
#   char[10] name (null-padded)
#   CRC extra byte = 44  (from MAVLink XML definition)

_NAMED_VALUE_INT_ID    = 252
_NAMED_VALUE_INT_EXTRA = 44


def _crc_accumulate(byte: int, crc: int) -> int:
    """CRC-16/MCRF4XX accumulator used by MAVLink."""
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = _crc_accumulate(b, crc)
    return crc


def _pack_named_value_int(sysid: int, compid: int, seq: int,
                          time_boot_ms: int, name: str, value: int) -> bytes:
    """Return a complete MAVLink v2 NAMED_VALUE_INT frame (bytes)."""
    name_b  = name[:10].encode('ascii').ljust(10, b'\x00')
    payload = struct.pack('<Ii', time_boot_ms, value) + name_b
    msgid   = _NAMED_VALUE_INT_ID
    header  = bytes([
        0xFD,               # STX
        len(payload),       # payload length
        0, 0,               # incompat / compat flags
        seq & 0xFF,
        sysid  & 0xFF,
        compid & 0xFF,
        msgid        & 0xFF,
        (msgid >> 8)  & 0xFF,
        (msgid >> 16) & 0xFF,
    ])
    # CRC covers: len, incompat, compat, seq, sysid, compid, msgid(3), payload, extra_crc
    crc_data = bytes([
        len(payload), 0, 0, seq & 0xFF,
        sysid & 0xFF, compid & 0xFF,
        msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF,
    ]) + payload + bytes([_NAMED_VALUE_INT_EXTRA])
    crc = _crc16(crc_data)
    return header + payload + struct.pack('<H', crc)


# ─── Node ─────────────────────────────────────────────────────────────────────

class QGCDollStatus(Node):

    NAMED_HZ       = 2.0
    REPEAT_SECONDS = 10.0

    def __init__(self):
        super().__init__('qgc_doll_status')

        host    = str(self.declare_parameter('qgc_host',      '255.255.255.255').value)
        port    = int(self.declare_parameter('qgc_port',      14550).value)
        self.sysid  = int(self.declare_parameter('system_id',    1).value)
        self.compid = int(self.declare_parameter('component_id', 191).value)

        self.total   = 0
        self.visible = 0
        self.report  = ''
        self.reported_final   = False
        self._last_sent_total = None
        self._last_text_time  = 0.0
        self._t0              = time.monotonic()
        self._seq             = 0

        # ── DDS publisher: STATUSTEXT via /fmu/in/log_message ─────────────
        # PX4's uXRCE-DDS agent reads this and forwards it as a MAVLink
        # STATUSTEXT to QGC over whatever link (radio/UDP) the FC uses.
        self._log_pub = self.create_publisher(LogMessage, '/uav_2/fmu/in/log_message', 10)

        # ── Optional UDP socket: NAMED_VALUE_INT (struct-packed, no pymavlink)
        self._sock     = None
        self._udp_addr = (host, port)
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.get_logger().info(
                f"Doll status NAMED_VALUE_INT -> QGC at {host}:{port} "
                f"(sysid={self.sysid} compid={self.compid}).")
        except Exception as exc:
            self.get_logger().warning(
                f"Could not open UDP socket for NAMED_VALUE_INT: {exc} — "
                "live values will not appear in the MAVLink Inspector.")

        # ── Subscriptions ──────────────────────────────────────────────────
        self.create_subscription(Int32,  'doll_count',    self.count_callback,   10)
        self.create_subscription(Int32,  'dolls_visible', self.visible_callback, 10)
        self.create_subscription(String, 'doll_report',   self.report_callback,  10)
        self.create_subscription(String, 'mission_phase', self.phase_callback,   10)

        self.create_timer(1.0 / self.NAMED_HZ, self.named_timer)

    # ─────────────────────────── send helpers ─────────────────────────────

    def _boot_ms(self) -> int:
        return int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF

    def _statustext(self, text: str, severity: int = MAV_SEVERITY_INFO):
        """Publish via /fmu/in/log_message so PX4 forwards as STATUSTEXT."""
        msg           = LogMessage()
        msg.timestamp = self._boot_ms() * 1000  # field is microseconds
        msg.severity  = severity
        # char[127] — encode, truncate, null-pad to exactly 127 elements
        encoded = text[:126].encode('ascii', 'replace')
        msg.text = list(encoded) + [0] * (127 - len(encoded))
        self._log_pub.publish(msg)

    def _named_int(self, name: str, value: int):
        """Send a raw MAVLink v2 NAMED_VALUE_INT frame over UDP (no pymavlink)."""
        if self._sock is None:
            return
        try:
            frame = _pack_named_value_int(
                self.sysid, self.compid, self._seq,
                self._boot_ms(), name, value)
            self._seq = (self._seq + 1) & 0xFF
            self._sock.sendto(frame, self._udp_addr)
        except Exception as exc:
            self.get_logger().warning(
                f"NAMED_VALUE_INT send failed: {exc}",
                throttle_duration_sec=5.0)

    # ────────────────────────── subscribers ───────────────────────────────

    def count_callback(self, msg: Int32):
        self.total = int(msg.data)
        if self.total != self._last_sent_total:
            self._statustext(
                f"DOLLS {self.total} (now {self.visible})", MAV_SEVERITY_NOTICE)
            self._last_sent_total = self.total
            self._last_text_time  = time.monotonic()

    def visible_callback(self, msg: Int32):
        self.visible = int(msg.data)

    def report_callback(self, msg: String):
        self.report = msg.data

    def phase_callback(self, msg: String):
        phase = msg.data.split('|')[0]
        if phase in ('LANDING', 'DONE') and not self.reported_final:
            self.reported_final = True
            self._send_final()

    def _send_final(self):
        self._statustext(
            f"FINAL DOLL COUNT: {self.total}", MAV_SEVERITY_WARNING)
        parts = self.report.split('|')[1:]
        for part in parts:
            if part:
                self._statustext(f"doll {part}"[:50])

    # ────────────────────────────── timers ────────────────────────────────

    def named_timer(self):
        self._named_int('dolls',     self.total)
        self._named_int('dolls_now', self.visible)

        now = time.monotonic()
        if now - self._last_text_time > self.REPEAT_SECONDS:
            self._statustext(f"DOLLS {self.total} (now {self.visible})")
            self._last_text_time = now

    def destroy_node(self):
        if not self.reported_final:
            self._statustext(
                f"FINAL DOLL COUNT: {self.total}", MAV_SEVERITY_WARNING)
        if self._sock is not None:
            self._sock.close()
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
