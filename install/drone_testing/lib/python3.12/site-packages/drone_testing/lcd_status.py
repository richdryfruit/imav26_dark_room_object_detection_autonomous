#!/usr/bin/env python3
"""
Bridge from the takeoff state machine to a parallel TFT shield on an
Arduino Uno.

Subscribes to /takeoff_status (published by offboard_takeoff) plus PX4's
vehicle status, formats a banner plus six body rows, and pushes them to the
Arduino over USB serial. Flash arduino/tft_status/tft_status.ino first.

The Arduino is intentionally dumb -- all formatting happens here, so the
layout can change without reflashing the board.

Screen:
    banner   the stage name, colour-coded by severity
    row 1    arming state
    row 2    altitude above the arming point
    row 3    horizontal control mode (velocity hold / latched position)
    row 4    optical flow health + window detection (/window_detected)
    row 5    stage-specific detail (countdown, target altitude)
    row 6    PX4 nav state

If offboard_takeoff is not running the screen still shows the PX4 link and
arming state, so it is useful during a bench session too.
"""

import glob
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleStatus

from drone_testing.px4_topics import subscribe_versioned

try:
    import serial
except ImportError:  # pragma: no cover - the node degrades to logging only
    serial = None


class LcdStatus(Node):

    # Rows are refreshed at this rate. The Arduino times out after 3 s of
    # silence, so anything above ~1 Hz keeps the link alive comfortably.
    # The sketch only repaints rows whose text actually changed, so a higher
    # rate here does not cost redraw time.
    UPDATE_HZ = 5.0
    STATUS_STALE_SECONDS = 2.0

    # Banner is 12 chars at text size 3. Labels are shortened to fit.
    STAGE_LABELS = {
        'PREPARATION': 'PREP',
        'OFFBOARD_REQUEST': 'WAIT OFB',
        'ARMING': 'ARMING',
        'GROUND_WAIT': 'GND WAIT',
        'TAKEOFF': 'TAKEOFF',
        'HOLD': 'HOLD',
        'SCAN': 'SCANNING',
        'LOCK': 'WIN LOCK',
        'STEP': 'STEP',
        'STEP_HOLD': 'SETTLING',
        'POST_HOLD': 'HOLD',
        'LANDING': 'LANDING',
        'DISARMING': 'DISARM',
        'KILLING': 'KILL',
        'DONE': 'DONE',
    }

    # 0 grey/idle, 1 green/ok, 2 amber/busy, 3 red/alarm.
    STAGE_SEVERITY = {
        'PREPARATION': 0,
        'OFFBOARD_REQUEST': 0,
        'ARMING': 2,
        'GROUND_WAIT': 2,
        'TAKEOFF': 2,
        'HOLD': 1,
        'SCAN': 2,
        'LOCK': 1,
        'STEP': 2,
        'STEP_HOLD': 2,
        'POST_HOLD': 1,
        'LANDING': 2,
        'DISARMING': 2,
        'KILLING': 3,
        'DONE': 0,
    }

    XY_LABELS = {
        'POS': 'pos hold',
        'FLO': 'vel hold',
        '---': 'vel hold',
    }

    NAV_STATE_OFFBOARD = 14

    def __init__(self):
        super().__init__('lcd_status')

        self.port = self.declare_parameter('port', '').value
        self.baud = self.declare_parameter('baud', 115200).value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(String, 'takeoff_status',
                                 self.status_callback, 10)
        # Window detection, straight from window_detect. Subscribed here rather
        # than routed through the flight node so the display is honest about the
        # camera even when no flight node is running.
        self.create_subscription(Bool, 'window_detected',
                                 self.window_callback, 10)
        self.vehicle_status_subs = subscribe_versioned(
            self, VehicleStatus, 'vehicle_status',
            self.vehicle_status_callback, sensor_qos)

        self.status_fields = None
        self.status_time = 0.0
        self.window_detected = False
        self.window_time = 0.0
        self.arming_state = None
        self.nav_state = None

        self.ser = None
        self._last_rows = [None] * 6
        self._last_banner = None
        self._last_severity = None
        self._open_serial()

        self.create_timer(1.0 / self.UPDATE_HZ, self.timer_callback)

    # ------------------------------------------------------------- serial

    def _find_port(self):
        """Auto-detect the Arduino unless a port was given explicitly."""
        if self.port:
            return self.port
        # ACM first: an Uno enumerates as ttyACM*. CH340 clones are ttyUSB*.
        candidates = sorted(glob.glob('/dev/ttyACM*')) + sorted(glob.glob('/dev/ttyUSB*'))
        return candidates[0] if candidates else None

    def _open_serial(self):
        if serial is None:
            self.get_logger().error("pyserial not installed; TFT output disabled.")
            return

        port = self._find_port()
        if port is None:
            self.get_logger().warning("No Arduino serial port found.",
                                      throttle_duration_sec=5.0)
            return

        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.1)
            # The Uno resets when the port opens; give the bootloader time to
            # finish or the first rows are swallowed.
            time.sleep(2.0)
            # Force a full repaint after a (re)connect.
            self._last_rows = [None] * 6
            self._last_banner = None
            self._last_severity = None
            self._send('C', '')
            self.get_logger().info(f"TFT connected on {port} @ {self.baud}.")
        except Exception as exc:
            self.ser = None
            self.get_logger().warning(f"Could not open {port}: {exc}",
                                      throttle_duration_sec=5.0)

    def _send(self, key, text):
        if self.ser is None:
            return
        try:
            self.ser.write(f"{key}:{text}\n".encode('ascii', 'replace'))
        except Exception as exc:
            self.get_logger().warning(f"LCD write failed: {exc}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # --------------------------------------------------------------- subs

    def status_callback(self, msg):
        parts = msg.data.split('|')
        if len(parts) == 5:
            self.status_fields = parts
            self.status_time = time.monotonic()

    def window_callback(self, msg):
        self.window_detected = msg.data
        self.window_time = time.monotonic()

    def window_text(self):
        """'win YES' / 'win no' / 'win --' when the detector is not talking."""
        if time.monotonic() - self.window_time > self.STATUS_STALE_SECONDS:
            return 'win --'
        return 'win YES' if self.window_detected else 'win no'

    def vehicle_status_callback(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    # -------------------------------------------------------------- render

    def _screen(self):
        """Return (banner, severity, [row1..row6]). Rows are <=20 chars."""
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        fresh = (self.status_fields is not None
                 and time.monotonic() - self.status_time < self.STATUS_STALE_SECONDS)

        if self.arming_state is None:
            return "NO PX4", 3, [
                "dds agent down?",
                "no vehicle_status",
                "", "", "", "",
            ]

        nav = f"nav {self.nav_state}"
        if self.nav_state == self.NAV_STATE_OFFBOARD:
            nav = "nav OFFBOARD"

        if not fresh:
            # offboard_takeoff is not running, or has stopped publishing.
            return "NO NODE", 0, [
                "ARMED" if armed else "disarmed",
                "takeoff node down",
                "", self.window_text(), "",
                nav,
            ]

        stage, arm, alt, xy, detail = self.status_fields
        banner = self.STAGE_LABELS.get(stage, stage[:12])
        severity = self.STAGE_SEVERITY.get(stage, 0)

        alt_str = "alt   --.-- m" if alt == 'nan' else f"alt {float(alt):+.2f} m"
        flow_ok = xy in ('POS', 'FLO')

        rows = [
            "ARMED" if armed else "disarmed",
            alt_str,
            f"xy  {self.XY_LABELS.get(xy, xy)}",
            f"flow {'ok' if flow_ok else 'NO'}  {self.window_text()}",
            detail,
            nav,
        ]
        return banner, severity, [r[:20] for r in rows]

    def timer_callback(self):
        if self.ser is None:
            self._open_serial()
            if self.ser is None:
                return

        banner, severity, rows = self._screen()

        # Only push what changed. The sketch also filters, but not sending it
        # at all keeps the 115200 link idle instead of pushing ~500 B/s of
        # identical text.
        if severity != self._last_severity:
            self._send('S', str(severity))
            self._last_severity = severity
        if banner != self._last_banner:
            self._send('T', banner)
            self._last_banner = banner
        for i, text in enumerate(rows):
            if text != self._last_rows[i]:
                self._send(str(i + 1), text)
                self._last_rows[i] = text

    def destroy_node(self):
        if self.ser is not None:
            try:
                self._send('S', '0')
                self._send('T', 'STOPPED')
                self._send('1', 'node exited')
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LcdStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
