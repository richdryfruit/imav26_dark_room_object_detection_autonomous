#!/usr/bin/env python3
"""
Bridge from the window-room mission to the Arduino TFT.

Flash arduino/room_status/room_status.ino first. This node does all the
formatting -- the Arduino stores strings and numbers and draws them -- so the
wording, the colours and the layout change here, without reflashing anything.

What it shows, which is what was asked for:

    the STATE        window detected -> inside the room -> window detected
                     again -> out of the room, plus the flying stages in
                     between and the landing at the end
    the TOTAL        cumulative dolls counted this flight
    the NOW          dolls in the current camera frame
    three rows       stage detail, altitude and flow, and the doll detector's
                     own state

It reads four topics and holds an opinion about none of them:

    mission_phase    String, PHASE|STAGE|detail, from window_room_traverse
    takeoff_status   String, stage|armed|alt|xy|detail, from the same node
                     (the format every flight node in this package publishes,
                     so this display also works against a plain traversal)
    doll_count       Int32, cumulative
    dolls_visible    Int32, this frame

Nothing here is required for flight. If the Arduino is absent, unplugged, or
enumerates on a different port, the node logs it once every five seconds and
carries on; it never blocks and it never throws into the executor.

This is a SEPARATE node from lcd_status.py rather than a change to it:
lcd_status drives the six-row tft_status.ino sketch that the existing
traversal flights use, and the two sketches are not protocol-compatible
(row count, plus the D: counts key). Run whichever matches the sketch that is
actually flashed on the board.
"""

import glob
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

try:
    import serial
except ImportError:  # pragma: no cover - degrades to logging only
    serial = None


class RoomDisplay(Node):

    # The sketch times out after 3 s of silence and shows NO LINK, so anything
    # above ~1 Hz keeps the link alive comfortably. It only repaints what
    # changed, so a higher rate costs nothing but a few hundred bytes a second
    # on an otherwise idle 115200 link.
    UPDATE_HZ = 5.0
    STALE_SECONDS = 3.0

    # PHASE (from mission_phase) -> (banner, severity). Severity is the
    # sketch's colour: 0 grey idle, 1 green ok, 2 amber busy, 3 red alarm.
    #
    # The four the mission is actually judged on -- window found, inside,
    # window found again, outside -- are GREEN. Everything that is a
    # transition is amber. That way a glance at the colour answers "has it got
    # somewhere yet?" without reading the word.
    PHASE_BANNERS = {
        'OUTSIDE': ('READY', 0),
        'SEARCH': ('SEARCHING', 2),
        'WINDOW_IN': ('WINDOW', 1),
        'ENTERING': ('ENTERING', 2),
        'INSIDE': ('IN ROOM', 1),
        'ROOM': ('ROOM RUN', 2),
        'SEARCH_OUT': ('FIND WIN2', 2),
        'WINDOW_OUT': ('WINDOW 2', 1),
        'EXITING': ('EXITING', 2),
        'OUT': ('OUTSIDE', 1),
        'LANDING': ('LANDING', 2),
        'DISARMING': ('DISARM', 2),
        'KILLING': ('KILL', 3),
        'DONE': ('DONE', 0),
    }

    # Fallback for the stage-only path, i.e. when mission_phase is absent
    # because a plain window_traverse (or nothing) is running.
    STAGE_BANNERS = {
        'PREPARATION': ('PREP', 0),
        'OFFBOARD_REQUEST': ('WAIT OFB', 0),
        'ARMING': ('ARMING', 2),
        'GROUND_WAIT': ('GND WAIT', 2),
        'TAKEOFF': ('TAKEOFF', 2),
        'HOLD': ('HOLD', 1),
        'SCAN': ('SEARCHING', 2),
        'LOCK': ('WINDOW', 1),
        'RECENTRE': ('RECENTRE', 2),
        'AIM': ('AIM', 2),
        'ALIGN': ('ALIGN', 2),
        'TRAVERSE': ('THROUGH', 2),
        'CLEAR': ('CLEAR', 1),
        'ROOM_MOVE': ('ROOM RUN', 2),
        'ROOM_TURN': ('ROOM TURN', 2),
        'ROOM_HOLD': ('ROOM RUN', 2),
        'RELOCK': ('FIND WIN2', 2),
        'LANDING': ('LANDING', 2),
        'DISARMING': ('DISARM', 2),
        'KILLING': ('KILL', 3),
        'DONE': ('DONE', 0),
    }

    def __init__(self):
        super().__init__('room_display')

        self.port = str(self.declare_parameter('port', '').value)
        self.baud = int(self.declare_parameter('baud', 115200).value)

        self.create_subscription(String, 'mission_phase',
                                 self.phase_callback, 10)
        self.create_subscription(String, 'takeoff_status',
                                 self.status_callback, 10)
        self.create_subscription(Int32, 'doll_count', self.count_callback, 10)
        self.create_subscription(Int32, 'dolls_visible',
                                 self.visible_callback, 10)

        self.phase = None
        self.phase_time = 0.0
        self.status = None
        self.status_time = 0.0
        self.total = 0
        self.visible = 0
        self.doll_time = 0.0

        self.ser = None
        self._last_rows = [None] * 3
        self._last_banner = None
        self._last_severity = None
        self._last_counts = None
        self._open_serial()

        self.create_timer(1.0 / self.UPDATE_HZ, self.timer_callback)

    # ------------------------------------------------------------- serial

    def _find_port(self):
        if self.port:
            return self.port
        # ACM first: an Uno enumerates as ttyACM*. CH340 clones are ttyUSB*.
        candidates = (sorted(glob.glob('/dev/ttyACM*'))
                      + sorted(glob.glob('/dev/ttyUSB*')))
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
            # The Uno resets when the port is opened; wait for the bootloader
            # or the first lines are swallowed.
            time.sleep(2.0)
            self._last_rows = [None] * 3
            self._last_banner = None
            self._last_severity = None
            self._last_counts = None
            self._send('C', '')
            self.get_logger().info(f"Room TFT connected on {port} @ {self.baud}.")
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
            self.get_logger().warning(f"TFT write failed: {exc}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # --------------------------------------------------------------- subs

    def phase_callback(self, msg):
        parts = msg.data.split('|')
        if len(parts) >= 2:
            self.phase = parts
            self.phase_time = time.monotonic()

    def status_callback(self, msg):
        parts = msg.data.split('|')
        if len(parts) == 5:
            self.status = parts
            self.status_time = time.monotonic()

    def count_callback(self, msg):
        self.total = int(msg.data)
        self.doll_time = time.monotonic()

    def visible_callback(self, msg):
        self.visible = int(msg.data)
        self.doll_time = time.monotonic()

    # -------------------------------------------------------------- render

    def _fresh(self, stamp):
        return time.monotonic() - stamp < self.STALE_SECONDS

    def _banner(self):
        """(text, severity) for the big state line.

        mission_phase wins when it is fresh -- it is the only source that
        knows which SIDE of the window the aircraft is on, which is the
        distinction the whole display exists to make. The stage table is the
        fallback for when only a flight node's status line is available, and
        'NO NODE' is what is shown when neither is.
        """
        if self.phase is not None and self._fresh(self.phase_time):
            label = self.phase[0]
            if label in self.PHASE_BANNERS:
                return self.PHASE_BANNERS[label]
            return label[:13], 2

        if self.status is not None and self._fresh(self.status_time):
            stage = self.status[0]
            if stage in self.STAGE_BANNERS:
                return self.STAGE_BANNERS[stage]
            return stage[:13], 2

        return 'NO NODE', 0

    def _rows(self):
        """Three body rows, each at most 20 characters at text size 2."""
        if self.status is not None and self._fresh(self.status_time):
            stage, arm, alt, xy, detail = self.status
            alt_str = 'alt  --.-- m' if alt == 'nan' else f"alt {float(alt):+.2f} m"
            row0 = f"{stage[:11].lower()} {detail}"[:20]
            row1 = f"{alt_str}  {'ARM' if arm == 'ARM' else 'dis'}"[:20]
            row2 = ('xy pos hold' if xy == 'POS'
                    else 'xy vel hold' if xy == 'FLO' else 'xy NO FLOW')
        else:
            row0 = 'flight node down'
            row1 = ''
            row2 = ''

        if self.doll_time == 0.0:
            dolls = 'yolo: not running'
        elif self._fresh(self.doll_time):
            dolls = 'yolo: running'
        else:
            dolls = 'yolo: idle'
        row2 = f"{row2}  {dolls}"[:20] if row2 else dolls[:20]

        return [row0[:20], row1[:20], row2[:20]]

    def timer_callback(self):
        if self.ser is None:
            self._open_serial()
            if self.ser is None:
                return

        banner, severity = self._banner()
        rows = self._rows()
        counts = (self.visible, self.total)

        # Push only what changed. The sketch filters too, but not sending it at
        # all keeps the link idle instead of pushing identical text five times
        # a second at a board that is also driving a slow parallel bus.
        if severity != self._last_severity:
            self._send('S', str(severity))
            self._last_severity = severity
        if banner != self._last_banner:
            self._send('T', banner)
            self._last_banner = banner
        if counts != self._last_counts:
            self._send('D', f"{counts[0]},{counts[1]}")
            self._last_counts = counts
        for i, text in enumerate(rows):
            if text != self._last_rows[i]:
                self._send(str(i + 1), text)
                self._last_rows[i] = text

    def destroy_node(self):
        if self.ser is not None:
            try:
                # Leave the total on screen. It is the result of the flight and
                # the node exiting is not a reason to stop showing it.
                self._send('S', '0')
                self._send('T', 'STOPPED')
                self._send('1', 'node exited')
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoomDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
