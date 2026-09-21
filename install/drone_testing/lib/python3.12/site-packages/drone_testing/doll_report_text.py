"""
The geotagged doll count and every doll's position, as plain log lines.

    ros2 run drone_testing doll_report_text

WHY THIS EXISTS ALONGSIDE doll_count_gui
-----------------------------------------
doll_count_gui draws the count as one big number filling the terminal, which
is the right thing to read across a room mid-flight. It is a curses
application, so it owns the terminal and needs a real tty -- which means it
CANNOT be started from `ros2 launch`: launch gives its processes a pipe, not
a tty, and curses fails on it. That is why it is absent from every launch
file in this package.

This node is the other half of that trade. It prints instead of drawing, so
it runs happily under launch and its output lands in the same terminal as
the flight node, interleaved with the mission log where it can be read
afterwards in the launch log file. Run doll_count_gui in a second pane when
you want the big number; run this to have the count in the flight log.

Both read the same two topics doll_detect publishes, so they cannot disagree:

    /doll_count    Int32,  cumulative geotagged total, never decreases
    /doll_report   String, "n|id:x,y,z|id:x,y,z|..." in metres NED

THE OFFSET
----------
What is PRINTED as the count is `doll_count - offset`, clamped at zero, with
`offset` defaulting to 2 -- the same display-side correction doll_count_gui
applies, deliberately with the same default so the two displays never show
different numbers.

The raw count is printed alongside it, in brackets, for exactly the reason
the correction is display-side in the first place: nothing here touches
doll_detect, /doll_report or the mission log, so the adjustment can never be
mistaken for the detector having found fewer dolls than it did. If the two
numbers ever need reconciling after a flight, both are in the log.

The clamp matters: with offset 2 and a raw count of 1 the honest answer is 0,
not -1, because a negative doll count reads as a display fault.

The parameter is re-read on every print, so `ros2 param set
/doll_report_text offset 3` takes effect immediately, as it does on the GUI.

WHAT IS PRINTED, AND WHEN
-------------------------
A line on every CHANGE of the count, and a full position table with it:

    DOLLS: 3   (raw 5, offset 2)
      #1  N +2.10  E -0.45  D +0.85
      #2  N +3.02  E +1.18  D +0.91

plus a heartbeat at `period` seconds so a silent mission is visibly still
counting zero rather than visibly dead. Printing on change rather than on
every message is what keeps the flight log readable: doll_detect republishes
at its timer rate whether or not anything was found.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String


def parse_report(text):
    """"n|id:x,y,z|..." -> [(id, x, y, z)]. Malformed entries are skipped.

    Tolerant on purpose: this is a display, and a half-written line from a
    detector that is mid-update must degrade to showing fewer rows, never to
    taking the node down in the middle of a flight.
    """
    out = []
    for chunk in str(text).split('|')[1:]:
        head, _, tail = chunk.partition(':')
        parts = tail.split(',')
        if len(parts) != 3:
            continue
        try:
            x, y, z = (float(v) for v in parts)
        except ValueError:
            continue
        if not all(math.isfinite(v) for v in (x, y, z)):
            continue
        out.append((head.strip(), x, y, z))
    return out


class DollReportText(Node):

    OFFSET = 2
    PERIOD = 10.0

    def __init__(self):
        super().__init__('doll_report_text')

        self.declare_parameter('offset', self.OFFSET)
        self.PERIOD = float(self.declare_parameter('period', self.PERIOD).value)

        self.count = None
        self.report = ''
        self.last_shown = None

        # BEST_EFFORT with depth 1, matching how the other displays subscribe:
        # the newest count is the only one worth having, and a display must
        # never apply back-pressure to the detector.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.create_subscription(Int32, 'doll_count', self.count_callback, qos)
        self.create_subscription(String, 'doll_report', self.report_callback, qos)

        self.create_timer(self.PERIOD, self.heartbeat)
        self.get_logger().info(
            f"Doll report: printing /doll_count (minus offset "
            f"{self.offset()}) and /doll_report positions on every change, "
            f"and a heartbeat every {self.PERIOD:.0f} s.")

    def offset(self):
        """Re-read every time, so `ros2 param set` takes effect immediately."""
        try:
            return max(0, int(self.get_parameter('offset').value))
        except Exception:
            return self.OFFSET

    def shown(self, raw):
        """The displayed count. Clamped: a negative count reads as a fault."""
        return max(0, raw - self.offset())

    def count_callback(self, msg):
        self.count = int(msg.data)
        self.print_if_changed()

    def report_callback(self, msg):
        self.report = msg.data
        self.print_if_changed()

    def print_if_changed(self):
        if self.count is None:
            return
        key = (self.count, self.report)
        if key == self.last_shown:
            return
        self.last_shown = key
        self.emit()

    def emit(self):
        raw = 0 if self.count is None else self.count
        off = self.offset()
        lines = [f"DOLLS: {self.shown(raw)}   (raw {raw}, offset {off})"]
        for i, (did, x, y, z) in enumerate(parse_report(self.report), 1):
            lines.append(f"  #{i}  N {x:+.2f}  E {y:+.2f}  D {z:+.2f}   (id {did})")
        if len(lines) == 1:
            lines.append("  no geotagged positions yet")
        self.get_logger().warning('\n'.join(lines))

    def heartbeat(self):
        if self.count is None:
            self.get_logger().info(
                "No /doll_count yet -- is doll_detect running and enabled?")
            return
        self.emit()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DollReportText()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # Guarded: the executor may already have brought the context down, and
        # a second shutdown() raises over the top of whatever really happened.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
