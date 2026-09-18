"""
Live doll count, as one big number filling the terminal.

    ros2 run drone_testing doll_count_gui
    ros2 run drone_testing doll_count_gui --ros-args -p offset:=3

Subscribes to /doll_count -- the cumulative geotagged total doll_detect
publishes, the same topic room_display and qgc_doll_status read -- and draws
it, and nothing else, centred and as large as the window allows.

THE OFFSET
----------
What is drawn is `doll_count - offset`, clamped at zero, with `offset`
defaulting to 2.

This is a display-side correction only. It does not touch doll_detect, the
geotagged report on /doll_report, or anything the mission logs -- those keep
the raw count, so the subtraction can never be mistaken for the detector
having found fewer dolls than it did. Two places read the adjusted number and
both are this node.

The clamp matters: with offset 2 and a raw count of 1 the honest answer is 0,
not -1. A negative doll count would be read as a display fault.

Set it three ways, in increasing order of convenience:

    ros2 run ... --ros-args -p offset:=3      at startup
    ros2 param set /doll_count_gui offset 3   while running, from elsewhere
    + and - keys                              while running, in the window

The parameter is re-read every frame, so all three agree at all times.

WHAT IS ON SCREEN
-----------------
The number. That is the whole design -- it is meant to be readable across a
room mid-flight, so there is no header, no topic name and no status line
competing with it.

Two things are drawn that are not the count, and only because the alternative
is worse:

    ----   no message has arrived on /doll_count yet. Drawing '0' here would
           be indistinguishable from a real zero and would claim the mission
           is running and has found nothing, which is a different fact from
           "nothing is publishing".
    dim    the count is stale (nothing for `stale_seconds`, default 5 s). The
           number is still the last real value; it is dimmed rather than
           hidden because the last known count is what you want if the
           detector has dropped out mid-flight.

Keys: q quits, + / - adjust the offset, r re-reads the parameter.
"""

import curses
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32

# 5x7 block digits. Deliberately a fixed bitmap rather than a font: the whole
# point is that it scales to whatever the window is, and a bitmap scales by an
# integer factor with no dependency and no antialiasing to go wrong over ssh.
GLYPHS = {
    '0': ("█████",
          "█   █",
          "█   █",
          "█   █",
          "█   █",
          "█   █",
          "█████"),
    '1': ("   ██",
          "  ███",
          "   ██",
          "   ██",
          "   ██",
          "   ██",
          "  ███"),
    '2': ("█████",
          "    █",
          "    █",
          "█████",
          "█    ",
          "█    ",
          "█████"),
    '3': ("█████",
          "    █",
          "    █",
          "█████",
          "    █",
          "    █",
          "█████"),
    '4': ("█   █",
          "█   █",
          "█   █",
          "█████",
          "    █",
          "    █",
          "    █"),
    '5': ("█████",
          "█    ",
          "█    ",
          "█████",
          "    █",
          "    █",
          "█████"),
    '6': ("█████",
          "█    ",
          "█    ",
          "█████",
          "█   █",
          "█   █",
          "█████"),
    '7': ("█████",
          "    █",
          "    █",
          "   █ ",
          "  █  ",
          "  █  ",
          "  █  "),
    '8': ("█████",
          "█   █",
          "█   █",
          "█████",
          "█   █",
          "█   █",
          "█████"),
    '9': ("█████",
          "█   █",
          "█   █",
          "█████",
          "    █",
          "    █",
          "█████"),
    '-': ("     ",
          "     ",
          "     ",
          "█████",
          "     ",
          "     ",
          "     "),
}
GLYPH_W, GLYPH_H = 5, 7
GAP = 1                 # blank columns between digits, in unscaled cells


class DollCountGui(Node):
    """Subscriber half. Owns no curses state: the draw loop reads these."""

    def __init__(self):
        super().__init__('doll_count_gui')

        self.offset = int(self.declare_parameter(
            'offset', 2,
            ParameterDescriptor(
                dynamic_typing=True,
                description='Subtracted from the published count before it is '
                            'displayed. Display-side only; clamped at 0.')).value)
        self.stale_seconds = float(self.declare_parameter(
            'stale_seconds', 5.0,
            ParameterDescriptor(dynamic_typing=True)).value)
        topic = str(self.declare_parameter('count_topic', 'doll_count').value)

        self._lock = threading.Lock()
        self.raw = None             # None = nothing has ever arrived
        self.last_rx = 0.0

        # Depth 1, RELIABLE: this is a low-rate cumulative counter, so the only
        # message worth having is the newest one, and doll_detect publishes it
        # reliably. Matching that avoids the silent no-data case an incompatible
        # best-effort subscription would give.
        self.create_subscription(
            Int32, topic, self._on_count,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        self.get_logger().info(
            f"Displaying {topic} minus offset {self.offset} (clamped at 0).")

    def _on_count(self, msg):
        with self._lock:
            self.raw = int(msg.data)
            self.last_rx = time.monotonic()

    def snapshot(self):
        """(displayed, raw, stale, have_data) under one lock acquisition."""
        # Re-read the parameter every frame so `ros2 param set` works live and
        # cannot disagree with the +/- keys.
        try:
            self.offset = int(self.get_parameter('offset').value)
        except Exception:
            pass
        with self._lock:
            raw, last = self.raw, self.last_rx
        if raw is None:
            return None, None, False, False
        shown = max(0, raw - self.offset)
        return shown, raw, (time.monotonic() - last) > self.stale_seconds, True

    def set_offset(self, value):
        value = max(0, int(value))
        self.offset = value
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('offset', value=value)])
        except Exception:
            pass


def _render(stdscr, node):
    curses.curs_set(0)
    stdscr.nodelay(True)
    have_colour = curses.has_colors()
    if have_colour:
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)   # live
        curses.init_pair(2, curses.COLOR_YELLOW, -1)  # stale
        curses.init_pair(3, curses.COLOR_BLUE, -1)    # no data

    while rclpy.ok():
        ch = stdscr.getch()
        if ch in (ord('q'), ord('Q')):
            return
        if ch in (ord('+'), ord('=')):
            node.set_offset(node.offset + 1)
        elif ch in (ord('-'), ord('_')):
            node.set_offset(node.offset - 1)

        shown, _raw, stale, have = node.snapshot()
        text = str(shown) if have else '-' * 4

        h, w = stdscr.getmaxyx()
        # Largest integer scale that fits, with a one-cell margin so the
        # bottom-right cell is never written (writing it throws on curses).
        span = len(text) * GLYPH_W + (len(text) - 1) * GAP
        scale = max(1, min((w - 2) // max(1, span), (h - 2) // GLYPH_H))
        blk_w, blk_h = span * scale, GLYPH_H * scale
        x0, y0 = max(0, (w - blk_w) // 2), max(0, (h - blk_h) // 2)

        attr = curses.A_BOLD
        if have_colour:
            attr |= curses.color_pair(3 if not have else (2 if stale else 1))
        if stale and have:
            attr |= curses.A_DIM

        stdscr.erase()
        for row in range(GLYPH_H):
            line = []
            for i, c in enumerate(text):
                g = GLYPHS.get(c, GLYPHS['-'])[row]
                if i:
                    line.append(' ' * (GAP * scale))
                line.append(''.join(p * scale for p in g))
            painted = ''.join(line)
            for rep in range(scale):
                y = y0 + row * scale + rep
                if 0 <= y < h - 1:
                    try:
                        stdscr.addstr(y, x0, painted[:max(0, w - x0 - 1)], attr)
                    except curses.error:
                        pass
        stdscr.refresh()
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = DollCountGui()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        curses.wrapper(_render, node)
    except KeyboardInterrupt:
        pass
    finally:
        shown, raw, _stale, have = node.snapshot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Printed AFTER curses has restored the terminal, so it survives in the
        # scrollback -- the count is the output of the flight and losing it to
        # a screen clear on exit would be the one unforgivable bug here.
        if have:
            print(f"doll count: {shown}   (raw {raw} - offset {node.offset})")
        else:
            print("doll count: no data received on /doll_count")


if __name__ == '__main__':
    main()
