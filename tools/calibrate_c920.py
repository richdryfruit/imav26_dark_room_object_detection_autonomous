#!/usr/bin/env python3
"""Calibrate the down camera (the Logitech "C920"), guided, on a screen.

    python3 ~/offboard_imav26_test/tools/calibrate_c920.py

Run it on a monitor plugged into the Jetson (or any machine with a desktop and
this camera). It walks through ~19 steps. Each step draws a box on the video
and says, in words, where to put the chessboard (9x6 squares = 8x5 inner
corners, 26 mm squares by default): middle, corners, close, far, tilted. Put the board there
and hold it still; it captures by itself and moves on. At the end it solves,
shows the result, and writes a ROS camera_info YAML (~/c920_320x240.yaml) for
usb_cam's camera_info_url.

Before starting it stops usb_cam (only one program can hold the camera) and
locks the focus with v4l2-ctl: UVC autofocus comes back on at every power-up
and would move the focal length under the calibration.

Keys: S skip this step, U undo the last capture, Q quit without saving.
"""

import argparse
import glob
import math
import os
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np

YAML = """image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{k}]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{d}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [{p}]
"""

MID, NEAR, FAR = (0.28, 0.50), (0.46, 1.00), (0.00, 0.28)
# (instruction, x, y, size range, tilt). x/y: 0 = left/top, 1 = right/bottom
# of the room the board has to move in. tilt: which edge must look shorter
# (turned away from the camera), 'any', or None for don't care.
STEPS = [
    ('Hold the board flat in the MIDDLE', .5, .5, MID, None),
    ('Move it to the TOP-LEFT', 0, 0, MID, None),
    ('Move it to the TOP-RIGHT', 1, 0, MID, None),
    ('Move it to the BOTTOM-RIGHT', 1, 1, MID, None),
    ('Move it to the BOTTOM-LEFT', 0, 1, MID, None),
    ('Bring it CLOSE, filling the picture', .5, .5, NEAR, None),
    ('Move it FAR away, small, in the middle', .5, .5, FAR, None),
    ('FAR away, TOP-LEFT', 0, 0, FAR, None),
    ('FAR away, TOP-RIGHT', 1, 0, FAR, None),
    ('FAR away, BOTTOM-RIGHT', 1, 1, FAR, None),
    ('FAR away, BOTTOM-LEFT', 0, 1, FAR, None),
    ('Middle, TILTED: turn the LEFT edge away from the camera', .5, .5, MID,
     'left'),
    ('Middle, TILTED: turn the RIGHT edge away from the camera', .5, .5, MID,
     'right'),
    ('Middle, TILTED: turn the TOP edge away from the camera', .5, .5, MID,
     'top'),
    ('Middle, TILTED: turn the BOTTOM edge away from the camera', .5, .5, MID,
     'bottom'),
    ('TOP-LEFT, TILTED (any way)', 0, 0, MID, 'any'),
    ('TOP-RIGHT, TILTED (any way)', 1, 0, MID, 'any'),
    ('BOTTOM-RIGHT, TILTED (any way)', 1, 1, MID, 'any'),
    ('BOTTOM-LEFT, TILTED (any way)', 0, 1, MID, 'any'),
]
POS_TOL = 0.25          # of the free room, per axis
TILT_RATIO = 0.85       # far edge at most this long relative to the near one
HOLD_S = 0.5            # still this long before capturing
STILL_PX = 1.5          # mean corner motion per frame that counts as still
MIN_VIEWS = 12
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN, YELLOW, ORANGE, RED = (0, 200, 0), (0, 220, 255), (0, 140, 255), (0, 0, 230)
OPPOSITE = {'left': 'right', 'right': 'left', 'top': 'bottom', 'bottom': 'top'}


def find_device():
    """First capture node of a non-RealSense V4L2 camera, like device_utils."""
    for path in sorted(glob.glob('/sys/class/video4linux/video*'),
                       key=lambda p: int(p.rsplit('video', 1)[1])):
        try:
            name = open(os.path.join(path, 'name')).read().strip()
            index = open(os.path.join(path, 'index')).read().strip()
        except OSError:
            continue
        if 'realsense' in name.lower() or index != '0':
            continue
        return '/dev/' + os.path.basename(path), name
    return None, None


def lock_focus(device, value):
    if not shutil.which('v4l2-ctl'):
        return 'v4l2-ctl missing (sudo apt install v4l-utils): focus NOT locked'
    for c in ('focus_automatic_continuous=0', f'focus_absolute={value}'):
        subprocess.run(['v4l2-ctl', '-d', device, '-c', c],
                       capture_output=True)
    r = subprocess.run(['v4l2-ctl', '-d', device, '-C',
                        'focus_automatic_continuous,focus_absolute'],
                       capture_output=True, text=True)
    return ' '.join(r.stdout.split()) or r.stderr.strip()


def pose(corners, cols, w, h):
    """x, y (0..1 in the room left), size (0..~0.8), and edge lengths by side."""
    c = corners.reshape(-1, 2)
    q = np.array([c[0], c[cols - 1], c[-1], c[-cols]])
    x_, y_ = q[:, 0], q[:, 1]
    area = 0.5 * abs(np.dot(x_, np.roll(y_, 1)) - np.dot(y_, np.roll(x_, 1)))
    border = math.sqrt(area)
    mx, my = c.mean(axis=0)
    x = min(1.0, max(0.0, (mx - border / 2) / max(1.0, w - border)))
    y = min(1.0, max(0.0, (my - border / 2) / max(1.0, h - border)))
    size = math.sqrt(area / (w * h))
    # Name each outer edge by where its midpoint sits relative to the board
    # centre, so it works whichever way up the board is held.
    centre = q.mean(axis=0)
    sides = {}
    for i in range(4):
        a, b = q[i], q[(i + 1) % 4]
        d = (a + b) / 2 - centre
        side = (('right' if d[0] > 0 else 'left') if abs(d[0]) > abs(d[1])
                else ('bottom' if d[1] > 0 else 'top'))
        sides[side] = float(np.linalg.norm(b - a))
    return x, y, size, sides


def check(step, p):
    """(matches, what to tell the user) for one detection against one step."""
    _, tx, ty, (s0, s1), tilt = step
    x, y, size, sides = p
    if size < s0:
        return False, 'Bring the board CLOSER'
    if size > s1:
        return False, 'Move the board FURTHER away'
    dx, dy = x - tx, y - ty
    if abs(dx) > POS_TOL or abs(dy) > POS_TOL:
        words = []
        if abs(dy) > POS_TOL:
            words.append('UP' if dy > 0 else 'DOWN')
        if abs(dx) > POS_TOL:
            words.append('LEFT' if dx > 0 else 'RIGHT')
        return False, 'Move the board ' + ' and '.join(words)
    ratios = ({s: sides[s] / max(1e-6, sides[OPPOSITE[s]]) for s in sides}
              if len(sides) == 4 else {})
    if tilt is None:
        pass
    elif tilt == 'any':
        if not ratios or min(ratios.values()) > TILT_RATIO:
            return False, 'TILT the board more (about 30-45 degrees)'
    elif ratios.get(tilt, 1.0) > TILT_RATIO:
        return False, f'Turn the {tilt.upper()} edge further AWAY'
    return True, 'GOOD - hold still'


def target_box(step, w, h, aspect):
    """Pixel rectangle roughly where the board's corner grid should sit."""
    _, tx, ty, (s0, s1), _ = step
    size = (s0 + min(s1, 0.6)) / 2
    border = size * math.sqrt(w * h)
    bw, bh = border * math.sqrt(aspect), border / math.sqrt(aspect)
    cx = tx * (w - border) + border / 2
    cy = ty * (h - border) + border / 2
    x0 = int(max(2, min(w - bw - 2, cx - bw / 2)))
    y0 = int(max(2, min(h - bh - 2, cy - bh / 2)))
    return x0, y0, int(x0 + bw), int(y0 + bh)


def wrap(text, n):
    out, line = [], ''
    for word in text.split():
        if line and len(line) + 1 + len(word) > n:
            out.append(line)
            line = word
        else:
            line = (line + ' ' + word).strip()
    return out + ([line] if line else [])


def put(img, text, org, scale=0.6, colour=(235, 235, 235), thick=1,
        label=False):
    if label:   # dark box behind text that sits on the video
        (tw, th), base = cv2.getTextSize(text, FONT, scale, thick)
        cv2.rectangle(img, (org[0] - 6, org[1] - th - 8),
                      (org[0] + tw + 6, org[1] + base + 4), (0, 0, 0), -1)
    cv2.putText(img, text, org, FONT, scale, colour, thick, cv2.LINE_AA)


def draw(frame, corners, pattern, step_i, n_views, message, colour, hold,
         flash):
    """Video (2x, target box, corners) on the left, instructions on the right."""
    h, w = frame.shape[:2]
    s = 2
    n_steps = len(STEPS)
    vis = cv2.resize(frame, (w * s, h * s), interpolation=cv2.INTER_LINEAR)
    if step_i < n_steps:
        aspect = (pattern[0] - 1) / (pattern[1] - 1)
        x0, y0, x1, y1 = (v * s for v in target_box(STEPS[step_i], w, h,
                                                    aspect))
        box_colour = GREEN if colour == GREEN else YELLOW
        cv2.rectangle(vis, (x0, y0), (x1, y1), box_colour, 3)
        put(vis, 'put the board here', (x0 + 10, y0 + 26), 0.55, box_colour,
            label=True)
    if corners is not None:
        cv2.drawChessboardCorners(vis, pattern, corners * s, True)
    cv2.rectangle(vis, (0, 0), (w * s - 1, h * s - 1),
                  (255, 255, 255) if flash else colour, 8)
    if flash:
        put(vis, 'CAPTURED!', (w * s // 2 - 110, h * s // 2), 1.5,
            (255, 255, 255), 3, label=True)

    pw = 460
    panel = np.full((h * s, pw, 3), 30, np.uint8)
    y = 96
    if step_i < n_steps:
        put(panel, f'STEP {step_i + 1} of {n_steps}', (18, 44), 1.1,
            (255, 255, 255), 2)
        for line in wrap(STEPS[step_i][0], 26):
            put(panel, line, (18, y), 0.85, YELLOW, 2)
            y += 38
    else:
        put(panel, 'ALL STEPS DONE', (18, 44), 1.1, GREEN, 2)
    y = max(y + 20, 230)
    cv2.rectangle(panel, (14, y - 34), (pw - 14, y + 14), colour, -1)
    cv2.putText(panel, message, (24, y), FONT, 0.62, (0, 0, 0), 2, cv2.LINE_AA)
    if hold > 0:
        cv2.rectangle(panel, (14, y + 22), (14 + int((pw - 28) * hold), y + 34),
                      GREEN, -1)
    y += 80
    put(panel, f'Pictures taken: {n_views}', (18, y), 0.65)
    cv2.rectangle(panel, (18, y + 16), (pw - 18, y + 34), (80, 80, 80), -1)
    cv2.rectangle(panel, (18, y + 16),
                  (18 + int((pw - 36) * step_i / n_steps), y + 34), GREEN, -1)
    put(panel, 'S = skip step    U = undo    Q = quit', (18, h * s - 18), 0.55,
        (170, 170, 170))
    return np.hstack([vis, panel])


def solve(objs, imgs, size):
    """calibrateCamera with k3 fixed (as ROS does), one outlier-rejection pass."""
    flags = cv2.CALIB_FIX_K3
    rms, k, d, rv, tv = cv2.calibrateCamera(objs, imgs, size, None, None,
                                            flags=flags)
    errs = []
    for o, i, r, t in zip(objs, imgs, rv, tv):
        proj, _ = cv2.projectPoints(o, r, t, k, d)
        # reshape both: OpenCV 5 returns (N, 2) here where 4.x gave (N, 1, 2)
        diff = proj.reshape(-1, 2) - i.reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))))
    errs = np.array(errs)
    keep = errs <= max(3.0 * np.median(errs), 0.5)
    if keep.sum() >= 10 and not keep.all():
        rms, k, d, _, _ = cv2.calibrateCamera(
            [o for o, kk in zip(objs, keep) if kk],
            [i for i, kk in zip(imgs, keep) if kk], size, None, None,
            flags=flags)
    else:
        keep[:] = True
    return rms, k, d, errs, keep


def write_yaml(path, name, w, h, k, d):
    proj = [k[0, 0], 0, k[0, 2], 0, 0, k[1, 1], k[1, 2], 0, 0, 0, 1, 0]
    with open(path, 'w') as f:
        f.write(YAML.format(
            w=w, h=h, name=name,
            k=', '.join(f'{v:.6f}' for v in k.flatten()),
            d=', '.join(f'{v:.6f}' for v in d.flatten()[:5]),
            p=', '.join(f'{v:.6f}' for v in proj)))


def result_screen(args, rms, k, d, errs, keep, w, h):
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    hfov = 2 * math.degrees(math.atan(0.5 * w / fx))
    vfov = 2 * math.degrees(math.atan(0.5 * h / fy))
    dd = d.flatten()[:5]
    verdict, colour = (('GOOD', GREEN) if rms < 0.5 else
                       ('OK', ORANGE) if rms < 1.0 else
                       ('POOR - run it again', RED))
    print('\n'.join([
        f"\n=== CALIBRATED {w}x{h}, {int(keep.sum())}/{len(errs)} pictures "
        f"used ===",
        f"reprojection error {rms:.3f} px -> {verdict}",
        f"fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}",
        f"distortion k1 k2 p1 p2 k3: {' '.join(f'{v:.5f}' for v in dd)}",
        f"horizontal FOV {hfov:.1f} deg, vertical FOV {vfov:.1f} deg",
        f"saved: {args.out}"]))
    panel = np.full((360, 900, 3), 30, np.uint8)
    put(panel, 'CALIBRATION DONE', (24, 50), 1.1, (255, 255, 255), 2)
    put(panel, f'Result: {verdict}  (error {rms:.2f} px)', (24, 110), 0.9,
        colour, 2)
    put(panel, f'Field of view: {hfov:.1f} deg across, {vfov:.1f} deg down',
        (24, 160), 0.7)
    put(panel, f'Saved to {args.out}', (24, 200), 0.7)
    put(panel, 'You can close this now (press any key).', (24, 300), 0.7,
        (170, 170, 170))
    cv2.imshow('Camera calibration', panel)
    cv2.waitKey(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--device', default='auto',
                    help='/dev/videoN, or auto (first non-RealSense camera)')
    ap.add_argument('--width', type=int, default=320)
    ap.add_argument('--height', type=int, default=240)
    ap.add_argument('--cols', type=int, default=8,
                    help='INNER corners across (squares across - 1)')
    ap.add_argument('--rows', type=int, default=5,
                    help='INNER corners down (squares down - 1)')
    ap.add_argument('--square', type=float, default=0.026, help='metres')
    ap.add_argument('--focus', type=int, default=0,
                    help='focus_absolute to lock (0 = infinity)')
    ap.add_argument('--out', default=os.path.expanduser('~/c920_320x240.yaml'))
    ap.add_argument('--name', default='c920')
    args = ap.parse_args()

    if not os.environ.get('DISPLAY'):
        sys.exit("No screen here. Run this in a terminal on the Jetson's own "
                 "monitor (log in on it, Ctrl+Alt+T).")

    if subprocess.run(['pkill', '-x', 'usb_cam_node_ex']).returncode == 0:
        print('stopped usb_cam so this script can use the camera')
        time.sleep(1.0)

    device, name = (find_device() if args.device == 'auto'
                    else (args.device, ''))
    if not device:
        sys.exit('No camera found. Is the Logitech plugged in?')
    print(f'camera: {device} {name}')
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f'Cannot open {device}: something else is using it.')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print('focus: ' + lock_focus(device, args.focus))

    ok, frame = cap.read()
    if not ok:
        sys.exit('Camera opened but gives no pictures.')
    h, w = frame.shape[:2]
    if (w, h) != (args.width, args.height):
        print(f'WARNING: camera gives {w}x{h}, not {args.width}x{args.height}')

    view_dir = os.path.splitext(args.out)[0] + '_views'
    os.makedirs(view_dir, exist_ok=True)
    pattern = (args.cols, args.rows)
    obj = np.zeros((args.cols * args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    obj *= args.square
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
             + cv2.CALIB_CB_FAST_CHECK)

    cv2.namedWindow('Camera calibration', cv2.WINDOW_AUTOSIZE)
    imgs, captured_at = [], []
    step_i, prev, still_since, last_capture = 0, None, None, 0.0
    try:
        while step_i < len(STEPS):
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern, flags)
            hold = 0.0
            if not found:
                message, colour = 'Show the WHOLE board to the camera', RED
                prev = still_since = None
            else:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                           crit)
                good, message = check(STEPS[step_i],
                                      pose(corners, args.cols, w, h))
                still = (prev is not None and float(np.mean(np.linalg.norm(
                    (corners - prev).reshape(-1, 2), axis=1))) < STILL_PX)
                prev = corners
                if not good:
                    colour, still_since = ORANGE, None
                elif not still:
                    message, colour = 'GOOD - now hold still', GREEN
                    still_since = None
                else:
                    colour = GREEN
                    still_since = still_since or time.time()
                    hold = min(1.0, (time.time() - still_since) / HOLD_S)
                    if hold >= 1.0:
                        imgs.append(corners)
                        captured_at.append(step_i)
                        cv2.imwrite(os.path.join(
                            view_dir, f'view_{len(imgs):02d}.png'), frame)
                        last_capture = time.time()
                        step_i += 1
                        still_since = None

            cv2.imshow('Camera calibration', draw(
                frame, corners if found else None, pattern, step_i, len(imgs),
                message, colour, hold, time.time() - last_capture < 0.5))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                print('quit, nothing saved')
                return
            if key == ord('s') and step_i < len(STEPS):
                step_i += 1
            if key == ord('u') and imgs:
                imgs.pop()
                step_i = captured_at.pop()
        if len(imgs) < MIN_VIEWS:
            print(f'only {len(imgs)} pictures (need {MIN_VIEWS}): too many '
                  f'steps skipped, nothing saved. Run it again.')
            return
        cap.release()   # free the camera now: the result screen waits for a key
        rms, k, d, errs, keep = solve([obj] * len(imgs), imgs, (w, h))
        write_yaml(args.out, args.name, w, h, k, d)
        result_screen(args, rms, k, d, errs, keep, w, h)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
