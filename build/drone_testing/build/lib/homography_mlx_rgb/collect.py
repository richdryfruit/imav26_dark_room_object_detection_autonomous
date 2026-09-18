#!/usr/bin/env python3
"""
Collect MLX↔RGB point correspondences at a fixed depth.

Usage:
    python3 collect.py --depth 1.0
    python3 collect.py --depth 2.0 --camera 0 --data ./data

Controls (live preview):
    Enter  - freeze current MLX + RGB frames for clicking
    q      - quit and save

Controls (pick mode, after Enter):
    Left click on MLX window  - set thermal point
    Left click on RGB window  - set RGB point
    Enter  - save the pair (needs both clicks) and return to live
    r      - discard this capture, back to live
    q      - quit
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

from common import (
    MLX90640Reader,
    THERMAL_DISPLAY_SIZE,
    depth_dir,
    display_xy_to_mlx,
    grid_to_display,
    load_pairs,
    pairs_path,
    save_pairs,
    wait_for_thermal_frame,
)


class ClickState:
    def __init__(self):
        self.mlx: Optional[Tuple[float, float]] = None  # (col, row)
        self.rgb: Optional[Tuple[float, float]] = None  # (x, y)
        self.mlx_disp: Optional[Tuple[int, int]] = None
        self.rgb_disp: Optional[Tuple[int, int]] = None


def _overlay_help(img: np.ndarray, lines: list, color=(255, 255, 255)) -> np.ndarray:
    out = img.copy()
    y = 24
    for line in lines:
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
        y += 22
    return out


def main():
    parser = argparse.ArgumentParser(description="Collect MLX↔RGB correspondences for one depth")
    parser.add_argument("--depth", type=float, required=True, help="Object depth in meters (e.g. 1.0)")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument(
        "--data",
        type=Path,
        default=Path(__file__).resolve().parent / "data",
        help="Root folder for calibration data",
    )
    args = parser.parse_args()

    out_dir = depth_dir(args.data, args.depth)
    out_json = pairs_path(out_dir)
    pairs = load_pairs(out_json)
    print(f"Depth {args.depth:g} m → {out_dir}")
    print(f"Existing pairs: {len(pairs)}")

    reader = MLX90640Reader()
    reader.start()
    cam = cv2.VideoCapture(args.camera)
    if not cam.isOpened():
        reader.close()
        raise RuntimeError(f"Could not open camera index {args.camera}")

    # Warm up thermal
    if wait_for_thermal_frame(reader) is None:
        cam.release()
        reader.close()
        raise RuntimeError("No thermal frame received — check MLX I2C wiring.")

    win_mlx_live = "MLX Live"
    win_rgb_live = "RGB Live"
    win_mlx_pick = "MLX Pick"
    win_rgb_pick = "RGB Pick"
    cv2.namedWindow(win_mlx_live, cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_rgb_live, cv2.WINDOW_NORMAL)

    state = ClickState()
    frozen_mlx = None
    frozen_rgb = None
    pick_mode = False

    def on_mlx_click(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or not pick_mode:
            return
        dw, dh = THERMAL_DISPLAY_SIZE
        col, row = display_xy_to_mlx(x, y, dw, dh)
        state.mlx = (col, row)
        state.mlx_disp = (x, y)
        print(f"  MLX click → col={col:.2f}, row={row:.2f}")

    def on_rgb_click(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or not pick_mode:
            return
        state.rgb = (float(x), float(y))
        state.rgb_disp = (x, y)
        print(f"  RGB click → x={x}, y={y}")

    try:
        print("\nLive: Enter=capture, q=quit")
        while True:
            thermal = reader.get_latest_frame()
            ret, rgb = cam.read()
            if not ret:
                time.sleep(0.01)
                continue
            rgb=cv2.flip(rgb, 1)  # horizontal flip to match MLX orientation
            if thermal is not None and not pick_mode:
                mlx_vis = grid_to_display(thermal.grid)
                mlx_vis = _overlay_help(
                    mlx_vis,
                    [
                        f"depth={args.depth:g}m  pairs={len(pairs)}",
                        "Enter: capture  |  q: quit",
                        f"hot={thermal.max_temp:.1f}C @ {thermal.max_pixel}",
                    ],
                )
                rgb_vis = _overlay_help(
                    rgb,
                    [
                        f"depth={args.depth:g}m  pairs={len(pairs)}",
                        "Enter: capture  |  q: quit",
                    ],
                )
                cv2.imshow(win_mlx_live, mlx_vis)
                cv2.imshow(win_rgb_live, rgb_vis)

            key = cv2.waitKey(1) & 0xFF

            if not pick_mode:
                if key == ord("q"):
                    break
                if key == 13:  # Enter
                    if thermal is None:
                        print("No thermal frame yet, wait a bit.")
                        continue
                    frozen_mlx = thermal.grid.copy()
                    frozen_rgb = rgb.copy()
                    state = ClickState()
                    pick_mode = True
                    cv2.destroyWindow(win_mlx_live)
                    cv2.destroyWindow(win_rgb_live)
                    cv2.namedWindow(win_mlx_pick, cv2.WINDOW_NORMAL)
                    cv2.namedWindow(win_rgb_pick, cv2.WINDOW_NORMAL)
                    cv2.setMouseCallback(win_mlx_pick, on_mlx_click)
                    cv2.setMouseCallback(win_rgb_pick, on_rgb_click)
                    print("Pick mode: click MLX point, click RGB point, Enter=save, r=redo")
                continue

            # ---- pick mode ----
            mlx_img = grid_to_display(frozen_mlx)
            rgb_img = frozen_rgb.copy()
            help_lines = [
                "Click MLX + RGB correspondence",
                "Enter: save pair  |  r: redo  |  q: quit",
            ]
            if state.mlx_disp is not None:
                cv2.circle(mlx_img, state.mlx_disp, 6, (0, 255, 0), 2)
                cv2.drawMarker(mlx_img, state.mlx_disp, (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
            if state.rgb_disp is not None:
                cv2.circle(rgb_img, state.rgb_disp, 8, (0, 255, 0), 2)
                cv2.drawMarker(rgb_img, state.rgb_disp, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

            cv2.imshow(win_mlx_pick, _overlay_help(mlx_img, help_lines, (0, 255, 255)))
            cv2.imshow(win_rgb_pick, _overlay_help(rgb_img, help_lines, (0, 255, 255)))

            if key == ord("q"):
                break
            if key == ord("r"):
                pick_mode = False
                cv2.destroyWindow(win_mlx_pick)
                cv2.destroyWindow(win_rgb_pick)
                cv2.namedWindow(win_mlx_live, cv2.WINDOW_NORMAL)
                cv2.namedWindow(win_rgb_live, cv2.WINDOW_NORMAL)
                print("Capture discarded.")
                continue
            if key == 13:
                if state.mlx is None or state.rgb is None:
                    print("Need both an MLX click and an RGB click before saving.")
                    continue
                pair = {
                    "mlx": [state.mlx[0], state.mlx[1]],
                    "rgb": [state.rgb[0], state.rgb[1]],
                }
                pairs.append(pair)
                save_pairs(out_json, args.depth, pairs)
                print(f"Saved pair #{len(pairs)}: mlx={pair['mlx']} rgb={pair['rgb']}")

                # Also dump a preview image of this correspondence
                preview = np.hstack(
                    [
                        cv2.resize(mlx_img, (frozen_rgb.shape[1], frozen_rgb.shape[0])),
                        rgb_img,
                    ]
                )
                prev_path = out_dir / f"pair_{len(pairs):03d}.jpg"
                cv2.imwrite(str(prev_path), preview)

                pick_mode = False
                cv2.destroyWindow(win_mlx_pick)
                cv2.destroyWindow(win_rgb_pick)
                cv2.namedWindow(win_mlx_live, cv2.WINDOW_NORMAL)
                cv2.namedWindow(win_rgb_live, cv2.WINDOW_NORMAL)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        save_pairs(out_json, args.depth, pairs)
        cam.release()
        cv2.destroyAllWindows()
        reader.close()
        print(f"Done. {len(pairs)} pairs in {out_json}")


if __name__ == "__main__":
    main()
