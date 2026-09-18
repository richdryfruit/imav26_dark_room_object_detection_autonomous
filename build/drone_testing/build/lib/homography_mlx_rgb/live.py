#!/usr/bin/env python3
"""
Live demo: map MLX hottest pixel → RGB using the homography for a given depth.

Usage:
    python3 live.py --depth 1.0
    python3 live.py --depth 2.0 --camera 0 --data ./data
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from common import (
    MLX90640Reader,
    THERMAL_DISPLAY_SIZE,
    apply_homography,
    depth_dir,
    grid_to_display,
    homography_path,
    mlx_to_display_xy,
    wait_for_thermal_frame,
)


def main():
    parser = argparse.ArgumentParser(description="Live hottest-point MLX→RGB mapping")
    parser.add_argument("--depth", type=float, required=True, help="Depth (m) whose H.npy to load")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument(
        "--data",
        type=Path,
        default=Path(__file__).resolve().parent / "data",
        help="Root folder for calibration data",
    )
    parser.add_argument("--box", type=int, default=60, help="Half-size of RGB marker box (px)")
    args = parser.parse_args()

    h_file = homography_path(depth_dir(args.data, args.depth))
    if not h_file.exists():
        raise SystemExit(f"Missing {h_file}. Run collect.py then fit.py for this depth.")

    H = np.load(h_file)
    print(f"Loaded H from {h_file}")

    reader = MLX90640Reader()
    reader.start()
    cam = cv2.VideoCapture(args.camera)
    if not cam.isOpened():
        reader.close()
        raise RuntimeError(f"Could not open camera index {args.camera}")

    if wait_for_thermal_frame(reader) is None:
        cam.release()
        reader.close()
        raise RuntimeError("No thermal frame received — check MLX I2C wiring.")

    win_mlx = "MLX Live"
    win_rgb = "RGB Mapped"
    cv2.namedWindow(win_mlx, cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_rgb, cv2.WINDOW_NORMAL)

    dw, dh = THERMAL_DISPLAY_SIZE

    try:
        print("Live mapping. Press q to quit.")
        while True:
            thermal = reader.get_latest_frame()
            ret, rgb = cam.read()
            if not ret or thermal is None:
                time.sleep(0.01)
                continue
            rgb=cv2.flip(rgb, 1)  # horizontal flip to match MLX orientation
            row, col = thermal.max_pixel  # (row, col) from unravel_index
            mlx_pt = np.array([[float(col), float(row)]], dtype=np.float32)
            rgb_pt = apply_homography(H, mlx_pt)[0]
            cx, cy = int(round(rgb_pt[0])), int(round(rgb_pt[1]))

            # Thermal viz
            mlx_vis = grid_to_display(thermal.grid)
            hx, hy = mlx_to_display_xy(col, row, dw, dh)
            cv2.drawMarker(mlx_vis, (hx, hy), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
            cv2.circle(mlx_vis, (hx, hy), 8, (0, 255, 255), 2)
            cv2.putText(
                mlx_vis,
                f"hot {thermal.max_temp:.1f}C  ({col},{row})",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # RGB viz
            h, w = rgb.shape[:2]
            cx_c = int(np.clip(cx, 0, w - 1))
            cy_c = int(np.clip(cy, 0, h - 1))
            half = args.box
            tl = (max(cx_c - half, 0), max(cy_c - half, 0))
            br = (min(cx_c + half, w - 1), min(cy_c + half, h - 1))
            cv2.rectangle(rgb, tl, br, (0, 0, 255), 2)
            cv2.drawMarker(rgb, (cx_c, cy_c), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
            cv2.putText(
                rgb,
                f"map=({cx},{cy})  depth={args.depth:g}m  {thermal.max_temp:.1f}C",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

            cv2.imshow(win_mlx, mlx_vis)
            cv2.imshow(win_rgb, rgb)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cam.release()
        cv2.destroyAllWindows()
        reader.close()


if __name__ == "__main__":
    main()
