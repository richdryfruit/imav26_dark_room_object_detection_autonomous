"""Shared helpers for MLX↔RGB homography calibration."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

# Import MLX90640Reader from the existing acquisition module.
_TFT_STATUS = Path(__file__).resolve().parents[1] / "arduino" / "tft_status"
if str(_TFT_STATUS) not in sys.path:
    sys.path.insert(0, str(_TFT_STATUS))

from thermal_image import MLX90640Reader  # noqa: E402

THERMAL_H, THERMAL_W = 24, 32
THERMAL_DISPLAY_SIZE = (640, 480)  # (width, height)

def flip_rgb_image(image: np.ndarray) -> np.ndarray:
    """Flip RGB image to match MLX orientation."""
    return cv2.flip(image, 1)  # horizontal flip
def depth_dir(data_root: Path, depth: float) -> Path:
    # Stable folder name, e.g. depth_1p5 for 1.5 m
    tag = f"{depth:g}".replace(".", "p")
    return data_root / f"depth_{tag}"


def pairs_path(depth_folder: Path) -> Path:
    return depth_folder / "pairs.json"


def homography_path(depth_folder: Path) -> Path:
    return depth_folder / "H.npy"


def grid_to_display(
    grid: np.ndarray,
    size: Tuple[int, int] = THERMAL_DISPLAY_SIZE,
) -> np.ndarray:
    """24x32 float °C → BGR heatmap at `size` (w, h)."""
    gmin, gmax = float(np.min(grid)), float(np.max(grid))
    if gmax <= gmin:
        gmax = gmin + 0.1
    norm = np.uint8((grid - gmin) * 255.0 / (gmax - gmin))
    heat = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
    return cv2.resize(heat, size, interpolation=cv2.INTER_CUBIC)


def display_xy_to_mlx(x: float, y: float, disp_w: int, disp_h: int) -> Tuple[float, float]:
    """Map click on resized thermal image → (col, row) in 32x24 sensor space."""
    col = x / disp_w * THERMAL_W
    row = y / disp_h * THERMAL_H
    return float(col), float(row)


def mlx_to_display_xy(
    col: float,
    row: float,
    disp_w: int,
    disp_h: int,
) -> Tuple[int, int]:
    """Map (col, row) in 32x24 → pixel on resized thermal image."""
    x = (col + 0.5) / THERMAL_W * disp_w
    y = (row + 0.5) / THERMAL_H * disp_h
    return int(round(x)), int(round(y))


def load_pairs(path: Path) -> List[dict]:
    if not path.exists():
        return []
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return list(data.get("pairs", []))


def save_pairs(path: Path, depth: float, pairs: List[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "depth_m": depth,
        "mlx_coords": "col_row_in_32x24",
        "rgb_coords": "x_y_pixels",
        "pairs": pairs,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def pairs_to_arrays(pairs: List[dict]) -> Tuple[np.ndarray, np.ndarray]:
    """Return Nx2 float32 arrays: mlx (col,row), rgb (x,y)."""
    mlx = np.array([p["mlx"] for p in pairs], dtype=np.float32)
    rgb = np.array([p["rgb"] for p in pairs], dtype=np.float32)
    return mlx, rgb


def apply_homography(H: np.ndarray, pts_xy: np.ndarray) -> np.ndarray:
    """pts_xy: (N,2) → (N,2) mapped with H."""
    pts = pts_xy.reshape(-1, 1, 2).astype(np.float32)
    out = cv2.perspectiveTransform(pts, H)
    return out.reshape(-1, 2)


def wait_for_thermal_frame(reader: MLX90640Reader, timeout_s: float = 10.0):
    import time

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        frame = reader.get_latest_frame()
        if frame is not None:
            return frame
        time.sleep(0.05)
    return None
