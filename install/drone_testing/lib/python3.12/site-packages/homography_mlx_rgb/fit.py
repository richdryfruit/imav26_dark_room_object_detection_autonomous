#!/usr/bin/env python3
"""
Fit a homography per depth from collected pairs.

Usage:
    python3 fit.py
    python3 fit.py --data ./data
    python3 fit.py --depth 1.0          # only this depth
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from common import (
    apply_homography,
    depth_dir,
    homography_path,
    load_pairs,
    pairs_path,
    pairs_to_arrays,
)


def fit_one(depth_folder: Path) -> bool:
    path = pairs_path(depth_folder)
    if not path.exists():
        print(f"[skip] no pairs.json in {depth_folder}")
        return False

    pairs = load_pairs(path)
    if len(pairs) < 4:
        print(f"[skip] {depth_folder.name}: need ≥4 pairs, have {len(pairs)}")
        return False

    mlx, rgb = pairs_to_arrays(pairs)
    H, mask = cv2.findHomography(mlx, rgb, method=cv2.RANSAC, ransacReprojThreshold=5.0)
    if H is None:
        print(f"[fail] {depth_folder.name}: findHomography returned None")
        return False

    mapped = apply_homography(H, mlx)
    err = np.linalg.norm(mapped - rgb, axis=1)
    inliers = int(mask.sum()) if mask is not None else len(pairs)

    out = homography_path(depth_folder)
    np.save(out, H)

    # Also save a readable sidecar
    meta = depth_folder / "H_meta.txt"
    with open(meta, "w", encoding="utf-8") as f:
        f.write(f"pairs={len(pairs)}\n")
        f.write(f"inliers={inliers}\n")
        f.write(f"mean_reproj_px={err.mean():.3f}\n")
        f.write(f"max_reproj_px={err.max():.3f}\n")
        f.write("H=\n")
        f.write(np.array2string(H, precision=6) + "\n")

    print(
        f"[ok] {depth_folder.name}: {len(pairs)} pairs, "
        f"{inliers} inliers, mean err={err.mean():.2f}px, max={err.max():.2f}px → {out}"
    )
    return True


def main():
    parser = argparse.ArgumentParser(description="Fit MLX→RGB homography per depth")
    parser.add_argument(
        "--data",
        type=Path,
        default=Path(__file__).resolve().parent / "data",
        help="Root folder containing depth_* subfolders",
    )
    parser.add_argument(
        "--depth",
        type=float,
        default=None,
        help="If set, only fit this depth; otherwise fit all depth_* folders",
    )
    args = parser.parse_args()

    if args.depth is not None:
        folders = [depth_dir(args.data, args.depth)]
    else:
        if not args.data.exists():
            raise SystemExit(f"No data root at {args.data}. Run collect.py first.")
        folders = sorted(p for p in args.data.iterdir() if p.is_dir() and p.name.startswith("depth_"))

    if not folders:
        raise SystemExit(f"No depth_* folders under {args.data}")

    ok = 0
    for folder in folders:
        if fit_one(folder):
            ok += 1
    print(f"Fitted {ok}/{len(folders)} depth(s).")


if __name__ == "__main__":
    main()
