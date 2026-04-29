#!/usr/bin/env python3
"""
Stage 1 — Intrinsic Camera Calibration
=======================================
Uses a checkerboard pattern to recover camera intrinsics (K) and
distortion coefficients via Zhang's method (OpenCV implementation).

Your board:  25 mm squares, 10×7 inner corners (11×8 squares)
Capture:     15–20 images at varied angles/positions

Usage:
    python intrinsic_calibration.py --image_dir ./calibration_images
                                    --square_size 0.025
                                    --cols 10
                                    --rows 7

Output:
    calibration_intrinsics.npz  (K, dist_coeffs, image_size, rms_error)
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np


def find_images(image_dir: str) -> list[str]:
    """Find all common image files in a directory."""
    extensions = ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.tiff", "*.tif")
    paths = []
    for ext in extensions:
        paths.extend(glob.glob(os.path.join(image_dir, ext)))
        paths.extend(glob.glob(os.path.join(image_dir, ext.upper())))
    paths = sorted(set(paths))
    return paths


def calibrate_intrinsics(
    image_dir: str,
    square_size: float,
    cols: int,
    rows: int,
    output_path: str,
    show_detections: bool = False,
):
    """
    Run intrinsic calibration from checkerboard images.

    Parameters
    ----------
    image_dir : str
        Folder containing calibration images.
    square_size : float
        Physical side length of one checkerboard square [metres].
    cols : int
        Number of inner corners per row  (horizontal count).
    rows : int
        Number of inner corners per column (vertical count).
    output_path : str
        Where to save the .npz calibration file.
    show_detections : bool
        If True, display each image with detected corners overlaid.
    """

    # ------------------------------------------------------------------
    # 1.  Build the known 3-D object points (Z = 0 for all)
    # ------------------------------------------------------------------
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    # objp[i] = (col_i * square_size, row_i * square_size, 0.0)

    object_points = []   # list of 3-D point arrays (one per accepted image)
    image_points = []    # list of 2-D corner arrays (one per accepted image)
    accepted_files = []  # filenames that passed corner detection

    # ------------------------------------------------------------------
    # 2.  Detect checkerboard corners in every image
    # ------------------------------------------------------------------
    image_paths = find_images(image_dir)
    if not image_paths:
        print(f"ERROR: No images found in '{image_dir}'")
        sys.exit(1)

    print(f"Found {len(image_paths)} images in '{image_dir}'\n")
    print(f"Board: {cols}×{rows} inner corners, square = {square_size*1000:.0f} mm\n")

    image_size = None  # (width, height) — set from first image

    criteria_subpix = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,    # max iterations
        0.001, # epsilon
    )

    for i, fpath in enumerate(image_paths):
        img = cv2.imread(fpath)
        if img is None:
            print(f"  [{i+1:>2}] SKIP (unreadable): {os.path.basename(fpath)}")
            continue

        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Record image dimensions (must be consistent across all images)
        h, w = grey.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif (w, h) != image_size:
            print(f"  [{i+1:>2}] SKIP (size mismatch {w}x{h}): {os.path.basename(fpath)}")
            continue

        found, corners = cv2.findChessboardCorners(
            grey,
            (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if not found:
            print(f"  [{i+1:>2}] FAIL (corners not found): {os.path.basename(fpath)}")
            continue

        # Sub-pixel refinement — critical for accuracy
        corners_refined = cv2.cornerSubPix(
            grey, corners, winSize=(11, 11), zeroZone=(-1, -1), criteria=criteria_subpix
        )

        object_points.append(objp)
        image_points.append(corners_refined)
        accepted_files.append(os.path.basename(fpath))
        print(f"  [{i+1:>2}]   OK : {os.path.basename(fpath)}")

        # Optional: show corners overlaid on image
        if show_detections:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, (cols, rows), corners_refined, found)
            cv2.imshow("Detected Corners", vis)
            key = cv2.waitKey(500)
            if key == 27:  # ESC to stop previewing
                show_detections = False

    if show_detections:
        cv2.destroyAllWindows()

    print(f"\nAccepted: {len(accepted_files)} / {len(image_paths)} images")

    if len(accepted_files) < 10:
        print(
            "WARNING: Fewer than 10 usable images. "
            "Aim for 15–20 with varied angles for reliable calibration."
        )
    if len(accepted_files) < 4:
        print("ERROR: Need at least 4 images to calibrate. Exiting.")
        sys.exit(1)

    # ------------------------------------------------------------------
    # 3.  Run calibration (Zhang's method via OpenCV)
    # ------------------------------------------------------------------
    print("\nRunning calibration (this may take a moment)...")

    rms, K, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,       # initial K  (None = auto-initialise)
        None,       # initial dist (None = auto-initialise)
    )

    # ------------------------------------------------------------------
    # 4.  Report results
    # ------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("  INTRINSIC CALIBRATION RESULTS")
    print("=" * 60)
    print(f"\n  RMS reprojection error:  {rms:.4f} px", end="")
    if rms < 0.5:
        print("  ✓  (target < 0.5 px)")
    else:
        print("  ⚠  (above 0.5 px — consider recapturing)")

    print(f"\n  Image size: {image_size[0]} × {image_size[1]} px")

    print(f"\n  Intrinsic matrix K:")
    print(f"    fx = {K[0, 0]:.2f} px")
    print(f"    fy = {K[1, 1]:.2f} px")
    print(f"    cx = {K[0, 2]:.2f} px")
    print(f"    cy = {K[1, 2]:.2f} px")
    print(f"\n    K = ")
    for row in K:
        print(f"        [{row[0]:>10.2f}  {row[1]:>10.2f}  {row[2]:>10.2f}]")

    dist = dist_coeffs.ravel()
    labels = ["k1", "k2", "p1", "p2", "k3"]
    print(f"\n  Distortion coefficients:")
    for j, lbl in enumerate(labels):
        if j < len(dist):
            print(f"    {lbl} = {dist[j]:>12.6f}")

    # ------------------------------------------------------------------
    # 5.  Per-image reprojection error breakdown
    # ------------------------------------------------------------------
    print(f"\n  Per-image reprojection errors:")
    per_image_errors = []
    for idx in range(len(object_points)):
        projected, _ = cv2.projectPoints(
            object_points[idx], rvecs[idx], tvecs[idx], K, dist_coeffs
        )
        err = cv2.norm(image_points[idx], projected, cv2.NORM_L2) / len(projected)
        per_image_errors.append(err)
        flag = "  ⚠" if err > 1.0 else ""
        print(f"    {accepted_files[idx]:<30s}  {err:.4f} px{flag}")

    worst = max(per_image_errors)
    best = min(per_image_errors)
    print(f"\n    Best:  {best:.4f} px")
    print(f"    Worst: {worst:.4f} px")
    print(f"    Mean:  {np.mean(per_image_errors):.4f} px")

    if worst > 1.0:
        worst_idx = per_image_errors.index(worst)
        print(
            f"\n  TIP: '{accepted_files[worst_idx]}' has high error. "
            "Consider removing it and re-running."
        )

    # ------------------------------------------------------------------
    # 6.  Save calibration to disk
    # ------------------------------------------------------------------
    np.savez(
        output_path,
        K=K,
        dist_coeffs=dist_coeffs,
        image_size=np.array(image_size),
        rms_error=rms,
        rvecs=np.array(rvecs, dtype=object),
        tvecs=np.array(tvecs, dtype=object),
    )
    print(f"\n  Saved to: {output_path}")
    print("=" * 60)

    return K, dist_coeffs, image_size, rms


# ==================================================================
#   CLI entry point
# ==================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Stage 1 — Intrinsic camera calibration from checkerboard images."
    )
    parser.add_argument(
        "--image_dir",
        type=str,
        default="./calibration_images",
        help="Folder containing checkerboard calibration photos.",
    )
    parser.add_argument(
        "--square_size",
        type=float,
        default=0.025,
        help="Side length of one checkerboard square in metres (default 0.025 = 25 mm).",
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=10,
        help="Inner corners per row (default 10 for your 11×8 square board).",
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=7,
        help="Inner corners per column (default 7 for your 11×8 square board).",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="calibration_intrinsics.npz",
        help="Output .npz filename.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show detected corners on each image (press ESC to stop).",
    )

    args = parser.parse_args()

    calibrate_intrinsics(
        image_dir=args.image_dir,
        square_size=args.square_size,
        cols=args.cols,
        rows=args.rows,
        output_path=args.output,
        show_detections=args.show,
    )
