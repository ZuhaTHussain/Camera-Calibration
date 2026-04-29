#!/usr/bin/env python3
"""
Stage 2 — Extrinsic Calibration & Ground-Plane Homography
==========================================================
Uses a single image of a checkerboard lying flat on the road
to recover the camera pose (R, t) and the ground-plane
homography H (image pixels ↔ ground-plane metres).

Requires: calibration_intrinsics.npz from Stage 1.

Your board:  40 mm squares, 6×4 inner corners (7×5 squares)

Usage:
    python extrinsic_calibration.py --image road_board.jpg
                                    --intrinsics calibration_intrinsics.npz
                                    --square_size 0.04
                                    --cols 6
                                    --rows 4

Output:
    calibration_extrinsics.npz  (R, t, H, H_inv, camera_height, K, dist_coeffs)
"""

import argparse
import os
import sys

import cv2
import numpy as np


def run_extrinsic_calibration(
    image_path: str,
    intrinsics_path: str,
    square_size: float,
    cols: int,
    rows: int,
    output_path: str,
    show: bool = False,
):
    # ------------------------------------------------------------------
    # 1.  Load intrinsics from Stage 1
    # ------------------------------------------------------------------
    if not os.path.exists(intrinsics_path):
        print(f"ERROR: Intrinsics file not found: {intrinsics_path}")
        sys.exit(1)

    data = np.load(intrinsics_path, allow_pickle=True)
    K = data["K"]
    dist_coeffs = data["dist_coeffs"]
    print("Loaded intrinsics from Stage 1")
    print(f"  fx={K[0,0]:.2f}  fy={K[1,1]:.2f}  cx={K[0,2]:.2f}  cy={K[1,2]:.2f}\n")

    # ------------------------------------------------------------------
    # 2.  Load and undistort the road-board image
    # ------------------------------------------------------------------
    img = cv2.imread(image_path)
    if img is None:
        print(f"ERROR: Cannot read image: {image_path}")
        sys.exit(1)

    h_img, w_img = img.shape[:2]
    print(f"Image size: {w_img} × {h_img} px")

    undistorted = cv2.undistort(img, K, dist_coeffs, None, K)
    grey = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

    # ------------------------------------------------------------------
    # 3.  Detect checkerboard corners in the undistorted image
    # ------------------------------------------------------------------
    found, corners = cv2.findChessboardCorners(
        grey,
        (cols, rows),
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_FAST_CHECK
        + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )

    if not found:
        print("ERROR: Could not detect checkerboard corners.")
        print("Tips:")
        print("  - Ensure the full board is visible in the image")
        print("  - Check that cols/rows match your board (inner corners)")
        print("  - Try a frame with better lighting / less blur")
        sys.exit(1)

    # Sub-pixel refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(grey, corners, (11, 11), (-1, -1), criteria)
    print(f"Detected {cols}×{rows} = {cols*rows} corners  ✓\n")

    # ------------------------------------------------------------------
    # 4.  Build 3-D object points (ground plane, Z = 0)
    # ------------------------------------------------------------------
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    # objp[i] = (col * sq_size, row * sq_size, 0.0) in metres

    # ------------------------------------------------------------------
    # 5.  Solve PnP → rotation and translation (extrinsics)
    # ------------------------------------------------------------------
    ret, rvec, tvec = cv2.solvePnP(
        objp,
        corners,
        K,
        None,  # already undistorted, so no distortion here
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    if not ret:
        print("ERROR: solvePnP failed.")
        sys.exit(1)

    R, _ = cv2.Rodrigues(rvec)  # 3×3 rotation matrix
    t = tvec.ravel()             # translation vector

    # Camera centre in world coordinates: C = -R^T @ t
    C = -R.T @ t
    camera_height = abs(C[2])

    print("=" * 60)
    print("  EXTRINSIC CALIBRATION RESULTS")
    print("=" * 60)

    print(f"\n  Rotation matrix R:")
    for row in R:
        print(f"    [{row[0]:>10.6f}  {row[1]:>10.6f}  {row[2]:>10.6f}]")

    print(f"\n  Translation vector t:")
    print(f"    [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")

    print(f"\n  Camera centre (world coords):")
    print(f"    X = {C[0]:.4f} m")
    print(f"    Y = {C[1]:.4f} m")
    print(f"    Z = {C[2]:.4f} m")
    print(f"\n  Camera height above road: {camera_height:.4f} m")

    # ------------------------------------------------------------------
    # 6.  Compute ground-plane homography H
    #     H = K @ [r1 | r2 | t]  (drop the Z-column since Z=0)
    # ------------------------------------------------------------------
    r1 = R[:, 0]
    r2 = R[:, 1]
    H = K @ np.column_stack([r1, r2, t])

    # Normalise so H[2,2] = 1 for consistency
    H = H / H[2, 2]

    # Inverse homography: image pixels → ground-plane metres
    H_inv = np.linalg.inv(H)
    H_inv = H_inv / H_inv[2, 2]

    print(f"\n  Ground-plane homography H (pixels → ground):")
    for row in H:
        print(f"    [{row[0]:>12.6f}  {row[1]:>12.6f}  {row[2]:>12.6f}]")

    print(f"\n  Inverse homography H⁻¹ (ground → pixels):")
    for row in H_inv:
        print(f"    [{row[0]:>12.6f}  {row[1]:>12.6f}  {row[2]:>12.6f}]")

    # ------------------------------------------------------------------
    # 7.  Verify: reproject corners through H and check error
    # ------------------------------------------------------------------
    image_pts_2d = corners.reshape(-1, 2)
    ground_pts_2d = objp[:, :2]  # X, Y only

    # Forward check: ground → image via H
    gnd_h = np.hstack([ground_pts_2d, np.ones((len(ground_pts_2d), 1))])  # (N, 3)
    proj = (H @ gnd_h.T).T  # (N, 3)
    proj_2d = proj[:, :2] / proj[:, 2:3]  # dehomogenise

    errors = np.linalg.norm(image_pts_2d - proj_2d, axis=1)
    rms_h = np.sqrt(np.mean(errors**2))

    print(f"\n  Homography reprojection check:")
    print(f"    RMS error: {rms_h:.4f} px")
    print(f"    Max error: {np.max(errors):.4f} px")
    print(f"    Min error: {np.min(errors):.4f} px")

    if rms_h > 2.0:
        print("    ⚠  High reprojection error — check board flatness and corner detection")
    else:
        print("    ✓  Homography looks good")

    # ------------------------------------------------------------------
    # 8.  Compute pixel-to-metre scale at board centroid
    # ------------------------------------------------------------------
    centroid_gnd = np.mean(ground_pts_2d, axis=0)  # (X, Y) in metres
    eps = 0.001  # 1 mm perturbation

    def project_ground_pt(xy):
        h_pt = np.array([xy[0], xy[1], 1.0])
        p = H @ h_pt
        return p[:2] / p[2]

    p0 = project_ground_pt(centroid_gnd)
    px = project_ground_pt(centroid_gnd + np.array([eps, 0]))
    py = project_ground_pt(centroid_gnd + np.array([0, eps]))

    scale = (np.linalg.norm(px - p0) + np.linalg.norm(py - p0)) / (2 * eps)
    print(f"\n  Local scale at board centroid: {scale:.2f} px/m")
    print(f"  (1 pixel ≈ {1.0/scale*1000:.2f} mm at this location)")

    # ------------------------------------------------------------------
    # 9.  Optional: show corners and axes overlay
    # ------------------------------------------------------------------
    if show:
        vis = undistorted.copy()
        cv2.drawChessboardCorners(vis, (cols, rows), corners, found)

        # Draw world axes on the image (X=red, Y=green, Z=blue)
        axis_len = square_size * 3  # 3 squares long
        axis_pts = np.float32([
            [0, 0, 0],
            [axis_len, 0, 0],
            [0, axis_len, 0],
            [0, 0, -axis_len],  # Z points up from road
        ])
        img_axis, _ = cv2.projectPoints(axis_pts, rvec, tvec, K, None)
        img_axis = img_axis.astype(int).reshape(-1, 2)

        origin = tuple(img_axis[0])
        cv2.line(vis, origin, tuple(img_axis[1]), (0, 0, 255), 3)   # X = red
        cv2.line(vis, origin, tuple(img_axis[2]), (0, 255, 0), 3)   # Y = green
        cv2.line(vis, origin, tuple(img_axis[3]), (255, 0, 0), 3)   # Z = blue

        cv2.putText(vis, "X", tuple(img_axis[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(vis, "Y", tuple(img_axis[2] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(vis, "Z", tuple(img_axis[3] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # Resize for display if image is very large
        max_dim = 900
        scale_disp = min(max_dim / vis.shape[1], max_dim / vis.shape[0], 1.0)
        if scale_disp < 1.0:
            vis = cv2.resize(vis, None, fx=scale_disp, fy=scale_disp)

        cv2.imshow("Extrinsic Calibration — Detected Corners + Axes", vis)
        print("\n  Press any key to close the preview window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # ------------------------------------------------------------------
    # 10. Save everything
    # ------------------------------------------------------------------
    np.savez(
        output_path,
        R=R,
        t=t,
        rvec=rvec,
        tvec=tvec,
        H=H,
        H_inv=H_inv,
        camera_height=camera_height,
        camera_centre=C,
        K=K,
        dist_coeffs=dist_coeffs,
        image_size=np.array([w_img, h_img]),
        scale_px_per_m=scale,
    )
    print(f"\n  Saved to: {output_path}")
    print("=" * 60)

    return R, t, H, H_inv, camera_height


# ==================================================================
#   CLI entry point
# ==================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Stage 2 — Extrinsic calibration & ground-plane homography."
    )
    parser.add_argument(
        "--image",
        type=str,
        required=True,
        help="Path to a single image of the checkerboard flat on the road.",
    )
    parser.add_argument(
        "--intrinsics",
        type=str,
        default="calibration_intrinsics.npz",
        help="Path to Stage 1 intrinsics .npz file.",
    )
    parser.add_argument(
        "--square_size",
        type=float,
        default=0.04,
        help="Checkerboard square size in metres (default 0.04 = 40 mm).",
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=6,
        help="Inner corners per row (default 6).",
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=4,
        help="Inner corners per column (default 4).",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="calibration_extrinsics.npz",
        help="Output .npz filename.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show detected corners and world axes overlaid on the image.",
    )

    args = parser.parse_args()

    run_extrinsic_calibration(
        image_path=args.image,
        intrinsics_path=args.intrinsics,
        square_size=args.square_size,
        cols=args.cols,
        rows=args.rows,
        output_path=args.output,
        show=args.show,
    )
