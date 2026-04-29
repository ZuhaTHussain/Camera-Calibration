#!/usr/bin/env python3
"""
Ground-Plane Distance Measurement Tool
=======================================
Load any image taken with your calibrated camera, click two points
on the ground plane, and get the real-world distance between them.

Requires: calibration_extrinsics.npz from Stage 2
          (contains K, dist_coeffs, H_inv)

Usage:
    python measure_distance.py --image photo.jpg
                               --calibration calibration_extrinsics.npz

Controls:
    Left click   — place a point (2 points needed per measurement)
    'r'          — reset / clear all points
    'q' or ESC   — quit
"""

import argparse
import os
import sys

import cv2
import numpy as np


class DistanceMeasurer:
    def __init__(self, image_path: str, calibration_path: str):
        # Load calibration
        if not os.path.exists(calibration_path):
            print(f"ERROR: Calibration file not found: {calibration_path}")
            sys.exit(1)

        cal = np.load(calibration_path, allow_pickle=True)
        self.K = cal["K"]
        self.dist_coeffs = cal["dist_coeffs"]
        self.H_inv = cal["H_inv"]
        self.camera_height = float(cal["camera_height"])

        print(f"Loaded calibration:")
        print(f"  Camera height: {self.camera_height:.3f} m")
        print(f"  Scale: {float(cal['scale_px_per_m']):.1f} px/m at board centroid\n")

        # Load and undistort image
        img = cv2.imread(image_path)
        if img is None:
            print(f"ERROR: Cannot read image: {image_path}")
            sys.exit(1)

        self.original = cv2.undistort(img, self.K, self.dist_coeffs, None, self.K)
        self.display = self.original.copy()
        self.h_img, self.w_img = self.original.shape[:2]

        # State
        self.points_px = []       # clicked pixel coordinates
        self.points_ground = []   # back-projected ground coordinates
        self.measurements = []    # list of (pt1, pt2, distance) tuples

        # Display scaling for large images
        self.max_display = 900
        self.scale = min(
            self.max_display / self.w_img,
            self.max_display / self.h_img,
            1.0,
        )

        print(f"Image: {self.w_img} × {self.h_img} px")
        if self.scale < 1.0:
            print(f"Display scaled to {self.scale:.0%} for viewing")
        print()
        print("=" * 50)
        print("  CONTROLS")
        print("=" * 50)
        print("  Left click  — place a point on the ground")
        print("  'r'         — reset all points")
        print("  'q' / ESC   — quit")
        print("=" * 50)
        print()

    def pixel_to_ground(self, u, v):
        """Back-project a pixel (u, v) to ground-plane (X, Y) in metres."""
        px_h = np.array([u, v, 1.0])
        gnd_h = self.H_inv @ px_h
        # Dehomogenise
        X = gnd_h[0] / gnd_h[2]
        Y = gnd_h[1] / gnd_h[2]
        return X, Y

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # Convert display coordinates back to original image coordinates
        u = int(x / self.scale)
        v = int(y / self.scale)

        # Back-project to ground plane
        X, Y = self.pixel_to_ground(u, v)
        self.points_px.append((u, v))
        self.points_ground.append((X, Y))

        print(f"  Point {len(self.points_px)}: pixel ({u}, {v}) → ground ({X:.4f}, {Y:.4f}) m")

        # If we have two points, compute distance
        if len(self.points_px) == 2:
            p1 = np.array(self.points_ground[0])
            p2 = np.array(self.points_ground[1])
            dist = np.linalg.norm(p2 - p1)

            dx = abs(p2[0] - p1[0])
            dy = abs(p2[1] - p1[1])

            self.measurements.append((self.points_px[0], self.points_px[1], dist))

            print(f"\n  ┌────────────────────────────────┐")
            print(f"  │  Distance: {dist:.4f} m ({dist*100:.2f} cm) │")
            print(f"  │  ΔX: {dx:.4f} m   ΔY: {dy:.4f} m  │")
            print(f"  └────────────────────────────────┘\n")

            # Reset for next measurement
            self.points_px = []
            self.points_ground = []

        self.redraw()

    def redraw(self):
        """Redraw the display image with all measurements and current points."""
        self.display = self.original.copy()

        # Draw completed measurements
        for i, (pt1, pt2, dist) in enumerate(self.measurements):
            color = (0, 255, 0)  # green
            cv2.circle(self.display, pt1, 8, color, -1)
            cv2.circle(self.display, pt2, 8, color, -1)
            cv2.line(self.display, pt1, pt2, color, 2)

            # Label
            mid_x = (pt1[0] + pt2[0]) // 2
            mid_y = (pt1[1] + pt2[1]) // 2

            label = f"{dist:.3f} m"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)

            # Background rectangle for readability
            cv2.rectangle(
                self.display,
                (mid_x - 5, mid_y - th - 10),
                (mid_x + tw + 5, mid_y + 5),
                (0, 0, 0),
                -1,
            )
            cv2.putText(
                self.display,
                label,
                (mid_x, mid_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        # Draw current (incomplete) point
        if len(self.points_px) == 1:
            pt = self.points_px[0]
            cv2.circle(self.display, pt, 8, (0, 0, 255), -1)  # red
            cv2.putText(
                self.display,
                "Click second point...",
                (pt[0] + 15, pt[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        # Instructions overlay at top
        overlay_text = "Click 2 points to measure | R=Reset | Q=Quit"
        cv2.rectangle(self.display, (0, 0), (self.w_img, 35), (0, 0, 0), -1)
        cv2.putText(
            self.display,
            overlay_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

    def run(self):
        """Main event loop."""
        window_name = "Ground-Plane Distance Measurement"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        # Set window size
        disp_w = int(self.w_img * self.scale)
        disp_h = int(self.h_img * self.scale)
        cv2.resizeWindow(window_name, disp_w, disp_h)

        cv2.setMouseCallback(window_name, self.on_mouse)

        self.redraw()

        while True:
            # Resize for display
            if self.scale < 1.0:
                show = cv2.resize(self.display, (disp_w, disp_h))
            else:
                show = self.display

            cv2.imshow(window_name, show)
            key = cv2.waitKey(30) & 0xFF

            if key in (ord("q"), 27):  # q or ESC
                break
            elif key == ord("r"):
                self.points_px = []
                self.points_ground = []
                self.measurements = []
                self.redraw()
                print("  [Reset] All points cleared.\n")

        cv2.destroyAllWindows()

        # Print summary
        if self.measurements:
            print("\n" + "=" * 50)
            print("  MEASUREMENT SUMMARY")
            print("=" * 50)
            for i, (pt1, pt2, dist) in enumerate(self.measurements):
                print(f"  {i+1}. ({pt1[0]},{pt1[1]}) → ({pt2[0]},{pt2[1]}): {dist:.4f} m ({dist*100:.2f} cm)")
            print("=" * 50)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Measure ground-plane distances by clicking on an image."
    )
    parser.add_argument(
        "--image",
        type=str,
        required=True,
        help="Path to an image taken with the calibrated camera.",
    )
    parser.add_argument(
        "--calibration",
        type=str,
        default="calibration_extrinsics.npz",
        help="Path to the extrinsic calibration .npz file from Stage 2.",
    )

    args = parser.parse_args()

    measurer = DistanceMeasurer(args.image, args.calibration)
    measurer.run()
