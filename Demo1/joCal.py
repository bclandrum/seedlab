#!/usr/bin/env python3
"""
Camera calibration with OpenCV, based on the OpenCV "Camera Calibration" tutorial.

Setup assumed:
  - Chessboard with 8x8 squares  -> 7x7 INNER corners (what OpenCV detects)
  - Square size: 20 mm
  - Image resolution: 640x480

Usage:
  1) Calibrate from a folder of saved images (default folder: calib_images/):
         python calibrate_camera.py
         python calibrate_camera.py --images "my_photos/*.png"

  2) Capture images from a webcam first (SPACE = save frame, ESC/q = finish):
         python calibrate_camera.py --capture

Outputs (in the current directory):
  cameraMatrix.json
  distortionCoef.json
"""

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

# ----------------------------- Configuration -----------------------------
# An 8x8-square board has 7x7 inner corners. If your board actually has
# 8x8 inner corners (9x9 squares), change this to (8, 8).
PATTERN_SIZE = (7, 7)       # inner corners per row, per column
SQUARE_SIZE_MM = 20.0       # size of one square in millimetres
IMAGE_SIZE = (640, 480)     # (width, height)

CAMERA_MATRIX_FILE = "cameraMatrix.json"
DISTORTION_FILE = "distortionCoef.json"
# -------------------------------------------------------------------------

CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def capture_images(out_dir, camera_index=0):
    """Grab frames from a webcam; press SPACE to save a frame with a found board."""
    os.makedirs(out_dir, exist_ok=True)
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_SIZE[1])
    if not cap.isOpened():
        sys.exit("Could not open the camera.")

    count = len(glob.glob(os.path.join(out_dir, "*.png")))
    print("SPACE = save frame (only when the board is detected), ESC or q = done")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, None)

        preview = frame.copy()
        if found:
            cv2.drawChessboardCorners(preview, PATTERN_SIZE, corners, found)
        cv2.putText(preview, f"saved: {count}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("capture", preview)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord(" ") and found:
            path = os.path.join(out_dir, f"calib_{count:03d}.png")
            cv2.imwrite(path, frame)
            count += 1
            print("saved", path)

    cap.release()
    cv2.destroyAllWindows()


def calibrate(image_glob, show=False):
    # Object points: (0,0,0), (20,0,0), (40,0,0), ... in millimetres
    objp = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    objpoints = []  # 3D points in real-world space
    imgpoints = []  # 2D points in the image plane

    images = sorted(glob.glob(image_glob))
    if not images:
        sys.exit(f"No images found matching: {image_glob}")

    used = 0
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"  [skip] could not read {fname}")
            continue

        if (img.shape[1], img.shape[0]) != IMAGE_SIZE:
            print(f"  [warn] {fname} is {img.shape[1]}x{img.shape[0]}, "
                  f"expected {IMAGE_SIZE[0]}x{IMAGE_SIZE[1]}; resizing")
            img = cv2.resize(img, IMAGE_SIZE)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, None)

        if not found:
            print(f"  [skip] no chessboard found in {fname}")
            continue

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
        objpoints.append(objp)
        imgpoints.append(corners2)
        used += 1
        print(f"  [ok]   {fname}")

        if show:
            cv2.drawChessboardCorners(img, PATTERN_SIZE, corners2, found)
            cv2.imshow("corners", img)
            cv2.waitKey(300)

    if show:
        cv2.destroyAllWindows()

    if used < 5:
        sys.exit(f"Only {used} usable image(s). Use at least 10-20 views "
                 f"of the board from different angles and distances.")

    print(f"\nCalibrating with {used} images...")
    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, IMAGE_SIZE, None, None)

    # Mean reprojection error (as in the OpenCV tutorial)
    total_error = 0.0
    for i in range(len(objpoints)):
        projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        total_error += cv2.norm(imgpoints[i], projected, cv2.NORM_L2) / len(projected)
    mean_error = total_error / len(objpoints)

    print(f"RMS reprojection error (calibrateCamera): {rms:.4f} px")
    print(f"Mean reprojection error:                   {mean_error:.4f} px")
    return mtx, dist


def save_json(path, array):
    with open(path, "w") as f:
        json.dump(np.asarray(array).tolist(), f, indent=4)
    print("Wrote", path)


def main():
    parser = argparse.ArgumentParser(description="OpenCV camera calibration")
    parser.add_argument("--images", default="calib_images/*.png",
                        help='glob for calibration images (default: "calib_images/*.png")')
    parser.add_argument("--capture", action="store_true",
                        help="capture images from a webcam into the folder first")
    parser.add_argument("--camera", type=int, default=0, help="webcam index")
    parser.add_argument("--show", action="store_true",
                        help="display detected corners while processing")
    args = parser.parse_args()

    if args.capture:
        capture_images(os.path.dirname(args.images) or "calib_images", args.camera)

    mtx, dist = calibrate(args.images, show=args.show)

    print("\nCamera matrix:\n", mtx)
    print("\nDistortion coefficients (k1, k2, p1, p2, k3):\n", dist.ravel())

    save_json(CAMERA_MATRIX_FILE, mtx)
    save_json(DISTORTION_FILE, dist)


if __name__ == "__main__":
    main()
