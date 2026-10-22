import cv2
import os
import sys
import glob
import json
import numpy as np

# --- Configuration --- #
chessboardSize = (7, 7)               # INNER corners (squares - 1). Count these on your printout!
size_of_chessboard_squares_mm = 20    # measure the printed squares to confirm
requestedFrameSize = (640, 480)       # requested capture resolution (camera may ignore it)
MIN_IMAGES = 10                       # minimum good images required; aim for 15-25
CLEAR_OLD_IMAGES = True               # delete leftover images from previous runs at startup
IMAGE_DIR = 'images'

# Flags for chessboard detection
detect_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
live_flags = detect_flags + cv2.CALIB_CB_FAST_CHECK  # fast check keeps live preview smooth

# Ensure the images directory exists, and optionally clear stale images
os.makedirs(IMAGE_DIR, exist_ok=True)
if CLEAR_OLD_IMAGES:
    for old in glob.glob(os.path.join(IMAGE_DIR, '*.png')):
        os.remove(old)

# --- Video / Image capture --- #
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: could not open camera.")
    sys.exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, requestedFrameSize[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, requestedFrameSize[1])

num = 0
quit_requested = False
print("Press 's' to save a chessboard image (only saves when the board is detected).")
print("Press 'c' to finish capturing and run calibration.")
print("Press 'q' to quit without calibrating.")

while cap.isOpened():
    success, img = cap.read()
    if not success:
        print("Ignoring empty camera frame.")
        continue

    # Live feedback: detect the board in the current frame and draw it on a copy
    gray_live = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, live_corners = cv2.findChessboardCorners(gray_live, chessboardSize, live_flags)

    display = img.copy()
    if found:
        cv2.drawChessboardCorners(display, chessboardSize, live_corners, found)
    status = f"Board detected - press 's' to save ({num} saved)" if found \
        else f"No board detected ({num} saved)"
    color = (0, 255, 0) if found else (0, 0, 255)
    cv2.putText(display, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.imshow('Img', display)

    k = cv2.waitKey(5) & 0xFF
    if k == ord('q'):
        quit_requested = True
        break
    elif k == ord('s'):
        if found:
            filename = os.path.join(IMAGE_DIR, f'img{num}.png')
            cv2.imwrite(filename, img)  # save the raw frame, not the one with corners drawn
            print(f"Image saved as {filename}!")
            num += 1
        else:
            print("Board not detected in this frame - image not saved.")
    elif k == ord('c'):
        print("\n[INFO] Starting camera calibration...")
        break

cap.release()
cv2.destroyAllWindows()

if quit_requested:
    print("Quitting without calibration.")
    sys.exit(0)

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (0,0,0), (1,0,0), (2,0,0) ..., scaled by square size
objpt = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objpt[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)
objpt = objpt * size_of_chessboard_squares_mm

objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane
frameSize = None  # (width, height), taken from the actual saved images

images = sorted(glob.glob(os.path.join(IMAGE_DIR, '*.png')))
print(f"Processing {len(images)} images for calibration...")

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Warning: could not read {fname}, skipping.")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Use the real image size (width, height), and make sure all images match
    size = gray.shape[::-1]
    if frameSize is None:
        frameSize = size
        print(f"Using image size {frameSize[0]}x{frameSize[1]} for calibration.")
    elif size != frameSize:
        print(f"Warning: {fname} is {size[0]}x{size[1]}, expected {frameSize[0]}x{frameSize[1]}. Skipping.")
        continue

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, detect_flags)

    if ret:
        objpoints.append(objpt)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv2.imshow('Calibration Corners', img)
        cv2.waitKey(200)
    else:
        print(f"No corners found in {fname}.")

cv2.destroyAllWindows()

# --- Calibration, Camera Matrix, and Distortion Coefficients --- #
if len(objpoints) < MIN_IMAGES:
    print(f"Error: only {len(objpoints)} usable chessboard images. Take at least {MIN_IMAGES} (15-25 is better).")
else:
    # performing calibration
    ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, frameSize, None, None)

    print("Success, calibration complete!")
    print("Camera Matrix:\n", cameraMatrix)
    print("Distortion Coefficients:\n", dist)

     # Save results as JSON so your main script can read them
    with open("cameraMatrix.json", "w") as f1:
      json.dump(cameraMatrix.tolist(), f1)
    with open("distortionCoef.json", "w") as f2:
      json.dump(dist.tolist(), f2)

    print("Saved camera parameters to 'cameraMatrix.json' and 'distortionCoef.json'")

    # finding reprojection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    print(f"Total Reprojection Error: {mean_error / len(objpoints):.4f} pixels")