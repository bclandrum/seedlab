import numpy as np
import cv2
import glob
import json

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points for an 8x8 checkerboard (7x7 internal corners)
# multiplied by 20 to represent the 20mm square dimensions
objp = np.zeros((7*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:7].T.reshape(-1, 2) * 20

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Load your 640x480 images
images = glob.glob('*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7, 7), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (7, 7), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate the camera to get the camera matrix (mtx) and distortion coefficients (dist)
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Convert NumPy arrays to standard Python lists for JSON serialization
camera_matrix_data = mtx.tolist()
distortion_data = dist.tolist()

# Write the camera matrix to a JSON file
with open('camera_matrix.json', 'w') as f:
    json.dump(camera_matrix_data, f, indent=4)

# Write the distortion coefficients to a JSON file
with open('distortion_coefficients.json', 'w') as f:
    json.dump(distortion_data, f, indent=4)

print("Calibration complete. Data saved to 'camera_matrix.json' and 'distortion_coefficients.json'.")
