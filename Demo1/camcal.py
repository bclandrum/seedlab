import cv2
import os
import glob
import numpy as np
import json

# Ensure the images directory exists
os.makedirs('images', exist_ok=True)

# --- Configuration --- #
chessboardSize = (7, 7) # count internal corners ie. grids n-1
size_of_chessboard_squares_mm = 20  # square size in mm
frameSize = (640, 480) #resolution

# --- Video / Image capture --- #
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frameSize[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frameSize[1])

num = 0
# Save / Calibrate / Quit UI
print("Press 's' to save a chessboard image.")
print("Press 'c' to finish capturing and run calibration.")
print("Press 'q' to quit without calibrating.")

while cap.isOpened():
    success, img = cap.read()
    if not success:
        print("Ignoring empty camera frame.")
        continue

    cv2.imshow('Img', img)
    
    k = cv2.waitKey(5) & 0xFF
    if k == ord('q'):  # Fixed string comparison to use ord()
        break
    elif k == ord('s'):  # Press 's' to save
        filename = f'images/img{num}.png'
        cv2.imwrite(filename, img)
        print(f"Image saved as {filename}!")
        num += 1
    elif k == ord('c'):  # Finish capturing and run calibration
        print("\n[INFO] Starting camera calibration...")
        break
cap.release()
cv2.destroyAllWindows()

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (0,0,0), (1,0,0), (2,0,0) ..., scaled by square size
objpt = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objpt[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)
objpt = objpt * size_of_chessboard_squares_mm

objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane

images = glob.glob('images/*.png')
print(f"Processing {len(images)} images for calibration...")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if 'cv' in globals() else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find and handle the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

    if ret == True:
        objpoints.append(objpt)
        corners2=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        cv2.drawChessboardCorners
        cv2.imshow('Calibration Corners',img)
        cv2.waitKey(200)

cv2.destroyAllWindows()
# --- Calibration, Camera Matrix, and Distortion Coefficients --- #
if len(objpoints)<5:
    print("Error: Not enough chessboard images... Take at least 10.")
else:
    # performing calibration
    ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints,imgpoints,frameSize,None,None)

    print("Success, calibration complete!")
    print("Camera Matrix:\n",cameraMatrix)
    print("Distortion Coefficients:\n", dist)

    # Save results as JSON so your main script can read them
    with open("cameraMatrix.json", "w") as f1:
      json.dump(cameraMatrix.tolist(), f1)
    with open("distortionCoef.json", "w") as f2:
      json.dump(dist.tolist(), f2)

    print("Saved camera parameters to 'cameraMatrix.json' and 'distortionCoef.json'")

    # finding reprojection error
    meanError = 0
    for i in range(len(objpoints)):
        imgpoints2,_=cv2.projectPoints(objpoints[i],rvecs[i],tvecs[i],cameraMatrix,dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        meanError += error

    print(f"Total Reprojection Error: {meanError / len(objpoints):.4f} pixels")


