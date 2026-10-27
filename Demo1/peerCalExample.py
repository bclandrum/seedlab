import numpy as np
import cv2 as cv
import json
from time import sleep
 
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# Define the size of your squares (e.g., 20.0 for 20mm)
square_size = 20.0 
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2) * square_size
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

camera = cv.VideoCapture(0)
camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
i = 0
images = [] 

print("Press 'f' to capture a clean image.")
print("Press 'c' to finish, calibrate, and save JSON files.")
print("Press 'q' to quit without saving.")

calibrated = False
while True:
    bol, img = camera.read()
    if not bol:
        print("Could not capture image from camera")
        break
    else:
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, (7,7), None)
     
        if ret == True:
            corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            
        key = cv.waitKey(1)
        if key == ord('f') and ret == True:
            # Append a clean copy of the original raw image (no overlays)
            images.append(img.copy())
            objpoints.append(objp)
            imgpoints.append(corners2)
            i = i + 1
            print(f"Captured clean image {i}")
        elif key == ord('c'):
            if i > 5:  # Require at least a few images to calibrate safely
                calibrated = True
                break
            else:
                print("Capture at least a few more images before calibrating!")
        elif key == ord('q'):
            break
            
        # Display clean live video feed with no text or drawing overlays
        cv.imshow('Video', img)
         
if calibrated or i > 0:
    print("Running calibration...")
    cap, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    if cap:
        h, w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        print(f"matrix: {mtx}")
        print(f"dist: {dist}")
        
        # Save camera matrix to JSON
        with open("cameraMatrix.json", "w") as file:
            json.dump(mtx.tolist(), file, indent=4)
            
        # Save distortion coefficients to JSON
        with open("distortionCoef.json", "w") as file:
            json.dump(dist.tolist(), file, indent=4)
            
        print("Saved cameraMatrix.json and distortionCoef.json successfully!")
        
        dst = cv.undistort(img, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imshow('fixed image', dst)
        sleep(5)
    else:
        print("failed to calibrate")
         
cv.destroyAllWindows()
camera.release()
