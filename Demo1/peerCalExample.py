import numpy as np
import cv2 as cv
import glob
import json
from time import sleep
 
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# put camera capture count on image
coordinates = (50, 50)       # (X, Y) position of the top-left corner of the text
font = cv.FONT_HERSHEY_SIMPLEX   # Font style
font_scale = 1               # Font size multiplier
color = (0, 255, 0)          # BGR format (This represents Green)
thickness = 2                # Line thickness
 
camera = cv.VideoCapture(0)
camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
i = 0
images = [] 

while i < 30:
    bol, img = camera.read()
    if not bol:
        print("Could not capture image from camera")
        break
    else:
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (7,7), None)
     
        # If found, add object points, image points (after refining them)
        if ret == True:
            corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
         
            # Draw and display the corners
            cv.drawChessboardCorners(img, (7,7), corners2, ret)
            if cv.waitKey(1) == ord('f'):
                images.append(img)
                objpoints.append(objp)
                imgpoints.append(corners2)
                i = i + 1
        text = f"images: {i}/30"

        # Draw the text onto the image
        cv.putText(img, text, coordinates, font, font_scale, color, thickness, cv.LINE_AA)
        cv.imshow('Video', img)
        if cv.waitKey(1) == ord('q'):
            break
         
if i > 29:
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
            
        dst = cv.undistort(img, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imshow('fixed image', dst)
        sleep(5)
    else:
        print("failed to calibrate")
         
cv.destroyAllWindows()
camera.release()
